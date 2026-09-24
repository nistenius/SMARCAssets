// DockSceneBuilder.cs — builds KTHTankDock.unity from KTHTankStrips.unity, by script (2026-09-23,
// docking work order R1). BUILT-UNFLOWN: written off-rig, never compiled or run by its author.
//
// Menu: SMARC ▸ Docking ▸ Build KTHTankDock scene
// (the work order says "SMaRC/Docking/..."; every other builder in this package lives under
//  "SMARC/", and Unity menu paths are case-sensitive, so "SMaRC" would open a SECOND top-level
//  menu. Filed under the existing one.)
//
// WHY BY SCRIPT (same reason as WaveTestSceneBuilder): the setup must be repeatable and
// reviewable, and the console must say exactly what was removed and every number it used.
//
// WHAT IT DOES, IN ORDER — and it REFUSES (logs an error, saves nothing) on any failed check:
//   1. opens KTHTankStrips.unity (never saves it — the result is written with saveAsCopy);
//   2. deletes the `sam_auv_v1.strips` instance (prefab guid 78dc5058…), recording its transform;
//   3. deletes every GameObject that is a SAMTankReplay / PitchSpeedRig / SAMDofTest diagnostic —
//      DELETED, not disabled (silencing is not disabling, SETTLED §3ae(52));
//   4. instantiates sam21.strips (guid ce851120…) at the old transform, names the instance
//      `VehicleName` (IN UNITY THE GAMEOBJECT NAME IS THE ROS NAMESPACE, §3ak), and REMOVES the
//      SAMTankReplay component that sam21.strips.prefab ITSELF CARRIES (RunOnStart 1,
//      TimeScale 4, WorldOffset (-151.14,0,-210.67) — read from the prefab 2026-09-23: dropping the
//      prefab into a tank scene as-is starts a 4x-speed replay that teleports the hull). The
//      removal is a scene-level prefab-instance override; the PREFAB IS NOT TOUCHED;
//   5. asserts exactly ONE WaterQueryModel in the scene;
//   6. READS the tank from the scene (never hardcoded): floor y from `The Tank/Collisions/Floor`
//      (the physics floor; the visual floor box is logged beside it), wall x from
//      `Collisions/Street` and `Collisions/Office`, lateral centre from `Left`/`Right`, water
//      surface y from `The Tank/Water`;
//   7. generates the dock meshes + materials + a low-friction physics material as assets under
//      Packages/com.smarc.assets/Runtime/Prefabs/Environment/KTH Tank/Dock/, builds the
//      DockStation hierarchy (NO Rigidbody — rigid and STATIC, Ivan 2026-09-23), saves it as
//      DockStation.prefab there, and places an instance at the Office end, axis along the tank's
//      long axis, mouth facing Street, axis 0.35 m above the physics floor;
//   8. checks the start box fits (tail clearance to the Street wall, depth of the axis below the
//      surface) and logs every number; saves Assets/Scenes/KTHTankDock.unity and opens it.
//
// WHERE THINGS PERSIST: the scene file KTHTankDock.unity (SMARCUnity repo); the Dock/ assets and
// DockStation.prefab (SMARCAssets repo). sam21.strips.prefab and hydro_overrides.txt: untouched.

using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using DefaultNamespace.Water;
using Docking;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace SmarcEditor
{
    public static class DockSceneBuilder
    {
        const string SourceScene = "Assets/Scenes/KTHTankStrips.unity";
        const string TargetScene = "Assets/Scenes/KTHTankDock.unity";
        const string StripsGuid = "ce8511200665c476fa5814bcf550d9a0";     // sam21.strips.prefab
        const string OldVehicleGuid = "78dc5058883a641faaee2929e01bb783"; // sam_auv_v1.strips.prefab
        const string AssetDir = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/KTH Tank/Dock";

        // The GameObject name IS the ROS namespace (ROSBehaviour: topic = /{robotGO.name}/{topic}).
        // This is the ONE place the tank-dock scene states it (DockStation holds a GameObject
        // REFERENCE, not a name string, so it cannot drift). It must equal fleet.yaml's sim
        // namespace for sam_mk2_02 (DC_ROBOT_NAME=sam21 after vm1_namespace_cutover.sh). Run
        // `SMARC ▸ Check ROS Namespace Consistency` on the built scene. Rule (§3ak): name the object
        // after the hull it DRAWS — this scene draws sam21.strips, the SAM 2.1 hull.
        const string VehicleName = "sam21";

        // Dock geometry (work order §2). Change here and rebuild; the Inspector shows the as-built values.
        const float MouthD = 0.60f, ThroatD = 0.21f,   // ROUND dock v1 (Ivan 2026-09-23): 0.16 could not admit the real hull (envelope Ø0.191, top rail r 0.0953); keyed throat is v2
                            ConeLen = 0.50f, TubeLen = 0.80f, Wall = 0.01f;
        const float AxisHeight = 0.35f;        // above the PHYSICS floor
        const float EndClearance = 0.30f;      // tube end cap to the Office wall
        // Marker board: the work order's "four discs in a 0.30 m square around the mouth rim" cannot
        // exist — a 0.30 m square's corners are at r = 0.21 m, INSIDE the 0.30 m mouth radius. Built
        // instead: a 0.80 x 0.44 m rectangle (corners at r = 0.46 m) + a fifth disc at (+0.45, 0)
        // (right-middle: breaks the left/right symmetry). Bottom discs clear the floor by 0.09 m.
        const float MarkerPitchX = 0.80f, MarkerPitchY = 0.44f, MarkerDiscD = 0.08f;
        static readonly Vector2 FifthDisc = new Vector2(0.45f, 0f);
        const float BoardHalfW = 0.55f, BoardTop = 0.32f, BoardThickness = 0.012f;
        const float StartFromMouth = 5.5f;     // must equal DockLaunchBox.StartFromMouth
        const float HullHalfLength = 0.66f;    // Collisions capsule tip (sam21.strips), for the fit check

        [MenuItem("SMARC/Docking/Build KTHTankDock scene")]
        public static void Build()
        {
            var log = new List<string>();
            System.Action<string> Fail = why =>
                Debug.LogError("[DockSceneBuilder] REFUSED, nothing saved: " + why + "\n  - " + string.Join("\n  - ", log));

            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            if (File.Exists(TargetScene) &&
                !EditorUtility.DisplayDialog("KTHTankDock exists", $"{TargetScene} exists. Rebuild it from {SourceScene}? (the old file is overwritten)", "Rebuild", "Cancel"))
                return;

            var scene = EditorSceneManager.OpenScene(SourceScene, OpenSceneMode.Single);
            if (!scene.IsValid()) { Fail($"could not open {SourceScene}"); return; }
            var roots = scene.GetRootGameObjects().ToList();

            // ---- 2. the old vehicle -------------------------------------------------------------
            var oldVeh = roots.FirstOrDefault(r => PrefabUtility.IsAnyPrefabInstanceRoot(r) &&
                AssetDatabase.AssetPathToGUID(PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(r)) == OldVehicleGuid);
            if (oldVeh == null) { Fail("no sam_auv_v1.strips instance (guid 78dc5058…) at scene root"); return; }
            Vector3 vPos = oldVeh.transform.position;
            Quaternion vRot = oldVeh.transform.rotation;
            log.Add($"deleted old vehicle '{oldVeh.name}' (sam_auv_v1.strips) at {vPos.ToString("F3")} rot {vRot.eulerAngles.ToString("F1")}");
            Object.DestroyImmediate(oldVeh);

            // ---- 3. diagnostics: delete the objects ----------------------------------------------
            string[] diag = { "SAMTankReplay", "PitchSpeedRig", "SAMDofTest" };
            foreach (var mb in Object.FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (mb == null || !diag.Contains(mb.GetType().Name)) continue;
                var go = mb.gameObject;
                if (PrefabUtility.IsPartOfPrefabInstance(go) && !PrefabUtility.IsOutermostPrefabInstanceRoot(go) && go.transform.parent != null)
                {
                    log.Add($"removed component {mb.GetType().Name} from '{go.name}' (inside a prefab instance)");
                    Object.DestroyImmediate(mb);
                }
                else
                {
                    log.Add($"deleted GameObject '{go.name}' ({mb.GetType().Name})");
                    Object.DestroyImmediate(go);
                }
            }

            // ---- 4. the new vehicle ---------------------------------------------------------------
            var prefabPath = AssetDatabase.GUIDToAssetPath(StripsGuid);
            var prefab = string.IsNullOrEmpty(prefabPath) ? null : AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
            if (prefab == null) { Fail("sam21.strips.prefab (guid ce851120…) not found"); return; }
            var sam = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
            sam.transform.SetPositionAndRotation(vPos, vRot);
            sam.name = VehicleName;
            log.Add($"instantiated {prefabPath} as '{VehicleName}' at the old transform");
            foreach (var mb in sam.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb == null || !diag.Contains(mb.GetType().Name)) continue;
                log.Add($"removed {mb.GetType().Name} from the new instance ('{mb.gameObject.name}') — scene override, prefab untouched");
                Object.DestroyImmediate(mb);
            }
            if (sam.GetComponentsInChildren<MonoBehaviour>(true).Any(m => m != null && diag.Contains(m.GetType().Name)))
            { Fail("a diagnostic component survived removal on the new vehicle"); return; }
            if (GameObject.FindGameObjectsWithTag("robot").Length != 1)
            { Fail($"{GameObject.FindGameObjectsWithTag("robot").Length} #robot objects in the scene (want exactly 1)"); return; }

            // ---- 5. one WaterQueryModel -----------------------------------------------------------
            var wqm = Object.FindObjectsByType<WaterQueryModel>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            log.Add($"WaterQueryModel count: {wqm.Length} ({string.Join(", ", wqm.Select(w => w.name + ":" + w.GetType().Name))})");
            if (wqm.Length != 1) { Fail($"{wqm.Length} WaterQueryModels (want exactly ONE)"); return; }

            // ---- 6. read the tank ------------------------------------------------------------------
            var world = roots.FirstOrDefault(r => r != null && r.name == "KTHTank World");
            if (world == null) { Fail("no 'KTHTank World' root"); return; }
            System.Func<string, Collider> Col = path =>
            {
                var t = world.transform.Find(path);
                return t != null ? t.GetComponent<Collider>() : null;
            };
            var floor = Col("The Tank/Collisions/Floor");
            var street = Col("The Tank/Collisions/Street");
            var office = Col("The Tank/Collisions/Office");
            var left = Col("The Tank/Collisions/Left");
            var right = Col("The Tank/Collisions/Right");
            var water = world.transform.Find("The Tank/Water");
            var visFloor = Col("The Tank/Visuals/Floor");
            if (floor == null || street == null || office == null || left == null || right == null)
            { Fail("tank colliders not found under 'KTHTank World/The Tank/Collisions/{Floor,Street,Office,Left,Right}'"); return; }
            float floorY = floor.bounds.max.y;
            float streetX = street.bounds.center.x, officeX = office.bounds.center.x;
            float midZ = 0.5f * (left.bounds.center.z + right.bounds.center.z);
            float waterY = water != null ? water.position.y : float.NaN;
            log.Add($"TANK (read from the scene): physics floor y {floorY:F4} (Collisions/Floor bounds {floor.bounds.min.y:F4}..{floor.bounds.max.y:F4})" +
                    (visFloor != null ? $" | visual floor box top y {visFloor.bounds.max.y:F4} (NOT used; {visFloor.bounds.max.y - floorY:+0.000;-0.000} m from physics floor)" : ""));
            log.Add($"TANK: Street x {streetX:F4}  Office x {officeX:F4}  => length {Mathf.Abs(officeX - streetX):F3} m | Left z {left.bounds.center.z:F4} Right z {right.bounds.center.z:F4} => centre z {midZ:F4}, width {Mathf.Abs(left.bounds.center.z - right.bounds.center.z):F3} m");
            log.Add($"TANK: water plane y {waterY:F4} (The Tank/Water transform; it may be an HDRP surface, not the rim) => depth {waterY - floorY:F3} m");

            // ---- 7. the station --------------------------------------------------------------------
            float sgn = Mathf.Sign(officeX - streetX);                // axis into the tube points Street -> Office
            Vector3 axis = new Vector3(sgn, 0f, 0f);
            float throatX = officeX - sgn * (EndClearance + Wall + TubeLen);
            var throat = new Vector3(throatX, floorY + AxisHeight, midZ);
            var station = BuildStationAssetsAndInstance(scene, throat, Quaternion.LookRotation(axis, Vector3.up), floorY, log);
            if (station == null) { Fail("station build failed (see above)"); return; }
            station.Vehicle = sam;
            PrefabUtility.RecordPrefabInstancePropertyModifications(station);
            log.Add($"DockStation throat centre {throat.ToString("F4")}, axis {axis}, mouth plane x {throatX - sgn * ConeLen:F4}, tube end x {throatX + sgn * (TubeLen + Wall):F4}");

            // ---- 8. fit checks ------------------------------------------------------------------------
            float startX = throatX - sgn * (ConeLen + StartFromMouth);
            float tailX = startX - sgn * HullHalfLength;
            float tailClear = sgn * (tailX - streetX);
            float axisDepth = waterY - throat.y;
            log.Add($"START BOX: base_link x {startX:F3} (x_D {-(ConeLen + StartFromMouth):F2}), tail x {tailX:F3}, clearance to Street wall {tailClear:F3} m" +
                    (tailClear < 0.30f ? "  <-- WARNING: < 0.30 m, shorten StartFromMouth" : ""));
            log.Add($"AXIS depth below the water plane {axisDepth:F3} m; start depth band +-0.2 m => {axisDepth - 0.2f:F2}..{axisDepth + 0.2f:F2} m");
            log.Add($"THROAT (round, v1): Ø {ThroatD:F3} m. Radial clearance about the axis with TIGHT colliders " +
                    $"(SamTightColliderBuilder): top rail {(ThroatD / 2f - 0.0953f) * 1000f:F1} mm (binding, upward), " +
                    $"side-scan pods {(ThroatD / 2f - 0.0768f) * 1000f:F1} mm, bare hull {(ThroatD / 2f - 0.0631f) * 1000f:F1} mm. " +
                    "With the OLD capsule (r 0.0696) it would read " + $"{(ThroatD / 2f - 0.0696f) * 1000f:F1} mm everywhere — run the tight-collider step first.");
            if (tailClear < 0f) { Fail("the start box puts the hull THROUGH the Street wall"); return; }

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, TargetScene, true);
            EditorSceneManager.OpenScene(TargetScene, OpenSceneMode.Single);
            Debug.Log("[DockSceneBuilder] built " + TargetScene + "\n  - " + string.Join("\n  - ", log));
        }

        // ---------------------------------------------------------------------------------------------
        static GameObject Child(string n, Transform parent)
        {
            var g = new GameObject(n);
            g.transform.SetParent(parent, false);
            return g;
        }

        static DockGate Gate(Transform parent, DockStation st, string n, string gateName, float planeX, float size)
        {
            var g = Child(n, parent);
            g.layer = 2;                                   // Ignore Raycast
            g.transform.localPosition = new Vector3(0f, 0f, planeX);
            var bc = g.AddComponent<BoxCollider>();
            bc.isTrigger = true; bc.size = new Vector3(size, size, 0.02f);
            var dg = g.AddComponent<DockGate>();
            dg.GateName = gateName; dg.PlaneX = planeX; dg.Station = st;
            return dg;
        }

        static T SaveAsset<T>(T obj, string path) where T : Object
        {
            if (AssetDatabase.LoadAssetAtPath<Object>(path) != null) AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(obj, path);
            return AssetDatabase.LoadAssetAtPath<T>(path);
        }

        static Material Mat(string name, Color c, bool unlit)
        {
            Shader sh = Shader.Find(unlit ? "HDRP/Unlit" : "HDRP/Lit") ?? Shader.Find(unlit ? "Unlit/Color" : "Standard");
            var m = new Material(sh) { name = name };
            if (m.HasProperty("_UnlitColor")) m.SetColor("_UnlitColor", c);
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            if (m.HasProperty("_Color")) m.SetColor("_Color", c);
            return m;
        }

        static DockStation BuildStationAssetsAndInstance(Scene scene, Vector3 pos, Quaternion rot, float floorY, List<string> log)
        {
            // AssetDatabase, not System.IO: "Packages/com.smarc.assets" is a VIRTUAL path (a file:
            // package), and Directory.CreateDirectory would create a real folder under
            // SMARCUnity/Packages/ that shadows the package.
            if (!AssetDatabase.IsValidFolder(AssetDir))
            {
                string parent = Path.GetDirectoryName(AssetDir).Replace('\\', '/');
                if (!AssetDatabase.IsValidFolder(parent)) { Debug.LogError($"[DockSceneBuilder] {parent} is not a valid asset folder"); return null; }
                AssetDatabase.CreateFolder(parent, Path.GetFileName(AssetDir));
            }

            float holeR = 0.5f * MouthD + Wall / Mathf.Cos(Mathf.Atan2(0.5f * (MouthD - ThroatD), ConeLen));
            float bottom = Mathf.Min(BoardTop, AxisHeight - 0.02f);
            var shell = SaveAsset(DockGeometry.Shell(MouthD, ThroatD, ConeLen, TubeLen, Wall), AssetDir + "/DockShell.asset");
            var board = SaveAsset(DockGeometry.Board(holeR, BoardHalfW, BoardTop, bottom, -ConeLen - 0.002f, BoardThickness), AssetDir + "/DockMarkerBoard.asset");
            var disc = SaveAsset(DockGeometry.Disc(0.5f * MarkerDiscD), AssetDir + "/DockMarkerDisc.asset");
            var mBody = SaveAsset(Mat("DockBody", new Color(0.55f, 0.57f, 0.60f), false), AssetDir + "/DockBody.mat");
            var mBoard = SaveAsset(Mat("DockBoard", new Color(0.05f, 0.05f, 0.06f), false), AssetDir + "/DockBoard.mat");
            var mDisc = SaveAsset(Mat("DockMarker", Color.white, true), AssetDir + "/DockMarker.mat");
            var pm = new PhysicsMaterial("DockLowFriction")
            {
                dynamicFriction = 0.05f, staticFriction = 0.05f, bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Minimum, bounceCombine = PhysicsMaterialCombine.Minimum
            };
            pm = SaveAsset(pm, AssetDir + "/DockLowFriction.physicMaterial");
            log.Add($"assets written to {AssetDir}: DockShell/DockMarkerBoard/DockMarkerDisc meshes, 3 materials, DockLowFriction (mu 0.05, combine Minimum — so a hull on the 23.7 deg cone wall slides in)");

            var root = new GameObject("DockStation");
            var st = root.AddComponent<DockStation>();
            st.MouthDiameter = MouthD; st.ThroatDiameter = ThroatD; st.ConeLength = ConeLen; st.TubeLength = TubeLen;
            st.WallThickness = Wall; st.AxisHeightAboveFloor = AxisHeight; st.FloorWorldY = floorY;
            var lb = root.AddComponent<DockLaunchBox>();
            lb.Station = st; lb.StartFromMouth = StartFromMouth;

            var goShell = Child("Shell", root.transform);
            goShell.AddComponent<MeshFilter>().sharedMesh = shell;
            goShell.AddComponent<MeshRenderer>().sharedMaterial = mBody;
            var mc = goShell.AddComponent<MeshCollider>();
            mc.sharedMesh = shell; mc.convex = false; mc.sharedMaterial = pm;
            goShell.AddComponent<DockContact>().Station = st;

            var goBoard = Child("MarkerBoard", root.transform);
            goBoard.AddComponent<MeshFilter>().sharedMesh = board;
            goBoard.AddComponent<MeshRenderer>().sharedMaterial = mBoard;
            var mcb = goBoard.AddComponent<MeshCollider>();
            mcb.sharedMesh = board; mcb.convex = false; mcb.sharedMaterial = pm;
            var dcb = goBoard.AddComponent<DockContact>(); dcb.Station = st; dcb.IsBoard = true;

            var goMarkers = Child("Markers", root.transform);
            var spots = new List<Vector2> {
                new Vector2(-0.5f * MarkerPitchX, 0.5f * MarkerPitchY), new Vector2(0.5f * MarkerPitchX, 0.5f * MarkerPitchY),
                new Vector2(-0.5f * MarkerPitchX, -0.5f * MarkerPitchY), new Vector2(0.5f * MarkerPitchX, -0.5f * MarkerPitchY),
                FifthDisc };
            var sb = new StringBuilder();
            for (int i = 0; i < spots.Count; i++)
            {
                var d = Child($"Disc{i}", goMarkers.transform);
                d.transform.localPosition = new Vector3(spots[i].x, spots[i].y, -ConeLen - 0.004f);
                d.AddComponent<MeshFilter>().sharedMesh = disc;
                d.AddComponent<MeshRenderer>().sharedMaterial = mDisc;
                sb.Append($" ({spots[i].x:F2},{spots[i].y:F2})");
            }
            log.Add($"marker discs (dock-local x right, y up, in the mouth plane) Ø{MarkerDiscD}:{sb}");

            Gate(root.transform, st, "Gate_Mouth", "mouth", -ConeLen, MouthD);
            Gate(root.transform, st, "Gate_Throat", "throat", 0f, ThroatD);

            var prefabPath = AssetDir + "/DockStation.prefab";
            var inst = PrefabUtility.SaveAsPrefabAssetAndConnect(root, prefabPath, InteractionMode.AutomatedAction);
            SceneManager.MoveGameObjectToScene(inst, scene);
            inst.transform.SetPositionAndRotation(pos, rot);
            log.Add($"saved {prefabPath} and placed an instance (NO Rigidbody: static by decision)");
            return inst.GetComponent<DockStation>();
        }
    }
}
