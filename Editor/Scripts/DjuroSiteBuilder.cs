using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

using GeoRef;
using Smarc.Environment;

/// <summary>
/// Builds the Djurö CURATED site from the scenario bundle produced by
/// data-cube/scripts/djuro-site/{build_heightmap.py, build_splatmap.py, make_bundle.py} :
///
///   DjuroCurated_C.asset      ONE TerrainData, 4096 m at 1 m (4097^2)
///   DjuroCuratedWorld.prefab  cloned from DjuroWorld.prefab for Ocean/Sun/Sky
///   DjuroCurated.unity        the scene
///
/// The heavy lifting is <see cref="CuratedSiteBuilder"/>, shared with Askö. What is Djurö's
/// own and lives here: the 119 charted rock obstructions, the measured launch point, and the
/// vehicle + GUI placed on it.
///
/// **THE SEABED HERE IS 100% CHART-DERIVED.** No bathymetric survey of Djurö exists. Every
/// water cell is interpolated from Sjöfartsverket S-57 contours, soundings, depth areas and
/// the coastline-as-0-m-isobath; the land has a REAL charted footprint and an INVENTED
/// height. So the gap-patch toggle means something different here than at Askö: switching
/// AskoGapPatchLayer.showSyntheticPatches off at Askö removes 59.6% of the seabed, and at
/// Djurö **it removes the entire seabed**. That is correct and is the point — with the patch
/// off, a bottom-following mission at Djurö is flying a chart, and the toggle proves it.
/// The rocks stay (separate colliders) and the land stays (its footprint is charted).
///
/// Frame: Unity +X = UTM 34N easting, +Z = UTM 34N northing, origin = Ivan's site pin
/// (data-cube/maps/sites.yaml `djuro`: 59.306517 N, 18.709717 E, confidence user_provided).
/// **UTM ZONE 34** — asserted on every build. Askö is 33 and this builder's ancestor was
/// Askö's; inheriting the zone would put the whole scene in the wrong hemisphere of the
/// grid. At this site UTM 34N and SWEREF99 TM differ by 5.1609 deg of grid north.
///
/// THE LEGACY DjuroWorld FRAME IS 6.24 m AWAY. DjuroWorld.prefab carries its own
/// "GLOBALREF - DjuroPier" at 59.3065052 N / 18.7096099 E = UTM 34 (369599.779, 6576424.052),
/// which is 6.24 m from the registered origin (dE -6.1, dN -1.1). Its hand-built jetty is
/// therefore NOT carried into the curated scene — it would sit 6 m from where it was placed.
/// The builder re-asserts the offset on every run.
///
/// Run from menu: SMARC -> Build Djuro Curated Site
/// Safe to re-run; overwrites the generated assets, never the saved scene.
/// </summary>
public static class DjuroSiteBuilder
{
    const string DataDir = "Packages/com.smarc.assets/Runtime/Terrain/Djuro";
    const string PrefabDir = CuratedSiteBuilder.PrefabDir;
    const string DjuroWorldPath = PrefabDir + "/DjuroWorld.prefab";
    const string WorldPrefabPath = PrefabDir + "/DjuroCuratedWorld.prefab";
    const string ScenePath = "Assets/Scenes/DjuroCurated.unity";
    const string WorldName = "DjuroCuratedWorld";

    // The legacy frame, measured from DjuroWorld.prefab (see the class comment).
    const double LegacyOriginUtmE = 369599.779, LegacyOriginUtmN = 6576424.052;

    // THE VEHICLE IS SAM 2.1 (Ivan, 2026-09-15): every Djurö bag (2024-10-25, 2024-11-18)
    // was recorded with the 2.1 hull, and the sim's 2.1 is `sam21.strips.prefab` — the
    // prefab that carries the 09-13/14 buoyancy, trim, drive and hydrodynamics fixes and
    // the tank-replay gate (SETTLED §3ae(29)-(33); it was named sam2.2.strips until the
    // 09-14 naming correction). Order of preference below; `sam_auv_v1.prefab` (the
    // untuned original) is the last resort, not the default. Whichever is found, THE OBJECT
    // NAME IS `sam21` (Ivan, 2026-09-15): the GameObject name IS the ROS namespace --
    // ROSBehaviour reads robot_name = robotGO.name and builds /{robot_name}/{topic} and tf
    // frame {robot_name}/{link} from it -- so the name must say 2.1 hull, and it must be
    // `sam21` and not `sam2.1`/`sam21.strips` because a dot is still ILLEGAL in a ROS name.
    static readonly string[] VehiclePrefabs = {
        "Packages/com.smarc.assets/Runtime/Prefabs/sam21.strips.prefab",
        "Packages/com.smarc.assets/Runtime/Prefabs/sam21.prefab",
        "Packages/com.smarc.assets/Runtime/Prefabs/sam_auv_v1.prefab",
    };
    const string VehicleName = "sam21";

    // THE SUPPORT VESSEL AND THE BASE STATION. Askö puts `datacube_station_01` on the SHORE
    // (TerrainHeightAt + 0.05). Djurö is not a shore deployment: Ivan, 2026-09-15 — "there is a
    // prefab of the boat Milou and the base station that sits on the boat with the uw-com
    // transducer in the water." So here the station is a CHILD of the boat, and the boat's
    // position is MEASURED, not eyeballed: `/cm_station/core/gps` on the campaign's station bag
    // (643 fixes, 70 x 199 m box, median 0.75 m/s — a boat tending an AUV), written into
    // djuro_site.json by `support_vessel.py`.
    //
    // DO NOT be tempted by `/sam/core/gps`: that is SAM's OWN receiver, and underwater it reads
    // 7.4 m/s and wanders 5.2 km. It is neither the vehicle nor the boat. SETTLED §3ae(50)/(51).
    //
    // The transducer needs nothing from us. `DeployedTransducer` keeps the case's X/Z and takes Y
    // from `waterPlaneY - DipDepthM`, so parenting the case to the boat puts the fish over the
    // side at its dip depth and keeps it there however the boat moves — which is the property the
    // real deployment guarantees and a fixed offset below the case does not.
    const string MilouPrefab   = "Packages/com.smarc.assets/Runtime/Prefabs/Milou.prefab";
    const string StationPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/datacube_station_01.prefab";
    const string MilouName     = "Milou";
    const string StationName   = "datacube_station_01";
    // On the fore deck, in Milou's own frame. The hull's ForcePoints span x ±0.92, z ±2.85.
    static readonly Vector3 StationOnDeck = new Vector3(0f, 0.60f, 1.90f);



    static string VehiclePrefabPath()
    {
        foreach (var p in VehiclePrefabs)
            if (AssetDatabase.LoadAssetAtPath<GameObject>(p) != null) return p;
        return null;
    }
    const string GuiPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/SmarcGUI/GUI.prefab";

    // Yaw at the launch: the manifest's measured course (launch_point.py: bearing from the
    // campaign's start fix to the first fix 50 m along the track). Before 2026-09-15 the
    // launch was the cove by the origin and the yaw a constant 270 (west, towards the bay);
    // that constant is only the fallback for a manifest without heading_deg.
    const float LaunchYawFallbackDeg = 270f;
    static float LaunchYaw(CuratedSiteBuilder.LaunchPointJ lp)
        => lp.heading_deg > 0f ? lp.heading_deg : LaunchYawFallbackDeg;

    public static readonly CuratedSiteConfig Config = new CuratedSiteConfig
    {
        prefix = "djuro",
        assetPrefix = "DjuroCurated",
        layerPrefix = "Djuro_",
        tag = "[Djuro]",
        dataDir = DataDir,
        bundleRel = "../smds-cloud-store/scenario-bundles/ov-site-Djuro-curated-v1/payload",
        sourceWorldPrefab = DjuroWorldPath,
        worldPrefabPath = WorldPrefabPath,
        worldObjectName = WorldName,
        scenePath = ScenePath,
        globalRefName = "GLOBALREF - DjuroSiteOrigin",
        expectedUtmZone = 34,
        legacyOriginUtm = new[] { LegacyOriginUtmE, LegacyOriginUtmN },
        legacyOriginWhat = "DjuroWorld.prefab's own GLOBALREF - DjuroPier",
        gapPatchSemantics =
            "with AskoGapPatchLayer.showSyntheticPatches OFF, THIS SITE HAS NO SEABED AT " +
            "ALL. 100% of the water is chart-derived, so every water cell becomes a terrain " +
            "hole: no render, no collision, no sonar return. A bottom-following mission " +
            "here is flying a chart, and the toggle is how you see that. Rocks stay (they " +
            "are separate colliders) and land stays (its footprint is charted; only its " +
            "height is invented). Unlike Askö, switching the patch off changes NOTHING " +
            "about rights — the FUK licence shaped every cell either way.",
        wireNeighbours = false,          // one tile; SetNeighbors would be a lie
        terrainPhysicMaterial = "Mud",
        sceneExistsHint =
            "To (re)place the rocks, the vehicle and the GUI run " +
            "SMARC -> Populate Djuro Scene. ",
        sceneCreatedHint =
            "\n       rocks, vehicle and GUI are added next by Populate Djuro Scene.",
    };

    // ---- mirror of djuro_rocks.json ---------------------------------------
    [Serializable]
    public class RockJ
    {
        public double lon, lat, utm_e, utm_n;
        public float unity_x, unity_z, top_z, radius_m;
        public int watlev;
        public bool absorbed_by_seabed;
        public float heightmap_top_z;
        public string cell, lnam, physics_material;
    }
    [Serializable]
    public class RocksJ
    {
        public string generated_utc, generator, frame, physics_material;
        public int count;
        public RockJ[] rocks;
    }

    [MenuItem("SMARC/Build Djuro Curated Site")]
    public static void Build()
    {
        CuratedSiteBuilder.Build(Config);
        PopulateScene();
    }

    /// <summary>
    /// ADDITIVE population of the Djurö scene: the charted rocks, the vehicle at the
    /// MEASURED launch point, and the GUI. Never deletes or replaces anything already there;
    /// existing objects are reused by name. Safe to run repeatedly.
    /// </summary>
    [MenuItem("SMARC/Populate Djuro Scene (rocks + vehicle + GUI)")]
    public static void PopulateScene()
    {
        var world = GameObject.Find(WorldName);
        if (world == null)
        {
            if (File.Exists(CuratedSiteBuilder.AssetPathToFull(ScenePath)))
            {
                EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
                world = GameObject.Find(WorldName);
            }
        }
        if (world == null)
        {
            Debug.LogError("[Djuro] the open scene has no '" + WorldName + "' — run " +
                           "SMARC -> Build Djuro Curated Site first.");
            return;
        }

        var site = CuratedSiteBuilder.LoadSite(Config);
        PlaceRocks(world, site);
        PlaceVehicle(world, site);

        EditorSceneManager.MarkAllScenesDirty();
        EditorSceneManager.SaveOpenScenes();
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log("[Djuro] populate DONE and scene saved.");
    }

    // ------------------------------------------------------------------------
    // The charted rocks.
    //
    // These are the one part of this scene that is not interpolated: an S-57 UWTROC is a
    // surveyed obstruction, recorded because it can sink a boat. They are separate colliders
    // rather than terrain, for two reasons that both matter: a 4 m bump on a 1 m lattice is
    // four samples and rounds away, and a collider can carry its OWN physics material, so a
    // sonar ray that hits a rock returns `Rock` and not the seabed's `Mud`.
    //
    // No UWTROC in these cells carries VALSOU, so the TOP is a convention: WATLEV 3 (always
    // submerged) -> -0.5 m, WATLEV 5 (awash) -> 0.0 m. djuro_rocks.json says so in its own
    // `top_z_convention` block; it is not a sounding and must never be quoted as one.
    // ------------------------------------------------------------------------
    static void PlaceRocks(GameObject world, CuratedSiteBuilder.SiteJ site)
    {
        string path;
        try { path = CuratedSiteBuilder.PayloadFull(Config, "djuro_rocks.json"); }
        catch (FileNotFoundException e)
        {
            Debug.LogWarning("[Djuro] no djuro_rocks.json — the charted obstructions are NOT " +
                             "in this scene: " + e.Message);
            return;
        }
        var data = JsonUtility.FromJson<RocksJ>(File.ReadAllText(path));
        if (data == null || data.rocks == null || data.rocks.Length == 0)
        {
            Debug.LogWarning("[Djuro] djuro_rocks.json parsed to no rocks.");
            return;
        }

        var parent = world.transform.Find("DjuroRocks");
        if (parent == null)
        {
            var go = new GameObject("DjuroRocks");
            go.transform.SetParent(world.transform, false);
            go.transform.localPosition = Vector3.zero;
            parent = go.transform;
        }

        // sharedMaterial, NOT material. In the editor, touching Collider.material
        // INSTANTIATES a copy that is never saved to an asset, so the scene ends up with 119
        // "Rock (Instance)" materials that vanish on reload and a sonar that reads the
        // default 0.5 for all of them. That is the Kristineberg trap (SETTLED §3f) and it
        // costs a session every time it is rediscovered.
        var rockMat = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(
            CuratedSiteBuilder.PhysicMaterialPath("Rock"));
        if (rockMat == null)
            Debug.LogError("[Djuro] Rock.physicMaterial not found — every rock would return " +
                           "the sonar's default hardness. " +
                           CuratedSiteBuilder.PhysicMaterialPath("Rock"));

        var visual = AssetDatabase.LoadAssetAtPath<Material>(
            CuratedSiteBuilder.MatDir + "/DefaultHDTerrainLitMaterial.mat");

        int made = 0, reused = 0, awash = 0, absorbed = 0;
        foreach (var r in data.rocks)
        {
            string name = "Rock_" + (string.IsNullOrEmpty(r.lnam) ? r.unity_x + "_" + r.unity_z
                                                                  : r.lnam);
            var t = parent.Find(name);
            GameObject go;
            if (t != null) { go = t.gameObject; reused++; }
            else
            {
                go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = name;
                go.transform.SetParent(parent, false);
                made++;
            }
            float rad = r.radius_m > 0 ? r.radius_m : 4f;
            // The sphere's TOP sits at the charted top; the rest is buried in the seabed.
            go.transform.localPosition = new Vector3(r.unity_x, r.top_z - rad, r.unity_z);
            go.transform.localScale = new Vector3(rad * 2f, rad * 2f, rad * 2f);
            var col = go.GetComponent<SphereCollider>();
            if (col != null && rockMat != null) col.sharedMaterial = rockMat;
            var mr = go.GetComponent<MeshRenderer>();
            if (mr != null && visual != null) mr.sharedMaterial = visual;
            if (r.watlev == 5) awash++;
            if (r.absorbed_by_seabed) absorbed++;
        }

        Debug.Log($"[Djuro] rocks: {data.rocks.Length} charted UWTROC obstructions " +
                  $"({made} created, {reused} reused), {awash} awash (WATLEV 5, top 0.0 m), " +
                  $"{data.rocks.Length - awash} always submerged (WATLEV 3, top -0.5 m). " +
                  $"{absorbed} of them stand no higher than the chart's own shallow-water " +
                  $"surface around them and are recorded as absorbed_by_seabed. Physics " +
                  $"material: {data.physics_material} (sharedMaterial, so it survives a " +
                  "reload). Tops are a WATLEV CONVENTION — no UWTROC here carries VALSOU.");

        // The check, run rather than claimed: drop a ray onto one rock and report what the
        // physics engine says the surface is. If this prints anything but Rock, the sonar
        // will read the seabed's hardness off an obstruction.
        Physics.SyncTransforms();
        var probe = data.rocks[0];
        var from = new Vector3(probe.unity_x, probe.top_z + 25f, probe.unity_z);
        if (Physics.Raycast(from, Vector3.down, out var hit, 60f))
            Debug.Log($"[Djuro] rock raycast self-check at ({probe.unity_x:F1}, " +
                      $"{probe.unity_z:F1}): hit '{hit.collider.gameObject.name}' at " +
                      $"y {hit.point.y:F2} m (charted top {probe.top_z:F2}), physics material " +
                      $"'{(hit.collider.sharedMaterial != null ? hit.collider.sharedMaterial.name : "NONE")}'");
        else
            Debug.LogWarning("[Djuro] rock raycast self-check hit nothing — colliders may not " +
                             "have been rebuilt yet; re-run Populate after the scene reloads.");
    }

    // ------------------------------------------------------------------------
    // The launch point is MEASURED, not chosen. Since 2026-09-15 (Ivan: "SAM is moved to
    // the approximate start point for the Djurö tests") it is the first dead-reckoned
    // surface fix of the earliest 2024-11-18 mission whose start has >= 3 m of charted
    // water — sam_good2.bag, Unity (-274.8, 134.4), 25 m from where Ivan dragged the
    // vehicle by eye — with the heading measured along that track (launch_point.py).
    // sam_good.bag's start was rejected and recorded: the chart says 1.4 m there, the DVL
    // measured 5.4 m. The earlier rule (nearest water cell to the origin, the cove) is kept
    // in the manifest as launch_point_cove. The site ANCHOR is Ivan's pin, which sits
    // within ~5 m of the charted shoreline — a reference point, not a place to put a vehicle.
    // ------------------------------------------------------------------------
    static void PlaceVehicle(GameObject world, CuratedSiteBuilder.SiteJ site)
    {
        if (site.launch_point == null || site.launch_point.unity == null)
        {
            Debug.LogWarning("[Djuro] djuro_site.json has no launch_point — run " +
                             "launch_point.py (or build_heightmap.py). Vehicle NOT placed.");
            return;
        }
        var lp = site.launch_point;
        var launchPos = new Vector3(lp.unity.x, -0.1f, lp.unity.z);
        float yaw = LaunchYaw(lp);
        float seabed = CuratedSiteBuilder.TerrainHeightAt(world, lp.unity.x, lp.unity.z);
        Debug.Log($"[Djuro] launch point (measured by {lp.measured_by}): Unity ({lp.unity.x:F1}, -0.1, " +
                  $"{lp.unity.z:F1}) = {lp.lat:F6} N {lp.lon:F6} E, heading {yaw:F1} deg; manifest " +
                  $"says the seabed is {lp.seabed_m:F2} m there, the built terrain samples " +
                  $"{seabed:F2} m. Rule: {lp.rule}");
        if (!float.IsNaN(seabed) && Mathf.Abs(seabed - lp.seabed_m) > 0.5f)
            Debug.LogWarning($"[Djuro] the terrain and the manifest disagree about the seabed " +
                             $"at the launch point by {Mathf.Abs(seabed - lp.seabed_m):F2} m — " +
                             "the heightmap and djuro_site.json are out of step.");

        string want = VehiclePrefabPath();
        if (want == null)
        {
            Debug.LogError("[Djuro] none of the vehicle prefabs exist: " + string.Join(", ", VehiclePrefabs));
            return;
        }
        var sam = GameObject.Find(VehicleName);
        if (sam == null)
        {
            sam = CuratedSiteBuilder.Place(Config, want, VehicleName, launchPos, yaw);
            if (sam != null)
                Debug.Log($"[Djuro] {VehicleName} instantiated from {want} at the launch, " +
                          $"heading {yaw:F1} deg ({lp.vehicle}).");
        }
        else
        {
            // Populate is ADDITIVE: an existing vehicle is never destroyed here. Two things
            // are checked and SAID, never silently fixed: which prefab it came from, and
            // whether it stands on land. Use "Replace Djuro Vehicle" to swap it.
            var src = PrefabUtility.GetCorrespondingObjectFromOriginalSource(sam);
            string have = src != null ? AssetDatabase.GetAssetPath(src) : "(not a prefab instance)";
            if (have != want)
                Debug.LogWarning($"[Djuro] existing {VehicleName} comes from {have}, but this site " +
                                 $"wants {want} (SAM 2.1). Run SMARC -> Replace Djuro Vehicle " +
                                 "to swap it — Populate never destroys what is in the scene.");
            float ground = CuratedSiteBuilder.TerrainHeightAt(
                world, sam.transform.position.x, sam.transform.position.z);
            float off = Vector3.Distance(
                new Vector3(sam.transform.position.x, 0, sam.transform.position.z),
                new Vector3(launchPos.x, 0, launchPos.z));
            if (!float.IsNaN(ground) && ground > 0f)
            {
                Debug.LogWarning($"[Djuro] existing {VehicleName} was at " +
                                 $"{sam.transform.position} — ON LAND (terrain {ground:+0.00} m). " +
                                 $"Moved to the measured launch ({launchPos}).");
                sam.transform.position = launchPos;
                sam.transform.rotation = Quaternion.Euler(0, yaw, 0);
            }
            else
            {
                Debug.Log($"[Djuro] existing {VehicleName} kept at {sam.transform.position} " +
                          $"({off:F1} m from the measured launch {launchPos}).");
            }
        }

        if (GameObject.Find("GUI") == null)
            CuratedSiteBuilder.Place(Config, GuiPrefab, "GUI", launchPos + new Vector3(0, 10f, 0));

        EnsureSupportVessel(site);
    }

    /// <summary>
    /// Milou at her MEASURED station-GPS position, with the base station parented to her deck.
    /// Additive like everything else in Populate: an existing boat or case is never destroyed or
    /// moved, only reported on.
    /// </summary>
    static void EnsureSupportVessel(CuratedSiteBuilder.SiteJ site)
    {
        var sv = site?.support_vessel;
        if (sv == null || sv.unity == null)
        {
            Debug.LogWarning("[Djuro] djuro_site.json has no support_vessel — run " +
                             "data-cube/scripts/djuro-site/support_vessel.py --write. " +
                             "Milou and the station NOT placed.");
            return;
        }

        var boat = GameObject.Find(MilouName);
        if (boat == null)
        {
            boat = CuratedSiteBuilder.Place(Config, MilouPrefab, MilouName,
                                            new Vector3(sv.unity.x, 0f, sv.unity.z));
            if (boat == null) return;
            Debug.Log($"[Djuro] {MilouName} placed at the MEASURED station-GPS start " +
                      $"({sv.unity.x:F2}, 0, {sv.unity.z:F2}) = {sv.lat:F6} N {sv.lon:F6} E " +
                      $"({sv.source}). Track: _test_sites/Djuro/tracks/milou_good2_unity.csv");
        }
        else
        {
            float off = Vector3.Distance(new Vector3(boat.transform.position.x, 0, boat.transform.position.z),
                                         new Vector3(sv.unity.x, 0, sv.unity.z));
            Debug.Log($"[Djuro] existing {MilouName} kept at {boat.transform.position} " +
                      $"({off:F1} m from the measured start).");
        }

        var station = GameObject.Find(StationName);
        if (station == null)
        {
            station = CuratedSiteBuilder.Place(Config, StationPrefab, StationName,
                                                boat.transform.TransformPoint(StationOnDeck));
            if (station == null) return;
            // Parent AFTER placing so the world position is the one we computed, then keep the
            // local offset: the case rides the boat, and the transducer rides the case in X/Z.
            station.transform.SetParent(boat.transform, worldPositionStays: true);
            Debug.Log($"[Djuro] {StationName} parented to {MilouName} at local {StationOnDeck} " +
                      "(fore deck). The UW-comms transducer takes its own depth from the water " +
                      "plane via DeployedTransducer — do not offset it from the case.");
        }
        else if (station.transform.parent != boat.transform)
        {
            Debug.LogWarning($"[Djuro] {StationName} exists but is NOT parented to {MilouName} " +
                             "— at Djurö the station rides the boat. Parent it by hand, or delete " +
                             "it and re-run Populate.");
        }
    }

    /// <summary>
    /// The one destructive vehicle operation, and it is a deliberate click: destroy the
    /// existing `sam_auv_v1` (and the GUI, which follows it) and instantiate the preferred
    /// SAM 2.1 prefab at the MEASURED campaign launch with the measured heading. Ivan,
    /// 2026-09-15: the first build placed sam_auv_v1.prefab at the cove; the Djurö tests
    /// were SAM 2.1 starting in the bay.
    /// </summary>
    [MenuItem("SMARC/Replace Djuro Vehicle (SAM 2.1 at the 2024 launch)")]
    public static void ReplaceVehicle()
    {
        var world = GameObject.Find(WorldName);
        if (world == null)
        {
            Debug.LogError("[Djuro] open Assets/Scenes/DjuroCurated.unity first.");
            return;
        }
        var site = CuratedSiteBuilder.LoadSite(Config);
        var old = GameObject.Find(VehicleName);
        if (old != null)
        {
            var src = PrefabUtility.GetCorrespondingObjectFromOriginalSource(old);
            Debug.Log($"[Djuro] destroying existing {VehicleName} at {old.transform.position} " +
                      $"(from {(src != null ? AssetDatabase.GetAssetPath(src) : "no prefab")})");
            UnityEngine.Object.DestroyImmediate(old);
        }
        var gui = GameObject.Find("GUI");
        if (gui != null) UnityEngine.Object.DestroyImmediate(gui);
        PlaceVehicle(world, site);
        EditorSceneManager.MarkAllScenesDirty();
        EditorSceneManager.SaveOpenScenes();
        Debug.Log("[Djuro] vehicle replaced and scene saved.");
    }
}
