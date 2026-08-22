using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

using SmarcGUI.Water;        // BalticWaterPreset, UnderwaterParticles, DockDrainDirector
using Smarc.Cinematics;      // CinematicDirector, CameraShot
using Visualizers;           // SonarMapAccumulator
using ROS.Subscribers;       // MissionWPHoop_Sub
using VehicleComponents.Sensors; // Sonar

/// <summary>
/// One-click setup for the Beckholmen mission video (plan doc
/// data-cube/docs/2026-08-21-beckholmen-video-plan.md).
///
/// Everything here is EDIT-MODE work that ends in a saved scene or a saved asset. That is the
/// point: a Play-mode Inspector edit is lost on Stop, and this project has already recorded a
/// "MEASURED" station position that was exactly that (SETTLED §3s2).
///
/// What it will NOT do, by construction:
///   * move, disable, or re-parent the Water object (SETTLED §3s, §3o),
///   * touch WaterSurface.scriptInteractions,
///   * add a collider to anything (a visualisation aid must not be a sonar target, §3o/§3g),
///   * run any prefab builder (SETTLED §5 — never press SMARC/Build SAM v2 Perception Prefabs).
/// </summary>
public static class VideoRigSetup
{
    const string PackageMatDir = "Packages/com.smarc.assets/Runtime/Materials/Video";
    const string FallbackMatDir = "Assets/VideoMaterials";

    static string MatDir => matDir ?? (matDir = ResolveMatDir());
    static string matDir;
    static string HoopMatPath => MatDir + "/WaypointHoop.mat";
    static string SnowMatPath => MatDir + "/MarineSnow.mat";
    static string MapMatPath => MatDir + "/SonarMap.mat";

    /// <summary>
    /// Materials belong beside the rest of the shared assets. If the package folder cannot be
    /// written — a local `file:` package is normally mutable, but say so rather than failing
    /// silently — they go into the project's own Assets instead. That is safe here because the
    /// references are stored on SCENE objects, and a scene may reference an Assets material.
    /// </summary>
    static string ResolveMatDir()
    {
        if (TryEnsureFolder(PackageMatDir)) return PackageMatDir;
        Debug.LogWarning($"[VideoRig] could not create {PackageMatDir} (a read-only package?) — " +
                         $"using {FallbackMatDir} instead. The references live on scene objects, so this works, " +
                         "but the materials will not travel with the shared assets package.");
        TryEnsureFolder(FallbackMatDir);
        return FallbackMatDir;
    }

    // ================================================================ 1. materials

    [MenuItem("SMARC/Video/1 - Create video materials", false, 100)]
    public static void CreateMaterials()
    {
        matDir = null;   // re-resolve, in case the folder situation changed

        var unlit = Shader.Find("HDRP/Unlit");
        if (unlit == null)
        {
            Debug.LogError("[VideoRig] Shader.Find(\"HDRP/Unlit\") returned null in the EDITOR. " +
                           "That means the HDRP package is not the active pipeline — nothing was created.");
            return;
        }

        // --- the waypoint hoop. Opaque and EMISSIVE: baltic water absorbs fully at ~6 m, so a
        //     non-emissive hoop 10 m away is gone whatever the shader does.
        var hoop = LoadOrCreate(HoopMatPath, unlit);
        SetFloat(hoop, "_SurfaceType", 0f);           // Opaque
        SetColor(hoop, "_UnlitColor", new Color(1f, 0.6f, 0f, 1f));
        SetEmissive(hoop, new Color(1f, 0.6f, 0f, 1f), 4f);
        SetFloat(hoop, "_DoubleSidedEnable", 0f);
        HDMaterial.ValidateMaterial(hoop);
        EditorUtility.SetDirty(hoop);

        // --- marine snow. Transparent, no depth write, soft dot texture supplied at runtime.
        var snow = LoadOrCreate(SnowMatPath, unlit);
        SetFloat(snow, "_SurfaceType", 1f);           // Transparent
        SetFloat(snow, "_BlendMode", 0f);             // Alpha
        SetFloat(snow, "_ZWrite", 0f);
        SetFloat(snow, "_TransparentZWrite", 0f);
        SetFloat(snow, "_DoubleSidedEnable", 1f);
        SetFloat(snow, "_CullMode", 0f);
        SetColor(snow, "_UnlitColor", new Color(0.85f, 0.88f, 0.78f, 0.55f));
        snow.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
        HDMaterial.ValidateMaterial(snow);
        EditorUtility.SetDirty(snow);

        // --- the sonar map. DOUBLE SIDED matters: each point is a flat flake lying in the
        //     surveyed surface, and half of them are seen from behind.
        var map = LoadOrCreate(MapMatPath, unlit);
        SetFloat(map, "_SurfaceType", 0f);
        SetFloat(map, "_DoubleSidedEnable", 1f);
        SetFloat(map, "_CullMode", 0f);
        SetColor(map, "_UnlitColor", Color.white);
        SetEmissive(map, Color.white, 2.5f);
        HDMaterial.ValidateMaterial(map);
        EditorUtility.SetDirty(map);

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log($"[VideoRig] materials ready:\n  {HoopMatPath}\n  {SnowMatPath}\n  {MapMatPath}");
    }

    static Material LoadOrCreate(string path, Shader shader)
    {
        var m = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (m != null)
        {
            if (m.shader != shader) m.shader = shader;
            return m;
        }
        m = new Material(shader) { name = Path.GetFileNameWithoutExtension(path) };
        AssetDatabase.CreateAsset(m, path);
        return m;
    }

    static void SetFloat(Material m, string p, float v) { if (m.HasProperty(p)) m.SetFloat(p, v); }
    static void SetColor(Material m, string p, Color v) { if (m.HasProperty(p)) m.SetColor(p, v); }

    static void SetEmissive(Material m, Color c, float intensity)
    {
        var e = new Color(c.r, c.g, c.b, 1f);
        SetColor(m, "_EmissiveColorLDR", e);
        SetFloat(m, "_EmissiveIntensity", intensity);
        SetFloat(m, "_EmissiveIntensityUnit", 0f);
        SetFloat(m, "_UseEmissiveIntensity", 1f);
        SetFloat(m, "_EmissiveExposureWeight", 0f);
        SetColor(m, "_EmissiveColor", e * intensity);
        m.EnableKeyword("_EMISSIVE_COLOR_MAP");
        m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
    }

    static bool TryEnsureFolder(string assetPath)
    {
        if (AssetDatabase.IsValidFolder(assetPath)) return true;
        var parent = Path.GetDirectoryName(assetPath).Replace('\\', '/');
        var leaf = Path.GetFileName(assetPath);
        if (string.IsNullOrEmpty(parent) || string.IsNullOrEmpty(leaf)) return false;
        if (!AssetDatabase.IsValidFolder(parent) && !TryEnsureFolder(parent)) return false;
        var guid = AssetDatabase.CreateFolder(parent, leaf);
        return !string.IsNullOrEmpty(guid) && AssetDatabase.IsValidFolder(assetPath);
    }

    // ================================================================ 2. the rig

    [MenuItem("SMARC/Video/2 - Add video rig to open scene", false, 101)]
    public static void AddRigToOpenScene()
    {
        CreateMaterials();

        var surface = Object.FindFirstObjectByType<WaterSurface>();
        if (surface == null)
        {
            Debug.LogError("[VideoRig] no WaterSurface in the open scene — the water preset has nothing to drive. Aborting.");
            return;
        }
        if (Mathf.Abs(surface.transform.position.y) > 1e-4f)
            Debug.LogError($"[VideoRig] the Water transform is at Y = {surface.transform.position.y:F3}, not 0 " +
                           "(SETTLED §3s). Fix that BEFORE filming: move the terrain, not the water. " +
                           "The rig was still added — it never writes the transform.");

        // --- P-V1: the preset component lives on the Water object itself.
        var preset = surface.GetComponent<BalticWaterPreset>();
        if (preset == null)
        {
            preset = Undo.AddComponent<BalticWaterPreset>(surface.gameObject);
            preset.Surface = surface;
            preset.SeedDefaultPresets();
            preset.CaptureIntoPreset("clear_demo");   // keep today's look before anything changes it
            PrefabUtility.RecordPrefabInstancePropertyModifications(preset);
            Debug.Log("[VideoRig] BalticWaterPreset added to the Water object; today's look captured as 'clear_demo'.");
        }

        // --- the rig root
        var rigGO = GameObject.Find("VideoRig");
        if (rigGO == null)
        {
            rigGO = new GameObject("VideoRig");
            Undo.RegisterCreatedObjectUndo(rigGO, "Create VideoRig");
        }

        // --- P-V2: particles
        var particles = rigGO.GetComponentInChildren<UnderwaterParticles>();
        if (particles == null)
        {
            var go = new GameObject("UnderwaterParticles");
            go.transform.SetParent(rigGO.transform, false);
            particles = go.AddComponent<UnderwaterParticles>();
        }
        particles.Surface = surface;
        particles.ParticleMaterial = AssetDatabase.LoadAssetAtPath<Material>(SnowMatPath);

        // --- P-V4: the sonar map
        var map = rigGO.GetComponentInChildren<SonarMapAccumulator>();
        if (map == null)
        {
            var go = new GameObject("SonarMapAccumulator");
            go.transform.SetParent(rigGO.transform, false);
            map = go.AddComponent<SonarMapAccumulator>();
            // AddComponent from a script does not call Reset(), so seed the ramp by hand or the
            // cloud ships Unity's default black-to-white gradient.
            map.SeedRamp();
        }
        map.MapMaterial = AssetDatabase.LoadAssetAtPath<Material>(MapMatPath);
        // DEFECT A, 2026-08-21: sam2.2 carries an FLS (Sonar3D15) and an SSS (DeepVision) and NO
        // MBES, so a rig created before today taps nothing and the map stays at 0 pts for the whole
        // take. Correct it here rather than leaving it to be noticed on camera again.
        if (!map.IncludeFLS)
        {
            map.IncludeFLS = true;
            Debug.LogWarning("[VideoRig] SonarMapAccumulator had IncludeFLS OFF — turned ON. The only " +
                             "forward sonar on sam2.2 is Sonar3D15 (Type = FLS); with this off the sonar " +
                             "map accumulates nothing, which is exactly what the 2026-08-21 take recorded.");
        }
        // ROUND 3, Ivan: "skip the side scan in the 3d point cloud map building here". The flag was
        // ticked on the rig for a test; the SEEDED state is FLS on, SSS off, and this is the place
        // that state is asserted rather than remembered.
        if (map.IncludeSSS)
        {
            map.IncludeSSS = false;
            Debug.LogWarning("[VideoRig] SonarMapAccumulator had IncludeSSS ticked — turned OFF (Ivan, round 3). " +
                             "The side scan draws the dock walls at a large cost in points, and §3l says it sees " +
                             "nothing at or above its own depth anyway. Tick it back by hand for a wall survey.");
        }
        // ROUND 3, Ivan: "color the mapped part in blue from dark (at depth) to lighter (at shallow)".
        // A default changed in code is NOT a change to the serialized component (SETTLED §3o, the
        // Collidable trap), so the gradient already in the scene is read and replaced if it is one
        // this tool authored earlier. A hand-edited ramp is left alone and said so.
        if (map.RampNeedsReseeding(out string rampWhy))
        {
            map.SeedRamp();
            Debug.LogWarning($"[VideoRig] SonarMapAccumulator's colour ramp was replaced with the BLUE depth " +
                             $"ramp (light cyan at the surface -> dark navy at depth) because {rampWhy}.");
        }
        else Debug.Log($"[VideoRig] SonarMapAccumulator's colour ramp left alone — {rampWhy}.");

        if (!map.DepthRampRelativeToWaterPlane)
        {
            map.DepthRampRelativeToWaterPlane = true;
            Debug.LogWarning("[VideoRig] SonarMapAccumulator now colours by DEPTH BELOW THE WATER PLANE " +
                             "rather than by world Y. The range is FIXED (see below) and never auto-fitted: " +
                             "a point's colour is baked into its vertex when it is added, so a rescaling ramp " +
                             "would change what already-drawn points mean.");
        }
        map.DepthRampShallowM = 0f;
        map.DepthRampDeepM = 8f;          // Beckholmen's dock floor is about 7 m down
        map.WaterPlaneSource = surface;   // read for its TRANSFORM only — never GetWaterLevelAt (§3s)

        // --- the dock drain (SETTLED §3s sanctioned exception; read DockDrainDirector's comment)
        var drain = rigGO.GetComponentInChildren<DockDrainDirector>();
        if (drain == null)
        {
            var go = new GameObject("DockDrainDirector");
            go.transform.SetParent(rigGO.transform, false);
            drain = go.AddComponent<DockDrainDirector>();
        }
        drain.Surface = surface;
        drain.VehicleName = "sam_auv_v1";

        // --- P-V3: hand the hoops their material
        var hoopSub = Object.FindFirstObjectByType<MissionWPHoop_Sub>(FindObjectsInactive.Include);
        if (hoopSub == null)
        {
            Debug.LogWarning("[VideoRig] no MissionWPHoop_Sub in the scene — run " +
                             "SMARC/Add Mission WP Hoop to Open Scene first, then this again.");
        }
        else
        {
            hoopSub.HoopMaterial = AssetDatabase.LoadAssetAtPath<Material>(HoopMatPath);
            hoopSub.UseEmissive = true;
            hoopSub.EmissiveIntensity = 4f;
            if (hoopSub.Collidable)
            {
                hoopSub.Collidable = false;
                Debug.LogWarning("[VideoRig] MissionWPHoop had Collidable TICKED — turned off. " +
                                 "A collidable hoop is a synthetic sonar target at every waypoint, " +
                                 "and the sonar map is about to accumulate exactly those returns (SETTLED §3o).");
            }
            PrefabUtility.RecordPrefabInstancePropertyModifications(hoopSub);
        }

        // --- P-V5: the director
        var director = rigGO.GetComponentInChildren<CinematicDirector>();
        bool freshDirector = director == null;
        if (freshDirector)
        {
            var go = new GameObject("CinematicDirector");
            go.transform.SetParent(rigGO.transform, false);
            director = go.AddComponent<CinematicDirector>();
        }
        director.Particles = particles;
        director.SonarMap = map;
        director.Hoops = hoopSub;
        director.Drain = drain;
        director.Surface = surface;
        director.WaterPreset = preset;   // round 3: CameraShot.WaterPresetName is driven through this
        if (freshDirector)
        {
            SeedStoryboard(director, surface, interactive: false);
        }
        else if (director.Shots.Count > 0)
        {
            Debug.Log($"[VideoRig] the director already has {director.Shots.Count} shot(s) — the storyboard was " +
                      "NOT touched, in case you have edited it. Use SMARC/Video/2b to re-seed it; 2b tells you " +
                      "exactly what it would replace and refuses if you say no.");
        }

        // --- ROUND 3: the underwater VOLUME, which is why "the water disappeared" mid-take.
        EnsureUnderwaterVolumeCovers(director, surface, apply: true, out _);

        EditorSceneManager.MarkSceneDirty(rigGO.scene);
        Selection.activeGameObject = rigGO;
        Debug.Log("[VideoRig] rig in place. SAVE THE SCENE (Cmd-S) — a preset is a saved scene state, not a Play-mode edit.");
    }

    // ================================================================ the underwater volume
    //
    // ROUND 3, AND IT IS A MEASUREMENT, NOT A THEORY. Ivan: "after wp2 the water suddenly
    // disappears, I like the effect but let's shift water to the clear version instead and not take
    // it away completely." The disappearance was not a look at all — it was HDRP switching the
    // underwater rendering off because the CAMERA had left the water's underwater volume.
    //
    // Read out of this project's own HDRP 17.3 source
    // (Runtime/Water/HDRenderPipeline.WaterSystem.Underwater.cs, ~line 85): for a surface that is
    // not an infinite ocean — Beckholmen's is `surfaceType: 2` (Pool), so `IsInfinite()` is false —
    // HDRP asks `volumeBounds.ClosestPoint(cameraWSPos)` and renders the underwater view ONLY if
    // that point IS the camera position. Outside the box: no underwater view, at all, however murky
    // the preset is. It is the CAMERA that is tested, not the vehicle — which is why the effect
    // arrived mid-mission, on the wide map-building shots where the lens sits 15-20 m behind and
    // 5-8 m above the hull.
    //
    // Measured out of the asset files: `BeckholmenWorld.prefab` puts `Water` under `Drydock` (local
    // (30.87, 0, 78.63), yaw -134.292 deg) at local (-0.003, 0, -3.6) with scale 17.6 x 1 x 102.39,
    // and its BoxCollider is `m_Size 1 x 5 x 1`, `m_Center (0, -2, 0)`. That is a 17.6 x 5 x 102.4 m
    // box, rotated with the dock, spanning only Y +0.5 down to Y -4.5 — while the dock FLOOR is
    // about 7 m down. So any camera deeper than 4.5 m, or more than ~8.8 m off the dock centreline,
    // is outside it by construction.
    //
    // NEVER FIXED BY MOVING THE WATER (SETTLED §3s: a non-zero water Y hands ForcePoints divergent
    // levels and the vehicle leaves at 67 m/s). The transform is not touched here at all — only the
    // collider's own `center` and `size`, which is exactly what §9a of the runbook said the answer
    // would have to be.

    /// <summary>
    /// Grow the WaterSurface's `volumeBounds` BoxCollider until it covers the whole dock channel —
    /// the mission, the seeded camera positions, and the dock floor — with margin. Idempotent: it
    /// computes the target, compares it in WORLD metres, and writes nothing when there is nothing to
    /// change. It NEVER SHRINKS the box (the existing corners are part of the target), and it
    /// refuses outright rather than enlarge a collider that is not a trigger.
    /// </summary>
    static bool EnsureUnderwaterVolumeCovers(CinematicDirector director, WaterSurface surface,
                                             bool apply, out string report)
    {
        report = "";
        if (surface == null) { report = "FAIL  no WaterSurface — no underwater volume to check."; return false; }
        if (director == null) { report = "FAIL  no CinematicDirector — nothing to size the volume against."; return false; }

        var box = surface.volumeBounds;
        if (box == null)
        {
            report = "FAIL  the WaterSurface has NO Volume Bounds BoxCollider, so HDRP has no region to " +
                     "render the underwater view in and every in-water shot will look like an above-water " +
                     "one. This tool will not create a collider (a visualisation must not become physics " +
                     "geometry by accident) — add it yourself: select BeckholmenWorld ▸ Water, and in the " +
                     "Water Surface component's Appearance ▸ Underwater section use the Volume Bounds " +
                     "field's dropdown to add a Box Collider.";
            if (apply) Debug.LogError("[VideoRig] " + report);
            return false;
        }
        if (!box.isTrigger)
        {
            report = "FAIL  the Volume Bounds BoxCollider is NOT a trigger. Enlarging it would put a solid " +
                     "wall around the whole dock. Tick 'Is Trigger' on BeckholmenWorld ▸ Water ▸ Box " +
                     "Collider first; nothing was changed. (The sonar is safe from it either way — " +
                     "Sonar.cs raycasts with QueryTriggerInteraction.Ignore — but the vehicle is not.)";
            if (apply) Debug.LogError("[VideoRig] " + report);
            return false;
        }

        var waterTf = surface.transform;

        // ---- the world points the box must contain ------------------------------------
        var pts = new List<Vector3>();

        // 1. The dock channel itself, from the vehicle and its heading — the same axis every shot
        //    is authored from, so the volume and the camera work cannot disagree about where the
        //    dock is.
        float waterY = waterTf.position.y;
        float floorY = waterY - 8f;
        string floorHow = "assumed 8 m below the water plane (no vehicle to raycast under)";
        if (TryDockAxis(director, out var vpos, out var fwd, out var side))
        {
            floorY = MeasureDockFloorY(vpos + fwd * 45f, out floorHow);
            for (int a = -1; a <= 1; a += 2)
                for (int s = -1; s <= 1; s += 2)
                {
                    Vector3 xz = vpos + fwd * (a < 0 ? -VolumeBehindM : VolumeAheadM) + side * (s * VolumeHalfWidthM);
                    pts.Add(WithY(xz, waterY + VolumeAboveM));
                    pts.Add(WithY(xz, floorY - VolumeBelowFloorM));
                }
        }

        // 2. Every seeded camera position, padded — the box is tested against the CAMERA, so a shot
        //    whose lens sits outside it renders no water however deep the vehicle is.
        foreach (Transform t in director.transform)
        {
            if (t == null) continue;
            AddPadded(pts, t.position, VolumeCameraPadM);
            foreach (Transform child in t) if (child != null) AddPadded(pts, child.position, VolumeCameraPadM);
        }

        if (pts.Count == 0)
        {
            report = "WARN  nothing to size the underwater volume against — no active vehicle to take the dock " +
                     "axis from, and no camera markers under the director. The volume was left exactly as it is.";
            if (apply) Debug.LogWarning("[VideoRig] " + report);
            return false;
        }

        // ---- into the collider's own space, which handles the dock's -134 deg yaw and the
        //      17.6 x 1 x 102.39 scale in one call ----------------------------------------
        Vector3 scale = waterTf.lossyScale;
        Vector3 lo = Vector3.one * float.MaxValue, hi = Vector3.one * float.MinValue;
        foreach (var p in pts)
        {
            var l = waterTf.InverseTransformPoint(p);
            lo = Vector3.Min(lo, l); hi = Vector3.Max(hi, l);
        }

        // QUANTISE OUTWARD TO A 5 m GRID, and this is what makes the menu item idempotent rather
        // than merely convergent. Half of the target is derived from the VEHICLE, which moves every
        // time the scene is played, so an exact fit would find "a change" on every press for the
        // rest of the project — the §3s8 shape, where a builder disagrees with itself. Snapped to
        // 5 m, a vehicle that has drifted a metre produces the identical box and the second press
        // genuinely writes nothing.
        for (int a = 0; a < 3; a++)
        {
            float step = VolumeGridM / Mathf.Max(1e-4f, Mathf.Abs(scale[a]));   // 5 world metres, in local units
            lo[a] = Mathf.Floor(lo[a] / step) * step;
            hi[a] = Mathf.Ceil(hi[a] / step) * step;
        }

        // NEVER SHRINK: whatever the box already covers stays covered. Applied after quantisation so
        // an already-correct box is bit-for-bit unchanged.
        Vector3 haveLo = box.center - box.size * 0.5f;
        Vector3 haveHi = box.center + box.size * 0.5f;
        lo = Vector3.Min(lo, haveLo);
        hi = Vector3.Max(hi, haveHi);

        Vector3 wantCenter = (lo + hi) * 0.5f;
        Vector3 wantSize = hi - lo;

        // Compare in WORLD metres: one local unit along Z is 102 m here, so a local-space tolerance
        // would be meaningless in one axis and absurd in another.
        Vector3 dC = Vector3.Scale(wantCenter - box.center, scale);
        Vector3 dS = Vector3.Scale(wantSize - box.size, scale);
        Vector3 worldSize = Vector3.Scale(wantSize, scale);
        string extent = $"{worldSize.x:F0} x {worldSize.y:F0} x {worldSize.z:F0} m about " +
                        $"{waterTf.TransformPoint(wantCenter)}, floor {floorY:F2} ({floorHow})";

        if (dC.magnitude < 0.05f && dS.magnitude < 0.05f)
        {
            report = $"ok    underwater volume already covers the dock channel: {extent}.";
            if (apply) Debug.Log("[VideoRig] " + report);
            return true;
        }

        Vector3 oldWorld = Vector3.Scale(box.size, scale);
        if (!apply)
        {
            report = $"FAIL  the underwater volume is {oldWorld.x:F0} x {oldWorld.y:F0} x {oldWorld.z:F0} m and " +
                     $"does NOT cover the dock channel ({extent} is needed). HDRP renders the underwater view " +
                     "ONLY while the CAMERA is inside this box, so shots will lose the water mid-mission. " +
                     "Run SMARC/Video/2 — it resizes the COLLIDER, never the Water transform.";
            return false;
        }
        Undo.RecordObject(box, "Cover the dock with the underwater volume");
        box.center = wantCenter;
        box.size = wantSize;
        EditorUtility.SetDirty(box);
        PrefabUtility.RecordPrefabInstancePropertyModifications(box);
        EditorSceneManager.MarkSceneDirty(surface.gameObject.scene);

        report = $"ok    underwater volume ENLARGED from {oldWorld.x:F0} x {oldWorld.y:F0} x {oldWorld.z:F0} m " +
                 $"to {extent}.";
        Debug.LogWarning("[VideoRig] the Water's Volume Bounds BoxCollider was " +
                         $"{oldWorld.x:F0} x {oldWorld.y:F0} x {oldWorld.z:F0} m and did not cover where the " +
                         $"mission flies. Enlarged to {extent}.\n" +
                         "  THIS IS THE ROOT CAUSE OF 'AFTER WP2 THE WATER SUDDENLY DISAPPEARS'. HDRP renders " +
                         "the underwater view only while the CAMERA is inside this box (HDRP 17.3, " +
                         "HDRenderPipeline.WaterSystem.Underwater.cs: volumeBounds.ClosestPoint(cameraWSPos)); " +
                         "the old box was 5 m TALL, from +0.5 to -4.5, over a dock floor about 7 m down.\n" +
                         "  Only the COLLIDER's center and size were written. The Water transform is untouched " +
                         "and stays at Y = 0 (SETTLED §3s). SAVE THE SCENE.");
        return true;
    }

    // Margins for the underwater volume, in metres. Generous on purpose: the cost of an over-large
    // trigger box is nil (the sonar ignores triggers; nothing else queries this one), and the cost
    // of an under-large one is a take with no water in it.
    const float VolumeAheadM = 130f;        // along the dock, south of the vehicle start
    const float VolumeBehindM = 40f;        // and north of it
    const float VolumeHalfWidthM = 35f;     // either side of the centreline
    const float VolumeAboveM = 4f;          // above the still-water plane
    const float VolumeBelowFloorM = 6f;     // below the measured dock floor
    const float VolumeCameraPadM = 8f;      // around every seeded camera node and marker
    const float VolumeGridM = 5f;           // the snap that makes a second press a genuine no-op

    static void AddPadded(List<Vector3> pts, Vector3 p, float pad)
    {
        pts.Add(p + new Vector3(pad, pad, pad));
        pts.Add(p - new Vector3(pad, pad, pad));
    }

    // ================================================================ 2b. the storyboard

    [MenuItem("SMARC/Video/2b - Re-seed the auto storyboard (12 shots)", false, 102)]
    public static void ReseedStoryboard()
    {
        var director = Object.FindFirstObjectByType<CinematicDirector>();
        if (director == null)
        {
            Debug.LogError("[VideoRig] no CinematicDirector in the open scene — run SMARC/Video/2 first.");
            return;
        }
        var surface = Object.FindFirstObjectByType<WaterSurface>();
        SeedStoryboard(director, surface, interactive: true);
    }

    // ================================================================ the shot geometry, ONE COPY
    //
    // Every number the fly-in, the hold, the drain camera and the final fly-through are built from
    // lives here, and BOTH the storyboard seeder (2b) and the geometry authors (2c, 2d) read it.
    // Two copies of "where the camera goes" is how a builder and its own checker start disagreeing
    // — SETTLED §3s8 cost a session to exactly that shape. If 2b and 2c ever disagree about shot 1,
    // it is because someone re-typed a number instead of changing it here.
    static class DockShotSpec
    {
        // ---- the fly-in. ROUND 3 (Ivan): "With flying in from the south, I meant flying along the
        //      centre of the dock from the south end (not exactly from south)." So: entry OVER THE
        //      SOUTH END of the dock — the same 80 m along the axis that DockSouthTarget stands at —
        //      and every node ON THE CENTRELINE, zero lateral offset, low over the water the whole
        //      way in. The round-2 version started 70 m out and swung 4 m toward a dock wall, which
        //      is a fly-in ALONG the dock rather than DOWN THE MIDDLE of it.
        //      "Along" is metres in the vehicle's own forward direction — the dock axis.
        public static readonly float[] FlyInAlong = { 80f, 55f, 28f, 7f };
        public static readonly float[] FlyInSide = { 0f, 0f, 0f, 0f };   // THE CENTRELINE. Round 3.
        public static readonly float[] FlyInAltAboveWater = { 1.5f, 1.5f, 1.5f, 4.0f };
        public const float FlyInDuration = 12f;

        // Where shot 1 ENDS and shot 1b STANDS. These are the same point by construction: shot 1's
        // EndOffsetInTargetFrame is (0, HoldHeight, 0) and shot 1b's anchor is the vehicle position
        // plus that. If they drift apart you get a jump cut between them.
        public const float HoldHeight = 6f;
        public const float HoldDuration = 4f;
        public const float HoldBlend = 2.5f;      // the blend IS the rotation to look down the dock
        public const float SouthTargetAlong = 80f;

        // ---- the final fly-through of the DRAINED dock (Ivan: "low altitude, see the built map
        //      in detail"). Same axis as the fly-in, same direction — in from the south — but now
        //      metres above the dock FLOOR, not the waterline.
        public static readonly float[] FlyThroughAlong = { 100f, 70f, 38f, 6f };
        public const float FlyThroughAboveFloor = 2.5f;
        public const float FlyThroughDuration = 20f;
        public const float FallbackDockFloorY = -7f;

        // ---- the drain reveal. ROUND 3: it no longer has a shot of its own — the zoom-out starts
        //      it (CameraShot.StartDrainAtShotStart) and ends when the dock is empty. These camera
        //      constants stay because Mode.DrainDock is kept for MANUAL use and its anchors are
        //      still authored, so a hand-directed drain wide is one Inspector change away.
        public const float DrainDepthM = 7.5f;
        public const float DrainSeconds = 25f;
        public const float DrainCamAlong = 55f;
        public const float DrainCamSide = 30f;
        public const float DrainCamHeight = 26f;
        public const float DrainLookAlong = 45f;

        // ---- where the zoom-out LANDS (Ivan, round 3): "rotating the camera so we look straight
        //      along the centre of the dry dock from south". The end pose is ELEVATED AT THE SOUTH
        //      END, on the centreline; the aim eases onto a point on the centreline near the NORTH
        //      end, so the last frame of the orbit looks straight down the dock. Both are authored
        //      from the dock axis, not from a generic bearing.
        public const float OrbitEndAlong = 105f;    // south of the vehicle start, past the dock end
        public const float OrbitEndHeight = 42f;
        public const float OrbitEndLookAlong = 8f;  // just south of the vehicle start = the north end
        public const float OrbitDegreesSwept = 170f;
        public const float OrbitDuration = 34f;     // the camera move; the CUT is on DrainComplete
    }

    /// <summary>
    /// The dock axis, taken from the vehicle: its forward is "south down the dock". STATED, because
    /// it is an assumption and not a measurement — if a sweep arrives from the wrong end, this is
    /// the assumption that failed, and the fix is to re-pose the vehicle and re-run, or drag nodes.
    /// </summary>
    static bool TryDockAxis(CinematicDirector director, out Vector3 vpos, out Vector3 fwd, out Vector3 side)
    {
        vpos = Vector3.zero; fwd = Vector3.forward; side = Vector3.right;
        var vehicleGO = GameObject.Find(director.VehicleName);
        if (vehicleGO == null) return false;
        vpos = vehicleGO.transform.position;
        fwd = vehicleGO.transform.forward; fwd.y = 0f;
        if (fwd.sqrMagnitude < 1e-6f) return false;
        fwd.Normalize();
        side = Vector3.Cross(Vector3.up, fwd);
        return true;
    }

    /// <summary>
    /// THE SELF-DRIVING STORYBOARD: Ivan's own beat list, with each cut triggered by an observable
    /// fact rather than by a stopwatch alone, so one press of RECORD produces the whole sequence
    /// in a single continuous take.
    ///
    /// IT NEVER SILENTLY OVERWRITES. If the director's shot list already IS this storyboard, it
    /// writes nothing at all and says NOTHING TO CHANGE — so a second press is a genuine no-op and
    /// you can prove it with `shasum` on the scene file. If the list differs, it names every shot
    /// it would replace and asks; Cancel means nothing is written. (2026-08-21: an untested
    /// "a second press does nothing" claim in this runbook turned out to be false six presses in a
    /// row — SETTLED §3s8. This one is built so the claim is checkable.)
    ///
    /// THE DOLLY NODES ARE REUSED IF THEY EXIST. Dragging the fly-in into shape is the one piece
    /// of hand work in this rig, and re-seeding must not throw it away.
    /// </summary>
    static void SeedStoryboard(CinematicDirector director, WaterSurface surface, bool interactive)
    {
        bool anythingCreated = false;
        var nodes = EnsureDollyNodes(director, surface, out bool nodesCreated);
        anythingCreated |= nodesCreated;
        var flyThrough = EnsureFlyThroughNodes(director, out bool ftCreated);
        anythingCreated |= ftCreated;
        var southTarget = EnsureMarker(director, "DockSouthTarget", () => SouthTargetPos(director, surface), out bool stCreated);
        anythingCreated |= stCreated;
        var holdAbove = EnsureMarker(director, "FlyInHoldAbove", () => HoldAbovePos(director), out bool haCreated);
        anythingCreated |= haCreated;
        var drainCam = EnsureMarker(director, "DrainCam", () => DrainCamPos(director, surface), out bool dcCreated);
        anythingCreated |= dcCreated;
        var drainTarget = EnsureMarker(director, "DrainCamTarget", () => DrainTargetPos(director, surface), out bool dtCreated);
        anythingCreated |= dtCreated;
        var orbitEnd = EnsureMarker(director, "OrbitEndPose", () => OrbitEndPosePos(director, surface), out bool oeCreated);
        anythingCreated |= oeCreated;
        var orbitEndLook = EnsureMarker(director, "OrbitEndLookAt", () => OrbitEndLookPos(director, surface), out bool olCreated);
        anythingCreated |= olCreated;

        var wanted = BuildStoryboard(nodes, flyThrough, southTarget, holdAbove, drainCam, drainTarget,
                                     orbitEnd, orbitEndLook);

        if (StoryboardMatches(director.Shots, wanted) && !anythingCreated)
        {
            Debug.Log($"[VideoRig] storyboard already matches the seeded {wanted.Count}-shot list — " +
                      "NOTHING TO CHANGE, and nothing was written. (Press it again: you must get this " +
                      "same line, and the scene must not go dirty.)");
            return;
        }

        if (interactive && director.Shots.Count > 0 && !StoryboardMatches(director.Shots, wanted))
        {
            var names = new List<string>();
            for (int i = 0; i < director.Shots.Count; i++)
            {
                var s = director.Shots[i];
                names.Add($"  {i + 1}. {(s == null ? "(null)" : s.Name)}" +
                          (s == null ? "" : $"  [{s.Kind}, {s.AdvanceSummary()}]"));
            }
            bool go = EditorUtility.DisplayDialog(
                "Replace the storyboard?",
                $"The CinematicDirector holds {director.Shots.Count} shot(s):\n\n" +
                string.Join("\n", names) +
                $"\n\nThese will be REPLACED by the seeded {wanted.Count}-shot auto storyboard.\n" +
                "The DollyNodes_FlyIn transforms are reused, not recreated, so any shaping you did " +
                "to the fly-in path survives.\n\nNothing else in the scene is touched.",
                "Replace them", "Cancel");
            if (!go)
            {
                Debug.Log("[VideoRig] re-seed REFUSED at the dialog — nothing was written.");
                return;
            }
        }

        Undo.RecordObject(director, "Re-seed cinematic storyboard");
        director.Shots = wanted;
        director.CurrentShot = 0;
        EditorUtility.SetDirty(director);
        EditorSceneManager.MarkSceneDirty(director.gameObject.scene);

        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"[VideoRig] seeded the {wanted.Count}-shot auto storyboard. Every cut is triggered by a " +
                      "fact, and every conditioned shot has a ceiling it falls through on:");
        for (int i = 0; i < wanted.Count; i++)
            sb.AppendLine($"  {i + 1}. {wanted[i].Name}   [{wanted[i].Kind}]  {wanted[i].AdvanceSummary()}");
        sb.AppendLine("SAVE THE SCENE (Cmd-S). Press SMARC/Video/2b again: it must say NOTHING TO CHANGE.");
        Debug.Log(sb.ToString());
    }

    /// <summary>
    /// The comparison that makes "nothing to change" a real answer: the fields the seed authors and
    /// that decide the sequence's behaviour. Cosmetic Inspector tweaks (a nudged FOV, a different
    /// damping) are deliberately NOT compared — re-seeding over those is the point of the dialog,
    /// not of this test.
    /// </summary>
    static bool StoryboardMatches(List<CameraShot> have, List<CameraShot> want)
    {
        // REWRITTEN 2026-08-21, after it lied on the rig: the hand-kept version compared SEVEN
        // fields, so when BuildStoryboard gained the shot-2 zoom-in (ZoomInSeconds /
        // ZoomStartMultiplier / a lower FollowOffset.Y) it reported "NOTHING TO CHANGE" against a
        // scene that did not have the dive. A comparator that must be extended by hand every time
        // the seeder grows a field is a comparator that drifts — the §3s5 hand-kept-copy shape.
        // JsonUtility serializes every [SerializeField] the Inspector shows, so this compares
        // exactly what the seeder authors, today's fields and every future one. Scene-object
        // references (Nodes, LookAt, …) serialize as instance IDs: `want` is built around the
        // SAME scene transforms EnsureDollyNodes returned, so identical wiring compares equal and
        // different wiring compares different, which is the point.
        if (have == null || have.Count != want.Count) return false;
        for (int i = 0; i < want.Count; i++)
        {
            var a = have[i]; var b = want[i];
            if (a == null) return false;
            if (JsonUtility.ToJson(a) != JsonUtility.ToJson(b)) return false;
        }
        return true;
    }

    /// <summary>
    /// The fly-in path. Seeded RELATIVE TO THE VEHICLE, not to the water quad: at Beckholmen the
    /// Water transform sits at (-0.003, 0, -3.6) with a 17.6 x 102.4 m footprint centred near the
    /// world origin, while the active `sam_auv_v1` starts at about (61.4, -0.15, 111.6) — roughly
    /// 130 m away. A path laid out from the quad's extents is therefore over open harbour, which
    /// is why the first take's fly-in was looking at the wrong end of everything.
    ///
    /// The director ALSO re-anchors this path to the vehicle at the moment the shot starts, so the
    /// video is correct even if the vehicle is moved afterwards. Seeding it in the right place is
    /// so that the orange gizmo in the Scene view is somewhere you can actually shape it.
    /// </summary>
    static Transform[] EnsureDollyNodes(CinematicDirector director, WaterSurface surface, out bool created)
    {
        created = false;
        var rootTf = director.transform.Find("DollyNodes_FlyIn");
        if (rootTf != null)
        {
            var have = new List<Transform>();
            foreach (Transform c in rootTf) have.Add(c);
            if (have.Count >= 2)
            {
                Debug.Log($"[VideoRig] reusing the existing DollyNodes_FlyIn ({have.Count} node(s)) — " +
                          "any shaping you did to the fly-in path is kept.");
                return have.ToArray();
            }
        }

        Vector3 origin; Vector3 fwd; Vector3 right; string from;
        var vehicleGO = GameObject.Find(director.VehicleName);
        if (vehicleGO != null)
        {
            origin = vehicleGO.transform.position;
            fwd = Vector3.ProjectOnPlane(vehicleGO.transform.forward, Vector3.up);
            if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
            fwd.Normalize();
            from = $"the vehicle '{director.VehicleName}' at {origin}";
        }
        else if (surface != null)
        {
            origin = surface.transform.position;
            fwd = surface.transform.forward;
            from = $"the water quad at {origin} (NO vehicle named '{director.VehicleName}' was found — " +
                   "the director re-anchors this path to the vehicle at Play anyway)";
        }
        else
        {
            origin = Vector3.zero; fwd = Vector3.forward;
            from = "the world origin (no vehicle and no WaterSurface found)";
        }
        right = Vector3.Cross(Vector3.up, fwd).normalized;

        float waterY = surface != null ? surface.transform.position.y : 0f;
        var p = new Vector3[4];
        for (int i = 0; i < 4; i++)
            p[i] = WithY(origin + fwd * DockShotSpec.FlyInAlong[i] + right * DockShotSpec.FlyInSide[i],
                         waterY + DockShotSpec.FlyInAltAboveWater[i]);

        if (rootTf == null)
        {
            var go = new GameObject("DollyNodes_FlyIn");
            Undo.RegisterCreatedObjectUndo(go, "Create fly-in dolly nodes");
            go.transform.SetParent(director.transform, false);
            rootTf = go.transform;
        }
        var nodes = new Transform[p.Length];
        for (int i = 0; i < p.Length; i++)
        {
            var n = new GameObject($"Node{i}");
            Undo.RegisterCreatedObjectUndo(n, "Create fly-in dolly node");
            n.transform.SetParent(rootTf, false);
            n.transform.position = p[i];
            nodes[i] = n.transform;
        }
        created = true;
        Debug.Log($"[VideoRig] created a 4-node fly-in path from {from}, coming in from the SOUTH " +
                  $"({DockShotSpec.FlyInAlong[0]:F0} m out at {DockShotSpec.FlyInAltAboveWater[0]:F1} m over the " +
                  $"water, skimming at {DockShotSpec.FlyInAltAboveWater[2]:F1} m). Shape it by dragging " +
                  "Node0..Node3 in the Scene view — the authored spline is the orange gizmo, and in Play the " +
                  "GREEN one is where it actually flies after being anchored to the vehicle.");
        return nodes;
    }

    /// <summary>
    /// The path for the FINAL fly-through of the drained dock: same axis and same direction as the
    /// fly-in (in from the south), but metres above the dock FLOOR rather than the waterline,
    /// because by then there is no waterline.
    ///
    /// The floor height is MEASURED with a downward raycast against the dock's own colliders and
    /// falls back to a named constant if the ray misses — a fly-through at an invented altitude
    /// would be a camera inside the concrete or ten metres over the map, and either way the log
    /// says which of the two numbers it used.
    /// </summary>
    static Transform[] EnsureFlyThroughNodes(CinematicDirector director, out bool created)
    {
        created = false;
        var rootTf = director.transform.Find("DollyNodes_FlyThrough");
        if (rootTf != null)
        {
            var have = new List<Transform>();
            foreach (Transform c in rootTf) have.Add(c);
            if (have.Count >= 2)
            {
                Debug.Log($"[VideoRig] reusing the existing DollyNodes_FlyThrough ({have.Count} node(s)).");
                return have.ToArray();
            }
        }

        if (!TryDockAxis(director, out var vpos, out var fwd, out _))
        {
            Debug.LogWarning($"[VideoRig] no ACTIVE '{director.VehicleName}' — the final fly-through cannot be " +
                             "laid out from the dock axis. Creating it at the world origin so the shot exists; " +
                             "drag DollyNodes_FlyThrough into place, or re-run SMARC/Video/2d with the vehicle present.");
            vpos = Vector3.zero; fwd = Vector3.forward;
        }

        float floorY = MeasureDockFloorY(vpos + fwd * 45f, out string floorHow);

        if (rootTf == null)
        {
            var go = new GameObject("DollyNodes_FlyThrough");
            Undo.RegisterCreatedObjectUndo(go, "Create fly-through dolly nodes");
            go.transform.SetParent(director.transform, false);
            rootTf = go.transform;
        }
        var nodes = new Transform[DockShotSpec.FlyThroughAlong.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            var n = new GameObject($"FT{i}");
            Undo.RegisterCreatedObjectUndo(n, "Create fly-through dolly node");
            n.transform.SetParent(rootTf, false);
            n.transform.position = WithY(vpos + fwd * DockShotSpec.FlyThroughAlong[i],
                                         floorY + DockShotSpec.FlyThroughAboveFloor);
            nodes[i] = n.transform;
        }
        created = true;
        Debug.Log($"[VideoRig] created the {nodes.Length}-node final fly-through down the empty dock at " +
                  $"{DockShotSpec.FlyThroughAboveFloor:F1} m above a floor of Y = {floorY:F2} ({floorHow}).");
        return nodes;
    }

    /// <summary>
    /// Dock floor height under a point, by raycast. Physics queries work in Edit mode against the
    /// colliders already in the scene, so this is a measurement and not a guess — and when it
    /// misses, it says so and hands back a constant that is labelled as a constant.
    /// </summary>
    static float MeasureDockFloorY(Vector3 xzAt, out string how)
    {
        var from = new Vector3(xzAt.x, xzAt.y + 60f, xzAt.z);
        if (Physics.Raycast(from, Vector3.down, out var hit, 200f, ~0, QueryTriggerInteraction.Ignore))
        {
            how = $"MEASURED by raycast onto '{hit.collider.name}'";
            return hit.point.y;
        }
        how = $"NOT measured — the downward ray from {from} hit nothing, so the fallback constant " +
              $"{DockShotSpec.FallbackDockFloorY:F1} m was used. Check the fly-through height by eye.";
        return DockShotSpec.FallbackDockFloorY;
    }

    /// <summary>
    /// A named marker Transform under the director, created once at a computed position and never
    /// moved by the seeder afterwards.
    ///
    /// WHY A TRANSFORM AND NOT A Vector3 IN THE SHOT: the storyboard has to compare equal to itself
    /// on a second press (§3s8). A position derived from the vehicle changes the moment the vehicle
    /// moves — after a Play, after a re-pose — and the seeder would then "find a difference" every
    /// time and offer to replace the shot list. A scene Transform is stable, so 2b is a genuine
    /// no-op and only 2c/2d ever move these, saying so when they do.
    /// </summary>
    static Transform EnsureMarker(CinematicDirector director, string name, System.Func<Vector3> where, out bool created)
    {
        created = false;
        var existing = director.transform.Find(name);
        if (existing != null) return existing;

        var go = new GameObject(name);
        Undo.RegisterCreatedObjectUndo(go, "Create " + name);
        go.transform.SetParent(director.transform, false);
        go.transform.position = where();
        created = true;
        Debug.Log($"[VideoRig] created marker '{name}' at {go.transform.position}.");
        return go.transform;
    }

    static Vector3 SouthTargetPos(CinematicDirector director, WaterSurface surface)
    {
        float waterY = surface != null ? surface.transform.position.y : 0f;
        if (!TryDockAxis(director, out var vpos, out var fwd, out _)) return Vector3.zero;
        return WithY(vpos + fwd * DockShotSpec.SouthTargetAlong, waterY);
    }

    static Vector3 HoldAbovePos(CinematicDirector director)
    {
        if (!TryDockAxis(director, out var vpos, out _, out _)) return Vector3.up * DockShotSpec.HoldHeight;
        return vpos + Vector3.up * DockShotSpec.HoldHeight;
    }

    static Vector3 DrainCamPos(CinematicDirector director, WaterSurface surface)
    {
        float waterY = surface != null ? surface.transform.position.y : 0f;
        if (!TryDockAxis(director, out var vpos, out var fwd, out var side))
            return new Vector3(0f, DockShotSpec.DrainCamHeight, 0f);
        return WithY(vpos + fwd * DockShotSpec.DrainCamAlong + side * DockShotSpec.DrainCamSide,
                     waterY + DockShotSpec.DrainCamHeight);
    }

    static Vector3 DrainTargetPos(CinematicDirector director, WaterSurface surface)
    {
        float waterY = surface != null ? surface.transform.position.y : 0f;
        if (!TryDockAxis(director, out var vpos, out var fwd, out _)) return Vector3.zero;
        return WithY(vpos + fwd * DockShotSpec.DrainLookAlong, waterY - DockShotSpec.DrainDepthM * 0.5f);
    }

    /// <summary>
    /// Where the zoom-out ENDS: high, at the south end of the dock, ON THE CENTRELINE. The director
    /// derives the orbit's final bearing, radius and height from this Transform relative to wherever
    /// the vehicle actually surfaced, so the sweep lands here whatever the mission did.
    /// </summary>
    static Vector3 OrbitEndPosePos(CinematicDirector director, WaterSurface surface)
    {
        float waterY = surface != null ? surface.transform.position.y : 0f;
        if (!TryDockAxis(director, out var vpos, out var fwd, out _))
            return Vector3.up * DockShotSpec.OrbitEndHeight;
        return WithY(vpos + fwd * DockShotSpec.OrbitEndAlong, waterY + DockShotSpec.OrbitEndHeight);
    }

    /// <summary>
    /// What that final frame LOOKS AT: a point on the dock centreline near the NORTH end. From the
    /// south-end pose above, the view direction through this point is straight down the dock —
    /// which is the shot Ivan asked for, expressed as two markers on one axis rather than as a
    /// bearing constant somebody would have to re-derive after the vehicle moved.
    /// </summary>
    static Vector3 OrbitEndLookPos(CinematicDirector director, WaterSurface surface)
    {
        float waterY = surface != null ? surface.transform.position.y : 0f;
        if (!TryDockAxis(director, out var vpos, out var fwd, out _)) return Vector3.zero;
        return WithY(vpos + fwd * DockShotSpec.OrbitEndLookAlong, waterY);
    }

    /// <summary>
    /// Ivan's beat list, 2026-08-21, in his order: fly in ending ON the vehicle at its start
    /// position; follow it down under; the map building on a third-person wide; a waypoint camera
    /// showing the approach and the pass, turning to follow it by; back to the wide with the map;
    /// the same again on a later hoop; follow it up through the surface; zoom out over the whole
    /// dock with the finished map.
    ///
    /// Read the Advance column as the answer to "why did it cut there".
    /// </summary>
    static List<CameraShot> BuildStoryboard(Transform[] nodes, Transform[] flyThrough,
                                            Transform southTarget, Transform holdAbove,
                                            Transform drainCam, Transform drainTarget,
                                            Transform orbitEnd, Transform orbitEndLook)
    {
        return new List<CameraShot>
        {
            // 1 — the fly-in, LOW and from the SOUTH, along the dock, ending over the hull.
            //     MinSeconds >= Duration so the dolly always LANDS before it can cut; it then holds
            //     that pose until the vehicle actually starts moving (Ivan: time the shots off
            //     where the vehicle is, not off a stopwatch).
            //
            //     AlignPathToTargetHeading is OFF, and that is a fix, not a default. The director
            //     rotates the authored path so its LAST SEGMENT points along the target's forward.
            //     This path's last segment runs BACKWARD along the dock (the camera is flying north
            //     toward a vehicle that faces south), so aligning it rotated the whole sweep ~180°
            //     and brought the camera in from behind the vehicle instead of from the south. The
            //     nodes are already derived from the vehicle's heading; they need translating to
            //     the vehicle, which AnchorEndToTarget does, and nothing else.
            new CameraShot
            {
                Name = "1 fly-in low from the south", Kind = CameraShot.Mode.DollySpline,
                Duration = DockShotSpec.FlyInDuration, FieldOfView = 58f,
                Advance = CameraShot.AdvanceWhen.VehicleMoving,
                // 120 s, not 45: the operator presses F9, then START RECORDING, then arms the
                // mission from MC. The sweep lands at 12 s and HOLDS its final pose until the
                // vehicle moves; the ceiling only exists so it cannot wait forever.
                MinSeconds = DockShotSpec.FlyInDuration, MaxSeconds = 120f,
                SteadySeconds = 0.8f, MoveSpeedThreshold = 0.12f,
                Nodes = nodes, LookAlongPath = true, LookAtName = "",
                AnchorEndToTarget = true, AlignPathToTargetHeading = false,
                EndOffsetInTargetFrame = new Vector3(0f, DockShotSpec.HoldHeight, 0f),
                LookAtOffset = Vector3.zero,
                HudVisible = false, ParticlesEnabled = false,
                SonarMapVisible = false, SonarMapLabelVisible = false, ClearMapAtStart = true,
                SonarBeamsVisible = false
            },

            // 1b — the rotation. The camera stands where shot 1 left it and the BLEND is the swing
            //      round to look south down the dock. Its anchor is the FlyInHoldAbove marker, which
            //      2c places at the vehicle + (0, HoldHeight, 0) — the same point shot 1 ends at.
            //      An anchor Transform rather than a Vector3 on purpose: EvalStatic falls back to
            //      PositionWS, and a PositionWS left at (0,0,0) parks the lens at the world origin.
            new CameraShot
            {
                Name = "1b hold above, look south down the dock", Kind = CameraShot.Mode.StaticLookAt,
                Duration = DockShotSpec.HoldDuration, BlendSeconds = DockShotSpec.HoldBlend,
                FieldOfView = 50f,
                Advance = CameraShot.AdvanceWhen.Duration,
                MinSeconds = DockShotSpec.HoldDuration, MaxSeconds = 0f,
                Anchor = holdAbove, LookAt = southTarget, LookAtOffset = Vector3.zero,
                HudVisible = false, ParticlesEnabled = false,
                SonarMapVisible = false, SonarMapLabelVisible = false,
                SonarBeamsVisible = false
            },

            // 2a — THE DIVE, FROM ABOVE (Ivan, 2026-08-21: "when the vehicle dives, follow from
            //      above until it disappears below the water surface"). The lens stays over the
            //      waterline looking down at the hull, and hands over when the vehicle is 1.5 m
            //      under — which in the baltic preset is the depth at which it has visually gone.
            new CameraShot
            {
                Name = "2a the dive, seen from above", Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 1.2f, FieldOfView = 52f,
                Advance = CameraShot.AdvanceWhen.VehicleSubmerged,
                MinSeconds = 4f, MaxSeconds = 90f, SteadySeconds = 0.6f, SubmergeDepth = 1.5f,
                FollowTargetName = "base_link", FollowOffset = new Vector3(1.5f, 4.0f, -6.0f),
                PositionDamping = 0.5f, RotationDamping = 0.3f, YawOnly = true,
                LookAtOffset = new Vector3(0f, 0.15f, 0f),
                HudVisible = true, ParticlesEnabled = false,
                SonarMapVisible = true, SonarMapLabelVisible = false,
                SonarBeamsVisible = true,
                WaterPresetName = "baltic"
            },

            // 2b — THE PLUNGE, ENDING ON THE VEHICLE'S OWN THIRD-PERSON CAMERA POSE.
            //      Round 2 (Ivan): "then dive in after the AUV and follow during the dive from an
            //      angle some meter below the vehicle" — that is the START offset below.
            //      Round 3 (Ivan): "follow with the camera down under the water (then zoom in closer
            //      to the same pos as the sam third person view cam is)" — so the shot now EASES
            //      from that below-and-behind plunge pose to the pose the vehicle carries itself.
            //
            //      (0.28, 0.21, -1.29) is MEASURED, not chosen: it is the localPosition of the
            //      `3rdPersonCam` GameObject in SMARCAssets/Runtime/Prefabs/sam2.2.prefab, whose
            //      parent is `base_link` with identity rotation — which is exactly the frame
            //      FollowOffset is expressed in (x right, y up, z forward). So the shot literally
            //      finishes looking through SAM's own third-person camera.
            //
            //      Nothing here moves the camera through the surface: the 1.2 s blend from shot 2a's
            //      above-water pose IS the plunge. Short on purpose — a slow zoom across the
            //      waterline reads as a mistake, a quick one reads as a dive.
            new CameraShot
            {
                Name = "2b plunge in after it, closing to SAM's own 3rd-person pose",
                Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 1.2f, FieldOfView = 55f,
                Advance = CameraShot.AdvanceWhen.VehicleSubmerged,
                MinSeconds = 8f, MaxSeconds = 60f, SteadySeconds = 1.0f, SubmergeDepth = 2.5f,
                FollowTargetName = "base_link",
                FollowOffset = new Vector3(0.28f, 0.21f, -1.29f),   // sam2.2.prefab ▸ base_link ▸ 3rdPersonCam
                ZoomInSeconds = 6f, ZoomStartOffset = new Vector3(1.8f, -2.5f, -5.0f),
                PositionDamping = 0.45f, RotationDamping = 0.3f, YawOnly = true,
                NearClip = 0.04f,                                   // the lens ends 1.3 m off the hull
                LookAtOffset = new Vector3(0f, 0.1f, 0f),
                HudVisible = true, ParticlesEnabled = true,
                SonarMapVisible = true, SonarMapLabelVisible = true,
                SonarBeamsVisible = true,
                WaterPresetName = "baltic"
            },

            // 3 — the map building. Far enough back that the cloud is the subject; cuts when the
            //     vehicle starts closing on a hoop, so the waypoint camera catches the APPROACH.
            new CameraShot
            {
                Name = "3 map building, wide third person", Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 0.8f, FieldOfView = 55f,
                Advance = CameraShot.AdvanceWhen.ApproachingHoop,
                MinSeconds = 12f, MaxSeconds = 80f, SteadySeconds = 0.5f, ApproachRange = 18f,
                FollowTargetName = "base_link",
                UseMapFraming = true, MapFramingDistance = 15f, MapFramingHeight = 5.5f, MapFramingSide = 2.5f,
                PositionDamping = 0.7f, RotationDamping = 0.4f, YawOnly = true,
                HudVisible = true, ParticlesEnabled = true,
                SonarMapVisible = true, SonarMapLabelVisible = true,
                SonarBeamsVisible = true,
                // THE MAP-READING SHOTS RUN IN CLEAR WATER (Ivan, round 3: "shift water to the clear
                // version instead and not take it away completely"). At 6 m absorption the blue cloud
                // 15 m behind the vehicle is absorbed to nothing; the point of these shots is the map.
                WaterPresetName = "clear_demo"
            },

            // 4 — the pass. Latches onto the hoop being approached, holds while it comes on, pans
            //     to follow it through, and hands over once it is astern.
            //
            //     ROUND 3: THE LENS IS ON THE HOOP CENTRELINE. Ivan: "make sure the wp cameras taking
            //     shots of the AUV approaching are not placed outside the dock walls. you can even
            //     place them in the middle of the wp." Lateral 0, 0.6 m below the hoop centre (well
            //     inside the 2 m hoop radius), 4.5 m past it — so the vehicle comes straight down the
            //     barrel, through the hoop, and over the lens. WPMaxLateralOffset is the hard stop
            //     that keeps any later hand-edit inside the 17.6 m dock channel.
            new CameraShot
            {
                Name = "4 waypoint pass", Kind = CameraShot.Mode.WaypointCams,
                FieldOfView = 46f, NearClip = 0.02f,   // the hull passes within centimetres
                Advance = CameraShot.AdvanceWhen.HoopPassed,
                MinSeconds = 3f, MaxSeconds = 50f, SteadySeconds = 2.5f,
                FollowTargetName = "base_link",
                WPLatchToOneHoop = true, WPPanDamping = 0.14f,
                WPLateralOffset = 0f, WPVerticalOffset = -0.6f, WPBeyondOffset = 4.5f,
                WPMaxLateralOffset = 2.0f,
                WPMinCutInterval = 3.5f, WPMaxRange = 60f,
                HudVisible = false, ParticlesEnabled = true,
                SonarMapVisible = true, SonarMapLabelVisible = false,
                SonarBeamsVisible = true,
                WaterPresetName = "baltic"
            },

            // 5 — back to the wide, map still growing.
            new CameraShot
            {
                Name = "5 map building again, wide", Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 0.8f, FieldOfView = 55f,
                Advance = CameraShot.AdvanceWhen.ApproachingHoop,
                MinSeconds = 14f, MaxSeconds = 100f, SteadySeconds = 0.5f, ApproachRange = 18f,
                FollowTargetName = "base_link",
                UseMapFraming = true, MapFramingDistance = 17f, MapFramingHeight = 6.5f, MapFramingSide = -3f,
                PositionDamping = 0.7f, RotationDamping = 0.4f, YawOnly = true,
                HudVisible = true, ParticlesEnabled = true,
                SonarMapVisible = true, SonarMapLabelVisible = true,
                SonarBeamsVisible = true,
                WaterPresetName = "clear_demo"
            },

            // 6 — the second pass, on a LATER hoop: the director's LastFilmedHoopIndex makes shot 5
            //     wait for a hoop this shot has not already used, so the two passes differ.
            new CameraShot
            {
                Name = "6 waypoint pass, later hoop", Kind = CameraShot.Mode.WaypointCams,
                FieldOfView = 42f, NearClip = 0.02f,
                Advance = CameraShot.AdvanceWhen.HoopPassed,
                MinSeconds = 3f, MaxSeconds = 50f, SteadySeconds = 2.5f,
                FollowTargetName = "base_link",
                WPLatchToOneHoop = true, WPPanDamping = 0.18f,
                // Also on the centreline; only the framing differs from shot 4 — a tighter lens and
                // a little further past the hoop, so the two passes are different shots without one
                // of them being pushed toward a wall.
                WPLateralOffset = 0f, WPVerticalOffset = -0.4f, WPBeyondOffset = 5.5f,
                WPMaxLateralOffset = 2.0f,
                WPMinCutInterval = 3.5f, WPMaxRange = 60f,
                HudVisible = false, ParticlesEnabled = true,
                SonarMapVisible = true, SonarMapLabelVisible = false,
                SonarBeamsVisible = true,
                WaterPresetName = "baltic"
            },

            // 7 — the long middle: the map fills in while the mission runs. It waits for the
            //     ASCENT, which is the cue that the surface break is coming — cutting on the break
            //     itself would put the camera in place a second late.
            //
            //     ROUND 3, AND THIS IS THE SHOT THAT COST TAKE 006 FOUR MINUTES. Its overlay read
            //     "FELL THROUGH at 240 s — 'VehicleAscending' never happened (vertical 0.00 m/s, at
            //     0.0 m down)": the vehicle had ALREADY SURFACED before the shot started, so the
            //     ascent was not late, it was over. The ceiling is still 240 s for the long mission,
            //     but the director now recognises a MOOT condition at shot start and hands over at
            //     MinSeconds instead — so the short mission, where shot 7 begins after the vehicle is
            //     already up, costs 10 s and not 240. MinSeconds is therefore the number that decides
            //     how long this shot holds the finished map when the mission ended early.
            new CameraShot
            {
                Name = "7 the map fills in, waiting for the ascent", Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 0.8f, FieldOfView = 58f,
                Advance = CameraShot.AdvanceWhen.VehicleAscending,
                MinSeconds = 10f, MaxSeconds = 240f, SteadySeconds = 2.0f, AscentRate = 0.05f,
                SurfaceDepth = 0.35f,   // what counts as "already up", i.e. the ascent is moot
                FollowTargetName = "base_link",
                UseMapFraming = true, MapFramingDistance = 20f, MapFramingHeight = 8f, MapFramingSide = 3.5f,
                PositionDamping = 0.8f, RotationDamping = 0.45f, YawOnly = true,
                HudVisible = true, ParticlesEnabled = true,
                SonarMapVisible = true, SonarMapLabelVisible = true,
                SonarBeamsVisible = true,
                WaterPresetName = "clear_demo"
            },

            // 8 — up through the water line, seen from below and behind. Ends on the vehicle
            //     actually crossing the plane, not on a timer. The map stays ON now (Ivan:
            //     "this draping should be visible in the different shots") — it used to be hidden
            //     here and that hid the one moment where the cloud and the surface are both in shot.
            //
            //     ROUND 3, THE OTHER HALF OF TAKE 006's DEAD AIR: its overlay read "waiting: the
            //     surface break — the vehicle has not been under DURING THIS SHOT". True, and moot:
            //     the vehicle was already up when the shot began, so there was no break left to
            //     film. That case now advances at MinSeconds. THE MISSION CAN END BEFORE THIS SHOT
            //     EVER STARTS, and every late shot has to be right for that.
            new CameraShot
            {
                Name = "8 follow it up through the surface", Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 0.6f, FieldOfView = 60f,
                Advance = CameraShot.AdvanceWhen.VehicleSurfaced,
                MinSeconds = 4f, MaxSeconds = 120f, SteadySeconds = 1.2f, SurfaceDepth = 0.15f,
                FollowTargetName = "base_link", FollowOffset = new Vector3(2.2f, -2.6f, -4.0f),
                PositionDamping = 0.7f, RotationDamping = 0.4f, YawOnly = true,
                HudVisible = false, ParticlesEnabled = true,
                SonarMapVisible = true, SonarMapLabelVisible = false,
                SonarBeamsVisible = true,
                WaterPresetName = "baltic"
            },

            // 9 — THE ENDING, IN ONE MOVE (Ivan, round 3: "why are you waiting 240s after the mission
            //     is done? cant we just start the draining while zooming out and rotating the camera
            //     so we look straight along the centre of the dry dock from south").
            //
            //     So this one shot does all three things at once:
            //       * it ASKS FOR THE DRAIN at shot start (StartDrainAtShotStart). The gate is
            //         unchanged — surfaced AND idle for 5 s, every ForcePoint frozen before the water
            //         moves, always restored (SETTLED §3s and its one sanctioned exception). The
            //         director POLLS that gate twice a second instead of asking once, so the pump
            //         starts the moment the run is genuinely over and the camera never waits for it.
            //       * it zooms out over the whole dock while the water goes down;
            //       * and it LANDS on an authored pose: OrbitEndPose is high at the SOUTH end on the
            //         dock centreline, and the aim eases onto OrbitEndLookAt near the north end, so
            //         the last frame looks straight down the middle of the dry dock. Both markers
            //         come from the dock axis (2c), which is why EndRadius/EndHeight/StartBearingDeg
            //         are not authored here — the anchor supplies them.
            //     It ends when the dock is EMPTY, not when a stopwatch says it ought to be.
            new CameraShot
            {
                Name = "9 zoom out from the south while the dock drains", Kind = CameraShot.Mode.OrbitZoomOut,
                Duration = DockShotSpec.OrbitDuration, BlendSeconds = 1.5f, FieldOfView = 55f,
                Advance = CameraShot.AdvanceWhen.DrainComplete,
                // MinSeconds >= Duration so the camera move always completes; the ceiling is what
                // rescues a rehearsal in which the gate never opens, and it says FELL THROUGH.
                MinSeconds = DockShotSpec.OrbitDuration, MaxSeconds = 75f, SteadySeconds = 0.5f,
                StartDrainAtShotStart = true,
                SurfaceDepth = 0.35f, MoveSpeedThreshold = 0.12f, IdleSeconds = 5f,
                StartRadius = 14f, StartHeight = 3f,
                DegreesSwept = DockShotSpec.OrbitDegreesSwept,
                OrbitEndAnchor = orbitEnd, OrbitEndLookAt = orbitEndLook,
                HudVisible = false, ParticlesEnabled = false,
                SonarMapVisible = true, SonarMapLabelVisible = true,
                SonarBeamsVisible = false,
                WaterPresetName = "clear_demo"
            },

            // 10 — the last shot: down the middle of the EMPTY dock, low, through the point cloud,
            //      SOUTH TO NORTH (Ivan, round 3 wish 9: "Once the basin is drained, fly along the
            //      centre from south to north highlighting the draped point cloud map"). The node
            //      spacing in DockShotSpec.FlyThroughAlong DECREASES — 100, 70, 38, 6 metres along
            //      the vehicle's forward, which is south — so the dolly travels from the south end
            //      back toward the vehicle's start at the north end. That direction is verified
            //      explicitly in 2d and in the readiness report, because the round-2 lesson was that
            //      a path's DIRECTION is exactly the thing nobody checks (AlignPathToTargetHeading
            //      quietly rotated the fly-in 180°).
            //      Manual, so it holds the finished frame until the recording is stopped.
            //      LookAlongPath: the subject is what the lens is flying through, not a target.
            new CameraShot
            {
                Name = "10 fly through the empty dock, south to north", Kind = CameraShot.Mode.DollySpline,
                Duration = DockShotSpec.FlyThroughDuration, BlendSeconds = 1.5f, FieldOfView = 62f,
                Advance = CameraShot.AdvanceWhen.Manual,
                MinSeconds = 0f, MaxSeconds = 0f,
                Nodes = flyThrough, LookAlongPath = true, LookAtName = "",
                // The path is authored in WORLD space down the dock floor and must NOT be moved to
                // the vehicle: by now the vehicle is surfaced at the far end and anchoring to it
                // would drag the fly-through out of the dock it is meant to fly through.
                AnchorEndToTarget = false, AlignPathToTargetHeading = false,
                NearClip = 0.05f,
                // WITHOUT THIS THE DOCK REFILLS ON THIS SHOT'S FIRST FRAME. The director puts the
                // water back whenever a shot that did not ask for the drain begins — right, for
                // every shot but this one, whose whole subject is the empty basin. Round 2 shipped
                // that defect and nobody caught it, because the sequence had never been run.
                KeepDockDrained = true,
                HudVisible = false, ParticlesEnabled = false,
                SonarMapVisible = true, SonarMapLabelVisible = true,
                SonarBeamsVisible = false,
                WaterPresetName = "clear_demo"
            }
        };
    }

    // ================================================================ 2c / 2d. the shot GEOMETRY

    /// <summary>
    /// Author the fly-in geometry (2026-08-21, retuned the same evening after Ivan flew it):
    /// a low sweep in from the SOUTH, along the dock, ending directly over the hull, plus the
    /// marker transforms the hold shot and the drain shot stand on.
    ///
    /// WHAT THIS OWNS, since it changed today. **2b owns the SHOT LIST; 2c owns the TRANSFORMS the
    /// shots point at.** They used to overlap — 2c edited shot 1's fields and inserted shot 1b —
    /// which made "2b then 2c" an order you had to remember and "press 2b again" a false alarm.
    /// Both now read the same `DockShotSpec` constants, so after 2b, 2c says NOTHING TO CHANGE, and
    /// after 2c, 2b says NOTHING TO CHANGE. Either order works, both are idempotent, and the two
    /// cannot drift apart because there is exactly one copy of every number.
    ///
    /// It still CHECKS shots 1 and 1b and repairs them if they have been hand-edited away from the
    /// geometry — a checker, not a second author.
    ///
    /// ASSUMPTION, stated out loud: "south down the dock" is the vehicle's own forward direction
    /// (it is parked facing its first leg). If the sweep arrives from the wrong end, that is the
    /// assumption that failed — drag Node0..Node3, or re-pose the vehicle and re-run.
    /// </summary>
    [MenuItem("SMARC/Video/2c - Author the south fly-in geometry", false, 103)]
    public static void AuthorSouthFlyIn()
    {
        var director = Object.FindFirstObjectByType<CinematicDirector>();
        if (director == null)
        {
            Debug.LogError("[VideoRig] no CinematicDirector in the open scene — run SMARC/Video/2 first.");
            return;
        }
        if (!TryDockAxis(director, out var vpos, out var fwd, out var side))
        {
            Debug.LogError($"[VideoRig] no ACTIVE GameObject named '{director.VehicleName}' with a horizontal " +
                           "heading — the fly-in is authored FROM the vehicle and refuses to guess a dock axis.");
            return;
        }

        var surface = Object.FindFirstObjectByType<WaterSurface>();
        float waterY = surface != null ? surface.transform.position.y : 0f;
        float headingDeg = Quaternion.LookRotation(fwd).eulerAngles.y;

        var nodes = EnsureDollyNodes(director, surface, out _);
        if (nodes.Length < 4)
        {
            Debug.LogError($"[VideoRig] expected 4 fly-in dolly nodes, found {nodes.Length}.");
            return;
        }

        var changes = new System.Text.StringBuilder();

        // ---- the four sweep nodes: low, close, inside the dock channel ------------------
        for (int i = 0; i < 4; i++)
        {
            var want = WithY(vpos + fwd * DockShotSpec.FlyInAlong[i] + side * DockShotSpec.FlyInSide[i],
                             waterY + DockShotSpec.FlyInAltAboveWater[i]);
            MoveIfNeeded(nodes[i], want, $"Node{i}", changes);
        }

        // ---- the markers ---------------------------------------------------------------
        var southTarget = EnsureMarker(director, "DockSouthTarget", () => SouthTargetPos(director, surface), out bool stNew);
        if (stNew) changes.AppendLine($"  * created DockSouthTarget at {southTarget.position}");
        else MoveIfNeeded(southTarget, SouthTargetPos(director, surface), "DockSouthTarget", changes);

        var holdAbove = EnsureMarker(director, "FlyInHoldAbove", () => HoldAbovePos(director), out bool haNew);
        if (haNew) changes.AppendLine($"  * created FlyInHoldAbove at {holdAbove.position}");
        else MoveIfNeeded(holdAbove, HoldAbovePos(director), "FlyInHoldAbove", changes);

        var drainCam = EnsureMarker(director, "DrainCam", () => DrainCamPos(director, surface), out bool dcNew);
        if (dcNew) changes.AppendLine($"  * created DrainCam at {drainCam.position}");
        else MoveIfNeeded(drainCam, DrainCamPos(director, surface), "DrainCam", changes);

        var drainTarget = EnsureMarker(director, "DrainCamTarget", () => DrainTargetPos(director, surface), out bool dtNew);
        if (dtNew) changes.AppendLine($"  * created DrainCamTarget at {drainTarget.position}");
        else MoveIfNeeded(drainTarget, DrainTargetPos(director, surface), "DrainCamTarget", changes);

        // ---- the zoom-out's LANDING POSE, round 3 ---------------------------------------
        // These two are the whole of "look straight along the centre of the dry dock from south":
        // OrbitEndPose stands high at the south end ON THE CENTRELINE, OrbitEndLookAt sits on the
        // same axis near the north end, and the director derives the orbit's final bearing from
        // where they are relative to wherever the vehicle actually surfaced.
        var orbitEnd = EnsureMarker(director, "OrbitEndPose", () => OrbitEndPosePos(director, surface), out bool oeNew);
        if (oeNew) changes.AppendLine($"  * created OrbitEndPose at {orbitEnd.position}");
        else MoveIfNeeded(orbitEnd, OrbitEndPosePos(director, surface), "OrbitEndPose", changes);

        var orbitEndLook = EnsureMarker(director, "OrbitEndLookAt", () => OrbitEndLookPos(director, surface), out bool olNew);
        if (olNew) changes.AppendLine($"  * created OrbitEndLookAt at {orbitEndLook.position}");
        else MoveIfNeeded(orbitEndLook, OrbitEndLookPos(director, surface), "OrbitEndLookAt", changes);

        // ---- check (do not re-author) the shots this geometry serves ---------------------
        RepairFlyInShots(director, nodes, southTarget, holdAbove, changes);
        RepairEndingShot(director, orbitEnd, orbitEndLook, changes);

        if (changes.Length == 0)
        {
            Debug.Log("[VideoRig] south fly-in geometry already authored — NOTHING TO CHANGE, and nothing was written.");
            return;
        }

        EditorSceneManager.MarkSceneDirty(director.gameObject.scene);
        Debug.Log($"[VideoRig] south fly-in authored from '{director.VehicleName}' at {vpos}, heading {headingDeg:F0} deg " +
                  $"(ASSUMED = south down the dock — if the sweep arrives from the wrong end, that is the assumption " +
                  $"that failed; drag Node0..Node3 or re-pose the vehicle and re-run). Changed:\n{changes}" +
                  "SAVE THE SCENE (Cmd-S). Press 2c again: it must say NOTHING TO CHANGE.");
    }

    /// <summary>
    /// Author the FINAL FLY-THROUGH of the drained dock: down the middle, low over the dock FLOOR,
    /// through the accumulated point cloud (Ivan, 2026-08-21, wish 7).
    ///
    /// The floor height is MEASURED by raycast against the dock's own colliders, and the log says
    /// whether it measured or fell back to the constant — a camera at an invented altitude is
    /// either inside the concrete or high over the map, and both read as "the shot is wrong"
    /// rather than as "the number was guessed".
    /// </summary>
    [MenuItem("SMARC/Video/2d - Author the final fly-through of the empty dock", false, 104)]
    public static void AuthorFinalFlyThrough()
    {
        var director = Object.FindFirstObjectByType<CinematicDirector>();
        if (director == null)
        {
            Debug.LogError("[VideoRig] no CinematicDirector in the open scene — run SMARC/Video/2 first.");
            return;
        }
        if (!TryDockAxis(director, out var vpos, out var fwd, out _))
        {
            Debug.LogError($"[VideoRig] no ACTIVE GameObject named '{director.VehicleName}' with a horizontal " +
                           "heading — the fly-through is authored along the dock axis and refuses to guess it.");
            return;
        }

        var nodes = EnsureFlyThroughNodes(director, out bool created);
        if (nodes.Length < 2)
        {
            Debug.LogError($"[VideoRig] expected {DockShotSpec.FlyThroughAlong.Length} fly-through nodes, found {nodes.Length}.");
            return;
        }

        float floorY = MeasureDockFloorY(vpos + fwd * 45f, out string floorHow);
        var changes = new System.Text.StringBuilder();
        if (created) changes.AppendLine($"  * created DollyNodes_FlyThrough ({nodes.Length} nodes)");

        int n = Mathf.Min(nodes.Length, DockShotSpec.FlyThroughAlong.Length);
        for (int i = 0; i < n; i++)
        {
            var want = WithY(vpos + fwd * DockShotSpec.FlyThroughAlong[i],
                             floorY + DockShotSpec.FlyThroughAboveFloor);
            MoveIfNeeded(nodes[i], want, $"FT{i}", changes);
        }

        // ---- VERIFY THE DIRECTION, out loud, before anything is wired -------------------
        // Round 2 lost a take to a path that flew the right shape the wrong way (a 180° rotation
        // nobody checked). Ivan asked for SOUTH → NORTH here, so this states which way the nodes
        // actually run, in metres along the dock axis, and FAILs rather than guessing.
        float alongFirst = Vector3.Dot(nodes[0].position - vpos, fwd);
        float alongLast = Vector3.Dot(nodes[nodes.Length - 1].position - vpos, fwd);
        bool southToNorth = alongFirst > alongLast;
        string dirLine = $"FT0 is {alongFirst:F0} m along +forward (south) and FT{nodes.Length - 1} is " +
                         $"{alongLast:F0} m, so the dolly flies " +
                         (southToNorth ? "SOUTH -> NORTH, which is what wish 9 asked for."
                                       : "NORTH -> SOUTH, which is BACKWARD for wish 9.");
        if (!southToNorth)
            Debug.LogError("[VideoRig] the final fly-through runs the WRONG WAY: " + dirLine +
                           " Reverse DockShotSpec.FlyThroughAlong (it must DECREASE) or re-order " +
                           "DollyNodes_FlyThrough's children. Nothing about the node positions is " +
                           "wrong — only their order, which is the half nobody checks.");

        // The shot itself, if 2b has already seeded it: only the node wiring and the two flags that
        // would silently ruin the shot are checked here. Everything else belongs to 2b. The Mode
        // filter matters — see FindShotByPrefix.
        var shot = FindShotByPrefix(director, "10 ", CameraShot.Mode.DollySpline);
        if (shot != null)
        {
            bool wired = shot.Nodes != null && shot.Nodes.Length == nodes.Length;
            if (wired) for (int i = 0; i < nodes.Length; i++) if (shot.Nodes[i] != nodes[i]) { wired = false; break; }
            if (!wired || shot.AnchorEndToTarget || shot.AlignPathToTargetHeading)
            {
                Undo.RecordObject(director, "Wire the final fly-through");
                shot.Nodes = nodes;
                // NEVER anchor this one to the vehicle: by the last shot the vehicle is surfaced at
                // the far end, and anchoring would drag the path out of the dock it flies through.
                shot.AnchorEndToTarget = false;
                shot.AlignPathToTargetHeading = false;
                changes.AppendLine("  * shot 10 re-wired to DollyNodes_FlyThrough, anchoring OFF");
                EditorUtility.SetDirty(director);
            }
        }
        else
        {
            Debug.Log("[VideoRig] the director has no DollySpline shot named '10 …' — run SMARC/Video/2b to " +
                      "seed the storyboard, then this again to wire the nodes into it. (Round 3 renumbered the " +
                      "ending: the drain no longer has a shot, so the fly-through is 10, not 11.)");
        }

        if (changes.Length == 0)
        {
            Debug.Log("[VideoRig] final fly-through already authored — NOTHING TO CHANGE, and nothing was " +
                      "written. Direction check: " + dirLine);
            return;
        }

        EditorSceneManager.MarkSceneDirty(director.gameObject.scene);
        Debug.Log($"[VideoRig] final fly-through authored along the dock axis at " +
                  $"{DockShotSpec.FlyThroughAboveFloor:F1} m above a floor of Y = {floorY:F2} ({floorHow}).\n" +
                  $"  DIRECTION: {dirLine}\n" +
                  $"Changed:\n{changes}SAVE THE SCENE (Cmd-S). Press 2d again: it must say NOTHING TO CHANGE.");
    }

    static void MoveIfNeeded(Transform t, Vector3 want, string label, System.Text.StringBuilder changes)
    {
        if (t == null) return;
        if ((t.position - want).sqrMagnitude <= 0.01f) return;
        Undo.RecordObject(t, "Author " + label);
        changes.AppendLine($"  * {label} {t.position} -> {want}");
        t.position = want;
    }

    /// <summary>
    /// Find a seeded shot by its leading number. `kind` is not optional decoration: round 3 renumbered
    /// the ending (the drain lost its own shot, so the fly-through moved from 11 to 10), and a scene
    /// still holding the round-2 list has a DrainDock shot sitting at "10 …". Matching the name alone
    /// would have let `2d` wire the fly-through dolly nodes into the drain shot on such a scene — a
    /// silent, plausible-looking miswiring of exactly the kind this file keeps paying for.
    /// </summary>
    static CameraShot FindShotByPrefix(CinematicDirector director, string prefix, CameraShot.Mode? kind = null)
    {
        if (director.Shots == null) return null;
        foreach (var s in director.Shots)
        {
            if (s == null || string.IsNullOrEmpty(s.Name) || !s.Name.StartsWith(prefix)) continue;
            if (kind.HasValue && s.Kind != kind.Value) continue;
            return s;
        }
        return null;
    }

    /// <summary>
    /// Check — and only if necessary repair — the two shots the fly-in geometry serves. It writes
    /// the SAME constants `BuildStoryboard` writes, read from the same place, so it can never
    /// author a value 2b then disagrees with.
    /// </summary>
    static void RepairFlyInShots(CinematicDirector director, Transform[] nodes,
                                 Transform southTarget, Transform holdAbove,
                                 System.Text.StringBuilder changes)
    {
        var s0 = FindShotByPrefix(director, "1 fly-in");
        if (s0 != null)
        {
            bool wired = s0.Nodes != null && s0.Nodes.Length == nodes.Length;
            if (wired) for (int i = 0; i < nodes.Length; i++) if (s0.Nodes[i] != nodes[i]) { wired = false; break; }

            bool wrong = !wired
                         || !s0.LookAlongPath
                         || !s0.AnchorEndToTarget
                         || s0.AlignPathToTargetHeading   // see BuildStoryboard: this rotated the sweep ~180 deg
                         || !string.IsNullOrEmpty(s0.LookAtName)
                         || (s0.EndOffsetInTargetFrame - new Vector3(0f, DockShotSpec.HoldHeight, 0f)).sqrMagnitude > 1e-4f
                         || Mathf.Abs(s0.Duration - DockShotSpec.FlyInDuration) > 1e-3f
                         || Mathf.Abs(s0.MinSeconds - DockShotSpec.FlyInDuration) > 1e-3f;
            if (wrong)
            {
                Undo.RecordObject(director, "Repair the fly-in shot");
                s0.Nodes = nodes;
                s0.LookAlongPath = true;
                s0.LookAtName = "";
                s0.AnchorEndToTarget = true;
                s0.AlignPathToTargetHeading = false;
                s0.EndOffsetInTargetFrame = new Vector3(0f, DockShotSpec.HoldHeight, 0f);
                s0.Duration = DockShotSpec.FlyInDuration;
                s0.MinSeconds = DockShotSpec.FlyInDuration;
                changes.AppendLine("  * shot 1 repaired: nodes wired, LookAlongPath on, heading-align OFF, " +
                                   $"end (0,{DockShotSpec.HoldHeight:F0},0) above the hull, {DockShotSpec.FlyInDuration:F0} s");
                EditorUtility.SetDirty(director);
            }
        }

        var s1b = FindShotByPrefix(director, "1b");
        if (s1b != null)
        {
            bool wrong = s1b.Kind != CameraShot.Mode.StaticLookAt
                         || s1b.Anchor != holdAbove
                         || s1b.LookAt != southTarget;
            if (wrong)
            {
                Undo.RecordObject(director, "Repair the look-south hold shot");
                s1b.Kind = CameraShot.Mode.StaticLookAt;
                s1b.Anchor = holdAbove;                  // NEVER rely on PositionWS: EvalStatic
                s1b.PositionWS = holdAbove.position;     // falls back to it, and (0,0,0) is the origin
                s1b.LookAt = southTarget;
                s1b.LookAtName = "";
                changes.AppendLine("  * shot 1b repaired: anchored to FlyInHoldAbove, looking at DockSouthTarget");
                EditorUtility.SetDirty(director);
            }
        }
    }

    /// <summary>
    /// Check — and only if necessary repair — the wiring of the ending shot: the zoom-out has to be
    /// pointed at the two markers this menu item authors, and it has to be the shot that starts the
    /// drain. Same contract as RepairFlyInShots: 2b OWNS the shot, this only repairs a hand-edit
    /// that broke the geometry, and it writes the same constants from the same place.
    /// </summary>
    static void RepairEndingShot(CinematicDirector director, Transform orbitEnd, Transform orbitEndLook,
                                 System.Text.StringBuilder changes)
    {
        var s9 = FindShotByPrefix(director, "9 ", CameraShot.Mode.OrbitZoomOut);
        if (s9 == null) return;

        bool wrong = s9.OrbitEndAnchor != orbitEnd
                     || s9.OrbitEndLookAt != orbitEndLook
                     || !s9.StartDrainAtShotStart
                     || Mathf.Abs(s9.DegreesSwept - DockShotSpec.OrbitDegreesSwept) > 1e-3f;
        if (!wrong) return;

        Undo.RecordObject(director, "Repair the ending shot");
        s9.OrbitEndAnchor = orbitEnd;
        s9.OrbitEndLookAt = orbitEndLook;
        s9.StartDrainAtShotStart = true;
        s9.DegreesSwept = DockShotSpec.OrbitDegreesSwept;
        changes.AppendLine("  * shot 9 repaired: orbit lands on OrbitEndPose looking at OrbitEndLookAt, " +
                           "and it starts the drain at shot start");
        EditorUtility.SetDirty(director);
    }

    static Vector3 WithY(Vector3 v, float y) { v.y = y; return v; }

    // ================================================================ 3. presets

    [MenuItem("SMARC/Video/3 - Apply water preset - baltic", false, 120)]
    public static void ApplyBaltic() => ApplyPreset("baltic");

    [MenuItem("SMARC/Video/3 - Apply water preset - baltic_murky", false, 121)]
    public static void ApplyBalticMurky() => ApplyPreset("baltic_murky");

    [MenuItem("SMARC/Video/3 - Apply water preset - clear_demo", false, 122)]
    public static void ApplyClear() => ApplyPreset("clear_demo");

    public static void ApplyPreset(string name)
    {
        var preset = Object.FindFirstObjectByType<BalticWaterPreset>();
        if (preset == null)
        {
            Debug.LogError("[VideoRig] no BalticWaterPreset in the open scene — run SMARC/Video/2 first.");
            return;
        }
        var surface = preset.ResolveSurface();
        if (surface == null) { Debug.LogError("[VideoRig] the preset has no WaterSurface."); return; }

        Undo.RecordObject(surface, $"Apply water preset {name}");
        Undo.RecordObject(preset, $"Apply water preset {name}");
        if (!preset.ApplyPreset(name)) return;

        EditorUtility.SetDirty(surface);
        EditorUtility.SetDirty(preset);
        PrefabUtility.RecordPrefabInstancePropertyModifications(surface);
        PrefabUtility.RecordPrefabInstancePropertyModifications(preset);
        EditorSceneManager.MarkSceneDirty(surface.gameObject.scene);
        Debug.Log($"[VideoRig] '{name}' applied in EDIT mode. SAVE THE SCENE — otherwise this is gone at the next open.");
    }

    // ================================================================ 4. lights

    [MenuItem("SMARC/Video/4 - Fix duplicate shadow-casting directional lights", false, 140)]
    public static void FixDirectionalShadows()
    {
        var lights = Object.FindObjectsByType<Light>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        var casters = new List<Light>();
        foreach (var l in lights)
            if (l.type == LightType.Directional && l.shadows != LightShadows.None) casters.Add(l);

        if (casters.Count <= 1)
        {
            Debug.Log($"[VideoRig] {casters.Count} shadow-casting directional light(s) — nothing to fix. " +
                      "If the 'Cascade Shadow atlasing has failed' error persists, one of them is being " +
                      "enabled at Play by a script rather than living in the scene.");
            return;
        }

        // Keep the brightest, or one literally called Sun; that is the key light by intent.
        Light keep = null;
        foreach (var l in casters) if (l.name == "Sun") keep = l;
        if (keep == null) { keep = casters[0]; foreach (var l in casters) if (l.intensity > keep.intensity) keep = l; }

        foreach (var l in casters)
        {
            if (l == keep) continue;
            Undo.RecordObject(l, "Disable duplicate directional shadows");
            l.shadows = LightShadows.None;
            EditorUtility.SetDirty(l);
            PrefabUtility.RecordPrefabInstancePropertyModifications(l);
            Debug.Log($"[VideoRig] shadows OFF on directional light '{HierarchyPath(l.transform)}' (intensity {l.intensity:F2}).");
        }
        Debug.Log($"[VideoRig] kept shadows on '{HierarchyPath(keep.transform)}'. SAVE THE SCENE.");
        EditorSceneManager.MarkSceneDirty(keep.gameObject.scene);
    }

    // ================================================================ 5. readiness

    [MenuItem("SMARC/Video/5 - Report scene readiness", false, 160)]
    public static void ReportReadiness()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("[VideoRig] readiness for the Beckholmen video take:");

        var surface = Object.FindFirstObjectByType<WaterSurface>();
        if (surface == null) sb.AppendLine("  FAIL  no WaterSurface in the scene.");
        else
        {
            float y = surface.transform.position.y;
            sb.AppendLine(Mathf.Abs(y) < 1e-4f
                ? "  ok    Water transform Y = 0 (SETTLED §3s trap not armed)."
                : $"  FAIL  Water transform Y = {y:F3}. Move the TERRAIN, not the water.");
            sb.AppendLine($"  info  absorption {surface.absorptionDistance:F1} m, underwater {(surface.underWater ? "on" : "OFF")}, " +
                          $"volume bounds {(surface.volumeBounds != null ? "present" : "MISSING — no underwater view")}, " +
                          $"scriptInteractions {(surface.scriptInteractions ? "ON (not the shipped setting)" : "off")}.");
            var preset = surface.GetComponent<BalticWaterPreset>();
            sb.AppendLine(preset != null
                ? $"  ok    BalticWaterPreset present, ActivePreset '{preset.ActivePreset}' " +
                  $"(known: {string.Join(", ", preset.PresetNames())})."
                : "  FAIL  no BalticWaterPreset — run SMARC/Video/2.");

            // ---- ROUND 3: does the underwater volume cover where the mission flies? -----------
            // The answer to "after wp2 the water suddenly disappears". HDRP tests the CAMERA against
            // this box (HDRP 17.3, HDRenderPipeline.WaterSystem.Underwater.cs) and renders no
            // underwater view at all outside it, whatever the preset says.
            var dirForVolume = Object.FindFirstObjectByType<CinematicDirector>();
            if (surface.volumeBounds != null)
            {
                var bc = surface.volumeBounds;
                Vector3 ws = Vector3.Scale(bc.size, surface.transform.lossyScale);
                sb.AppendLine($"  info  underwater volume BoxCollider {ws.x:F0} x {ws.y:F0} x {ws.z:F0} m, " +
                              $"trigger {(bc.isTrigger ? "yes" : "NO — it is a solid wall")}, centred at " +
                              $"{surface.transform.TransformPoint(bc.center)}.");
                if (!bc.isTrigger)
                    sb.AppendLine("  FAIL  the underwater volume collider is NOT a trigger — tick Is Trigger on " +
                                  "BeckholmenWorld ▸ Water ▸ Box Collider before enlarging it.");
            }
            EnsureUnderwaterVolumeCovers(dirForVolume, surface, apply: false, out string volReport);
            if (!string.IsNullOrEmpty(volReport)) sb.AppendLine("  " + volReport);
        }

        var hoopSub = Object.FindFirstObjectByType<MissionWPHoop_Sub>(FindObjectsInactive.Include);
        if (hoopSub == null) sb.AppendLine("  FAIL  no MissionWPHoop_Sub — no hoops will appear.");
        else
        {
            sb.AppendLine(hoopSub.HoopMaterial != null
                ? $"  ok    hoop material '{hoopSub.HoopMaterial.name}' assigned (no Shader.Find fallback)."
                : "  WARN  no hoop material — hoops fall back to Shader.Find, the §3o suspect.");
            sb.AppendLine(hoopSub.Collidable
                ? "  FAIL  hoops are COLLIDABLE — every waypoint becomes a sonar target and the map will draw them."
                : "  ok    hoops are not sonar targets.");
            sb.AppendLine($"  info  hoops emissive {(hoopSub.UseEmissive ? hoopSub.EmissiveIntensity.ToString("F1") : "OFF")}, " +
                          $"topic {hoopSub.topic}, robot '{hoopSub.RobotName}'.");
        }

        var map = Object.FindFirstObjectByType<SonarMapAccumulator>();
        if (map == null) sb.AppendLine("  FAIL  no SonarMapAccumulator.");
        else
        {
            sb.AppendLine(map.MapMaterial != null ? "  ok    sonar map material assigned." : "  WARN  no sonar map material.");
            sb.AppendLine($"  info  label \"{map.MapLabel}\" / \"{map.MapSubLabel}\".");

            // ---- ROUND 3: the above-surface filter -------------------------------------------
            if (!map.DiscardHitsAboveWater)
                sb.AppendLine("  WARN  the ABOVE-WATER FILTER is off — returns from the quay tops will be drawn " +
                              "into the map, which take 006 did. A raycast sonar images concrete standing in " +
                              "the air; a real sonar cannot.");
            else if (map.WaterPlaneSource == null && surface == null)
                sb.AppendLine("  FAIL  the above-water filter is on but there is no WaterSurface to read the " +
                              "still-water plane from. It will turn ITSELF off at Play and say so; it never " +
                              "assumes Y = 0.");
            else
            {
                float planeY = map.WaterPlaneSource != null
                    ? map.WaterPlaneSource.transform.position.y
                    : surface.transform.position.y;
                sb.AppendLine($"  ok    above-water filter ON: everything above Y = " +
                              $"{planeY - map.AboveWaterMarginM:F2} m is discarded (plane {planeY:F2} m off the " +
                              $"WaterSurface TRANSFORM, margin {map.AboveWaterMarginM:F2} m). Never GetWaterLevelAt (§3s).");
            }

            // ---- ROUND 3: the blue depth ramp -------------------------------------------------
            sb.AppendLine(map.IncludeSSS
                ? "  WARN  Include SSS is TICKED — Ivan asked for the side scan to be skipped in the map " +
                  "(round 3). Untick it, or expect the dock walls and a great many more points."
                : "  ok    Include SSS off (the map is the FLS draping the floor, as asked).");
            if (map.ColorBy != SonarMapAccumulator.ColorSource.Depth)
                sb.AppendLine($"  WARN  the map is coloured by {map.ColorBy}, not Depth — the blue depth ramp " +
                              "will not be what you see.");
            else if (!map.DepthRampRelativeToWaterPlane)
                sb.AppendLine($"  WARN  depth colouring is on the LEGACY world-Y fields " +
                              $"({map.DepthRampTop:F1} .. {map.DepthRampBottom:F1}), not depth below the water " +
                              "plane. Run SMARC/Video/2.");
            else
                sb.AppendLine($"  ok    blue depth ramp over a FIXED {map.DepthRampShallowM:F1}–" +
                              $"{map.DepthRampDeepM:F1} m below the water plane (light cyan shallow → dark navy " +
                              "deep). Fixed, not auto-rescaled: a point's colour is baked in when it is added.");
            if (map.RampNeedsReseeding(out string rampState))
                sb.AppendLine($"  WARN  the colour gradient is not the blue depth ramp — {rampState}. Run SMARC/Video/2.");
        }

        // ---- the check that would have caught defect A before the 643 s take ----------------
        string robotName = map != null ? map.RobotName : "sam_auv_v1";
        int named = 0, namedActive = 0;
        foreach (var t in Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (t == null || t.parent != null || t.name != robotName) continue;
            named++;
            if (t.gameObject.activeInHierarchy) namedActive++;
        }
        if (named > 1)
            sb.AppendLine($"  WARN  {named} root objects are named '{robotName}' ({namedActive} active). " +
                          "Beckholmen holds an INACTIVE sam_auv_v1.prefab and an ACTIVE sam2.2.prefab both " +
                          "renamed sam_auv_v1 — everything binds to the active one, which is the one with " +
                          "no MBES on it.");

        var robot = GameObject.Find(robotName);
        if (robot == null) sb.AppendLine("  FAIL  the vehicle GameObject was not found by name.");
        else
        {
            var sonars = robot.GetComponentsInChildren<Sonar>(true);
            sb.AppendLine($"  info  {sonars.Length} Sonar component(s) under '{robot.name}':");
            int wouldTap = 0;
            foreach (var s in sonars)
            {
                bool included = (s.Type == SonarType.MBES && map != null && map.IncludeMBES)
                             || (s.Type == SonarType.SSS && map != null && map.IncludeSSS)
                             || (s.Type == SonarType.FLS && map != null && map.IncludeFLS);
                if (included && s.gameObject.activeInHierarchy) wouldTap++;
                sb.AppendLine($"          {s.name}  type {s.Type}  {s.frequency:F0} Hz  {s.TotalRayCount} rays  " +
                              $"range {s.MaxRange:F0} m  active {s.gameObject.activeInHierarchy}  " +
                              $"{(included ? "TAPPED by the map" : "excluded (Include" + s.Type + " is off)")}");
            }
            if (map != null)
                sb.AppendLine(wouldTap > 0
                    ? $"  ok    the sonar map would tap {wouldTap} of {sonars.Length} sonar(s)."
                    : "  FAIL  the sonar map would tap NOTHING — the Include* flags exclude every sonar this " +
                      "vehicle has. THIS IS DEFECT A: it is what produced 'sonar map · 0 pts' for the whole " +
                      "643 s take on 2026-08-21. Tick Include FLS on VideoRig ▸ SonarMapAccumulator.");
        }

        var drain = Object.FindFirstObjectByType<DockDrainDirector>();
        if (drain == null) sb.AppendLine("  WARN  no DockDrainDirector — the drain shot will play over a full dock.");
        else
            sb.AppendLine($"  ok    DockDrainDirector present: {drain.DrainDepthM:F1} m over {drain.DrainSeconds:F0} s, " +
                          $"gate {(drain.BypassGateForRehearsal ? "BYPASSED — untick Bypass Gate For Rehearsal before a take" : "surfaced+idle")}. " +
                          "It refuses in Edit mode, so the scene can never be saved with the water off zero.");

        var director = Object.FindFirstObjectByType<CinematicDirector>();
        if (director == null) sb.AppendLine("  FAIL  no CinematicDirector.");
        else
        {
            sb.AppendLine($"  ok    CinematicDirector with {director.Shots.Count} shot(s); overlay " +
                          $"{(director.ShowOverlay ? "ON — TURN OFF BEFORE THE TAKE" : "off")}; auto-advance " +
                          $"{(director.AutoAdvanceEnabled ? "ON (one-take)" : "OFF (every shot manual)")}.");
            sb.AppendLine(director.Surface != null
                ? "  ok    director has a WaterSurface — the depth conditions can be evaluated."
                : "  WARN  director has no WaterSurface assigned; it will find one at Play, and if there is none " +
                  "the submerged/surfaced/ascending conditions will SAY they cannot be evaluated and fall through.");

            // The property that makes a rehearsal safe: no shot can wait forever.
            var hangs = new List<string>();
            for (int i = 0; i < director.Shots.Count; i++)
            {
                var s = director.Shots[i];
                if (s == null) { hangs.Add($"{i + 1} (null shot)"); continue; }
                bool last = i == director.Shots.Count - 1;
                if (!last && s.EffectiveAdvance() == CameraShot.AdvanceWhen.Manual && s.MaxSeconds <= 0f)
                    hangs.Add($"{i + 1} '{s.Name}' is Manual with no ceiling");
            }
            sb.AppendLine(hangs.Count == 0
                ? "  ok    every non-final shot either has a condition or a ceiling — the storyboard cannot hang."
                : $"  WARN  these shots stop the take dead if you are not on the keyboard: {string.Join("; ", hangs)}.");

            // ---- ROUND 3: the per-shot water presets ------------------------------------------
            var wp = surface != null ? surface.GetComponent<BalticWaterPreset>() : null;
            var missing = new List<string>();
            // `presetShots`, not `named`: this method already declares `int named` (the
            // two-sam_auv_v1 counter, ~line 1793) and C# CS0136 forbids the shadowing. Caught on
            // first compile 2026-08-22.
            var presetShots = new List<string>();
            int drainStarters = 0;
            foreach (var s in director.Shots)
            {
                if (s == null) continue;
                if (s.StartDrainAtShotStart || s.Kind == CameraShot.Mode.DrainDock) drainStarters++;
                if (string.IsNullOrEmpty(s.WaterPresetName)) continue;
                presetShots.Add($"{s.Name} → '{s.WaterPresetName}'");
                if (wp == null || wp.Find(s.WaterPresetName) == null) missing.Add($"{s.Name} → '{s.WaterPresetName}'");
            }
            sb.AppendLine(missing.Count == 0
                ? $"  ok    {presetShots.Count} shot(s) drive the water look, and every preset they name exists."
                : $"  FAIL  these shots name a water preset that does not exist: {string.Join("; ", missing)}. " +
                  "Those shots will leave the water as saved rather than substitute a look.");
            if (director.WaterPreset == null)
                sb.AppendLine("  WARN  the director has no BalticWaterPreset assigned; it will find one at Play. " +
                              "With none in the scene, WaterPresetName is a no-op that says so.");

            // ---- ROUND 3: the merged ending ----------------------------------------------------
            sb.AppendLine(drainStarters == 1
                ? "  ok    exactly one shot starts the dock drain."
                : drainStarters == 0
                    ? "  WARN  NO shot starts the dock drain — the take will end over a full dock. The zoom-out " +
                      "is supposed to carry it (Start Drain At Shot Start on shot 9)."
                    : $"  WARN  {drainStarters} shots start a drain; the dock refills between them, which works " +
                      "but is almost certainly not what was meant.");
            // Every shot AFTER the one that starts the drain must keep the dock empty, or the water
            // comes back on its first frame. This is a whole-sequence property, so it is checked as
            // one rather than trusted per shot.
            int drainAt = -1;
            for (int i = 0; i < director.Shots.Count; i++)
            {
                var s = director.Shots[i];
                if (s != null && (s.StartDrainAtShotStart || s.Kind == CameraShot.Mode.DrainDock)) { drainAt = i; break; }
            }
            if (drainAt >= 0)
            {
                var refills = new List<string>();
                for (int i = drainAt + 1; i < director.Shots.Count; i++)
                {
                    var s = director.Shots[i];
                    if (s == null) continue;
                    if (!s.KeepDockDrained && !s.StartDrainAtShotStart && s.Kind != CameraShot.Mode.DrainDock)
                        refills.Add($"{i + 1} '{s.Name}'");
                }
                sb.AppendLine(refills.Count == 0
                    ? "  ok    every shot after the drain keeps the dock empty."
                    : $"  FAIL  these shots come after the drain WITHOUT 'Keep Dock Drained', so the water " +
                      $"returns on their first frame: {string.Join("; ", refills)}. The fly-through through an " +
                      "empty dock is the last beat of the video; this is the field that decides whether it is " +
                      "empty.");
            }

            var orbit = FindShotByPrefix(director, "9 ", CameraShot.Mode.OrbitZoomOut);
            if (orbit != null)
                sb.AppendLine(orbit.OrbitEndAnchor != null
                    ? $"  ok    the zoom-out LANDS on '{orbit.OrbitEndAnchor.name}'" +
                      (orbit.OrbitEndLookAt != null ? $", looking at '{orbit.OrbitEndLookAt.name}'" : "") +
                      " — an end bearing authored from the dock axis, not a generic orbit."
                    : "  WARN  the zoom-out has no Orbit End Anchor, so it stops on whatever bearing " +
                      "StartBearingDeg + DegreesSwept happens to reach. Run SMARC/Video/2c.");

            // ---- ROUND 3: the fly-through DIRECTION, stated rather than assumed -----------------
            var ftRoot = director.transform.Find("DollyNodes_FlyThrough");
            if (ftRoot != null && ftRoot.childCount >= 2 &&
                TryDockAxis(director, out var vp, out var fw, out _))
            {
                var f0 = ftRoot.GetChild(0);
                var fN = ftRoot.GetChild(ftRoot.childCount - 1);
                float a0 = Vector3.Dot(f0.position - vp, fw);
                float aN = Vector3.Dot(fN.position - vp, fw);
                sb.AppendLine(a0 > aN
                    ? $"  ok    the final fly-through runs SOUTH → NORTH ({f0.name} at {a0:F0} m along the dock " +
                      $"axis → {fN.name} at {aN:F0} m), which is wish 9."
                    : $"  FAIL  the final fly-through runs NORTH → SOUTH ({f0.name} at {a0:F0} m → {fN.name} at " +
                      $"{aN:F0} m) — BACKWARD. Re-run SMARC/Video/2d; the positions are fine, the ORDER is not.");
            }

            for (int i = 0; i < director.Shots.Count; i++)
            {
                var s = director.Shots[i];
                if (s != null)
                    sb.AppendLine($"          {i + 1}. {s.Name}  [{s.Kind}]  {s.AdvanceSummary()}" +
                                  (string.IsNullOrEmpty(s.WaterPresetName) ? "" : $"  water '{s.WaterPresetName}'") +
                                  (s.StartDrainAtShotStart ? "  +DRAIN" : ""));
            }
        }

        var casters = 0;
        foreach (var l in Object.FindObjectsByType<Light>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
            if (l.type == LightType.Directional && l.shadows != LightShadows.None) casters++;
        sb.AppendLine(casters <= 1
            ? "  ok    one shadow-casting directional light."
            : $"  WARN  {casters} shadow-casting directional lights — HDRP will log 'Cascade Shadow atlasing has failed'. Run SMARC/Video/4.");

        var recorder = File.Exists("Packages/manifest.json") && File.ReadAllText("Packages/manifest.json").Contains("com.unity.recorder");
        sb.AppendLine(recorder ? "  ok    com.unity.recorder is in the manifest." : "  WARN  com.unity.recorder is NOT in Packages/manifest.json.");

        Debug.Log(sb.ToString());
    }

    static string HierarchyPath(Transform t)
    {
        var s = t.name;
        while (t.parent != null) { t = t.parent; s = t.name + "/" + s; }
        return s;
    }
}
