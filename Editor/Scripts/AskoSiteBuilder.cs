using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

using GeoRef;
using Smarc.Environment;

/// <summary>
/// Builds the Askö CURATED site from every real survey we hold, via the data produced by
/// data-cube/scripts/asko-site/{build_heightmap.py, build_splatmap.py} :
///
///   AskoCurated_&lt;tile&gt;.asset   four TerrainDatas, 4096 m each at 1 m (4097^2), 2x2
///   AskoCuratedWorld.prefab    cloned from AskoWorld.prefab so Ocean/Sky/Sun come from the
///                              proven Askö setup; the GLOBALREF and terrains are new
///   AskoCurated.unity          the scene
///
/// REFACTORED 2026-09-15. The 700 lines that built the terrains, the world prefab and the
/// scene now live in <see cref="CuratedSiteBuilder"/> and are driven by the
/// <see cref="CuratedSiteConfig"/> below — because Djurö became the second curated site and
/// SETTLED §3d says one pipeline, never a second implementation. Behaviour here is
/// unchanged: same menu item, same asset paths, same log lines, same order. What stays in
/// this file is what is genuinely Askö's: the nested 0.125 m Deep Vision patch, and the
/// hand-placed mission set (Ivan's MMT Mini, the bay line, the station).
///
/// Scene: 8192 m x 8192 m covering the whole island group AND the FM2022-10168:2 measurement
/// permit. Four terrains because Unity's heightmapResolution maxes at 4097: one terrain
/// could hold 8192 m only at 2 m/sample, and halving the sampling to fit the box would throw
/// away half of what the lidar measured — the same rule as never letting a lower-accuracy
/// source overwrite a higher-accuracy one, applied to geometry.
///
/// Frame: Unity +X = UTM 33N easting, +Z = UTM 33N northing, origin = the REGISTERED Askö
/// site origin (data-cube/maps/sites.yaml `asko`: 58.8228662 N, 17.6378932 E). Water at y = 0.
///
/// **THE LEGACY ASKÖ ASSETS ARE IN A DIFFERENT FRAME.** AskoWorld.prefab's own
/// "GLOBALREF - AskoPierTip" sits at Unity (-871, 0, -1934) carrying 58.823332 N /
/// 17.635227 E, which puts that frame's origin at UTM33 (653020.81, 6525308.63) —
/// **2105 m from the registered site origin** (dE +715.1, dN +1979.8). AskoEvolo / AskoEmpty
/// / Asko.prefab and this curated scene do NOT share coordinates. Measured 2026-08-27; the
/// builder re-asserts it on every build so the discrepancy cannot quietly disappear.
///
/// SYNTHETIC SEABED: cells no survey reached carry a chart-extrapolated patch. Each terrain
/// gets an AskoGapPatchLayer whose toggle turns those cells into terrain HOLES, and a second
/// splat layer tints them while they are shown.
///
/// Run from menu: SMARC -> Build Asko Curated Site
/// Safe to re-run; overwrites the generated assets.
/// </summary>
public static class AskoSiteBuilder
{
    const string DataDir = "Packages/com.smarc.assets/Runtime/Terrain/Asko";
    const string PrefabDir = CuratedSiteBuilder.PrefabDir;
    const string AskoWorldPath = PrefabDir + "/AskoWorld.prefab";
    const string WorldPrefabPath = PrefabDir + "/AskoCuratedWorld.prefab";
    const string ScenePath = "Assets/Scenes/AskoCurated.unity";
    const string MatDir = CuratedSiteBuilder.MatDir;
    const string MatTerrain = CuratedSiteBuilder.MatTerrain;
    const string TileMaterialPath = CuratedSiteBuilder.TileMaterialPath;
    const string MudPhysicPath = MatDir + "/Physic/Mud.physicMaterial";

    // The legacy frame, measured from AskoWorld.prefab (see the class comment).
    const double LegacyOriginUtmE = 653020.81, LegacyOriginUtmN = 6525308.63;

    /// <summary>Everything about Askö that the shared builder needs to know.</summary>
    public static readonly CuratedSiteConfig Config = new CuratedSiteConfig
    {
        prefix = "asko",
        assetPrefix = "AskoCurated",
        layerPrefix = "Asko_",
        tag = "[Asko]",
        dataDir = DataDir,
        bundleRel = "../smds-cloud-store/scenario-bundles/ov-site-Asko-curated-v1/payload",
        sourceWorldPrefab = AskoWorldPath,
        worldPrefabPath = WorldPrefabPath,
        worldObjectName = "AskoCuratedWorld",
        scenePath = ScenePath,
        globalRefName = "GLOBALREF - AskoSiteOrigin",
        expectedUtmZone = 33,
        legacyOriginUtm = new[] { LegacyOriginUtmE, LegacyOriginUtmN },
        legacyOriginWhat = "the legacy Askö assets (AskoWorld/AskoEvolo/Asko.prefab)",
        gapPatchSemantics =
            "the chart-extrapolated 59.6% of the seabed becomes terrain HOLES when " +
            "AskoGapPatchLayer.showSyntheticPatches is off. The measured 40.4% stays, and " +
            "so does its rights status: the real-only surface does not carry the FUK licence.",
        wireNeighbours = true,
        terrainPhysicMaterial = "Mud",
        sceneExistsHint =
            "To add the mission set (vehicle, GUI, station, sonar HUDs, hoop) run " +
            "SMARC -> Populate Asko Scene. ",
        sceneCreatedHint =
            "\n       now run SMARC -> Populate Asko Scene for the mission set.",
    };

    // ---- the nested-patch manifest model. Askö's alone: no other site has a hi-res patch.
    [Serializable]
    public class PatchStatsJ { public float min_m, max_m; }
    [Serializable]
    public class PatchJ
    {
        public string generated_utc, generator, source_key, name;
        public string heightmap, source_mask, hole_mask, seabed_class_map, control_map;
        public int resolution, mask_resolution;
        public float terrain_size_x_m, terrain_size_z_m, terrain_size_y_m, terrain_base_y_m;
        public CuratedSiteBuilder.Vec3J unity_position;
        public PatchStatsJ statistics;
    }

    [MenuItem("SMARC/Build Asko Curated Site")]
    public static void Build()
    {
        CuratedSiteBuilder.Build(Config);
    }

    static string AssetPathToFull(string p) { return CuratedSiteBuilder.AssetPathToFull(p); }
    static string PayloadFull(string f) { return CuratedSiteBuilder.PayloadFull(Config, f); }
    static CuratedSiteBuilder.SiteJ LoadSite() { return CuratedSiteBuilder.LoadSite(Config); }
    static CuratedSiteBuilder.SplatJ LoadSplatManifest()
    { return CuratedSiteBuilder.LoadSplatManifest(Config); }
    static SeabedAcousticMap.ClassEntry MakeClass(string n, CuratedSiteBuilder.AcousticClassJ j)
    { return CuratedSiteBuilder.MakeClass(n, j); }
    static float TerrainHeightAt(GameObject world, float x, float z)
    { return CuratedSiteBuilder.TerrainHeightAt(world, x, z); }
    static GameObject Place(string prefabPath, string name, Vector3 pos, float yawDeg = 0)
    { return CuratedSiteBuilder.Place(Config, prefabPath, name, pos, yawDeg); }

    // ---- the bay line (Ivan, 2026-08-27). THE CAR'S POSITION IS IVAN'S OWN — he placed
    // MMTMiniCooper by hand at (-171.8, -155.4), which sits in 8.0 m of water on REAL
    // MULTIBEAM bathymetry, and the line is routed OVER it rather than the car moved to the
    // line. Launch and waypoints are CHOSEN (not surveyed), on measured seabed throughout:
    //     launch (-150,-158)  -3.05 m [lidar]   rocky shore slope
    //     wp1    (-171.8,-155.4) -8.02 [MB]     THE CAR — Ivan's placement
    //     wp2    (-250,-145)  -8.96 [MB]        sand
    //     wp3    (-330,-132) -11.20 [MB]        sand
    //     wp4    (-400,-160) -12.98 [MB]        crossing the wave base -> clay
    //     wp5    (-470,-190) -15.70 [MB]        clay
    static readonly Vector3 LaunchXZ = new Vector3(-150f, 0f, -158f);
    static readonly Vector3 StationXZ = new Vector3(30f, 0f, 10f);   // shore by the lab, +1.3 m [lidar]

    const string VehiclePrefab = "Packages/com.smarc.assets/Runtime/Prefabs/sam21.prefab";
    // The GameObject name IS the ROS namespace: ROSBehaviour reads robot_name = robotGO.name
    // and builds /{robot_name}/{topic} and tf frame {robot_name}/{link} from it. `sam21` = the
    // SAM 2.1 hull (VehiclePrefab above is sam21.prefab); a dot is illegal in a ROS name.
    const string VehicleName = "sam21";   // prefixes every ROS topic and tf frame
    const string GuiPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/SmarcGUI/GUI.prefab";
    const string StationPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/datacube_station_01.prefab";
    const string MiniPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/MMTMini/MMTMiniCooper.prefab";

    // =========================================================================
    // THE NESTED 0.125 m DEEP VISION PATCH  (added 2026-08-30, Phase 2)
    //
    // A second, higher-accuracy source layer for the bay outside the station: the same
    // 2024-06-04 Deep Vision delivery the 1 m base already uses, gridded at 0.125 m into ONE
    // 512 m terrain of 4097 samples. This is the sea-chart gap patch pattern pointed the
    // other way — instead of a lower-accuracy source filling holes, a higher-accuracy one
    // takes over an area — and it obeys the same three rules:
    //
    //   * SEPARABLE. Its own manifest, its own per-cell source_id raster, its own terrain.
    //     Deleting the patch files and re-running the base build returns the scene exactly to
    //     where it was; the base grids were never rewritten
    //     (data-cube/scripts/asko-site/test_ingest_reversibility.py).
    //   * ADDITIVE IN THE EDITOR. This runs on the OPEN scene, reuses anything already there
    //     by name, and touches nothing else — §3f0 v5: the scene is hand-edited (Ivan placed
    //     the Mini himself) and a builder may never regenerate it.
    //   * ONE SURFACE PER PLACE. The parent terrain is holed under the patch's full coverage
    //     and the patch is holed where the survey never reached, so a sonar ray meets exactly
    //     one seabed — or, where neither has data, none, which is the honest answer (§3f0b).
    //
    // Run from menu: SMARC -> Add Asko DV Hi-Res Patch (0.125 m)
    // Prerequisite:  data-cube/scripts/asko-site/build_hires_patch.py
    // =========================================================================
    const string PatchManifest = "asko_dvhires.json";
    const string RockPhysicPath = MatDir + "/Physic/Rock.physicMaterial";

    [MenuItem("SMARC/Add Asko DV Hi-Res Patch (0.125 m)")]
    public static void AddHiResPatch()
    {
        var world = GameObject.Find("AskoCuratedWorld");
        if (world == null)
        {
            Debug.LogError("[Asko] the open scene has no 'AskoCuratedWorld' — open " +
                           "Assets/Scenes/AskoCurated.unity first.");
            return;
        }

        string mp;
        try { mp = PayloadFull(PatchManifest); }
        catch (FileNotFoundException e)
        {
            Debug.LogError("[Asko] " + e.Message + "\n       Run: python3 " +
                           "data-cube/scripts/asko-site/build_hires_patch.py " +
                           "--survey-root \"<Askölaboratoriet>\"");
            return;
        }
        var p = JsonUtility.FromJson<PatchJ>(File.ReadAllText(mp));
        if (p == null || p.resolution == 0 || p.unity_position == null)
            throw new Exception(PatchManifest + " did not parse into a patch manifest");
        float cell = p.terrain_size_x_m / (p.resolution - 1);
        Debug.Log($"[Asko/patch] {p.name}: {p.resolution}x{p.resolution} over " +
                  $"{p.terrain_size_x_m} m = {cell:F6} m/cell, y {p.terrain_base_y_m:F1}.." +
                  $"{p.terrain_base_y_m + p.terrain_size_y_m:F1} m, heights " +
                  $"{p.statistics.min_m:F2}..{p.statistics.max_m:F2} m\n" +
                  $"             generated {p.generated_utc} by {p.generator}");

        // --- TerrainData -----------------------------------------------------
        int res = p.resolution;
        var bytes = File.ReadAllBytes(PayloadFull(p.heightmap));
        long expected = (long)res * res * 2;
        if (bytes.LongLength != expected)
            throw new Exception($"patch heightmap is {bytes.LongLength} bytes, expected " +
                                $"{expected} for res {res}");
        var heights = new float[res, res];
        int k = 0;
        for (int z = 0; z < res; z++)
            for (int x = 0; x < res; x++, k += 2)
                heights[z, x] = (bytes[k] | (bytes[k + 1] << 8)) / 65535f;

        string assetPath = DataDir + "/AskoCurated_" + p.name + ".asset";
        var td = AssetDatabase.LoadAssetAtPath<TerrainData>(assetPath);
        bool isNew = td == null;
        if (isNew) td = new TerrainData();
        td.name = "AskoCurated_" + p.name;
        td.heightmapResolution = res;
        td.size = new Vector3(p.terrain_size_x_m, p.terrain_size_y_m, p.terrain_size_z_m);
        td.SetHeights(0, 0, heights);
        if (isNew)
        {
            var dir = Path.GetDirectoryName(AssetPathToFull(assetPath));
            if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
            AssetDatabase.CreateAsset(td, assetPath);
        }
        else EditorUtility.SetDirty(td);
        ApplyPatchSplat(td, p);

        // --- the GameObject, reused by name ----------------------------------
        string goName = "AskoCurated_" + p.name;
        var go = GameObject.Find(goName);
        if (go == null)
        {
            go = Terrain.CreateTerrainGameObject(td);
            go.name = goName;
            go.transform.SetParent(world.transform, false);
        }
        else
        {
            go.GetComponent<Terrain>().terrainData = td;
            var tc0 = go.GetComponent<TerrainCollider>();
            if (tc0 != null) tc0.terrainData = td;
        }
        go.transform.localPosition = new Vector3(
            p.unity_position.x, p.unity_position.y, p.unity_position.z);
        go.transform.localScale = Vector3.one;      // never scale a georeferenced terrain
        var terr = go.GetComponent<Terrain>();
        // A 0.125 m terrain carries 64x the vertices of a 1 m one over the same ground, so
        // it is the one place in this scene where the LOD settings matter.
        terr.heightmapPixelError = 1f;
        terr.basemapDistance = 512f;
        var mat = AssetDatabase.LoadAssetAtPath<Material>(MatTerrain);
        if (mat != null) terr.materialTemplate = mat;
        var tc = go.GetComponent<TerrainCollider>();
        var rock = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(RockPhysicPath)
                   ?? AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(MudPhysicPath);
        if (tc != null && rock != null) tc.sharedMaterial = rock;

        // --- holes on the PATCH: cells the survey never reached ---------------
        var pl = go.GetComponent<AskoGapPatchLayer>() ?? go.AddComponent<AskoGapPatchLayer>();
        pl.maskAssetPath = "";                  // nothing synthetic here to toggle
        pl.showSyntheticPatches = true;
        pl.maskResolution = p.mask_resolution;
        pl.alwaysHoleMaskAssetPath = DataDir + "/" + p.hole_mask;
        pl.provenance =
            "Deep Vision MBES 2024-06-04 gridded at 0.125 m, MEDIAN per cell. Cells with no " +
            "sounding within 1 m are HOLES: the 1 m base shows through, and a sonar ray gets " +
            "the base's answer rather than an interpolated invention. Per-cell attribution " +
            "in " + p.source_mask + ".";
        pl.ApplyNow();

        // --- bottom type, re-derived at 0.125 m -------------------------------
        var sm = LoadSplatManifest();
        if (sm != null && sm.acoustics != null)
        {
            var acou = go.GetComponent<SeabedAcousticMap>() ?? go.AddComponent<SeabedAcousticMap>();
            acou.classMapAssetPath = DataDir + "/" + p.seabed_class_map;
            acou.resolution = res;
            acou.classes = new[]
            {
                MakeClass("land", sm.acoustics.land),
                MakeClass("bedrock", sm.acoustics.bedrock),
                MakeClass("sand", sm.acoustics.sand),
                MakeClass("clay", sm.acoustics.clay),
            };
            acou.provenance =
                "Bottom type INFERRED from slope, roughness and depth at 0.125 m, with the " +
                "SAME thresholds build_splatmap.py uses at 1 m. Bedrock rises from 34.9 % to " +
                "46.7 % over this footprint: the rules did not change, the cell size did — a " +
                "1 m grid averages a boulder into gentle ground. Still inferred, not surveyed.";
        }

        // --- cut the parent terrain out from under the patch -------------------
        var site = LoadSite();
        foreach (var t in site.tiles)
        {
            if (string.IsNullOrEmpty(t.hires_cutout_mask)) continue;
            var ptGo = GameObject.Find("AskoCurated_" + t.name);
            if (ptGo == null)
            {
                Debug.LogWarning($"[Asko/patch] parent terrain AskoCurated_{t.name} is not in " +
                                 "the open scene — its cutout was NOT applied, so the 1 m " +
                                 "base still draws under the patch.");
                continue;
            }
            var ppl = ptGo.GetComponent<AskoGapPatchLayer>()
                      ?? ptGo.AddComponent<AskoGapPatchLayer>();
            ppl.alwaysHoleMaskAssetPath = DataDir + "/" + t.hires_cutout_mask;
            ppl.ApplyNow();
            Debug.Log($"[Asko/patch] AskoCurated_{t.name}: cutout applied, " +
                      $"{ppl.alwaysHoleFraction * 100f:F3} % of the tile is now a hole under " +
                      "the hi-res patch.");

            // and give it the seam-collared surface, without a full site rebuild
            if (string.IsNullOrEmpty(t.heightmap_collared)) continue;
            try
            {
                var cb = File.ReadAllBytes(PayloadFull(t.heightmap_collared));
                int pr = t.resolution;
                if (cb.LongLength != (long)pr * pr * 2)
                {
                    Debug.LogError($"[Asko/patch] {t.heightmap_collared} is {cb.LongLength} " +
                                   $"bytes, expected {(long)pr * pr * 2} — not applied.");
                    continue;
                }
                var ph = new float[pr, pr];
                int j = 0;
                for (int z = 0; z < pr; z++)
                    for (int x = 0; x < pr; x++, j += 2)
                        ph[z, x] = (cb[j] | (cb[j + 1] << 8)) / 65535f;
                var ptd = ptGo.GetComponent<Terrain>().terrainData;
                ptd.SetHeights(0, 0, ph);
                EditorUtility.SetDirty(ptd);
                Debug.Log($"[Asko/patch] AskoCurated_{t.name}: seam-collared heightmap " +
                          $"applied ({t.heightmap_collared}). The un-collared {t.heightmap} " +
                          "is untouched on disk and the correction is kept as its own raster.");
            }
            catch (FileNotFoundException e)
            {
                Debug.LogWarning("[Asko/patch] collared base not applied: " + e.Message);
            }
        }

        // The patch is NESTED, not adjacent, so it is deliberately NOT wired into
        // SetNeighbors: telling Unity a 512 m terrain neighbours a 4096 m one it sits inside
        // would stitch LOD across a seam that does not exist.
        EditorSceneManager.MarkAllScenesDirty();
        EditorSceneManager.SaveOpenScenes();
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log($"[Asko/patch] DONE. {goName} at local " +
                  $"({p.unity_position.x:F3}, {p.unity_position.y:F1}, " +
                  $"{p.unity_position.z:F3}), {cell:F3} m cells. Scene saved.");
    }

    /// <summary>
    /// Splat for the patch: Bedrock, Sand, Clay, SyntheticMark — and NO orthophoto layer.
    /// The patch is 100 % submerged (its shallowest cell is -1.45 m), so draping the 5 cm
    /// aerial photo over it would be painting the water surface onto the seabed.
    /// </summary>
    static void ApplyPatchSplat(TerrainData td, PatchJ p)
    {
        var sm = LoadSplatManifest();
        if (sm == null || sm.layers == null)
        {
            Debug.LogWarning("[Asko/patch] no asko_splat.json — patch left untextured. " +
                             "Run build_splatmap.py.");
            return;
        }
        var use = new List<TerrainLayer>();
        foreach (var L in sm.layers)
        {
            if (L.texture == "PER_TILE") continue;             // the orthophoto: skipped
            var layerPath = DataDir + "/Asko_" + L.name + ".terrainlayer";
            var layer = AssetDatabase.LoadAssetAtPath<TerrainLayer>(layerPath);
            if (layer == null)
            {
                layer = new TerrainLayer();
                AssetDatabase.CreateAsset(layer, layerPath);
                layer.diffuseTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(
                    DataDir + "/" + L.texture);
                layer.tileSize = new Vector2(sm.tile_size_m, sm.tile_size_m);
                EditorUtility.SetDirty(layer);
            }
            use.Add(layer);
        }
        td.terrainLayers = use.ToArray();

        string ctrlPath;
        try { ctrlPath = PayloadFull(p.control_map); }
        catch (FileNotFoundException e)
        { Debug.LogWarning("[Asko/patch] control map missing: " + e.Message); return; }
        var raw = File.ReadAllBytes(ctrlPath);
        int n = (int)Math.Round(Math.Sqrt(raw.LongLength / (double)use.Count));
        long expect = (long)n * n * use.Count;
        if (expect != raw.LongLength)
        {
            Debug.LogError($"[Asko/patch] control map is {raw.LongLength} bytes, which is not " +
                           $"n x n x {use.Count} for any n — regenerate it.");
            return;
        }
        td.alphamapResolution = n;
        var maps = new float[n, n, use.Count];
        int k = 0;
        for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
                for (int L = 0; L < use.Count; L++, k++)
                    maps[y, x, L] = raw[k] / 255f;
        td.SetAlphamaps(0, 0, maps);
        Debug.Log($"[Asko/patch] splat: {use.Count} layers (no ortho), control {n}x{n}");
    }

    /// <summary>
    /// ADDITIVE population of the OPEN scene with the parts the Beckholmen scene carries:
    /// vehicle, HUD, station, sonar map, SSS waterfall, WP hoop. Never deletes or replaces
    /// anything already there; existing objects are reused by name. Safe to run repeatedly.
    /// </summary>
    [MenuItem("SMARC/Populate Asko Scene (vehicle + GUI + mission set)")]
    public static void PopulateScene()
    {
        var world = GameObject.Find("AskoCuratedWorld");
        if (world == null)
        {
            Debug.LogError("[Asko] the open scene has no 'AskoCuratedWorld' — open " +
                           "Assets/Scenes/AskoCurated.unity first (or run Build Asko Curated " +
                           "Site to create it).");
            return;
        }

        // --- vehicle: reuse Ivan's instance if present, else instantiate ------------
        var sam = GameObject.Find(VehicleName);
        float seabedLaunch = TerrainHeightAt(world, LaunchXZ.x, LaunchXZ.z);
        var launchPos = new Vector3(LaunchXZ.x, -0.1f, LaunchXZ.z);
        if (sam == null)
        {
            sam = Place(VehiclePrefab, VehicleName, launchPos, yawDeg: 262f);
            Debug.Log($"[Asko] {VehicleName} instantiated at the launch " +
                      $"({LaunchXZ.x}, -0.1, {LaunchXZ.z}), seabed below {seabedLaunch:F2} m");
        }
        else
        {
            // An existing SAM parked on land (e.g. carried-over Beckholmen coordinates)
            // is moved to the launch and the move is SAID; a SAM already in the water is
            // left where the operator put it.
            float ground = TerrainHeightAt(world, sam.transform.position.x, sam.transform.position.z);
            if (!float.IsNaN(ground) && ground > 0f)
            {
                Debug.LogWarning($"[Asko] existing {VehicleName} was at " +
                                 $"{sam.transform.position} — ON LAND (terrain {ground:+0.00} m). " +
                                 $"Moved to the launch ({launchPos}), heading down the line.");
                sam.transform.position = launchPos;
                sam.transform.rotation = Quaternion.Euler(0, 262f, 0);
            }
            else
            {
                Debug.Log($"[Asko] existing {VehicleName} kept at {sam.transform.position}");
            }
        }

        // --- the car: IVAN'S PLACEMENT IS AUTHORITATIVE ------------------------------
        var car = GameObject.Find("MMTMiniCooper");
        if (car != null)
        {
            float seabed = TerrainHeightAt(world, car.transform.position.x, car.transform.position.z);
            Debug.Log($"[Asko] MMTMiniCooper kept at {car.transform.position} " +
                      $"(seabed there {seabed:F2} m) — Ivan's placement; the mission line " +
                      "runs over it (wp1).");
        }
        else
        {
            float seabed = TerrainHeightAt(world, -171.8f, -155.4f);
            Place(MiniPrefab, "MMTMiniCooper",
                  new Vector3(-171.8f, float.IsNaN(seabed) ? -8f : seabed + 0.02f, -155.4f),
                  yawDeg: 145f);
            Debug.Log("[Asko] MMTMiniCooper placed at (-171.8, seabed, -155.4) — the position " +
                      "Ivan chose on 2026-08-27, on multibeam bathymetry.");
        }

        // --- the DETECTED FEATURE: the pipeline (added 2026-08-30, Phase 7) ----------
        // Idempotent and additive like everything else here. It is silently skipped when the
        // prefab has not been built, because the pipeline is a SEPARABLE derived feature: a
        // scene without it is a valid scene, and that is the whole point of registering it in
        // `features[]` rather than baking it into the terrain.
        AskoPipelineBuilder.Ensure(world);

        // --- HUD, station, sonar map, waterfall, hoop — added only if missing --------
        if (GameObject.Find("GUI") == null)
            Place(GuiPrefab, "GUI", launchPos + new Vector3(0, 10f, 0));
        if (GameObject.Find("datacube_station_01") == null)
        {
            float shoreH = TerrainHeightAt(world, StationXZ.x, StationXZ.z);
            Place(StationPrefab, "datacube_station_01",
                  new Vector3(StationXZ.x, float.IsNaN(shoreH) ? 1.3f : shoreH + 0.05f, StationXZ.z),
                  yawDeg: 250f);
        }
        if (GameObject.Find("SonarMapAccumulator") == null)
        {
            var acc = new GameObject("SonarMapAccumulator");
            acc.AddComponent<Visualizers.SonarMapAccumulator>().RobotName = VehicleName;
        }
        if (sam != null && sam.GetComponent<Visualizers.SSSWaterfallHUD>() == null)
        {
            var wf = sam.AddComponent<Visualizers.SSSWaterfallHUD>();
            wf.vehicleRoot = sam.transform;
        }
        MissionWPHoopSetup.Ensure(VehicleName);      // idempotent, reads GLOBALREF

        EditorSceneManager.MarkAllScenesDirty();
        EditorSceneManager.SaveOpenScenes();
        Debug.Log("[Asko] populate DONE and scene saved. SSS waterfall: F6 or the " +
                  "bottom-right button, in Play mode. Mission: " +
                  "data-cube/scripts/seed_asko_bay_mission.py seeds the bay line into MC " +
                  "(launch -> over the car -> sand -> clay).");
    }
}
