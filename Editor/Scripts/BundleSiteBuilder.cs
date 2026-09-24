using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

using UnityEngine.Rendering.HighDefinition;   // WaterSurface: the Ocean's swell/wind fields
using Force;
using GeoRef;
using Smarc.Environment;

/// <summary>
/// The Unity end of the OCEANVERSE -> SMaRC Unity bridge (ADR-013). Sites are created and
/// curated in OCEANVERSE (simulator-agnostic site package in <bundle>/site/); the bridge
/// `python3 -m bridges.smarc_unity.export` writes <bundle>/engines/smarc_unity/, and this
/// builder reads ONLY that folder — no per-site C#. Its unity_build.json carries:
///
///   georef       the site frame: anchor, derived UTM zone/band/EPSG, grid convergence,
///                and the bridge's axis mapping (Unity X = east, Z = grid north, Y = up)
///   everything   prefixes, engine-cache dir, scene/prefab names, expected zone/band,
///   else         vehicle prefabs, gap-patch semantics
///
/// and this builder turns that into a <see cref="CuratedSiteConfig"/> and hands it to
/// <see cref="CuratedSiteBuilder"/> — the same code path Askö and Djurö use. So the
/// GLOBALREF is created from the bundle's own anchor and CHECKED against the bundle's own
/// zone AND band (CoordinateSharp vs pyproj, two independent implementations). Nothing about
/// the frame is typed in, inherited or guessed: a Monterey bundle gets UTM 10S because its
/// anchor is at 121.9 W, not because anyone configured it.
///
/// Then it populates the scene additively (never destroys): charted hazards as sphere
/// colliders with the Rock physics material, SAM 2.1 at the MEASURED launch point with the
/// measured heading, the GUI — and the ENVIRONMENT: the site package's sea state at one point
/// in time (site/environment.json, mapped by the bridge into <prefix>_environment.json): the
/// Ocean's WaterSurface gets the real wind and swell direction, a CurrentField carries the
/// measured surface current (OFF by default — a scenario knob, see CurrentField.cs), and the
/// vertical reference is Unity y = 0 = the sea surface at that time (the bridge placed the
/// terrain sea_level lower than MSL; the Ocean transform stays at the origin — SETTLED).
///
/// Menu: SMARC -> Build Curated Site from Bundle...   (pick scenario-bundle.manifest.json)
///       SMARC -> Rebuild Last Curated Bundle          (same bundle again, no dialog)
/// </summary>
public static class BundleSiteBuilder
{
    const string LastKey = "SMARC.BundleSiteBuilder.LastManifest";
    const string GuiPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/SmarcGUI/GUI.prefab";

    // Ocean / Sun / Sky are cloned from an existing georeferenced world prefab; the builder
    // DROPS every other child (including that prefab's own GLOBALREF), so which one is used
    // only decides the look of the water, never the frame.
    static readonly string[] EnvironmentPrefabs =
    {
        CuratedSiteBuilder.PrefabDir + "/DjuroWorld.prefab",
        CuratedSiteBuilder.PrefabDir + "/AskoWorld.prefab",
    };

    // ---- manifest mirror (JsonUtility ignores everything not declared here) ----------
    [Serializable]
    public class UnityBuildJ
    {
        public string builder, prefix, asset_prefix, layer_prefix, tag, data_dir;
        public string bundle_payload_rel_to_unity_project, scene_path, world_prefab_name;
        public string global_ref_name, expected_utm_band, vehicle_name, terrain_physic_material;
        public string gap_patch_semantics;
        public int expected_utm_zone;
        public string[] vehicle_prefabs;
        public string environment_file, environment_summary;     // <prefix>_environment.json, or empty
        public string structures_file;                           // <prefix>_structures.json, or empty
        public string live_traffic_file;                         // <bundle>/live/traffic.json (ovsite.traffic live), absolute
        public string replay_traffic_file;                       // <bundle>/live/replay.json (ovsite.traffic replay), absolute
        public ScenarioClockJ scenario_clock;                    // the scene's ONE clock (OCEANVERSE time.json)
        public WaterMaskJ water_mask;                            // swell SHELTER mask for the Ocean, or null
        public DryBasinsRefJ dry_basins;                         // land below the sea level the sea cannot reach, or null
    }
    // ---- dry basins (bridges.smarc_unity.dry_basins): an HDRP water-exclusion sheet ------------------
    [Serializable] public class DryBasinsRefJ { public string file, rule; public float level_msl_m, basin_km2; public int rects; }
    [Serializable] public class DryBasinsJ { public float level_msl_m, excluder_unity_y, basin_km2; public string rule; public float[] vertices; public int[] triangles; }
    // ---- the swell shelter mask (bridges.smarc_unity.shelter_mask): HDRP WaterSurface.waterMask ----
    [Serializable] public class WaterMaskJ { public string file; public float[] extent_m, offset_m; public float hs_ref_m; }
    // ---- <prefix>_structures.json (bridges.smarc_unity.structures_mesh) — SI metres, Unity coords --
    [Serializable] public class StructGroupJ { public string render_material, physics_material; public float[] vertices; public int[] triangles; }
    [Serializable] public class StructTileJ { public string tile; public StructGroupJ[] groups; }
    [Serializable] public class StructFeatJ { public string source_id, kind, tile, source, height_source, material; public int synthetic_piles; public bool in_terrain; }
    [Serializable] public class StructuresJ { public StructTileJ[] tiles; public StructFeatJ[] features; public string[] notes; public int heights_assumed, synthetic_piles; }
    // ---- <prefix>_environment.json (bridges.smarc_unity.export.unity_environment) --------
    // Sentinels: JsonUtility leaves a field the file does not carry at its default.
    // SI throughout: m, s, m/s, compass degrees. Sentinels: JsonUtility leaves absent fields at the default.
    [Serializable] public class EnvMeasuredJ
    {
        public float sea_level_msl_m, wave_hs_m = float.NaN, wave_tp_s = float.NaN, wave_from_deg = float.NaN,
                     wind_speed_ms = float.NaN, wind_from_deg = float.NaN, current_speed_ms = float.NaN, current_to_deg = float.NaN;
        public string sea_level_source, waves_source, wind_source, current_source;
    }
    [Serializable] public class EnvWaterJ { public float swell_wind_ms = -1, ripple_wind_ms = -1, swell_toward_deg = -999; }
    [Serializable] public class EnvVecJ { public float x, y, z; }
    [Serializable] public class EnvVolumeJ { public EnvVecJ center, size; }
    [Serializable] public class EnvCurrentJ { public bool CurrentEnabled; public float HeadingDeg = -999, SpeedMS = -1; public string source, time_utc, note; public float distance_km; public EnvVolumeJ volume; }
    [Serializable] public class EnvWaterPlaneJ { public float unity_y, sea_level_msl_m; public string rule; }
    [Serializable] public class ScenarioClockJ { public string time_file, mode, start_utc, end_utc, rule; public double lat, lon; public float grid_convergence_deg; }
    [Serializable] public class EnvTimelineJ
    {
        public float[] cloud_cover_pct, visibility_m, precipitation_mm_h, air_temperature_c, sun_elevation_deg, sun_azimuth_deg;
        public string kind_weather, kind_air_temperature;
        public string start_utc, end_utc, generated_utc, kind_sea_level, kind_waves, kind_wind, kind_current, kind_legend, gaps;
        public float step_s, build_sea_level_msl_m; public int samples; public string[] epochs, sources;
        public float[] t_s, sea_level_msl_m, tide_dy_m, swell_wind_ms, ripple_wind_ms, swell_toward_deg,
                       wave_hs_m, wave_tp_s, wave_from_deg, wind_speed_ms, wind_from_deg, current_speed_ms, current_to_deg;
    }
    [Serializable] public class EnvironmentJ { public string units, applied_from, at_utc, summary; public EnvWaterPlaneJ water_plane; public EnvMeasuredJ measured; public EnvWaterJ water_surface; public EnvCurrentJ current_field; public EnvTimelineJ timeline; }
    [Serializable] public class UtmJ { public string epsg, band, hemisphere; public int zone; public double easting, northing; }
    [Serializable] public class GeoAnchorJ { public double lat, lon; public string origin_confidence; }
    [Serializable] public class GeoRefJ { public GeoAnchorJ anchor; public UtmJ utm; public string unity_axes; public double grid_convergence_deg; }
    [Serializable] public class ManifestJ { public string bundle_id; }
    [Serializable] public class RegionJ { public float tile_m; public int n_cols, n_rows, total; public string[] curated; }
    [Serializable] public class StreamingJ { public float active_radius_m; public string note; }
    [Serializable]
    public class UnityBuildFileJ : UnityBuildJ
    {
        public string bridge, bridge_version;
        public GeoRefJ georef;
        public RegionJ region;          // null for a single site
        public StreamingJ streaming;
    }

    [Serializable]
    public class HazardJ
    {
        public double lon, lat;
        public float unity_x, unity_z, top_z, radius_m, heightmap_top_z;
        public bool absorbed_by_seabed;
        public string kind, lnam, cell, top_rule, physics_material;
    }
    [Serializable] public class HazardsJ { public int count; public HazardJ[] rocks; }

    // ------------------------------------------------------------------------------------
    [MenuItem("SMARC/Build Curated Site from Bundle...")]
    public static void BuildFromBundleMenu()
    {
        var projectRoot = Path.GetDirectoryName(Application.dataPath);
        var start = Path.GetFullPath(Path.Combine(projectRoot, "..", "smds-cloud-store", "scenario-bundles"));
        var path = EditorUtility.OpenFilePanel("Pick the bundle's scenario-bundle.manifest.json",
                                               Directory.Exists(start) ? start : projectRoot, "json");
        if (string.IsNullOrEmpty(path)) return;
        EditorPrefs.SetString(LastKey, path);
        BuildFromManifest(path);
    }

    [MenuItem("SMARC/Rebuild Last Curated Bundle")]
    public static void RebuildLast()
    {
        var p = EditorPrefs.GetString(LastKey, "");
        if (string.IsNullOrEmpty(p) || !File.Exists(p))
        {
            Debug.LogWarning("[Bundle] no previous bundle — use SMARC -> Build Curated Site from Bundle...");
            return;
        }
        BuildFromManifest(p);
    }

    public static void BuildFromManifest(string manifestPath)
    {
        var man = JsonUtility.FromJson<ManifestJ>(File.ReadAllText(manifestPath));
        var bundleDir = Path.GetDirectoryName(manifestPath);
        var engineDir = Path.Combine(bundleDir, "engines", "smarc_unity");
        var ubPath = Path.Combine(engineDir, "unity_build.json");
        if (!File.Exists(ubPath))
        {
            Debug.LogError($"[Bundle] {man?.bundle_id}: no engines/smarc_unity/unity_build.json. Sites are " +
                           "curated in OCEANVERSE and reach Unity through the smarc_unity bridge — run it first:\n" +
                           $"    cd \"<workspace>/oceanverse\" && python3 -m bridges.smarc_unity.export --bundle \"{bundleDir}\"\n" +
                           "(or 'export smarc_unity' in the OCEANVERSE Site pipeline console).");
            return;
        }
        var ub = JsonUtility.FromJson<UnityBuildFileJ>(File.ReadAllText(ubPath));
        if (ub == null || string.IsNullOrEmpty(ub.prefix) || ub.georef == null || ub.georef.anchor == null || ub.georef.utm == null)
        {
            Debug.LogError($"[Bundle] {ubPath} is incomplete (prefix or georef missing) — re-run the bridge; " +
                           "refusing to build a scene whose frame would have to be guessed.");
            return;
        }
        var tag = string.IsNullOrEmpty(ub.tag) ? "[" + ub.prefix + "]" : ub.tag;
        var conf = Path.Combine(engineDir, "conformance.json");
        if (File.Exists(conf) && File.ReadAllText(conf).Contains("\"result\": \"FAIL\""))
            Debug.LogWarning($"{tag} the bridge's conformance.json says FAIL — this export does not reproduce " +
                             "the OCEANVERSE site. Building anyway so you can look, but do not fly it.");

        // The payload is the bridge's engine folder beside the picked manifest (not wherever the
        // exporting machine was): a bundle copied to another disk still builds.
        var projectRoot = Path.GetDirectoryName(Application.dataPath);
        var payloadFull = engineDir;
        var payloadRel = Path.GetRelativePath(projectRoot, payloadFull).Replace('\\', '/');

        SyncEngineCache(tag, ub, payloadFull);

        string env = null;
        foreach (var p in EnvironmentPrefabs)
            if (AssetDatabase.LoadAssetAtPath<GameObject>(p) != null) { env = p; break; }
        if (env == null)
        {
            Debug.LogError($"{tag} none of the environment prefabs exist: {string.Join(", ", EnvironmentPrefabs)}");
            return;
        }

        var cfg = new CuratedSiteConfig
        {
            prefix = ub.prefix,
            assetPrefix = ub.asset_prefix,
            layerPrefix = ub.layer_prefix,
            tag = tag,
            dataDir = ub.data_dir,
            bundleRel = payloadRel,
            sourceWorldPrefab = env,
            worldPrefabPath = CuratedSiteBuilder.PrefabDir + "/" + ub.world_prefab_name + ".prefab",
            worldObjectName = ub.world_prefab_name,
            scenePath = ub.scene_path,
            globalRefName = ub.global_ref_name,
            expectedUtmZone = ub.expected_utm_zone,
            legacyOriginUtm = null,
            gapPatchSemantics = ub.gap_patch_semantics,
            wireNeighbours = false,
            terrainPhysicMaterial = string.IsNullOrEmpty(ub.terrain_physic_material) ? "Mud" : ub.terrain_physic_material,
            sceneExistsHint = "Hazards, vehicle and GUI are (re)placed additively next. ",
            sceneCreatedHint = "\n       hazards, vehicle and GUI are added next.",
        };

        var g = ub.georef;
        Debug.Log($"{tag} bundle {man.bundle_id} (bridge {ub.bridge} v{ub.bridge_version})\n       frame from the bundle: anchor {g.anchor.lat:F6}, " +
                  $"{g.anchor.lon:F6} ({g.anchor.origin_confidence}) -> {g.utm.epsg} zone {g.utm.zone}{g.utm.band} " +
                  $"E {g.utm.easting:F2} N {g.utm.northing:F2}; grid convergence {g.grid_convergence_deg:+0.000;-0.000} deg\n" +
                  $"       {g.unity_axes}");

        CuratedSiteBuilder.Build(cfg);
        if (!File.Exists(CuratedSiteBuilder.AssetPathToFull(cfg.scenePath))) return;
        if (EditorSceneManager.GetActiveScene().path != cfg.scenePath)
            EditorSceneManager.OpenScene(cfg.scenePath, OpenSceneMode.Single);

        var world = GameObject.Find(cfg.worldObjectName);
        if (world == null)
        {
            Debug.LogError($"{tag} the scene has no '{cfg.worldObjectName}' after the build.");
            return;
        }
        CheckGlobalRef(tag, cfg, g);
        var site = CuratedSiteBuilder.LoadSite(cfg);
        ResetTerrainsAsBuilt(tag, cfg, world, site);          // BEFORE anything is placed relative to a tile
        if (ub.region != null && ub.region.n_cols > 0) WireRegion(tag, ub, world);
        PlaceHazards(tag, cfg, world, site);
        PlaceVehicle(tag, cfg, ub, world, site);
        ApplyEnvironment(tag, cfg, ub, world);
        PlaceStructures(tag, cfg, ub, world);
        PlaceDryBasins(tag, cfg, ub, world);
        var clock = SetupClock(tag, cfg, ub, world);
        SetupTimeline(tag, cfg, ub, world, clock, site);
        SetupSun(tag, world, clock);
        PlaceLiveTraffic(tag, cfg, ub, world, clock);

        EditorSceneManager.MarkAllScenesDirty();
        EditorSceneManager.SaveOpenScenes();
        AssetDatabase.SaveAssets();
        Debug.Log($"{tag} populate DONE and scene saved: {cfg.scenePath}");
    }

    /// Runtime files the engine needs from the package dir (textures by GUID, class maps,
    /// synthetic mask). build.py copies them already; this makes a bundle built on ANOTHER
    /// machine buildable too. Bundle -> cache only, never back.
    static void SyncEngineCache(string tag, UnityBuildJ ub, string payloadFull)
    {
        var dst = CuratedSiteBuilder.AssetPathToFull(ub.data_dir + "/x");
        dst = Path.GetDirectoryName(dst);
        Directory.CreateDirectory(dst);
        int n = 0;
        foreach (var src in Directory.GetFiles(payloadFull, ub.prefix + "_*"))
        {
            var name = Path.GetFileName(src);
            // build-time-only files stay in the bundle: the heightmap, the source mask, the
            // splat control map (all read through PayloadFull, bundle first)
            if (name.EndsWith("_heightmap.r16") || name.EndsWith("_source.u8") || name.EndsWith("_splat.u8"))
                continue;
            var d = Path.Combine(dst, name);
            if (File.Exists(d) && File.GetLastWriteTimeUtc(d) >= File.GetLastWriteTimeUtc(src) &&
                new FileInfo(d).Length == new FileInfo(src).Length) continue;
            File.Copy(src, d, true);
            n++;
        }
        if (n > 0)
        {
            AssetDatabase.Refresh();
            Debug.Log($"{tag} engine cache: {n} file(s) refreshed at {dst}");
        }
    }

    /// REGION tiles (names r##c##): stitch neighbours so Unity's LOD meets at the seams, and add
    /// the streamer that keeps only tiles near the vehicle active. Tiles that were not curated
    /// are simply absent — the region is populated on a need-to-be basis.
    static void WireRegion(string tag, UnityBuildJ ub0, GameObject world)
    {
        var ub = ub0 as UnityBuildFileJ;
        var byName = new Dictionary<string, Terrain>();
        foreach (var t in world.GetComponentsInChildren<Terrain>(true))
        {
            var i = t.name.LastIndexOf('_');
            if (i >= 0 && i + 1 < t.name.Length) byName[t.name.Substring(i + 1)] = t;
        }
        int wired = 0;
        foreach (var kv in byName)
        {
            if (kv.Key.Length != 6 || kv.Key[0] != 'r' || kv.Key[3] != 'c') continue;
            int r = int.Parse(kv.Key.Substring(1, 2)), c = int.Parse(kv.Key.Substring(4, 2));
            byName.TryGetValue($"r{r:00}c{c - 1:00}", out var left);
            byName.TryGetValue($"r{r:00}c{c + 1:00}", out var right);
            byName.TryGetValue($"r{r + 1:00}c{c:00}", out var top);      // +row = +Z = north
            byName.TryGetValue($"r{r - 1:00}c{c:00}", out var bottom);
            kv.Value.SetNeighbors(left, top, right, bottom);
            if (left != null || right != null || top != null || bottom != null) wired++;
        }
        var streamer = world.GetComponent<TerrainStreamer>();
        if (streamer == null) streamer = world.AddComponent<TerrainStreamer>();
        streamer.trackedName = string.IsNullOrEmpty(ub.vehicle_name) ? "sam21" : ub.vehicle_name;
        if (ub?.streaming != null && ub.streaming.active_radius_m > 0) streamer.activeRadius = ub.streaming.active_radius_m;
        Debug.Log($"{tag} REGION: {byName.Count} curated tile(s) of {ub?.region?.total ?? 0} in the grid " +
                  $"({ub?.region?.n_cols}x{ub?.region?.n_rows} of {ub?.region?.tile_m:F0} m); {wired} with a neighbour " +
                  $"stitched; TerrainStreamer keeps tiles within {streamer.activeRadius:F0} m of {streamer.trackedName} active.");
    }

    /// Two independent geodesy implementations must agree on the zone AND the band: the
    /// band is what CuratedSiteBuilder does not check, and a band slip at a zone boundary
    /// latitude is ~900 km of northing.
    static void CheckGlobalRef(string tag, CuratedSiteConfig cfg, GeoRefJ g)
    {
        var go = GameObject.Find(cfg.globalRefName);
        var grp = go != null ? go.GetComponent<GlobalReferencePoint>() : null;
        if (grp == null)
        {
            Debug.LogError($"{tag} no GlobalReferencePoint named '{cfg.globalRefName}' in the scene.");
            return;
        }
        bool ok = grp.UTMZone == g.utm.zone && string.Equals(grp.UTMBand, g.utm.band, StringComparison.OrdinalIgnoreCase);
        double de = grp.UTMEasting - g.utm.easting, dn = grp.UTMNorthing - g.utm.northing;
        if (!ok)
            Debug.LogError($"{tag} GEOREF MISMATCH: Unity's GLOBALREF says UTM {grp.UTMZone}{grp.UTMBand}, " +
                           $"the bundle says {g.utm.zone}{g.utm.band}. Do not fly this scene.");
        else
            Debug.Log($"{tag} GEOREF OK: GLOBALREF {grp.Lat:F6}, {grp.Lon:F6} = UTM {grp.UTMZone}{grp.UTMBand} " +
                      $"(bundle {g.utm.zone}{g.utm.band}), easting/northing agree to ({de:F3}, {dn:F3}) m. " +
                      $"The vehicle's estimator will derive frame utm_{grp.UTMZone}_{grp.UTMBand} from the GPS fix.");
    }

    static void PlaceHazards(string tag, CuratedSiteConfig cfg, GameObject world, CuratedSiteBuilder.SiteJ site)
    {
        if (string.IsNullOrEmpty(site.rocks_file)) return;
        string path;
        try { path = CuratedSiteBuilder.PayloadFull(cfg, site.rocks_file); }
        catch (FileNotFoundException) { Debug.LogWarning($"{tag} no {site.rocks_file} — hazards not placed."); return; }
        var data = JsonUtility.FromJson<HazardsJ>(File.ReadAllText(path));
        if (data == null || data.rocks == null || data.rocks.Length == 0)
        {
            Debug.Log($"{tag} no charted point hazards in this box.");
            return;
        }
        var parentName = cfg.assetPrefix + "Hazards";
        var parent = world.transform.Find(parentName);
        if (parent == null)
        {
            var go = new GameObject(parentName);
            go.transform.SetParent(world.transform, false);
            parent = go.transform;
        }
        // sharedMaterial, never .material: the editor would instantiate unsaved copies and the
        // sonar would read the 0.5 default on every hazard (the Kristineberg trap, SETTLED §3f).
        var rockMat = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(CuratedSiteBuilder.PhysicMaterialPath("Rock"));
        var visual = AssetDatabase.LoadAssetAtPath<Material>(CuratedSiteBuilder.MatTerrain);
        int made = 0, reused = 0, wrecks = 0;
        foreach (var r in data.rocks)
        {
            var name = (string.IsNullOrEmpty(r.kind) ? "Hazard" : char.ToUpper(r.kind[0]) + r.kind.Substring(1)) + "_" +
                       (string.IsNullOrEmpty(r.lnam) ? $"{r.unity_x:F0}_{r.unity_z:F0}" : r.lnam);
            var t = parent.Find(name);
            GameObject go;
            if (t != null) { go = t.gameObject; reused++; }
            else { go = GameObject.CreatePrimitive(PrimitiveType.Sphere); go.name = name; go.transform.SetParent(parent, false); made++; }
            float rad = r.radius_m > 0 ? r.radius_m : 3f;
            go.transform.localPosition = new Vector3(r.unity_x, r.top_z - rad, r.unity_z);
            go.transform.localScale = Vector3.one * rad * 2f;
            var col = go.GetComponent<SphereCollider>();
            if (col != null && rockMat != null) col.sharedMaterial = rockMat;
            var mr = go.GetComponent<MeshRenderer>();
            if (mr != null && visual != null) mr.sharedMaterial = visual;
            if (r.kind == "wreck") wrecks++;
        }
        Debug.Log($"{tag} hazards: {data.rocks.Length} charted point features ({made} created, {reused} reused, " +
                  $"{wrecks} wrecks). Tops from VALSOU (below MLLW, shifted to MSL) where charted, else a " +
                  $"WATLEV convention — see {site.rocks_file}. Physics material Rock.");
    }

    /// The sea at the site at one point in time — from the site package, through the bridge.
    /// Sets what the scene has a consumer for: the Ocean's HDRP WaterSurface (distant wind
    /// speed and swell orientation; ripples from the local wind) and a CurrentField with the
    /// measured surface current, OFF. The water plane stays at y = 0; the bridge already
    /// lowered every terrain and hazard by the sea level, so y = 0 IS the sea surface.
    static void ApplyEnvironment(string tag, CuratedSiteConfig cfg, UnityBuildJ ub, GameObject world)
    {
        if (string.IsNullOrEmpty(ub.environment_file))
        {
            Debug.Log($"{tag} environment: none in this export (the site package has no environment.json) — " +
                      "Unity y = 0 is MSL, the Ocean keeps the source prefab's sea state, no current.");
            return;
        }
        string path;
        try { path = CuratedSiteBuilder.PayloadFull(cfg, ub.environment_file); }
        catch (FileNotFoundException) { Debug.LogWarning($"{tag} environment: {ub.environment_file} missing — re-run the bridge."); return; }
        var e = JsonUtility.FromJson<EnvironmentJ>(File.ReadAllText(path));
        if (e == null) { Debug.LogWarning($"{tag} environment: {ub.environment_file} unreadable."); return; }
        float sl = e.water_plane != null ? e.water_plane.sea_level_msl_m : 0f;
        Debug.Log($"{tag} ENVIRONMENT at {e.at_utc} ({e.applied_from}): {e.summary}\n" +
                  $"       vertical reference: Unity y = 0 is the sea surface at that time; MSL is at y = {-sl:+0.000;-0.000} m " +
                  "(terrain and hazards were placed by the bridge accordingly; the Ocean transform stays at the origin).");

        // --- the Ocean: SiteEnvironment (SI) drives the HDRP WaterSurface ------------------
        var ws = world.GetComponentInChildren<WaterSurface>(true);
        SiteEnvironment se = null;
        if (ws == null)
            Debug.LogWarning($"{tag} environment: no WaterSurface under {world.name} — sea state not applied.");
        else
        {
            se = ws.GetComponent<SiteEnvironment>();
            if (se == null) se = ws.gameObject.AddComponent<SiteEnvironment>();
            Undo.RecordObject(se, "OCEANVERSE environment");
            Undo.RecordObject(ws, "OCEANVERSE environment");
            se.Ocean = ws;
            var m = e.measured ?? new EnvMeasuredJ();
            se.MeasuredAtUtc = e.at_utc;
            se.SeaLevelMsl_m = sl;
            se.WaveHs_m = m.wave_hs_m; se.WaveTp_s = m.wave_tp_s; se.WaveFrom_deg = m.wave_from_deg;
            se.WindSpeed_ms = m.wind_speed_ms; se.WindFrom_deg = m.wind_from_deg;
            se.CurrentSpeed_ms = m.current_speed_ms; se.CurrentToward_deg = m.current_to_deg;
            se.Sources = $"sea level: {m.sea_level_source ?? "absent"}\nwaves: {m.waves_source ?? "absent"}\n" +
                         $"wind: {m.wind_source ?? "absent"}\ncurrent: {m.current_source ?? "absent"}";
            var w = e.water_surface;
            if (w != null)
            {
                if (w.swell_wind_ms >= 0) se.SwellWind_ms = w.swell_wind_ms;
                if (w.ripple_wind_ms >= 0) se.RippleWind_ms = Mathf.Min(w.ripple_wind_ms, 13.9f);
                if (w.swell_toward_deg > -900) { se.SwellToward_deg = w.swell_toward_deg; se.RippleToward_deg = w.swell_toward_deg; }
            }
            // 2026-09-23 (Ivan): drive HDRP to the same statistical sea state as the site, not to a wind.
            // With a measured Hs and Tp the Ocean is SOLVED for them (HdrpSeaStateModel); without, the wind rules.
            bool haveWaves = !float.IsNaN(se.WaveHs_m) && !float.IsNaN(se.WaveTp_s) && se.WaveTp_s > 0f;
            se.Drive = haveWaves ? SiteEnvironment.OceanDrive.SeaState : SiteEnvironment.OceanDrive.Wind;
            se.UseMeasuredWaves = true;
            // physics = pixels: the query must see every band that is rendered (HDRPWaterQueryModel enforces it at runtime too)
            ws.scriptInteractions = true;
            if (ws.ripples) ws.cpuEvaluateRipples = true;
            ApplyWaterMask(tag, ub, ws);
            se.Apply();
            Debug.Log($"{tag} Ocean drive {se.Drive}: {se.SeaStateNote.Replace("\n", " | ")}");
            // 2026-09-23: the site's water density has ONE owner, SiteWater on the Ocean (VBS, buoyancy and
            // hydrodynamics all read it). The Ocean is cloned from another world prefab, so its SiteWater is
            // THAT site's water — say so loudly instead of silently flying Pacific water at Baltic density.
            var sw = ws.GetComponent<SiteWater>();
            if (sw == null) { sw = ws.gameObject.AddComponent<SiteWater>(); sw.Density_kgm3 = 1025f; sw.Source = "ASSUMED open-ocean class by BundleSiteBuilder — set the site's value"; }
            Debug.LogWarning($"{tag} SiteWater on {ws.name}: {sw.Density_kgm3:F0} kg/m3 ({sw.Source}). It came with the cloned Ocean — " +
                             "confirm it is THIS site's water (fresh 997, Baltic ~1005, ocean ~1025) on the Ocean > Site Water component.");
            EditorUtility.SetDirty(sw);
            EditorUtility.SetDirty(se);
            EditorUtility.SetDirty(ws);
            Debug.Log($"{tag} SiteEnvironment (SI) on {ws.name}: swell wind {se.SwellWind_ms:F2} m/s toward compass {se.SwellToward_deg:F0} deg, " +
                      $"ripple wind {se.RippleWind_ms:F2} m/s. Measured: Hs {se.WaveHs_m:F2} m, Tp {se.WaveTp_s:F1} s from {se.WaveFrom_deg:F0} deg; " +
                      $"wind {se.WindSpeed_ms:F2} m/s from {se.WindFrom_deg:F0} deg.\n" +
                      $"       Edit the sea state on {ws.name} > Site Environment (SI), not on Water Surface (HDRP keeps km/h internally). " +
                      $"Check in the Scene view that the swell travels TOWARD compass {se.SwellToward_deg:F0} deg (the orientation convention is assumed).");
        }

        // --- the current: Smarc.Environment.CurrentField, OFF ----------------------------
        var c = e.current_field;
        var curName = cfg.assetPrefix + "CurrentField (SCENARIO — off by default)";
        var existing = world.transform.Find(curName);
        if (c == null || c.SpeedMS < 0)
        {
            Debug.Log($"{tag} environment: no surface current layer" + (existing != null ? $" — existing {curName} left as it is." : "."));
            return;
        }
        GameObject go = existing != null ? existing.gameObject : new GameObject(curName);
        if (existing == null) go.transform.SetParent(world.transform, false);
        var box = go.GetComponent<BoxCollider>();
        if (box == null) box = go.AddComponent<BoxCollider>();
        box.isTrigger = true;                                   // ForceFieldBase applies via OnTriggerStay
        if (c.volume != null && c.volume.size != null && c.volume.size.x > 0)
        {
            go.transform.localPosition = new Vector3(c.volume.center.x, c.volume.center.y, c.volume.center.z);
            box.size = new Vector3(c.volume.size.x, c.volume.size.y, c.volume.size.z);
        }
        var ffs = go.GetComponent<ForceFieldStatic>();
        if (ffs == null) ffs = go.AddComponent<ForceFieldStatic>();
        ffs.onlyUnderwater = true;                              // a current is water, not air
        var cf = go.GetComponent<CurrentField>();
        if (cf == null) cf = go.AddComponent<CurrentField>();
        bool wasOn = existing != null && cf.CurrentEnabled;
        cf.HeadingDeg = c.HeadingDeg;
        cf.SpeedMS = c.SpeedMS;
        cf.CurrentEnabled = wasOn;                              // never switched on by a build; kept if the user had it on
        EditorUtility.SetDirty(go);
        if (se != null) { se.Current = cf; EditorUtility.SetDirty(se); }
        Debug.Log($"{tag} CurrentField: {c.SpeedMS:F2} m/s toward compass {c.HeadingDeg:F0} deg ({c.source}, {c.time_utc}, cell {c.distance_km:F1} km) — " +
                  (wasOn ? "ENABLED (it was on before this build)." : "OFF (scenario knob: tick Current Enabled on " + curName + " to let the vehicle feel it; that changes its dynamics)."));
    }

    /// LIVE AIS: one AISTraffic object with Smarc.Environment.AisTrafficLive pointed at the bundle's
    /// live/traffic.json. OFF by default (scenario knob); a rebuild keeps the user's on/off.
    /// THE SCENARIO CLOCK — time as its own object: one per scene, every time-dependent component
    /// reads it (EnvironmentTimeline, SunFromClock, AisTrafficLive). A rebuild keeps the user's mode,
    /// advance and time scale; a NEW window (the site was re-timed) resets the offset to its start.
    /// THE SWELL SHELTER MASK (2026-09-24, Moss Landing looked "flooded"): the Ocean is one infinite
    /// surface driven by the OFFSHORE sea state; unmasked, a 1.6 m swell ran through the harbour, up the
    /// slough and over the salt marsh. The bridge computes a mask from the terrain (land 0; depth-limited;
    /// decaying with distance through water from the open sea) — R/G attenuate the swell bands, B the
    /// ripples. HDRP applies it in the renderer AND in the CPU query the buoyancy reads, so physics stays =
    /// pixels. To see the unmasked offshore sea everywhere, clear Ocean > Water Surface > Water Mask.
    static void ApplyWaterMask(string tag, UnityBuildJ ub, WaterSurface ws)
    {
        var wm = ub.water_mask;
        if (wm == null || string.IsNullOrEmpty(wm.file) || wm.extent_m == null || wm.extent_m.Length < 2 ||
            wm.offset_m == null || wm.offset_m.Length < 2)
        {
            Debug.Log($"{tag} no swell shelter mask in the bundle (re-export with the current bridge) — the offshore swell runs everywhere, " +
                      "including harbours and sloughs." + (ws.waterMask != null ? $" Existing mask '{ws.waterMask.name}' left on {ws.name}." : ""));
            return;
        }
        string ap = ub.data_dir + "/" + wm.file;
        AssetDatabase.ImportAsset(ap, ImportAssetOptions.ForceSynchronousImport);
        var imp = AssetImporter.GetAtPath(ap) as TextureImporter;
        if (imp == null) { Debug.LogWarning($"{tag} swell shelter mask {ap} is not in the project (engine cache) — not applied."); return; }
        bool re = false;
        if (imp.sRGBTexture) { imp.sRGBTexture = false; re = true; }                       // a mask is linear data
        if (imp.mipmapEnabled) { imp.mipmapEnabled = false; re = true; }
        if (imp.wrapMode != TextureWrapMode.Clamp) { imp.wrapMode = TextureWrapMode.Clamp; re = true; }   // beyond the region: edge values
        if (imp.textureCompression != TextureImporterCompression.Uncompressed) { imp.textureCompression = TextureImporterCompression.Uncompressed; re = true; }
        if (imp.npotScale != TextureImporterNPOTScale.None) { imp.npotScale = TextureImporterNPOTScale.None; re = true; }
        if (imp.maxTextureSize < 4096) { imp.maxTextureSize = 4096; re = true; }
        if (re) imp.SaveAndReimport();
        var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(ap);
        if (tex == null) { Debug.LogWarning($"{tag} swell shelter mask {ap} did not load as a Texture2D — not applied."); return; }
        ws.waterMask = tex;
        ws.waterMaskExtent = new Vector2(wm.extent_m[0], wm.extent_m[1]);
        ws.waterMaskOffset = new Vector2(wm.offset_m[0], wm.offset_m[1]);
        ws.waterMaskRemap = new Vector2(0f, 1f);
        Debug.Log($"{tag} swell SHELTER mask on {ws.name}: {tex.width}x{tex.height} px over {wm.extent_m[0]:F0} x {wm.extent_m[1]:F0} m centred at " +
                  $"Unity ({wm.offset_m[0]:F0}, {wm.offset_m[1]:F0}); Hs_ref {wm.hs_ref_m:F2} m. Harbours, sloughs and land get no offshore swell " +
                  "(renderer and buoyancy alike). Clear Ocean > Water Surface > Water Mask to see the unmasked offshore sea.");
    }

    static ScenarioClock SetupClock(string tag, CuratedSiteConfig cfg, UnityBuildJ ub, GameObject world)
    {
        var c = ub.scenario_clock;
        var name = cfg.assetPrefix + "ScenarioClock";
        var tr = world.transform.Find(name);
        var go = tr != null ? tr.gameObject : new GameObject(name);
        if (tr == null) go.transform.SetParent(world.transform, false);
        var clock = go.GetComponent<ScenarioClock>();
        if (clock == null) clock = go.AddComponent<ScenarioClock>();
        Undo.RecordObject(clock, "OCEANVERSE scenario clock");
        if (c != null)
        {
            if (clock.WindowStartUtc != c.start_utc || clock.WindowEndUtc != c.end_utc) clock.Offset_h = 0f;
            clock.WindowStartUtc = c.start_utc; clock.WindowEndUtc = c.end_utc;
            clock.Latitude = c.lat; clock.Longitude = c.lon; clock.GridConvergence_deg = c.grid_convergence_deg;
        }
        EditorUtility.SetDirty(clock);
        Debug.Log($"{tag} SCENARIO CLOCK {go.name}: {clock.ClockMode}, window {clock.WindowStartUtc} → {clock.WindowEndUtc}, " +
                  $"offset {clock.Offset_h:F2} h, {(clock.AdvanceDuringPlay ? $"ADVANCING x{clock.TimeScale:g}" : "frozen")}; " +
                  $"place {clock.Latitude:F5}, {clock.Longitude:F5} (grid convergence {clock.GridConvergence_deg:+0.000;-0.000}°). " +
                  "It drives the sea, the weather, the sun and the live traffic; it is NOT the ROS clock.");
        return clock;
    }

    /// The SUN: SunFromClock on the scene's directional light — direction computed from the clock's
    /// time and place; intensity scaled by elevation and cloud cover (clear-sky value captured once).
    static void SetupSun(string tag, GameObject world, ScenarioClock clock)
    {
        Light dir = null;
        foreach (var l in world.GetComponentsInChildren<Light>(true)) if (l.type == LightType.Directional) { dir = l; break; }
        if (dir == null)
            foreach (var l in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsSortMode.None)) if (l.type == LightType.Directional) { dir = l; break; }
        if (dir == null) { Debug.LogWarning($"{tag} sun: no directional light in the scene — SunFromClock not added."); return; }
        var sun = dir.GetComponent<SunFromClock>();
        if (sun == null) sun = dir.gameObject.AddComponent<SunFromClock>();
        Undo.RecordObject(sun, "OCEANVERSE sun");
        sun.Clock = clock;
        sun.Weather = world.GetComponentInChildren<EnvironmentTimeline>(true);
        if (sun.ClearSkyIntensity < 0) sun.ClearSkyIntensity = dir.intensity;
        EditorUtility.SetDirty(sun);
        SunFromClock.SolarPosition(clock.Utc, clock.Latitude, clock.Longitude, out var el, out var az);
        Debug.Log($"{tag} SUN on '{dir.name}': computed from the clock ({clock.ScenarioUtc}): elevation {el:F1}°, azimuth {az:F1}° true; " +
                  $"clear-sky intensity {sun.ClearSkyIntensity:g}, scaled by elevation and cloud cover (untick Apply Intensity to keep it fixed).");
    }

    static void PlaceLiveTraffic(string tag, CuratedSiteConfig cfg, UnityBuildJ ub, GameObject world, ScenarioClock clock)
    {
        if (string.IsNullOrEmpty(ub.live_traffic_file)) return;
        var name = cfg.assetPrefix + "AISTraffic (LIVE — off by default)";
        var tr = world.transform.Find(name);
        var go = tr != null ? tr.gameObject : new GameObject(name);
        if (tr == null) go.transform.SetParent(world.transform, false);
        go.transform.localPosition = Vector3.zero; go.transform.localRotation = Quaternion.identity;
        var ais = go.GetComponent<AisTrafficLive>();
        if (ais == null) ais = go.AddComponent<AisTrafficLive>();
        Undo.RecordObject(ais, "OCEANVERSE live AIS");
        ais.TrafficFile = ub.live_traffic_file;
        ais.ReplayFile = ub.replay_traffic_file ?? "";
        ais.Clock = clock;
        if (ais.HullPhysicsMaterial == null)
            ais.HullPhysicsMaterial = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(CuratedSiteBuilder.PhysicMaterialPath("Steel"));
        EditorUtility.SetDirty(ais);
        Debug.Log($"{tag} LIVE AIS: {go.name} reads {ub.live_traffic_file} — " +
                  (ais.LiveTrafficEnabled ? "ENABLED (it was on before this build)." :
                   "OFF: start the feed in the site-selector console (`traffic live`; PAST ships: `traffic replay`), then tick Live Traffic Enabled. " +
                   "Hulls are scaled from each ship's AIS dimensions (A+B x C+D, draught), Steel colliders, dead-reckoned between fixes."));
    }

    /// SCENARIO TIME: the environment timeline (past / present / forecast) onto an
    /// EnvironmentTimeline next to SiteEnvironment on the Ocean. It drives the sea state, the
    /// CurrentField's speed/heading (never its on/off) and the tide — by moving the terrain tiles,
    /// the structures and the hazards (the Ocean stays at the origin). Starts FROZEN at the window
    /// start (the ScenarioClock is frozen unless someone asks for time): a build changes nothing about a run.
    static void SetupTimeline(string tag, CuratedSiteConfig cfg, UnityBuildJ ub, GameObject world, ScenarioClock clock, CuratedSiteBuilder.SiteJ site)
    {
        // 2026-09-24 (Ivan: "still looks flooded"): the terrain tiles are the tide movers, and edit-mode playback
        // moves them. The World is a PREFAB INSTANCE, so a moved tile is a scene OVERRIDE that survives the
        // rebuild — and the base below was then recorded from the moved height. Three rebuilds after tides of
        // +0.8 m left every tile 1.96 m low (scene -695.98, bundle -694.02): the marsh "flooded" at any tide.
        // Put every tile back AS BUILT (the bundle's unity_position) before anything reads it — always, even
        // without a timeline, so a stale tide offset can never outlive a build.
        // (the reset itself runs first thing in the populate step — see ResetTerrainsAsBuilt)
        var ws = world.GetComponentInChildren<WaterSurface>(true);
        if (ws == null || string.IsNullOrEmpty(ub.environment_file)) return;
        EnvironmentJ e;
        try { e = JsonUtility.FromJson<EnvironmentJ>(File.ReadAllText(CuratedSiteBuilder.PayloadFull(cfg, ub.environment_file))); }
        catch (Exception) { return; }
        var tl = ws.GetComponent<EnvironmentTimeline>();
        var t = e?.timeline;
        if (t == null || t.t_s == null || t.t_s.Length == 0)
        {
            if (tl != null) { tl.T_s = new float[0]; tl.StateNow = "no timeline in this export"; EditorUtility.SetDirty(tl); }
            Debug.Log($"{tag} scenario time: no timeline in this export (snapshot only). Add one: console `time at now for 6h`, then export.");
            return;
        }
        if (tl == null) tl = ws.gameObject.AddComponent<EnvironmentTimeline>();
        Undo.RecordObject(tl, "OCEANVERSE timeline");
        tl.enabled = false;                                   // no Evaluate while half-filled
        tl.WindowStartUtc = t.start_utc; tl.WindowEndUtc = t.end_utc; tl.GeneratedUtc = t.generated_utc;
        tl.Epochs = t.epochs != null ? string.Join(", ", t.epochs) : "";
        tl.Sources = t.sources != null ? string.Join("\n", t.sources) : "";
        tl.Gaps = t.gaps;
        tl.StepSeconds = t.step_s; tl.BuildSeaLevelMsl_m = t.build_sea_level_msl_m;
        tl.T_s = t.t_s; tl.SeaLevelMsl_m = t.sea_level_msl_m; tl.TideDy_m = t.tide_dy_m;
        tl.SwellWind_ms = t.swell_wind_ms; tl.RippleWind_ms = t.ripple_wind_ms; tl.SwellToward_deg = t.swell_toward_deg;
        tl.WaveHs_m = t.wave_hs_m; tl.WaveTp_s = t.wave_tp_s; tl.WaveFrom_deg = t.wave_from_deg;
        tl.WindSpeed_ms = t.wind_speed_ms; tl.WindFrom_deg = t.wind_from_deg;
        tl.CurrentSpeed_ms = t.current_speed_ms; tl.CurrentToward_deg = t.current_to_deg;
        tl.KindSeaLevel = t.kind_sea_level; tl.KindWaves = t.kind_waves; tl.KindWind = t.kind_wind; tl.KindCurrent = t.kind_current;
        tl.CloudCover_pct = t.cloud_cover_pct ?? new float[0]; tl.Visibility_m = t.visibility_m ?? new float[0];
        tl.Precipitation_mm_h = t.precipitation_mm_h ?? new float[0]; tl.AirTemperature_c = t.air_temperature_c ?? new float[0];
        tl.KindWeather = t.kind_weather ?? ""; tl.KindAirTemperature = t.kind_air_temperature ?? "";
        tl.Clock = clock;
        tl.Environment = ws.GetComponent<SiteEnvironment>();
        tl.Current = world.GetComponentInChildren<CurrentField>(true);
        // tide movers: every terrain tile (its structures are its children), the hazards root, the
        // structures fallback root — all AS BUILT (sea surface at the window start)
        var movers = new List<Transform>();
        foreach (var ter in world.GetComponentsInChildren<Terrain>(true)) movers.Add(ter.transform);
        foreach (var n in new[] { cfg.assetPrefix + "Hazards", cfg.assetPrefix + "Structures" })
        {
            var tr = world.transform.Find(n);
            if (tr == null || movers.Contains(tr)) continue;
            // these roots are AS BUILT at local y = 0 (their children carry Unity coordinates); an
            // earlier timeline may have left them tide-shifted — reset before recording the base
            var lp = tr.localPosition; lp.y = 0f; tr.localPosition = lp;
            movers.Add(tr);
        }
        tl.TideMovers = movers.ToArray();
        tl.TideMoverBaseY = movers.Select(m => m.position.y).ToArray();
        tl.enabled = true;
        EditorUtility.SetDirty(tl);
        int nM = (t.kind_sea_level ?? "").Count(ch => ch == 'M'), nF = (t.kind_waves ?? "").Count(ch => ch == 'F');
        Debug.Log($"{tag} SCENARIO TIME {t.start_utc} → {t.end_utc} ({tl.Epochs}), {t.samples} samples every {t.step_s:F0} s — " +
                  $"EnvironmentTimeline on {ws.name} plays the sea + weather at the ScenarioClock's time (scrub / advance THERE). " +
                  $"Tide moves {movers.Count} object(s) (terrain tiles, structures, hazards); the Ocean stays at y = 0. " +
                  $"Kinds: sea level {t.kind_sea_level?.Distinct().Count()} kind(s), {nM} measured; waves {nF} forecast samples. Gaps: {t.gaps}. " +
                  "Scenario time is NOT the ROS clock (that stays wall-anchored).");
    }

    static void ResetTerrainsAsBuilt(string tag, CuratedSiteConfig cfg, GameObject world, CuratedSiteBuilder.SiteJ site)
    {
        if (site?.tiles == null) return;
        var tl = world.GetComponentInChildren<EnvironmentTimeline>(true);
        if (tl != null) { tl.TideMovers = new Transform[0]; tl.TideMoverBaseY = new float[0]; EditorUtility.SetDirty(tl); }  // nothing moves them mid-reset
        int n = 0; float worst = 0f;
        foreach (var t in site.tiles)
        {
            var tr = world.transform.Find(cfg.assetPrefix + "_" + t.name);
            if (tr == null || t.unity_position == null) continue;
            var want = new Vector3(t.unity_position.x, t.unity_position.y, t.unity_position.z);
            float dy = tr.localPosition.y - want.y;
            if (Mathf.Abs(dy) > Mathf.Abs(worst)) worst = dy;
            Undo.RecordObject(tr, "OCEANVERSE terrain as built");
            tr.localPosition = want;
            if (PrefabUtility.IsPartOfPrefabInstance(tr))
                PrefabUtility.RevertObjectOverride(tr, InteractionMode.AutomatedAction);   // drop the stale override itself
            n++;
        }
        if (Mathf.Abs(worst) > 0.005f)
            Debug.LogWarning($"{tag} TERRAIN RESET: {n} tile(s) put back at their built height (base y {site.tiles[0].unity_position.y:F2} m); " +
                             $"they were {worst:+0.00;-0.00} m off — a tide offset left over from edit-mode playback (a prefab-instance override). " +
                             "Anything judged in the scene before this rebuild saw the terrain that much too " + (worst < 0 ? "LOW (flooded)." : "HIGH."));
        else
            Debug.Log($"{tag} terrain: {n} tile(s) at their built height (base y {site.tiles[0].unity_position.y:F2} m).");
    }

    /// Buildings, piers, breakwaters, seawalls, pontoons… from the site package's structures layer,
    /// as meshes the bridge already built in Unity coordinates. One mesh per tile and material,
    /// parented to that tile's terrain so the TerrainStreamer shows/hides them with it; a
    /// MeshCollider with the Rock/Steel physics material so the vehicle and the sonar meet them.
    /// Rebuilt in place on every build (meshes are assets in the data dir, overwritten).
    static void PlaceStructures(string tag, CuratedSiteConfig cfg, UnityBuildJ ub, GameObject world)
    {
        var rootName = cfg.assetPrefix + "Structures";
        if (string.IsNullOrEmpty(ub.structures_file))
        {
            Debug.Log($"{tag} structures: none in this export (the site package has no structures.geojson).");
            return;
        }
        string path;
        try { path = CuratedSiteBuilder.PayloadFull(cfg, ub.structures_file); }
        catch (FileNotFoundException) { Debug.LogWarning($"{tag} structures: {ub.structures_file} missing — re-run the bridge."); return; }
        var sj = JsonUtility.FromJson<StructuresJ>(File.ReadAllText(path));
        if (sj == null || sj.tiles == null) { Debug.LogWarning($"{tag} structures: {ub.structures_file} unreadable."); return; }

        // remove what an earlier build placed (the meshes are regenerated from the bundle)
        foreach (var t in world.GetComponentsInChildren<Transform>(true))
            if (t != null && t.name == rootName) UnityEngine.Object.DestroyImmediate(t.gameObject);
        var fallbackRoot = new GameObject(rootName);
        fallbackRoot.transform.SetParent(world.transform, false);

        int meshes = 0, tris = 0;
        foreach (var tj in sj.tiles)
        {
            if (tj.groups == null) continue;
            var terrainT = world.transform.Find(cfg.assetPrefix + "_" + tj.tile);
            Transform parent;
            if (terrainT != null)
            {
                var holder = new GameObject(rootName);
                holder.transform.SetParent(terrainT, false);
                // the terrain sits at its SW corner and base; structures are in world coordinates
                holder.transform.position = Vector3.zero;
                parent = holder.transform;
            }
            else parent = fallbackRoot.transform;
            foreach (var g in tj.groups)
            {
                if (g.vertices == null || g.triangles == null || g.triangles.Length == 0) continue;
                var assetPath = $"{cfg.dataDir}/{cfg.assetPrefix}_{tj.tile}_{g.render_material}_structures.asset";
                var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
                bool isNew = mesh == null;
                if (isNew) mesh = new Mesh();
                mesh.Clear();
                mesh.name = $"{cfg.assetPrefix}_{tj.tile}_{g.render_material}";
                int nv = g.vertices.Length / 3;
                if (nv > 65535) mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
                var verts = new Vector3[nv];
                for (int i = 0; i < nv; i++) verts[i] = new Vector3(g.vertices[3 * i], g.vertices[3 * i + 1], g.vertices[3 * i + 2]);
                mesh.vertices = verts;
                mesh.triangles = g.triangles;
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                if (isNew)
                {
                    var dir = Path.GetDirectoryName(CuratedSiteBuilder.AssetPathToFull(assetPath));
                    if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
                    AssetDatabase.CreateAsset(mesh, assetPath);
                }
                else EditorUtility.SetDirty(mesh);

                var go = new GameObject($"{g.render_material}");
                go.transform.SetParent(parent, false);
                go.transform.position = Vector3.zero;
                go.isStatic = true;
                go.AddComponent<MeshFilter>().sharedMesh = mesh;
                go.AddComponent<MeshRenderer>().sharedMaterial = StructureMaterial(cfg, g.render_material);
                var col = go.AddComponent<MeshCollider>();
                col.sharedMesh = mesh;
                // sharedMaterial, never .material (SETTLED §3f): the sonar reads the physics material's name
                var pm = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(CuratedSiteBuilder.PhysicMaterialPath(
                    string.IsNullOrEmpty(g.physics_material) ? "Rock" : g.physics_material));
                if (pm != null) col.sharedMaterial = pm;
                meshes++; tris += g.triangles.Length / 3;
            }
        }
        if (fallbackRoot.transform.childCount == 0) UnityEngine.Object.DestroyImmediate(fallbackRoot);
        var kinds = new Dictionary<string, int>();
        int inTerrain = 0;
        foreach (var f in sj.features ?? new StructFeatJ[0])
        {
            if (f.in_terrain) { inTerrain++; continue; }
            kinds[f.kind] = kinds.TryGetValue(f.kind, out var n) ? n + 1 : 1;
        }
        var parts = new List<string>();
        foreach (var kv in kinds) parts.Add($"{kv.Key} {kv.Value}");
        Debug.Log($"{tag} structures: {string.Join(", ", parts)} — {meshes} mesh(es), {tris} triangles, MeshColliders (Rock/Steel). " +
                  $"{sj.heights_assumed} heights ASSUMED (default by kind), {sj.synthetic_piles} pier piles SYNTHETIC; " +
                  $"{inTerrain} coastal works already in the terrain. Parented to their tile's terrain (streamed with it).");
    }

    /// DRY BASINS (2026-09-24, Ivan vs Google Earth): the Ocean is ONE infinite plane, so at a +0.82 m tide it
    /// "flooded" the Moss Landing salt ponds, the Moro Cojo fields and the marsh behind the Hwy 1 embankment —
    /// land the real sea cannot reach (levees, embankments, tide gates). The bridge flood-fills from the open
    /// sea at the window's HIGHEST sea level; what stays cut off gets a flat sheet with HDRP's water-exclusion
    /// material just above the sea surface: the water is not drawn there, the terrain is untouched. The sheet
    /// is NOT a tide mover (the sea surface stays at y = 0). Rendering only: the Ocean's CPU query still sees
    /// water there. To compare, untick the DryBasins object.
    const string WaterExclusionMat = "Packages/com.unity.render-pipelines.high-definition/Runtime/RenderPipelineResources/Material/MaterialWaterExclusion.mat";

    static void PlaceDryBasins(string tag, CuratedSiteConfig cfg, UnityBuildJ ub, GameObject world)
    {
        var name = cfg.assetPrefix + "DryBasins (water excluder)";
        foreach (var t in world.GetComponentsInChildren<Transform>(true))
            if (t != null && t.name == name) UnityEngine.Object.DestroyImmediate(t.gameObject);
        var r = ub.dry_basins;
        if (r == null || string.IsNullOrEmpty(r.file))
        {
            Debug.Log($"{tag} dry basins: none in this export (no land below the sea level cut off from the sea, or an older bridge).");
            return;
        }
        DryBasinsJ d;
        try { d = JsonUtility.FromJson<DryBasinsJ>(File.ReadAllText(CuratedSiteBuilder.PayloadFull(cfg, r.file))); }
        catch (Exception ex) { Debug.LogWarning($"{tag} dry basins: {r.file} unreadable ({ex.Message}) — re-run the bridge. The Ocean floods them."); return; }
        if (d == null || d.vertices == null || d.triangles == null || d.triangles.Length == 0) return;
        var mat = AssetDatabase.LoadAssetAtPath<Material>(WaterExclusionMat);
        if (mat == null) { Debug.LogWarning($"{tag} dry basins: HDRP water-exclusion material not found at {WaterExclusionMat} — not applied."); return; }
        var hd = UnityEngine.Rendering.GraphicsSettings.currentRenderPipeline as HDRenderPipelineAsset;
        if (hd != null && !hd.currentPlatformRenderPipelineSettings.supportWaterExclusion)
            Debug.LogWarning($"{tag} dry basins: the active HDRP asset '{hd.name}' has Water > Exclusion OFF — tick it, or the basins stay flooded.");

        var assetPath = $"{cfg.dataDir}/{cfg.assetPrefix}_drybasins.asset";
        var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
        bool isNew = mesh == null;
        if (isNew) mesh = new Mesh();
        mesh.Clear();
        mesh.name = cfg.assetPrefix + "_drybasins";
        int nv = d.vertices.Length / 3;
        mesh.indexFormat = nv > 65535 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16;
        var verts = new Vector3[nv];
        for (int i = 0; i < nv; i++) verts[i] = new Vector3(d.vertices[3 * i], d.vertices[3 * i + 1], d.vertices[3 * i + 2]);
        mesh.vertices = verts;
        mesh.triangles = d.triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        if (isNew)
        {
            var dir = Path.GetDirectoryName(CuratedSiteBuilder.AssetPathToFull(assetPath));
            if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
            AssetDatabase.CreateAsset(mesh, assetPath);
        }
        else EditorUtility.SetDirty(mesh);
        int up = 0;
        foreach (var n in mesh.normals) if (n.y > 0.99f) up++;

        var go = new GameObject(name);
        go.transform.SetParent(world.transform, false);
        go.transform.localPosition = Vector3.zero;
        go.AddComponent<MeshFilter>().sharedMesh = mesh;
        var mr = go.AddComponent<MeshRenderer>();
        mr.sharedMaterial = mat;
        mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        mr.receiveShadows = false;
        mr.lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.Off;
        mr.reflectionProbeUsage = UnityEngine.Rendering.ReflectionProbeUsage.Off;
        mr.rayTracingMode = UnityEngine.Experimental.Rendering.RayTracingMode.Off;
        EditorUtility.SetDirty(go);
        Debug.Log($"{tag} DRY BASINS: {d.basin_km2:F2} km2 of land below the window's highest sea level (+{d.level_msl_m:F3} m MSL) that the sea " +
                  $"cannot reach — water not drawn there ({mesh.triangles.Length / 3} triangles at Unity y = {d.excluder_unity_y:F2} m, " +
                  $"{up}/{nv} normals up; HDRP water excluder). Rule: {d.rule}. Untick '{name}' to compare; the terrain is unchanged.");
    }

    static Material StructureMaterial(CuratedSiteConfig cfg, string name)
    {
        var p = $"{cfg.dataDir}/{name}.mat";
        var m = AssetDatabase.LoadAssetAtPath<Material>(p);
        if (m != null) return m;
        var sh = Shader.Find("HDRP/Lit");
        if (sh == null) return AssetDatabase.LoadAssetAtPath<Material>(CuratedSiteBuilder.MatTerrain);
        m = new Material(sh);
        Color c = name.EndsWith("Rock") ? new Color(0.45f, 0.43f, 0.40f) : name.EndsWith("Steel") ? new Color(0.35f, 0.37f, 0.40f)
                : name.EndsWith("Wood") ? new Color(0.45f, 0.33f, 0.22f) : new Color(0.62f, 0.62f, 0.60f);
        m.SetColor("_BaseColor", c);
        if (name.EndsWith("Steel")) m.SetFloat("_Metallic", 0.7f);
        var dir = Path.GetDirectoryName(CuratedSiteBuilder.AssetPathToFull(p));
        if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
        AssetDatabase.CreateAsset(m, p);
        return m;
    }

    static void PlaceVehicle(string tag, CuratedSiteConfig cfg, UnityBuildJ ub, GameObject world, CuratedSiteBuilder.SiteJ site)
    {
        var lp = site.launch_point;
        if (lp == null || lp.unity == null) { Debug.LogWarning($"{tag} no launch_point in the site manifest."); return; }
        var pos = new Vector3(lp.unity.x, -0.1f, lp.unity.z);
        float seabed = CuratedSiteBuilder.TerrainHeightAt(world, lp.unity.x, lp.unity.z);
        Debug.Log($"{tag} launch point ({lp.measured_by}): Unity ({lp.unity.x:F1}, -0.1, {lp.unity.z:F1}) = " +
                  $"{lp.lat:F6}, {lp.lon:F6}, grid heading {lp.heading_deg:F1} deg; manifest seabed {lp.seabed_m:F2} m below the surface, " +
                  $"built terrain {seabed:F2} m. Rule: {lp.rule}");
        if (!float.IsNaN(seabed) && Mathf.Abs(seabed - lp.seabed_m) > 0.5f)
            Debug.LogWarning($"{tag} terrain and manifest disagree at the launch point by {Mathf.Abs(seabed - lp.seabed_m):F2} m.");

        var vname = string.IsNullOrEmpty(ub.vehicle_name) ? "sam21" : ub.vehicle_name;
        var sam = GameObject.Find(vname);
        if (sam == null)
        {
            string want = null;
            foreach (var p in ub.vehicle_prefabs ?? new string[0])
                if (AssetDatabase.LoadAssetAtPath<GameObject>(p) != null) { want = p; break; }
            if (want == null) { Debug.LogError($"{tag} none of the vehicle prefabs exist: {string.Join(", ", ub.vehicle_prefabs ?? new string[0])}"); return; }
            sam = CuratedSiteBuilder.Place(cfg, want, vname, pos, lp.heading_deg);
            Debug.Log($"{tag} {vname} instantiated from {want} at the launch point. The GameObject name IS the " +
                      $"ROS robot name (/{vname}/...), so it is not renamed.");
        }
        else
            Debug.Log($"{tag} existing {vname} kept at {sam.transform.position} (populate is additive; " +
                      $"{Vector3.Distance(new Vector3(sam.transform.position.x, 0, sam.transform.position.z), new Vector3(pos.x, 0, pos.z)):F1} m from the launch point).");
        if (GameObject.Find("GUI") == null)
            CuratedSiteBuilder.Place(cfg, GuiPrefab, "GUI", pos + new Vector3(0, 10f, 0));
    }
}
