using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

using GeoRef;
using Smarc.Environment;

/// <summary>
/// Builds a CURATED site — terrains, world prefab and scene — from an OCEANVERSE scenario
/// bundle produced by `data-cube/scripts/<site>-site/{build_heightmap.py, build_splatmap.py}`.
///
/// THIS IS THE ONE BUILDER. It was lifted out of AskoSiteBuilder.cs on 2026-09-15 when Djurö
/// became the second curated site to need it. SETTLED §3d: one pipeline, never a second
/// implementation — a copied 982-line C# builder would have been two heightmap readers, two
/// splat appliers and two zone assertions that drift apart the first time one of them is
/// fixed. `AskoSiteBuilder` and `DjuroSiteBuilder` are now thin: they own a
/// <see cref="CuratedSiteConfig"/>, a menu item and whatever is genuinely site-specific
/// (Askö's hi-res DV patch and its hand-placed mission set; Djurö's charted rocks and its
/// measured launch point). The Askö behaviour is unchanged — same menu item, same asset
/// paths, same log lines, same order of operations.
///
/// Frame: Unity +X = UTM easting, +Z = UTM northing, +Y = up, origin = the site anchor from
/// the bundle's own site manifest. Water at y = 0. The UTM ZONE is asserted against the
/// manifest on every build and is never inherited: Askö is 33, Beckholmen and Djurö are 34.
/// </summary>
public class CuratedSiteConfig
{
    /// File-name prefix inside the bundle payload: "asko" -> asko_site.json, asko_C_*.
    public string prefix;
    /// Prefix for generated Unity assets: "AskoCurated" -> AskoCurated_SW.asset.
    public string assetPrefix;
    /// Prefix for the generated .terrainlayer assets: "Asko_" -> Asko_Bedrock.terrainlayer.
    public string layerPrefix;
    /// Log tag, e.g. "[Asko]".
    public string tag;
    /// Package folder holding the engine cache (textures, class maps, manifests).
    public string dataDir;
    /// The bundle payload, RELATIVE TO THE UNITY PROJECT ROOT. Source of truth.
    public string bundleRel;
    /// Prefab cloned for Ocean / Sun / Sky.
    public string sourceWorldPrefab;
    /// Where the generated world prefab is written, and what it is called in the scene.
    public string worldPrefabPath, worldObjectName;
    /// The scene this builder creates when it does not already exist.
    public string scenePath;
    /// Name of the GlobalReferencePoint object placed at the scene origin.
    public string globalRefName;
    /// The zone the anchor MUST be in. A mismatch is a hard error, not a warning.
    public int expectedUtmZone;
    /// A legacy frame for this site whose origin is somewhere else, or null. Re-asserted on
    /// every build so the discrepancy cannot quietly disappear.
    public double[] legacyOriginUtm;          // {easting, northing} or null
    public string legacyOriginWhat;           // what carries it, for the warning text
    /// What turning the gap patch off MEANS at this site. Printed in the build log, because
    /// it is not the same sentence everywhere: at Askö it removes 59.6% of the seabed, at
    /// Djurö it removes all of it.
    public string gapPatchSemantics;
    /// Wire Terrain.SetNeighbors across a 2x2 {S,N}x{W,E} tile grid.
    public bool wireNeighbours;
    /// Physics material for the terrain collider itself.
    public string terrainPhysicMaterial = "Mud";
    /// Extra sentence appended to the "scene exists" log — the site's own follow-up menu
    /// items. Kept in the config so the Askö log line is unchanged by the refactor.
    public string sceneExistsHint = "";
    /// Extra sentence appended to the "scene created" log.
    public string sceneCreatedHint = "";
}

public static class CuratedSiteBuilder
{
    public const string PrefabDir =
        "Packages/com.smarc.assets/Runtime/Prefabs/Environment/GeoReferenced";
    public const string MatDir = "Packages/com.smarc.assets/Runtime/Materials";
    public const string MatTerrain = MatDir + "/DefaultHDTerrainLitMaterial.mat";
    public const string TileMaterialPath = MatDir + "/2DMapMaterial.mat";

    public static string PhysicMaterialPath(string name)
    {
        return MatDir + "/Physic/" + name + ".physicMaterial";
    }

    // ---- mirror of <site>_site.json ----------------------------------------
    [Serializable] public class Vec3J { public float x, y, z; }
    [Serializable]
    public class AnchorJ
    {
        public double lat, lon; public string utm_epsg; public int utm_zone;
        public double utm_easting, utm_northing;
        public string origin_confidence;
    }
    [Serializable]
    public class TileStatsJ
    {
        public float min_m, max_m, water_fraction, real_fraction, synthetic_fraction;
        public float land_fraction, hole_fraction_when_patch_off;
    }
    [Serializable]
    public class TileJ
    {
        public string name, heightmap, synthetic_mask, source_mask;
        public int resolution, mask_resolution;
        public float terrain_size_x_m, terrain_size_z_m, terrain_size_y_m, terrain_base_y_m;
        public Vec3J unity_position;
        public TileStatsJ statistics;
        // OPTIONAL, absent on a tile that carries no nested patch (Askö's DV hi-res ingest).
        public string heightmap_collared, hires_cutout_mask;
    }
    [Serializable] public class CoverageJ
    { public long cells, real; public float real_fraction, synthetic_fraction; }
    [Serializable]
    public class ChartPatchJ
    {
        public bool enabled_at_build; public long cells, cells_without_chart;
        public float fraction_of_scene; public string method, source, rights, unity_toggle;
    }
    [Serializable]
    public class LaunchPointJ
    {
        public Vec3J unity; public double lat, lon; public float seabed_m; public string rule;
        // Optional (launch_point.py, Djurö 2026-09-15): measured course at the launch and
        // which hull the campaign was flown with. JsonUtility leaves them 0/null when absent.
        public float heading_deg; public string source, measured_by, vehicle;
    }
    /// <summary>
    /// The support vessel and what it carries. Optional: JsonUtility leaves it null when the
    /// manifest has no such block, which is every site but Djurö today. Written by
    /// `data-cube/scripts/djuro-site/support_vessel.py` from the campaign's STATION GPS.
    /// </summary>
    [Serializable]
    public class SupportVesselJ
    {
        public Vec3J unity; public double lat, lon;
        public string vessel, carries, measured_by, source, rule, track_csv;
        public float median_speed_mps;
    }
    [Serializable]
    public class SiteJ
    {
        public SupportVesselJ support_vessel;
        public string generated_utc, generator, site, scenario_bundle;
        public string data_status, data_status_note;
        public AnchorJ anchor; public TileJ[] tiles;
        public CoverageJ coverage; public ChartPatchJ chart_patch;
        public LaunchPointJ launch_point;
        public string[] priority; public string priority_rule;
        public int rocks_count; public string rocks_file;
    }
    [Serializable] public class SplatLayerJ
    { public string name, texture, means; public float tile_size_m; }
    [Serializable]
    public class AcousticClassJ
    { public float reflectivity; public int label; public string physics_material, note; }
    [Serializable]
    public class AcousticsJ { public AcousticClassJ bedrock, sand, clay, land; }
    [Serializable]
    public class SplatJ
    {
        public int control_resolution; public float tile_size_m, ortho_tile_size_m;
        public SplatLayerJ[] layers; public AcousticsJ acoustics;
    }

    // ---- the build ---------------------------------------------------------
    public static void Build(CuratedSiteConfig cfg)
    {
        try
        {
            splatCache.Remove(cfg.prefix);          // re-read the manifest on every build
            var site = LoadSite(cfg);
            var tds = new Dictionary<string, TerrainData>();
            foreach (var t in site.tiles) tds[t.name] = BuildTerrain(cfg, site, t);
            var world = BuildWorldPrefab(cfg, site, tds);
            BuildScene(cfg, world, site);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log(cfg.tag + " DONE. Scene: " + cfg.scenePath);
        }
        catch (Exception e)
        {
            Debug.LogError(cfg.tag + " build failed: " + e);
            throw;
        }
    }

    public static string AssetPathToFull(string assetPath)
    {
        var pi = UnityEditor.PackageManager.PackageInfo.FindForAssetPath(assetPath);
        if (pi != null)
            return Path.Combine(pi.resolvedPath,
                assetPath.Substring(("Packages/" + pi.name + "/").Length));
        return Path.Combine(Application.dataPath, assetPath.Substring("Assets/".Length));
    }

    // ------------------------------------------------------------------------
    // WHERE THE CURATED DATA LIVES (rehomed 2026-08-27, Ivan's call):
    // the OCEANVERSE scenario bundle in the SMDS cloud store is the SOURCE OF
    // TRUTH for every curated artefact. This builder is the smarcsim import
    // step: it reads build-time payloads (heightmaps, masks, splat controls)
    // straight from the bundle, and the package keeps only what the ENGINE
    // needs at runtime or by GUID as a cache the bundle can always regenerate.
    // ------------------------------------------------------------------------
    /// <summary>Full path of a curated payload file: the bundle first (source of
    /// truth), the package cache second (runtime copies). Errors name both.</summary>
    public static string PayloadFull(CuratedSiteConfig cfg, string fileName)
    {
        // Application.dataPath = <project>/Assets; the workspace root is two up.
        var projectRoot = Path.GetDirectoryName(Application.dataPath);
        var bundle = Path.GetFullPath(Path.Combine(projectRoot, cfg.bundleRel, fileName));
        if (File.Exists(bundle)) return bundle;
        var cache = AssetPathToFull(cfg.dataDir + "/" + fileName);
        if (File.Exists(cache))
        {
            Debug.LogWarning($"{cfg.tag} {fileName}: not in the scenario bundle ({bundle}) — " +
                             "using the engine cache copy. The bundle is supposed to be " +
                             "the source of truth; re-run the generators.");
            return cache;
        }
        throw new FileNotFoundException(
            $"{fileName} found neither in the scenario bundle ({bundle}) nor the engine " +
            $"cache ({cache}) — run the site's build_heightmap.py first.");
    }

    public static SiteJ LoadSite(CuratedSiteConfig cfg)
    {
        var p = PayloadFull(cfg, cfg.prefix + "_site.json");
        var s = JsonUtility.FromJson<SiteJ>(File.ReadAllText(p));
        if (s == null || s.tiles == null || s.tiles.Length == 0 || s.anchor == null)
            throw new Exception(cfg.prefix + "_site.json did not parse into a site manifest");
        Debug.Log($"{cfg.tag} manifest {s.generated_utc}\n" +
                  $"       {s.tiles.Length} tiles; REAL {s.coverage.real_fraction * 100f:F2}% of the " +
                  $"scene, SYNTHETIC {s.coverage.synthetic_fraction * 100f:F2}% " +
                  $"({s.chart_patch.cells:N0} chart-extrapolated cells)\n" +
                  $"       priority: {string.Join(" > ", s.priority)}\n" +
                  $"       {s.priority_rule}");
        if (!string.IsNullOrEmpty(s.data_status) && s.data_status != "baked_real")
            Debug.LogWarning($"{cfg.tag} data_status = {s.data_status}. {s.data_status_note}");
        return s;
    }

    public static TerrainData BuildTerrain(CuratedSiteConfig cfg, SiteJ site, TileJ t)
    {
        // The collared variant, when the manifest names one and the file is there.
        var raw = PayloadFull(cfg, t.heightmap);
        if (!string.IsNullOrEmpty(t.heightmap_collared))
        {
            try
            {
                raw = PayloadFull(cfg, t.heightmap_collared);
                Debug.Log($"{cfg.tag} {t.name}: using the SEAM-COLLARED heightmap " +
                          $"{t.heightmap_collared} (the un-collared {t.heightmap} is " +
                          "untouched on disk).");
            }
            catch (FileNotFoundException)
            {
                Debug.LogWarning($"{cfg.tag} {t.name}: manifest names {t.heightmap_collared} " +
                                 "but it is not there — building the UN-COLLARED tile, so " +
                                 "expect a step at the hi-res patch boundary.");
            }
        }

        int res = t.resolution;
        var bytes = File.ReadAllBytes(raw);
        long expected = (long)res * res * 2;
        if (bytes.LongLength != expected)
            throw new Exception($"{t.name}: heightmap is {bytes.LongLength} bytes, expected " +
                                $"{expected} for res {res}");

        // r16: uint16 little-endian, row 0 = SOUTH, col 0 = WEST.
        // Unity's SetHeights takes [y, x] where y indexes +Z. Same order, direct copy.
        var heights = new float[res, res];
        int k = 0;
        for (int z = 0; z < res; z++)
            for (int x = 0; x < res; x++, k += 2)
                heights[z, x] = (bytes[k] | (bytes[k + 1] << 8)) / 65535f;

        string assetPath = cfg.dataDir + "/" + cfg.assetPrefix + "_" + t.name + ".asset";
        var td = AssetDatabase.LoadAssetAtPath<TerrainData>(assetPath);
        bool isNew = td == null;
        if (isNew) td = new TerrainData();
        td.name = cfg.assetPrefix + "_" + t.name;
        td.heightmapResolution = res;
        td.size = new Vector3(t.terrain_size_x_m, t.terrain_size_y_m, t.terrain_size_z_m);
        td.SetHeights(0, 0, heights);
        if (isNew)
        {
            var dir = Path.GetDirectoryName(AssetPathToFull(assetPath));
            if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
            AssetDatabase.CreateAsset(td, assetPath);
        }
        else EditorUtility.SetDirty(td);

        ApplySplat(cfg, td, t);

        Debug.Log($"{cfg.tag} {t.name}: {res}x{res} @1 m, size {td.size}, base {t.terrain_base_y_m:F1} m, " +
                  $"range {t.statistics.min_m:F2}..{t.statistics.max_m:F2} m, " +
                  $"{t.statistics.water_fraction * 100f:F1}% water, " +
                  $"REAL {t.statistics.real_fraction * 100f:F1}% / " +
                  $"SYNTHETIC {t.statistics.synthetic_fraction * 100f:F1}%");
        return td;
    }

    /// Terrain layers from the site's splat manifest. A layer whose texture is the literal
    /// "PER_TILE" is the orthophoto: a different image per terrain, draped ONCE across it.
    public static void ApplySplat(CuratedSiteConfig cfg, TerrainData td, TileJ t)
    {
        var manifestPath = AssetPathToFull(cfg.dataDir + "/" + cfg.prefix + "_splat.json");
        if (!File.Exists(manifestPath))
        {
            Debug.LogWarning($"{cfg.tag} no {cfg.prefix}_splat.json — terrain left untextured, " +
                             "so REAL and SYNTHETIC will look identical. Run build_splatmap.py.");
            return;
        }
        var sm = JsonUtility.FromJson<SplatJ>(File.ReadAllText(manifestPath));
        var layers = new TerrainLayer[sm.layers.Length];
        for (int i = 0; i < sm.layers.Length; i++)
        {
            var L = sm.layers[i];
            bool perTile = L.texture == "PER_TILE";
            string texPath = perTile ? cfg.dataDir + "/" + cfg.prefix + "_" + t.name + "_ortho.png"
                                     : cfg.dataDir + "/" + L.texture;
            // Unity's default maxTextureSize is 2048, so a 4096 orthophoto would be silently
            // halved to 2 m/px on import. Set it explicitly.
            if (perTile) EnsureMaxTextureSize(cfg, texPath, 4096);
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(texPath);
            if (tex == null)
            {
                Debug.LogWarning($"{cfg.tag} missing texture {texPath} — layer '{L.name}' will be " +
                                 "flat. Run build_ortho.py / build_splatmap.py.");
            }
            float tile = perTile ? sm.ortho_tile_size_m
                                 : (L.tile_size_m > 0 ? L.tile_size_m : sm.tile_size_m);
            var layerPath = perTile
                ? cfg.dataDir + "/" + cfg.layerPrefix + "Ortho_" + t.name + ".terrainlayer"
                : cfg.dataDir + "/" + cfg.layerPrefix + L.name + ".terrainlayer";
            var layer = AssetDatabase.LoadAssetAtPath<TerrainLayer>(layerPath);
            if (layer == null) { layer = new TerrainLayer(); AssetDatabase.CreateAsset(layer, layerPath); }
            layer.diffuseTexture = tex;
            layer.tileSize = new Vector2(tile, tile);
            layer.tileOffset = Vector2.zero;
            EditorUtility.SetDirty(layer);
            layers[i] = layer;
        }
        td.terrainLayers = layers;

        string ctrlPath;
        try { ctrlPath = PayloadFull(cfg, cfg.prefix + "_" + t.name + "_splat.u8"); }
        catch (FileNotFoundException e)
        { Debug.LogWarning($"{cfg.tag} control map missing: " + e.Message); return; }
        var raw = File.ReadAllBytes(ctrlPath);
        int n = sm.control_resolution;
        long expect = (long)n * n * layers.Length;
        if (raw.LongLength != expect)
        {
            Debug.LogError($"{cfg.tag} {t.name} control map is {raw.LongLength} bytes, expected " +
                           $"{expect} for {n}x{n}x{layers.Length} — regenerate it");
            return;
        }
        td.alphamapResolution = n;
        var maps = new float[n, n, layers.Length];
        int k = 0;
        for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
                for (int L = 0; L < layers.Length; L++, k++)
                    maps[y, x, L] = raw[k] / 255f;
        td.SetAlphamaps(0, 0, maps);
    }

    public static void EnsureMaxTextureSize(CuratedSiteConfig cfg, string assetPath, int max)
    {
        var imp = AssetImporter.GetAtPath(assetPath) as TextureImporter;
        if (imp == null) return;
        if (imp.maxTextureSize >= max) return;
        imp.maxTextureSize = max;
        imp.SaveAndReimport();
        Debug.Log($"{cfg.tag} {assetPath}: maxTextureSize -> {max}");
    }

    static readonly Dictionary<string, SplatJ> splatCache = new Dictionary<string, SplatJ>();

    public static SplatJ LoadSplatManifest(CuratedSiteConfig cfg)
    {
        if (splatCache.TryGetValue(cfg.prefix, out var cached) && cached != null) return cached;
        string p;
        try { p = PayloadFull(cfg, cfg.prefix + "_splat.json"); }
        catch (FileNotFoundException) { return null; }
        var sm = JsonUtility.FromJson<SplatJ>(File.ReadAllText(p));
        splatCache[cfg.prefix] = sm;
        return sm;
    }

    public static SeabedAcousticMap.ClassEntry MakeClass(string name, AcousticClassJ j)
    {
        return new SeabedAcousticMap.ClassEntry
        {
            name = name,
            reflectivity = j != null ? j.reflectivity : 0.5f,
            label = j != null ? j.label : 1,
        };
    }

    public static GameObject BuildWorldPrefab(CuratedSiteConfig cfg, SiteJ site,
                                              Dictionary<string, TerrainData> tds)
    {
        var src = AssetDatabase.LoadAssetAtPath<GameObject>(cfg.sourceWorldPrefab);
        if (src == null)
            throw new FileNotFoundException("world prefab not found at " + cfg.sourceWorldPrefab);

        var root = (GameObject)PrefabUtility.InstantiatePrefab(src);
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely,
                                           InteractionMode.AutomatedAction);
        root.name = cfg.worldObjectName;
        root.transform.position = Vector3.zero;

        // Keep the environment essentials; drop the legacy GLOBALREF, which carries a
        // different frame. Two GlobalReferencePoints disagreeing about where the world is
        // is exactly the ambiguity this project has paid for before.
        var keep = new HashSet<string> { "Ocean", "Sun", "Sky and Fog Global Volume" };
        var dropped = new List<string>();
        for (int i = root.transform.childCount - 1; i >= 0; i--)
        {
            var c = root.transform.GetChild(i).gameObject;
            if (!keep.Contains(c.name))
            {
                dropped.Add(c.name);
                UnityEngine.Object.DestroyImmediate(c);
            }
        }
        if (dropped.Count > 0)
            Debug.Log($"{cfg.tag} kept Ocean/Sun/Sky from {Path.GetFileName(cfg.sourceWorldPrefab)}; " +
                      $"dropped {dropped.Count} other child(ren): {string.Join(", ", dropped)}. " +
                      "Anything hand-placed there belonged to the SOURCE prefab's frame, not " +
                      "this one — re-place it deliberately if it is wanted here.");

        // The Ocean must sit at exactly (0,0,0): the water plane is the vertical datum every
        // depth in this pipeline is referenced to, and raising it "for realism" is what sent
        // SAM to 67 m/s on 2026-08-18 (SETTLED §3s).
        var ocean = root.transform.Find("Ocean");
        if (ocean != null && ocean.localPosition != Vector3.zero)
        {
            Debug.LogWarning($"{cfg.tag} Ocean was at {ocean.localPosition}; forcing (0,0,0) — " +
                             "the water plane is the datum, not a dressing choice.");
            ocean.localPosition = Vector3.zero;
        }

        // --- GLOBALREF at the scene origin, carrying the registered site anchor ----
        var gref = new GameObject(cfg.globalRefName);
        gref.transform.SetParent(root.transform, false);
        gref.transform.localPosition = Vector3.zero;
        var grp = gref.AddComponent<GlobalReferencePoint>();
        grp.OriginMode = OriginMode.LatLon;
        grp.UnityCoordinateFrame = UnityCoordinateFrame.UTM;
        grp.Lat = site.anchor.lat;
        grp.Lon = site.anchor.lon;
        var so = new SerializedObject(grp);
        so.ApplyModifiedProperties();
        // Reflection, not SendMessage: SendMessage on a just-added component trips Unity's
        // ShouldRunBehaviour() assertion and logs a red error with no message.
        var onValidate = typeof(GlobalReferencePoint).GetMethod("OnValidate",
            System.Reflection.BindingFlags.Instance |
            System.Reflection.BindingFlags.Public |
            System.Reflection.BindingFlags.NonPublic);
        if (onValidate != null) onValidate.Invoke(grp, null);

        var tiler = gref.AddComponent<WMSTiler>();
        tiler.TileMaterial = AssetDatabase.LoadAssetAtPath<Material>(TileMaterialPath);
        tiler.Radius = 4096;
        tiler.TileSizePx = 400;
        tiler.TileSizeMeters = 256;
        if (tiler.TileMaterial == null)
            Debug.LogWarning($"{cfg.tag} tile material not found at " + TileMaterialPath);

        // Cross-check Unity's own geodesy against the pipeline's pyproj — two independent
        // implementations, one number. A silent disagreement here offsets everything.
        Debug.Log($"{cfg.tag} GLOBALREF {grp.Lat}, {grp.Lon} -> CoordinateSharp UTM " +
                  $"{grp.UTMZone}{grp.UTMBand} {grp.UTMEasting:F2} {grp.UTMNorthing:F2}" +
                  $"  | pyproj said {site.anchor.utm_zone} {site.anchor.utm_easting:F2} " +
                  $"{site.anchor.utm_northing:F2}");
        if (grp.UTMZone != site.anchor.utm_zone || site.anchor.utm_zone != cfg.expectedUtmZone)
            Debug.LogError($"{cfg.tag} UTM ZONE MISMATCH: Unity says {grp.UTMZone}, the manifest " +
                           $"says {site.anchor.utm_zone}, this builder expects " +
                           $"{cfg.expectedUtmZone}. The scene frame is wrong. " +
                           "(Askö is 33; Beckholmen and Djurö are 34. Never inherit a zone.)");
        double de = grp.UTMEasting - site.anchor.utm_easting;
        double dn = grp.UTMNorthing - site.anchor.utm_northing;
        if (Math.Abs(de) > 1.0 || Math.Abs(dn) > 1.0)
            Debug.LogWarning($"{cfg.tag} CoordinateSharp/pyproj UTM differ by ({de:F2}, {dn:F2}) m.");
        else
            Debug.Log($"{cfg.tag} geodesy agrees to ({de:F3}, {dn:F3}) m");

        if (cfg.legacyOriginUtm != null && cfg.legacyOriginUtm.Length == 2)
        {
            double lde = cfg.legacyOriginUtm[0] - site.anchor.utm_easting;
            double ldn = cfg.legacyOriginUtm[1] - site.anchor.utm_northing;
            Debug.LogWarning($"{cfg.tag} FRAME NOTE: {cfg.legacyOriginWhat} uses an origin " +
                             $"{Math.Sqrt(lde * lde + ldn * ldn):F2} m away " +
                             $"(dE {lde:F1}, dN {ldn:F1}). Do not mix objects between those " +
                             "scenes and this one.");
        }

        // --- the terrains --------------------------------------------------------
        var mat = AssetDatabase.LoadAssetAtPath<Material>(MatTerrain);
        var bottom = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(
            PhysicMaterialPath(cfg.terrainPhysicMaterial));
        var made = new Dictionary<string, Terrain>();
        foreach (var t in site.tiles)
        {
            var go = Terrain.CreateTerrainGameObject(tds[t.name]);
            go.name = cfg.assetPrefix + "_" + t.name;
            go.transform.SetParent(root.transform, false);
            go.transform.localPosition = new Vector3(
                t.unity_position.x, t.unity_position.y, t.unity_position.z);
            go.transform.localScale = Vector3.one;   // never scale a georeferenced terrain
            var terr = go.GetComponent<Terrain>();
            if (mat != null) terr.materialTemplate = mat;
            // Without a physics material the TerrainCollider returns Sonar.cs's 0.5 default
            // and label 0, so the bottom reads as "something unidentified" on every ping.
            var tc = go.GetComponent<TerrainCollider>();
            if (tc != null && bottom != null) tc.sharedMaterial = bottom;

            // Per-cell bottom type, so sonar hardness depends on where the ray landed rather
            // than on the terrain's single PhysicsMaterial.
            var sm2 = LoadSplatManifest(cfg);
            if (sm2 != null && sm2.acoustics != null)
            {
                var acou = go.AddComponent<SeabedAcousticMap>();
                acou.classMapAssetPath =
                    cfg.dataDir + "/" + cfg.prefix + "_" + t.name + "_seabed_class.u8";
                acou.resolution = t.resolution;
                acou.classes = new[]
                {
                    MakeClass("land", sm2.acoustics.land),
                    MakeClass("bedrock", sm2.acoustics.bedrock),
                    MakeClass("sand", sm2.acoustics.sand),
                    MakeClass("clay", sm2.acoustics.clay),
                };
                acou.provenance =
                    "Bottom type INFERRED from slope, roughness and depth (Baltic erosion / " +
                    "transport / accumulation model) — not surveyed. Cells whose SHAPE is " +
                    "chart-extrapolated carry the 0x80 flag and are inferred twice over. " +
                    "Reflectivities are literature-typical, not measured at this site.";
            }

            var patch = go.AddComponent<AskoGapPatchLayer>();
            patch.maskAssetPath = cfg.dataDir + "/" + t.synthetic_mask;
            patch.maskResolution = t.mask_resolution;
            patch.showSyntheticPatches = true;
            patch.syntheticFraction = t.statistics.synthetic_fraction;
            if (!string.IsNullOrEmpty(t.hires_cutout_mask))
                patch.alwaysHoleMaskAssetPath = cfg.dataDir + "/" + t.hires_cutout_mask;
            patch.provenance =
                $"{t.statistics.real_fraction * 100f:F1}% measured, " +
                $"{t.statistics.synthetic_fraction * 100f:F1}% chart-extrapolated. " +
                site.chart_patch.method;
            made[t.name] = terr;
        }

        // Neighbours, so Unity stitches LOD across the tile seams instead of drawing a crack.
        if (cfg.wireNeighbours && site.tiles.Length > 1)
        {
            foreach (var t in site.tiles)
            {
                Terrain L = null, R = null, B = null, T = null;
                var n = t.name;                                  // SW, SE, NW, NE
                string we = n.Substring(1, 1), sn = n.Substring(0, 1);
                made.TryGetValue(sn + (we == "E" ? "W" : "E"), out var horiz);
                made.TryGetValue((sn == "N" ? "S" : "N") + we, out var vert);
                if (we == "E") L = horiz; else R = horiz;
                if (sn == "N") B = vert; else T = vert;
                made[n].SetNeighbors(L, T, R, B);
            }
        }

        if (!string.IsNullOrEmpty(cfg.gapPatchSemantics))
            Debug.Log($"{cfg.tag} GAP PATCH MEANS, AT THIS SITE: {cfg.gapPatchSemantics}");

        var dirp = Path.GetDirectoryName(AssetPathToFull(cfg.worldPrefabPath));
        if (!Directory.Exists(dirp)) Directory.CreateDirectory(dirp);
        var prefab = PrefabUtility.SaveAsPrefabAsset(root, cfg.worldPrefabPath);
        UnityEngine.Object.DestroyImmediate(root);
        Debug.Log($"{cfg.tag} world prefab -> " + cfg.worldPrefabPath);
        return prefab;
    }

    public static float TerrainHeightAt(GameObject world, float x, float z)
    {
        foreach (var terr in world.GetComponentsInChildren<Terrain>())
        {
            var p = terr.transform.position;
            var s = terr.terrainData.size;
            if (x < p.x || x > p.x + s.x || z < p.z || z > p.z + s.z) continue;
            return terr.SampleHeight(new Vector3(x, 0, z)) + p.y;
        }
        return float.NaN;
    }

    public static GameObject Place(CuratedSiteConfig cfg, string prefabPath, string name,
                                   Vector3 pos, float yawDeg = 0)
    {
        var p = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
        if (p == null) { Debug.LogWarning($"{cfg.tag} prefab not found: " + prefabPath); return null; }
        var go = (GameObject)PrefabUtility.InstantiatePrefab(p);
        go.name = name;
        go.transform.position = pos;
        go.transform.rotation = Quaternion.Euler(0, yawDeg, 0);
        return go;
    }

    public static void BuildScene(CuratedSiteConfig cfg, GameObject worldPrefab, SiteJ site)
    {
        // NEVER overwrite an existing scene: a curated scene carries HAND-PLACED objects
        // (Ivan placed the MMT Mini himself on 2026-08-27, on real multibeam bathymetry, and
        // a regenerate-from-scratch here would have silently deleted it). Terrain, prefab
        // and materials above always rebuild; the SCENE regenerates only when absent.
        if (File.Exists(AssetPathToFull(cfg.scenePath)))
        {
            Debug.Log($"{cfg.tag} scene exists — NOT regenerating {cfg.scenePath}. Terrains/prefab " +
                      "are rebuilt in place. " + cfg.sceneExistsHint +
                      "To truly start over, delete the scene file first.");
            return;
        }
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var world = (GameObject)PrefabUtility.InstantiatePrefab(worldPrefab);
        world.transform.position = Vector3.zero;
        var dir = Path.GetDirectoryName(cfg.scenePath);
        if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
        EditorSceneManager.SaveScene(scene, cfg.scenePath);
        Debug.Log($"{cfg.tag} scene -> {cfg.scenePath}  (bundle: {site.scenario_bundle})\n" +
                  $"       gap patch: {site.chart_patch.unity_toggle}" + cfg.sceneCreatedHint);
    }
}
