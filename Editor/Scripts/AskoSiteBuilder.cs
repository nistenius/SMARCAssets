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
///   AskoCurated_<tile>.asset   four TerrainDatas, 4096 m each at 1 m (4097^2), 2x2
///   AskoCuratedWorld.prefab    cloned from AskoWorld.prefab so Ocean/Sky/Sun come from the
///                              proven Askö setup; the GLOBALREF and terrains are new
///   AskoCurated.unity          the scene
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
/// builder re-asserts it below so the discrepancy cannot quietly disappear.
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
    const string PrefabDir = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/GeoReferenced";
    const string AskoWorldPath = PrefabDir + "/AskoWorld.prefab";
    const string WorldPrefabPath = PrefabDir + "/AskoCuratedWorld.prefab";
    const string ScenePath = "Assets/Scenes/AskoCurated.unity";
    const string MatDir = "Packages/com.smarc.assets/Runtime/Materials";
    const string MatTerrain = MatDir + "/DefaultHDTerrainLitMaterial.mat";
    const string TileMaterialPath = MatDir + "/2DMapMaterial.mat";
    const string MudPhysicPath = MatDir + "/Physic/Mud.physicMaterial";

    // The legacy frame, measured from AskoWorld.prefab (see the class comment).
    const double LegacyOriginUtmE = 653020.81, LegacyOriginUtmN = 6525308.63;

    // ---- mirror of asko_site.json -------------------------------------------
    [Serializable] public class Vec3J { public float x, y, z; }
    [Serializable]
    public class AnchorJ
    {
        public double lat, lon; public string utm_epsg; public int utm_zone;
        public double utm_easting, utm_northing;
    }
    [Serializable]
    public class TileStatsJ
    {
        public float min_m, max_m, water_fraction, real_fraction, synthetic_fraction;
    }
    [Serializable]
    public class TileJ
    {
        public string name, heightmap, synthetic_mask, source_mask;
        public int resolution, mask_resolution;
        public float terrain_size_x_m, terrain_size_z_m, terrain_size_y_m, terrain_base_y_m;
        public Vec3J unity_position;
        public TileStatsJ statistics;
        // Added 2026-08-30 by the DV hi-res ingest. Both are OPTIONAL and absent on a tile
        // that carries no nested patch, so a manifest written before the ingest still loads
        // and builds exactly as it did.
        //   heightmap_collared  the same tile WITH the seam collar blended in. The
        //                       un-collared file stays on disk untouched and stays named in
        //                       `heightmap`, so removing the ingest is deleting a file.
        //   hires_cutout_mask   1 = this 1 m quad is fully covered by the 0.125 m patch and
        //                       must become a terrain hole, or the two surfaces both render.
        public string heightmap_collared, hires_cutout_mask;
    }

    // ---- mirror of asko_dvhires.json (the nested 0.125 m patch) --------------
    [Serializable]
    public class PatchStatsJ { public float min_m, max_m; }
    [Serializable]
    public class PatchJ
    {
        public string generated_utc, generator, source_key, name;
        public string heightmap, source_mask, hole_mask, seabed_class_map, control_map;
        public int resolution, mask_resolution;
        public float terrain_size_x_m, terrain_size_z_m, terrain_size_y_m, terrain_base_y_m;
        public Vec3J unity_position;
        public PatchStatsJ statistics;
    }
    [Serializable]
    public class CoverageJ { public long cells, real; public float real_fraction, synthetic_fraction; }
    [Serializable]
    public class ChartPatchJ
    {
        public bool enabled_at_build; public long cells, cells_without_chart;
        public float fraction_of_scene; public string method, source, rights, unity_toggle;
    }
    [Serializable]
    public class SiteJ
    {
        public string generated_utc, generator, site, scenario_bundle;
        public AnchorJ anchor; public TileJ[] tiles;
        public CoverageJ coverage; public ChartPatchJ chart_patch;
        public string[] priority; public string priority_rule;
    }
    [Serializable]
    public class SplatLayerJ { public string name, texture, means; public float tile_size_m; }
    [Serializable]
    public class AcousticClassJ
    {
        public float reflectivity; public int label; public string physics_material, note;
    }
    [Serializable]
    public class AcousticsJ
    {
        public AcousticClassJ bedrock, sand, clay, land;
    }
    [Serializable]
    public class SplatJ
    {
        public int control_resolution; public float tile_size_m, ortho_tile_size_m;
        public SplatLayerJ[] layers; public AcousticsJ acoustics;
    }

    [MenuItem("SMARC/Build Asko Curated Site")]
    public static void Build()
    {
        try
        {
            splatCache = null;              // re-read the manifest on every build
            var site = LoadSite();
            var tds = new Dictionary<string, TerrainData>();
            foreach (var t in site.tiles) tds[t.name] = BuildTerrain(site, t);
            var world = BuildWorldPrefab(site, tds);
            BuildScene(world, site);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("[Asko] DONE. Scene: " + ScenePath);
        }
        catch (Exception e)
        {
            Debug.LogError("[Asko] build failed: " + e);
            throw;
        }
    }

    static string AssetPathToFull(string assetPath)
    {
        var pi = UnityEditor.PackageManager.PackageInfo.FindForAssetPath(assetPath);
        if (pi != null)
            return Path.Combine(pi.resolvedPath, assetPath.Substring(("Packages/" + pi.name + "/").Length));
        return Path.Combine(Application.dataPath, assetPath.Substring("Assets/".Length));
    }

    // ------------------------------------------------------------------------
    // WHERE THE CURATED DATA LIVES (rehomed 2026-08-27, Ivan's call):
    // the OCEANVERSE scenario bundle in the SMDS cloud store is the SOURCE OF
    // TRUTH for every curated artefact. This builder is the smarcsim import
    // step: it reads build-time payloads (heightmaps, masks, splat controls)
    // straight from the bundle, and the package keeps only what the ENGINE
    // needs at runtime or by GUID (ortho textures, seabed_class + synthetic
    // masks, the small manifests) as a cache the bundle can always regenerate.
    // Earlier the bundle pointed backwards into this package — the consumer's
    // copy was posing as the source, which is exactly what the data-cube
    // concept exists to prevent.
    // ------------------------------------------------------------------------
    const string BundleRel = "../smds-cloud-store/scenario-bundles/ov-site-Asko-curated-v1/payload";

    /// <summary>Full path of a curated payload file: the bundle first (source of
    /// truth), the package cache second (runtime copies). Errors name both.</summary>
    static string PayloadFull(string fileName)
    {
        // Application.dataPath = <project>/Assets; the workspace root is two up.
        var projectRoot = Path.GetDirectoryName(Application.dataPath);
        var bundle = Path.GetFullPath(Path.Combine(projectRoot, BundleRel, fileName));
        if (File.Exists(bundle)) return bundle;
        var cache = AssetPathToFull(DataDir + "/" + fileName);
        if (File.Exists(cache))
        {
            Debug.LogWarning($"[Asko] {fileName}: not in the scenario bundle ({bundle}) — " +
                             "using the engine cache copy. The bundle is supposed to be " +
                             "the source of truth; re-run the generators.");
            return cache;
        }
        throw new FileNotFoundException(
            $"{fileName} found neither in the scenario bundle ({bundle}) nor the engine " +
            $"cache ({cache}) — run data-cube/scripts/asko-site/build_heightmap.py first.");
    }

    static SiteJ LoadSite()
    {
        var p = PayloadFull("asko_site.json");
        var s = JsonUtility.FromJson<SiteJ>(File.ReadAllText(p));
        if (s == null || s.tiles == null || s.tiles.Length == 0 || s.anchor == null)
            throw new Exception("asko_site.json did not parse into a site manifest");
        Debug.Log($"[Asko] manifest {s.generated_utc}\n" +
                  $"       {s.tiles.Length} tiles; REAL {s.coverage.real_fraction * 100f:F2}% of the " +
                  $"scene, SYNTHETIC {s.coverage.synthetic_fraction * 100f:F2}% " +
                  $"({s.chart_patch.cells:N0} chart-extrapolated cells)\n" +
                  $"       priority: {string.Join(" > ", s.priority)}\n" +
                  $"       {s.priority_rule}");
        return s;
    }

    static TerrainData BuildTerrain(SiteJ site, TileJ t)
    {
        // The collared variant, when the manifest names one and the file is there. Same
        // base/size_y, so it is a drop-in: it is the un-collared grid plus a correction
        // raster that is kept separately and can be subtracted straight back out
        // (test_ingest_reversibility.py guard 2). If it is missing we build the un-collared
        // tile and SAY SO, rather than failing — the seam is cosmetic, a broken build is not.
        var raw = PayloadFull(t.heightmap);
        if (!string.IsNullOrEmpty(t.heightmap_collared))
        {
            try
            {
                raw = PayloadFull(t.heightmap_collared);
                Debug.Log($"[Asko] {t.name}: using the SEAM-COLLARED heightmap " +
                          $"{t.heightmap_collared} (the un-collared {t.heightmap} is " +
                          "untouched on disk).");
            }
            catch (FileNotFoundException)
            {
                Debug.LogWarning($"[Asko] {t.name}: manifest names {t.heightmap_collared} " +
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

        string assetPath = DataDir + "/AskoCurated_" + t.name + ".asset";
        var td = AssetDatabase.LoadAssetAtPath<TerrainData>(assetPath);
        bool isNew = td == null;
        if (isNew) td = new TerrainData();
        td.name = "AskoCurated_" + t.name;
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

        ApplySplat(td, t);

        Debug.Log($"[Asko] {t.name}: {res}x{res} @1 m, size {td.size}, base {t.terrain_base_y_m:F1} m, " +
                  $"range {t.statistics.min_m:F2}..{t.statistics.max_m:F2} m, " +
                  $"{t.statistics.water_fraction * 100f:F1}% water, " +
                  $"REAL {t.statistics.real_fraction * 100f:F1}% / " +
                  $"SYNTHETIC {t.statistics.synthetic_fraction * 100f:F1}%");
        return td;
    }

    /// Two layers: measured seabed, and the chart-extrapolated patch, so the difference is
    /// visible without opening a mask file.
    static void ApplySplat(TerrainData td, TileJ t)
    {
        var manifestPath = AssetPathToFull(DataDir + "/asko_splat.json");
        if (!File.Exists(manifestPath))
        {
            Debug.LogWarning("[Asko] no asko_splat.json — terrain left untextured, so REAL and " +
                             "SYNTHETIC will look identical. Run build_splatmap.py.");
            return;
        }
        var sm = JsonUtility.FromJson<SplatJ>(File.ReadAllText(manifestPath));
        var layers = new TerrainLayer[sm.layers.Length];
        for (int i = 0; i < sm.layers.Length; i++)
        {
            var L = sm.layers[i];
            // "PER_TILE" = the orthophoto, which is a different image for each terrain and is
            // draped ONCE across it (tileSize = terrain size) rather than repeated. Everything
            // else is a small repeating material texture shared by all four tiles.
            bool perTile = L.texture == "PER_TILE";
            string texPath = perTile ? DataDir + "/asko_" + t.name + "_ortho.png"
                                     : DataDir + "/" + L.texture;
            // Unity's default maxTextureSize is 2048, so a 4096 orthophoto would be silently
            // halved to 2 m/px on import — the imagery would still LOOK fine and would simply
            // be half the resolution we went and fetched. Set it explicitly.
            if (perTile) EnsureMaxTextureSize(texPath, 4096);
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(texPath);
            if (tex == null)
            {
                Debug.LogWarning($"[Asko] missing texture {texPath} — layer '{L.name}' will be " +
                                 "flat. Run build_ortho.py / build_splatmap.py.");
            }
            float tile = perTile ? sm.ortho_tile_size_m
                                 : (L.tile_size_m > 0 ? L.tile_size_m : sm.tile_size_m);
            var layerPath = perTile ? DataDir + "/Asko_Ortho_" + t.name + ".terrainlayer"
                                    : DataDir + "/Asko_" + L.name + ".terrainlayer";
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
        try { ctrlPath = PayloadFull("asko_" + t.name + "_splat.u8"); }
        catch (FileNotFoundException e)
        { Debug.LogWarning("[Asko] control map missing: " + e.Message); return; }
        var raw = File.ReadAllBytes(ctrlPath);
        int n = sm.control_resolution;
        long expect = (long)n * n * layers.Length;
        if (raw.LongLength != expect)
        {
            Debug.LogError($"[Asko] {t.name} control map is {raw.LongLength} bytes, expected " +
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

    static void EnsureMaxTextureSize(string assetPath, int max)
    {
        var imp = AssetImporter.GetAtPath(assetPath) as TextureImporter;
        if (imp == null) return;
        if (imp.maxTextureSize >= max) return;
        imp.maxTextureSize = max;
        imp.SaveAndReimport();
        Debug.Log($"[Asko] {assetPath}: maxTextureSize -> {max}");
    }

    static SplatJ splatCache;
    static SplatJ LoadSplatManifest()
    {
        if (splatCache != null) return splatCache;
        string p;
        try { p = PayloadFull("asko_splat.json"); }
        catch (FileNotFoundException) { return null; }
        splatCache = JsonUtility.FromJson<SplatJ>(File.ReadAllText(p));
        return splatCache;
    }

    static SeabedAcousticMap.ClassEntry MakeClass(string name, AcousticClassJ j)
    {
        return new SeabedAcousticMap.ClassEntry
        {
            name = name,
            reflectivity = j != null ? j.reflectivity : 0.5f,
            label = j != null ? j.label : 1,
        };
    }

    static GameObject BuildWorldPrefab(SiteJ site, Dictionary<string, TerrainData> tds)
    {
        var asko = AssetDatabase.LoadAssetAtPath<GameObject>(AskoWorldPath);
        if (asko == null) throw new FileNotFoundException("AskoWorld.prefab not found at " + AskoWorldPath);

        var root = (GameObject)PrefabUtility.InstantiatePrefab(asko);
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        root.name = "AskoCuratedWorld";
        root.transform.position = Vector3.zero;

        // Keep the environment essentials; drop the legacy GLOBALREF, which carries the
        // 2105 m-away frame. Two GlobalReferencePoints disagreeing about where the world is
        // is exactly the ambiguity this project has paid for before.
        var keep = new HashSet<string> { "Ocean", "Sun", "Sky and Fog Global Volume" };
        for (int i = root.transform.childCount - 1; i >= 0; i--)
        {
            var c = root.transform.GetChild(i).gameObject;
            if (!keep.Contains(c.name)) UnityEngine.Object.DestroyImmediate(c);
        }
        // The Ocean must sit at exactly (0,0,0): the water plane is the vertical datum every
        // depth in this pipeline is referenced to, and raising it "for realism" is what sent
        // SAM to 67 m/s on 2026-08-18 (SETTLED §3s).
        var ocean = root.transform.Find("Ocean");
        if (ocean != null && ocean.localPosition != Vector3.zero)
        {
            Debug.LogWarning($"[Asko] Ocean was at {ocean.localPosition}; forcing (0,0,0) — " +
                             "the water plane is the datum, not a dressing choice.");
            ocean.localPosition = Vector3.zero;
        }

        // --- GLOBALREF at the scene origin, carrying the registered site anchor ----
        var gref = new GameObject("GLOBALREF - AskoSiteOrigin");
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
            Debug.LogWarning("[Asko] tile material not found at " + TileMaterialPath);

        // Cross-check Unity's own geodesy against the pipeline's pyproj — two independent
        // implementations, one number. A silent disagreement here offsets everything.
        Debug.Log($"[Asko] GLOBALREF {grp.Lat}, {grp.Lon} -> CoordinateSharp UTM " +
                  $"{grp.UTMZone}{grp.UTMBand} {grp.UTMEasting:F2} {grp.UTMNorthing:F2}" +
                  $"  | pyproj said {site.anchor.utm_zone} {site.anchor.utm_easting:F2} " +
                  $"{site.anchor.utm_northing:F2}");
        if (grp.UTMZone != site.anchor.utm_zone)
            Debug.LogError($"[Asko] UTM ZONE MISMATCH: Unity says {grp.UTMZone}, pipeline assumed " +
                           $"{site.anchor.utm_zone}. The scene frame is wrong. (Askö is 33.)");
        double de = grp.UTMEasting - site.anchor.utm_easting;
        double dn = grp.UTMNorthing - site.anchor.utm_northing;
        if (Math.Abs(de) > 1.0 || Math.Abs(dn) > 1.0)
            Debug.LogWarning($"[Asko] CoordinateSharp/pyproj UTM differ by ({de:F2}, {dn:F2}) m.");
        else
            Debug.Log($"[Asko] geodesy agrees to ({de:F3}, {dn:F3}) m");

        double lde = LegacyOriginUtmE - site.anchor.utm_easting;
        double ldn = LegacyOriginUtmN - site.anchor.utm_northing;
        Debug.LogWarning($"[Asko] FRAME NOTE: the legacy Askö assets (AskoWorld/AskoEvolo/Asko.prefab) " +
                         $"use an origin {Math.Sqrt(lde * lde + ldn * ldn):F0} m away " +
                         $"(dE {lde:F1}, dN {ldn:F1}). Do not mix objects between those scenes and this one.");

        // --- the four terrains ---------------------------------------------------
        var mat = AssetDatabase.LoadAssetAtPath<Material>(MatTerrain);
        var mud = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(MudPhysicPath);
        var made = new Dictionary<string, Terrain>();
        foreach (var t in site.tiles)
        {
            var go = Terrain.CreateTerrainGameObject(tds[t.name]);
            go.name = "AskoCurated_" + t.name;
            go.transform.SetParent(root.transform, false);
            go.transform.localPosition = new Vector3(
                t.unity_position.x, t.unity_position.y, t.unity_position.z);
            go.transform.localScale = Vector3.one;   // never scale a georeferenced terrain
            var terr = go.GetComponent<Terrain>();
            if (mat != null) terr.materialTemplate = mat;
            // Without a physics material the TerrainCollider returns Sonar.cs's 0.5 default
            // and label 0, so the bottom reads as "something unidentified" on every ping.
            var tc = go.GetComponent<TerrainCollider>();
            if (tc != null && mud != null) tc.sharedMaterial = mud;

            // Per-cell bottom type, so sonar hardness depends on where the ray landed rather
            // than on the terrain's single PhysicsMaterial. Values come from the generator's
            // manifest; nothing acoustic is hardcoded here.
            var sm2 = LoadSplatManifest();
            if (sm2 != null && sm2.acoustics != null)
            {
                var acou = go.AddComponent<SeabedAcousticMap>();
                acou.classMapAssetPath = DataDir + "/asko_" + t.name + "_seabed_class.u8";
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
            patch.maskAssetPath = DataDir + "/" + t.synthetic_mask;
            patch.maskResolution = t.mask_resolution;
            patch.showSyntheticPatches = true;
            patch.syntheticFraction = t.statistics.synthetic_fraction;
            // Under a nested hi-res patch this tile must not draw at all: two coincident
            // surfaces z-fight AND return two echoes. Same component, so there is still
            // exactly ONE writer of this terrain's hole state.
            if (!string.IsNullOrEmpty(t.hires_cutout_mask))
                patch.alwaysHoleMaskAssetPath = DataDir + "/" + t.hires_cutout_mask;
            patch.provenance =
                $"{t.statistics.real_fraction * 100f:F1}% measured, " +
                $"{t.statistics.synthetic_fraction * 100f:F1}% chart-extrapolated. " +
                site.chart_patch.method;
            made[t.name] = terr;
        }

        // Neighbours, so Unity stitches LOD across the tile seams instead of drawing a crack.
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

        var dirp = Path.GetDirectoryName(AssetPathToFull(WorldPrefabPath));
        if (!Directory.Exists(dirp)) Directory.CreateDirectory(dirp);
        var prefab = PrefabUtility.SaveAsPrefabAsset(root, WorldPrefabPath);
        UnityEngine.Object.DestroyImmediate(root);
        Debug.Log("[Asko] world prefab -> " + WorldPrefabPath);
        return prefab;
    }

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

    const string VehiclePrefab = "Packages/com.smarc.assets/Runtime/Prefabs/sam2.2.prefab";
    const string VehicleName = "sam_auv_v1";   // prefixes every ROS topic and tf frame
    const string GuiPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/SmarcGUI/GUI.prefab";
    const string StationPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/datacube_station_01.prefab";
    const string MiniPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/MMTMini/MMTMiniCooper.prefab";

    static float TerrainHeightAt(GameObject world, float x, float z)
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

    static GameObject Place(string prefabPath, string name, Vector3 pos, float yawDeg = 0)
    {
        var p = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
        if (p == null) { Debug.LogWarning("[Asko] prefab not found: " + prefabPath); return null; }
        var go = (GameObject)PrefabUtility.InstantiatePrefab(p);
        go.name = name;
        go.transform.position = pos;
        go.transform.rotation = Quaternion.Euler(0, yawDeg, 0);
        return go;
    }

    static void BuildScene(GameObject worldPrefab, SiteJ site)
    {
        // NEVER overwrite an existing scene: AskoCurated.unity carries HAND-PLACED objects
        // (Ivan placed the MMT Mini himself on 2026-08-27, on real multibeam bathymetry, and
        // a regenerate-from-scratch here would have silently deleted it). Terrain, prefab
        // and materials above always rebuild; the SCENE regenerates only when absent.
        if (File.Exists(AssetPathToFull(ScenePath)))
        {
            Debug.Log($"[Asko] scene exists — NOT regenerating {ScenePath}. Terrains/prefab " +
                      "are rebuilt in place. To add the mission set (vehicle, GUI, station, " +
                      "sonar HUDs, hoop) run SMARC -> Populate Asko Scene. To truly start " +
                      "over, delete the scene file first.");
            return;
        }
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        var world = (GameObject)PrefabUtility.InstantiatePrefab(worldPrefab);
        world.transform.position = Vector3.zero;
        var dir = Path.GetDirectoryName(ScenePath);
        if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
        EditorSceneManager.SaveScene(scene, ScenePath);
        Debug.Log($"[Asko] scene -> {ScenePath}  (bundle: {site.scenario_bundle})\n" +
                  $"       gap patch: {site.chart_patch.unity_toggle}\n" +
                  "       now run SMARC -> Populate Asko Scene for the mission set.");
    }

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
