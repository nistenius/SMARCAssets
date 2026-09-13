using System;
using System.IO;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Builds and places the Askö pipeline — the first DETECTED FEATURE in this project: a thing
/// nobody drew, found in the 0.125 m Deep Vision patch, measured, reconstructed and put back
/// where the data says it is.
///
/// Generator: data-cube/scripts/asko-site/build_asko_pipe.py
/// Evidence:  docs/2026-08-30-asko-dv-phase0-audit.md §5, 2026-08-30-asko-run12-pipeline-sss.md
/// Write-up:  docs/2026-08-30-asko-pipe-object.md
///
/// WHAT THE PREFAB CLAIMS, AND WHAT IT DOES NOT
///   * The CENTRELINE is measured — 105.9 m of traced crest, positioned to decimetres, and
///     the run-12 side-scan puts the real feature within 0.02-0.18 m of it.
///   * The DIAMETER IS NOT. The DEM cannot separate a small pipe under a sediment drape from
///     a big one barely proud: the cross-section fit is flat from O.D. 0.10 m to 3.3 m. The
///     0.45 m modelled here is pinned by the side-scan's shadow geometry and is a FIT UNDER
///     ASSUMPTION. Anyone reading a diameter off this prefab is reading our guess.
///   * The MATERIAL is a modelling choice. `Steel` (Sonar reflectivity 0.95, label 5) because
///     a man-made target must out-reflect the seabed it lies on; nothing in the bathymetry
///     says what the thing is made of.
///   * The SECOND, BEADED LINE 5-7 m east is NOT in this prefab and must not be added to it.
///   * The object STOPS at both ends of the evidence. It does not run to the station.
///
/// Two menu items, in this order:
///   SMARC -> Build Asko Pipeline Prefab          (asset build; safe to re-run)
///   SMARC -> Add Asko Pipeline to Scene          (ADDITIVE; reuses by name; never moves a
///                                                 hand-moved instance, only says so)
/// </summary>
public static class AskoPipelineBuilder
{
    const string Dir = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/AskoPipeline";
    const string ObjPath = Dir + "/asko_pipeline.obj";
    const string TexPath = Dir + "/asko_pipeline_coating.png";
    const string MetaPath = Dir + "/asko_pipeline.json";
    const string PrefabPath = Dir + "/AskoPipeline.prefab";
    const string PhysDir = "Packages/com.smarc.assets/Runtime/Materials/Physic";
    const string SteelPhys = PhysDir + "/Steel.physicMaterial";
    public const string SceneObjectName = "AskoPipeline";

    // ---- the slice of asko_pipeline.json this editor needs -------------------
    [Serializable] public class GeomJ
    {
        public float polyline_length_m, straight_line_m, sinuosity, crown_epsilon_m;
        public int vertices, spans, triangles, radial_divisions;
    }
    [Serializable] public class MeasJ { public float diameter_adopted_m; public string diameter_basis; }
    [Serializable] public class PtJ { public double E, N; public float y_m; }
    [Serializable] public class EndJ { public PtJ south, north; }
    [Serializable] public class MetaJ
    {
        public string generated_utc, generator, feature_key, @object;
        public GeomJ geometry; public MeasJ measurement; public EndJ endpoints;
    }

    static MetaJ LoadMeta()
    {
        var p = AssetPathToFull(MetaPath);
        return File.Exists(p) ? JsonUtility.FromJson<MetaJ>(File.ReadAllText(p)) : null;
    }

    [MenuItem("SMARC/Build Asko Pipeline Prefab")]
    public static void Build()
    {
        var full = AssetPathToFull(ObjPath);
        if (!File.Exists(full))
        {
            Debug.LogError("[AskoPipe] asko_pipeline.obj not found at " + full +
                           " — run data-cube/scripts/asko-site/build_asko_pipe.py first.");
            return;
        }
        var meta = LoadMeta();

        // The OBJ is authored in SCENE metres already (Unity +X = UTM33N east, +Z = north),
        // with X pre-mirrored to cancel this importer's own negation. Any scale or axis
        // change here moves a georeferenced object, so pin them and say so.
        var imp = AssetImporter.GetAtPath(ObjPath) as ModelImporter;
        if (imp != null)
        {
            bool dirty = false;
            if (Math.Abs(imp.globalScale - 1f) > 1e-4f) { imp.globalScale = 1f; dirty = true; }
            if (!imp.importNormals.Equals(ModelImporterNormals.Import))
            { imp.importNormals = ModelImporterNormals.Import; dirty = true; }
            if (!imp.isReadable) { imp.isReadable = true; dirty = true; }   // sonar raycasts
            if (dirty) imp.SaveAndReimport();
        }

        var model = AssetDatabase.LoadAssetAtPath<GameObject>(ObjPath);
        if (model == null) { Debug.LogError("[AskoPipe] could not load " + ObjPath); return; }

        var coating = MakeMaterial("AskoPipeline_Coating", new Color(0.66f, 0.64f, 0.62f),
                                   0.72f, 0.0f,
                                   AssetDatabase.LoadAssetAtPath<Texture2D>(TexPath));

        var root = (GameObject)PrefabUtility.InstantiatePrefab(model);
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely,
                                           InteractionMode.AutomatedAction);
        root.name = SceneObjectName;
        root.transform.position = Vector3.zero;
        root.transform.rotation = Quaternion.identity;
        root.transform.localScale = Vector3.one;

        // ONE CHILD PER SPAN. The generator emits the sweep as OBJ groups span_00..span_NN of
        // 15 m each, so the importer gives us one GameObject per span for free. That is the
        // point: a later survey, a repair, or a better model can replace ONE span without
        // touching the rest, and a span can be switched off to ask "was that echo the pipe?"
        // the same way the Mini's activate/deactivate discriminator works (SETTLED §3f0e).
        int spans = 0;
        foreach (var r in root.GetComponentsInChildren<MeshRenderer>(true))
        {
            var mats = r.sharedMaterials;
            for (int i = 0; i < mats.Length; i++) mats[i] = coating;
            r.sharedMaterials = mats;

            if (r.gameObject.name.StartsWith("span_"))
            {
                int n;
                var tail = r.gameObject.name.Substring(5);
                r.gameObject.name = int.TryParse(tail, out n)
                    ? string.Format("Span_{0:00} (s {1}-{2} m)", n, n * 15, (n + 1) * 15)
                    : r.gameObject.name;
                spans++;
            }

            var mf = r.GetComponent<MeshFilter>();
            if (mf != null && mf.sharedMesh != null)
            {
                var mc = r.gameObject.GetComponent<MeshCollider>() ??
                         r.gameObject.AddComponent<MeshCollider>();
                mc.sharedMesh = mf.sharedMesh;
                mc.convex = false;                      // a curved swept tube is not convex
                mc.sharedMaterial = EnsureSteelPhysics();
            }
        }

        var prefab = PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        UnityEngine.Object.DestroyImmediate(root);
        AssetDatabase.SaveAssets();

        var b = BoundsOf(prefab);
        Debug.Log($"[AskoPipe] prefab -> {PrefabPath}\n" +
                  $"       {spans} spans, measured bounds {b.size} m, centre {b.center}\n" +
                  (meta != null
                      ? $"       generator: {meta.geometry.polyline_length_m:F2} m of centreline, " +
                        $"{meta.geometry.vertices} vertices, {meta.geometry.triangles} tris, " +
                        $"O.D. {meta.measurement.diameter_adopted_m:F3} m (FITTED, not measured)\n"
                      : "") +
                  "       physics material: Steel (Sonar reflectivity 0.95, label 5) — a " +
                  "MODELLING CHOICE, not a finding: nothing in the bathymetry says what this " +
                  "object is made of.");

        // A SILENT IMPORTER CHANGE IS THE ONE FAILURE THAT LOOKS LIKE NOTHING. The Mini
        // caught it on size; a pipeline is caught on LENGTH, because a mirrored or rescaled
        // sweep still looks like a pipe and just stops being where the survey put it.
        if (meta != null && meta.geometry != null)
        {
            float diag = new Vector2(b.size.x, b.size.z).magnitude;
            if (Mathf.Abs(diag - meta.geometry.straight_line_m) > 0.05f)
                Debug.LogError($"[AskoPipe] IMPORTED SIZE DISAGREES WITH THE GENERATOR — " +
                               $"end-to-end {diag:F3} m against {meta.geometry.straight_line_m:F3} m. " +
                               "The OBJ importer changed scale or axes; everything placed by " +
                               "coordinate is now wrong.");
        }
    }

    /// <summary>
    /// ADDITIVE placement into the OPEN scene. Reuses the object by name, never regenerates
    /// and never silently relocates: the scene is hand-edited and Ivan's edits win (§3f0 v5).
    /// </summary>
    [MenuItem("SMARC/Add Asko Pipeline to Scene (detected feature)")]
    public static void AddToScene()
    {
        var world = GameObject.Find("AskoCuratedWorld");
        if (world == null)
        {
            Debug.LogError("[AskoPipe] the open scene has no 'AskoCuratedWorld' — open " +
                           "Assets/Scenes/AskoCurated.unity first.");
            return;
        }
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null)
        {
            Debug.LogError("[AskoPipe] " + PrefabPath + " does not exist — run " +
                           "SMARC -> Build Asko Pipeline Prefab first.");
            return;
        }
        Ensure(world);
    }

    /// <summary>Idempotent add/refresh; also called from the Populate path.</summary>
    public static GameObject Ensure(GameObject world)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null) return null;
        var meta = LoadMeta();

        var go = GameObject.Find(SceneObjectName);
        if (go == null)
        {
            go = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            go.name = SceneObjectName;
            go.transform.SetParent(world.transform, false);
            go.transform.localPosition = Vector3.zero;
            go.transform.localRotation = Quaternion.identity;
            go.transform.localScale = Vector3.one;
            Debug.Log("[AskoPipe] added to the scene at the origin — THE MESH CARRIES THE " +
                      "GEOREFERENCE, so this object belongs at localPosition (0,0,0) under " +
                      "AskoCuratedWorld. Moving it moves the pipeline off the survey." +
                      (meta != null
                          ? $"\n       {meta.geometry.polyline_length_m:F1} m, " +
                            $"E {meta.endpoints.south.E:F1}/N {meta.endpoints.south.N:F1} -> " +
                            $"E {meta.endpoints.north.E:F1}/N {meta.endpoints.north.N:F1}, " +
                            $"crown y {meta.endpoints.south.y_m:F2} .. {meta.endpoints.north.y_m:F2} m"
                          : ""));
        }
        else
        {
            // HAND EDITS WIN. If it has been moved we say so loudly and leave it: the operator
            // may be testing something. What we never do is quietly drag it back, which would
            // make a georeference argument invisible.
            var lp = go.transform.localPosition;
            if (lp.magnitude > 0.05f || go.transform.localScale != Vector3.one)
                Debug.LogWarning($"[AskoPipe] existing '{SceneObjectName}' is at localPosition " +
                                 $"{lp} scale {go.transform.localScale} — NOT (0,0,0)/1. Left as " +
                                 "found (hand edits are authoritative), but it is no longer " +
                                 "where the survey puts it. Reset the transform to restore.");
            else
                Debug.Log($"[AskoPipe] existing '{SceneObjectName}' kept, transform is clean.");
        }
        return go;
    }

    static Bounds BoundsOf(GameObject go)
    {
        var rs = go.GetComponentsInChildren<MeshRenderer>(true);
        if (rs.Length == 0) return new Bounds();
        var b = rs[0].bounds;
        foreach (var r in rs) b.Encapsulate(r.bounds);
        return b;
    }

    static Material MakeMaterial(string name, Color col, float roughness, float metallic,
                                 Texture2D tex)
    {
        string path = Dir + "/" + name + ".mat";
        var m = AssetDatabase.LoadAssetAtPath<Material>(path);
        var shader = Shader.Find("HDRP/Lit") ?? Shader.Find("Standard");
        if (m == null) { m = new Material(shader); AssetDatabase.CreateAsset(m, path); }
        m.shader = shader;
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 1f - roughness);
        if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", 1f - roughness);
        if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", metallic);
        if (tex != null)
        {
            if (m.HasProperty("_BaseColorMap")) m.SetTexture("_BaseColorMap", tex);
            if (m.HasProperty("_MainTex")) m.SetTexture("_MainTex", tex);
            col = Color.white;                 // the atlas carries the colour; tinting doubles it
        }
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", col);
        if (m.HasProperty("_Color")) m.SetColor("_Color", col);
        EditorUtility.SetDirty(m);
        return m;
    }

    /// The sonar reads the physics material's NAME, so this asset must be called "Steel".
    static PhysicsMaterial EnsureSteelPhysics()
    {
        var p = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(SteelPhys);
        if (p != null) return p;
        var dir = AssetPathToFull(PhysDir);
        if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
        p = new PhysicsMaterial("Steel") { dynamicFriction = 0.4f, staticFriction = 0.5f,
                                           bounciness = 0.05f };
        AssetDatabase.CreateAsset(p, SteelPhys);
        Debug.Log("[AskoPipe] created " + SteelPhys +
                  " (Sonar maps the name 'Steel' to reflectivity 0.95)");
        return p;
    }

    static string AssetPathToFull(string assetPath)
    {
        var pi = UnityEditor.PackageManager.PackageInfo.FindForAssetPath(assetPath);
        if (pi != null)
            return Path.Combine(pi.resolvedPath,
                assetPath.Substring(("Packages/" + pi.name + "/").Length));
        return Path.Combine(Application.dataPath, assetPath.Substring("Assets/".Length));
    }
}
