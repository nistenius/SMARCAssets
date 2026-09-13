using System;
using System.IO;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Builds the MMT Mini Cooper search-target prefab from the model produced by
/// data-cube/scripts/asko-site/build_mmt_mini.py.
///
/// The car is a real object: a 1965 Austin Mini Cooper S that MMT painted yellow with a red
/// roof and their wordmark, sank at Askö as a sonar/search training target, and recovered
/// years later. This prefab is that car as it came up — rusted, roof bleached — because that
/// is what the photographs in the CAD folder show and what a search exercise would find.
///
/// Materials are created here rather than taken from the .mtl, so the sim gets HDRP Lit
/// materials with sensible roughness instead of the OBJ importer's guesses. The one that
/// matters beyond looks is the PHYSICS material: `Steel`, which `Sonar.SonarHit` reads by
/// NAME and maps to reflectivity 0.95 — a steel shell has to out-reflect the seabed it is
/// lying on, or the target the exercise is built around does not appear on the sonar.
///
/// Run from menu: SMARC -> Build MMT Mini Prefab
/// Safe to re-run; overwrites the generated assets.
/// </summary>
public static class MMTMiniBuilder
{
    const string Dir = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/MMTMini";
    const string ObjPath = Dir + "/mmt_mini.obj";
    const string PaintTex = Dir + "/mmt_mini_paint.png";
    const string PrefabPath = Dir + "/MMTMiniCooper.prefab";
    const string PhysDir = "Packages/com.smarc.assets/Runtime/Materials/Physic";
    const string SteelPhys = PhysDir + "/Steel.physicMaterial";

    /// The windows were not in the car when it was sunk. The part exists so it CAN be shown,
    /// and is named so nobody ticks it back on without knowing what they are claiming.
    const string Glazing = "Glazing (absent on the real car)";

    [Serializable] public class DimJ { public float width, height, length; }
    [Serializable]
    public class MetaJ
    {
        public string generated_utc, generator, @object, source_cad, frame, placement;
        public DimJ dimensions_m;
    }

    [MenuItem("SMARC/Build MMT Mini Prefab")]
    public static void Build()
    {
        var full = AssetPathToFull(ObjPath);
        if (!File.Exists(full))
        {
            Debug.LogError("[MMTMini] mmt_mini.obj not found at " + full +
                           " — run data-cube/scripts/asko-site/build_mmt_mini.py first.");
            return;
        }

        MetaJ meta = null;
        var mp = AssetPathToFull(Dir + "/mmt_mini.json");
        if (File.Exists(mp)) meta = JsonUtility.FromJson<MetaJ>(File.ReadAllText(mp));

        // The OBJ importer must not swap handedness or rescale: the model is authored in
        // metres, Y up, +Z = nose, and every decal is placed at a known model coordinate.
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
        if (model == null) { Debug.LogError("[MMTMini] could not load " + ObjPath); return; }

        // One material per Rhino layer group (see build_mmt_mini.py RHINO_LAYERS):
        // Kaross, Fönster, Däck, Default, Framlyktor, Framblinkers, Baklampor.
        var paint = MakeMaterial("MMTMini_Paint", new Color(0.84f, 0.75f, 0.18f), 0.55f, 0.10f,
                                 AssetDatabase.LoadAssetAtPath<Texture2D>(PaintTex));
        // `glazingMat`, not `glazing`: the GameObject holding the panes is already called
        // that below, and the collision is a compile error, not a shadowing warning.
        var glazingMat = MakeMaterial("MMTMini_Glazing", new Color(0.07f, 0.08f, 0.09f), 0.15f, 0f, null);
        var rubber = MakeMaterial("MMTMini_Rubber", new Color(0.045f, 0.045f, 0.045f), 0.92f, 0f, null);
        var trim = MakeMaterial("MMTMini_Trim", new Color(0.35f, 0.34f, 0.31f), 0.45f, 0.85f, null);
        var lamp = MakeMaterial("MMTMini_Lamp", new Color(0.70f, 0.70f, 0.66f), 0.25f, 0.2f, null);
        var amber = MakeMaterial("MMTMini_Amber", new Color(0.62f, 0.28f, 0.03f), 0.30f, 0f, null);
        var tail = MakeMaterial("MMTMini_Tail", new Color(0.45f, 0.04f, 0.05f), 0.30f, 0f, null);

        var root = (GameObject)PrefabUtility.InstantiatePrefab(model);
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely,
                                           InteractionMode.AutomatedAction);
        root.name = "MMTMiniCooper";
        root.transform.position = Vector3.zero;
        root.transform.rotation = Quaternion.identity;

        // Replace whatever the OBJ importer made, matching on the usemtl group names the
        // generator wrote. Matching by NAME rather than by submesh index because the
        // importer is free to reorder, and a silent reorder would paint the tyres yellow.
        int assigned = 0, unmatched = 0;
        GameObject glazing = null;
        foreach (var r in root.GetComponentsInChildren<MeshRenderer>(true))
        {
            var mats = r.sharedMaterials;
            string group = null;
            for (int i = 0; i < mats.Length; i++)
            {
                string n = mats[i] != null ? mats[i].name.ToLowerInvariant() : "";
                if (n.Contains("paint")) { mats[i] = paint; group = "Body_Kaross"; assigned++; }
                else if (n.Contains("glazing")) { mats[i] = glazingMat; group = Glazing; assigned++; }
                else if (n.Contains("rubber")) { mats[i] = rubber; group = "Tyres_Dack"; assigned++; }
                else if (n.Contains("trim")) { mats[i] = trim; group = "Trim_Bumpers_Rims"; assigned++; }
                else if (n.Contains("lamp")) { mats[i] = lamp; group = "Headlamps_Framlyktor"; assigned++; }
                else if (n.Contains("amber")) { mats[i] = amber; group = "Indicators_Framblinkers"; assigned++; }
                else if (n.Contains("tail")) { mats[i] = tail; group = "TailLamps_Baklampor"; assigned++; }
                else { unmatched++; Debug.LogWarning($"[MMTMini] unmatched material '{n}' on {r.name}"); }
            }
            r.sharedMaterials = mats;
            // The OBJ importer already split the groups into separate child GameObjects, so
            // each part is independently switchable — it just needed names that say what it
            // is rather than what the generator called it.
            if (group != null) r.gameObject.name = group;
            if (group == Glazing) glazing = r.gameObject;

            var mf = r.GetComponent<MeshFilter>();
            if (mf != null && mf.sharedMesh != null)
            {
                var mc = r.gameObject.GetComponent<MeshCollider>();
                if (mc == null) mc = r.gameObject.AddComponent<MeshCollider>();
                mc.sharedMesh = mf.sharedMesh;
                mc.convex = false;
                mc.sharedMaterial = EnsureSteelPhysics();
            }
        }
        Debug.Log($"[MMTMini] materials assigned: {assigned}, unmatched: {unmatched}");

        // THE CAR HAD NO GLASS. Ivan, 2026-08-27 — the windows were already out when it went
        // down, and every recovery photograph shows open apertures. So the glazing ships
        // DISABLED: the honest default is the car as it actually lay on the seabed, and
        // anyone who wants glass can tick one box.
        //
        // Disabling the GameObject takes its MeshCollider with it, which is the point rather
        // than a side effect: a pane that is not there must not return a sonar ping or stop a
        // raycast either. Leaving the renderer off but the collider on would have produced an
        // invisible window that the sonar could still see — the same class of lie as an
        // invented seabed answering a range query.
        if (glazing != null)
        {
            glazing.SetActive(false);
            Debug.Log($"[MMTMini] '{Glazing}' is a separate child and ships DISABLED — the " +
                      "real car had no windows. Tick it in the Inspector to show glass; its " +
                      "MeshCollider follows the toggle, so with it off nothing blocks a " +
                      "sonar ray through the cabin.");
        }
        else
        {
            Debug.LogWarning("[MMTMini] no glazing child was found to disable — the OBJ's " +
                             "'opening' group did not import as its own object.");
        }

        var prefab = PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        UnityEngine.Object.DestroyImmediate(root);
        AssetDatabase.SaveAssets();

        var b = prefab.GetComponentInChildren<MeshRenderer>() != null
            ? BoundsOf(prefab) : new Bounds();
        Debug.Log($"[MMTMini] prefab -> {PrefabPath}\n" +
                  $"       measured bounds {b.size} m " +
                  (meta != null && meta.dimensions_m != null
                      ? $"| generator said {meta.dimensions_m.width:F3} x " +
                        $"{meta.dimensions_m.height:F3} x {meta.dimensions_m.length:F3}"
                      : "") + "\n" +
                  $"       frame: {(meta != null ? meta.frame : "metres, Y up, +Z = nose")}\n" +
                  $"       {(meta != null ? meta.placement : "")}");
        if (meta != null && meta.dimensions_m != null)
        {
            // A silent handedness flip or unit change on import would show up here and
            // nowhere else until someone wondered why the car was 3 mm long.
            if (Mathf.Abs(b.size.x - meta.dimensions_m.width) > 0.02f ||
                Mathf.Abs(b.size.y - meta.dimensions_m.height) > 0.02f ||
                Mathf.Abs(b.size.z - meta.dimensions_m.length) > 0.02f)
                Debug.LogError("[MMTMini] IMPORTED SIZE DISAGREES WITH THE GENERATOR — the " +
                               "OBJ importer changed scale or axes. Everything placed by " +
                               "coordinate will be wrong.");
        }
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
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", col);
        if (m.HasProperty("_Color")) m.SetColor("_Color", col);
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 1f - roughness);
        if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", 1f - roughness);
        if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", metallic);
        if (tex != null)
        {
            if (m.HasProperty("_BaseColorMap")) m.SetTexture("_BaseColorMap", tex);
            if (m.HasProperty("_MainTex")) m.SetTexture("_MainTex", tex);
            // The atlas carries the paint; tinting it as well would double-darken.
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", Color.white);
            if (m.HasProperty("_Color")) m.SetColor("_Color", Color.white);
        }
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
        Debug.Log("[MMTMini] created " + SteelPhys +
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
