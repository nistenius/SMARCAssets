using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Builds the Kristineberg experimental algae farm as a prefab, from the surveyed buoy
/// positions in kristineberg_unity.json.
///
/// SONAR-TRUE BY CONSTRUCTION. Measured in Sonar.cs 2026-08-16: return intensity and material
/// label come from the collider's PHYSICS MATERIAL NAME, and the rays use
/// QueryParameters.Default — no layer mask, so every collider is already sonar-visible. So the
/// only thing that matters is that each part has a collider carrying the right physics
/// material: "Buoy" (reflectivity 0.99, label 3), "Rope" (0.4, label 4), "Algae" (0.25,
/// label 2). Those tables already existed; Buoy.physicMaterial and Algae.physicMaterial did
/// not, and were created in the same session.
///
/// Topology follows Ivan's own papers rather than a guess:
///   * two CULTURE LINES (the long N-S sides), each mooring buoy -> intermediate buoy ->
///     mooring buoy, held at ~2 m depth — SSS-SLAM paper Fig. 2
///   * corner buoys moored to seabed concrete blocks — sensors-22-05064 §5
///   * Saccharina blades hanging from the culture lines
/// M1/M2 are the intermediate buoys because they sit at EXACTLY t=0.500 on their sides.
///
/// Run from menu: SMARC -> Build Kristineberg Algae Farm
/// </summary>
public static class KristinebergFarmBuilder
{
    const string DataDir = "Packages/com.smarc.assets/Runtime/Terrain/Kristineberg";
    const string PrefabPath = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/AlgaeFarm/KristinebergAlgaeFarm.prefab";
    const string PhysDir = "Packages/com.smarc.assets/Runtime/Materials/Physic";
    const string BuoyMeshPath = "Packages/com.smarc.assets/Runtime/Models/algae_farm/a0_buoy.obj";
    const string MatDir = "Packages/com.smarc.assets/Runtime/Materials";

    // Geometry of the parts. Rope radius is what the sonar actually sees, so it is a
    // modelling decision, not decoration: 25 mm is a realistic culture-rope diameter.
    const float RopeRadius = 0.025f;
    const float MooringRadius = 0.04f;
    const float RopeSegmentLen = 2.0f;     // one collider per 2 m of rope
    const float BuoyRadius = 0.30f;

    [Serializable] public class FarmBuoy { public string name; public float x, z, seabed; public bool moored, intermediate; }
    [Serializable] public class FarmLine { public string[] nodes; }
    [Serializable]
    public class Farm
    {
        public float ropeDepth, bladeLength, bladesPerMetre;
        public FarmBuoy[] buoys;
        public FarmLine[] cultureLines, crossLines;
    }
    [Serializable] public class Root { public Farm farm; }

    [MenuItem("SMARC/Build Kristineberg Algae Farm")]
    public static void Build()
    {
        var path = KristinebergSiteBuilder.AssetPathToFullPublic(DataDir + "/kristineberg_unity.json");
        if (!File.Exists(path))
        { Debug.LogError("[Farm] no manifest at " + path + " — run make_unity_manifest.py"); return; }
        var farm = JsonUtility.FromJson<Root>(File.ReadAllText(path)).farm;
        if (farm == null || farm.buoys == null || farm.buoys.Length == 0)
        { Debug.LogError("[Farm] manifest has no farm block — regenerate it"); return; }

        var physBuoy = LoadPhys("Buoy");
        var physRope = LoadPhys("Rope");
        var physAlgae = LoadPhys("Algae");
        if (physBuoy == null || physRope == null || physAlgae == null) return;

        var root = new GameObject("KristinebergAlgaeFarm");
        var byName = farm.buoys.ToDictionary(b => b.name, b => b);
        Vector3 Pos(string n) => new Vector3(byName[n].x, 0f, byName[n].z);

        // --- buoys -----------------------------------------------------------
        var buoyMesh = AssetDatabase.LoadAssetAtPath<GameObject>(BuoyMeshPath);
        var buoyHolder = new GameObject("Buoys"); buoyHolder.transform.SetParent(root.transform, false);
        foreach (var b in farm.buoys)
        {
            var go = new GameObject(b.name);
            go.transform.SetParent(buoyHolder.transform, false);
            go.transform.localPosition = new Vector3(b.x, 0f, b.z);
            // Intermediate buoys are visibly smaller than the mooring buoys in the papers'
            // Fig. 2 (white vs yellow) and in the 2022 nadir frame.
            float r = b.intermediate ? BuoyRadius * 0.6f : BuoyRadius;
            if (buoyMesh != null)
            {
                var vis = (GameObject)PrefabUtility.InstantiatePrefab(buoyMesh);
                vis.name = "mesh";
                vis.transform.SetParent(go.transform, false);
                vis.transform.localScale = Vector3.one * (r / 0.135f);   // a0_buoy is 0.27 m across
            }
            else
            {
                var vis = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                vis.name = "mesh"; vis.transform.SetParent(go.transform, false);
                vis.transform.localScale = Vector3.one * (2f * r);
                UnityEngine.Object.DestroyImmediate(vis.GetComponent<Collider>());
            }
            var col = go.AddComponent<SphereCollider>();
            col.radius = r;
            col.sharedMaterial = physBuoy;          // <- this is what makes it read as a buoy on sonar
        }

        // --- ropes -----------------------------------------------------------
        var ropeHolder = new GameObject("Ropes"); ropeHolder.transform.SetParent(root.transform, false);
        int ropeSegs = 0;
        foreach (var line in farm.cultureLines)
            ropeSegs += MakeRope(ropeHolder.transform, "culture", line.nodes, Pos, farm.ropeDepth,
                                 RopeRadius, physRope);
        foreach (var line in farm.crossLines)
            ropeSegs += MakeRope(ropeHolder.transform, "cross", line.nodes, Pos, farm.ropeDepth,
                                 RopeRadius, physRope);

        // --- moorings: corner buoy -> concrete block on the seabed -----------
        var moorHolder = new GameObject("Moorings"); moorHolder.transform.SetParent(root.transform, false);
        int moorSegs = 0;
        foreach (var b in farm.buoys.Where(b => b.moored))
        {
            // Anchors are offset outboard from the farm centre, as a real mooring spread is.
            var c = new Vector2(farm.buoys.Average(q => q.x), farm.buoys.Average(q => q.z));
            var dir = (new Vector2(b.x, b.z) - c).normalized;
            var top = new Vector3(b.x, 0f, b.z);
            var anchor = new Vector3(b.x + dir.x * 8f, b.seabed, b.z + dir.y * 8f);
            moorSegs += MakeSegments(moorHolder.transform, $"mooring_{b.name}", top, anchor,
                                     MooringRadius, physRope);
            var blk = GameObject.CreatePrimitive(PrimitiveType.Cube);
            blk.name = $"anchor_{b.name}";
            blk.transform.SetParent(moorHolder.transform, false);
            blk.transform.localPosition = anchor + new Vector3(0, 0.25f, 0);
            blk.transform.localScale = new Vector3(0.8f, 0.5f, 0.8f);
            blk.GetComponent<Collider>().sharedMaterial = LoadPhys("Rock");   // concrete reads as rock
        }

        // --- algae hanging from the culture lines ----------------------------
        var algaeHolder = new GameObject("Algae"); algaeHolder.transform.SetParent(root.transform, false);
        int clusters = 0;
        foreach (var line in farm.cultureLines)
            clusters += MakeAlgae(algaeHolder.transform, line.nodes, Pos, farm.ropeDepth,
                                  farm.bladeLength, farm.bladesPerMetre, physAlgae);

        Directory.CreateDirectory(Path.GetDirectoryName(
            KristinebergSiteBuilder.AssetPathToFullPublic(PrefabPath)));
        var prefab = PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        UnityEngine.Object.DestroyImmediate(root);

        Debug.Log($"[Farm] built: {farm.buoys.Length} buoys, {ropeSegs} rope colliders, " +
                  $"{moorSegs} mooring colliders, {clusters} algae clusters -> {PrefabPath}");
        Debug.Log("[Farm] sonar labels in play: Buoy=3 (0.99), Rope=4 (0.40), Algae=2 (0.25), " +
                  "Rock=1 (0.80). Reflectivity values are the source's own 'wild guesses' — " +
                  "detection is qualitative until calibrated against real SSS.");
    }

    static PhysicsMaterial LoadPhys(string name)
    {
        var m = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>($"{PhysDir}/{name}.physicMaterial");
        if (m == null) Debug.LogError($"[Farm] missing {PhysDir}/{name}.physicMaterial — the sonar " +
                                      $"looks up material NAMES, so without this the part reads as unclassified");
        return m;
    }

    /// <summary>A rope through a node list, sagging under gravity between supports.</summary>
    static int MakeRope(Transform parent, string kind, string[] nodes, Func<string, Vector3> pos,
                        float depth, float radius, PhysicsMaterial phys)
    {
        int n = 0;
        var go = new GameObject($"{kind}_{nodes[0]}_{nodes[nodes.Length - 1]}");
        go.transform.SetParent(parent, false);
        for (int i = 0; i < nodes.Length - 1; i++)
        {
            var a = pos(nodes[i]); var b = pos(nodes[i + 1]);
            a.y = depth; b.y = depth;
            n += MakeSegments(go.transform, $"seg{i}", a, b, radius, phys, sag: 0.35f);
        }
        return n;
    }

    /// <summary>
    /// A straight or sagging run as a chain of capsule colliders with thin cylinder visuals.
    /// One collider per RopeSegmentLen — a LineRenderer alone has NO collider and would be
    /// completely invisible to the sonar, which is the trap this exists to avoid.
    /// </summary>
    static int MakeSegments(Transform parent, string name, Vector3 a, Vector3 b,
                            float radius, PhysicsMaterial phys, float sag = 0f)
    {
        float len = Vector3.Distance(a, b);
        int count = Mathf.Max(1, Mathf.RoundToInt(len / RopeSegmentLen));
        var holder = new GameObject(name); holder.transform.SetParent(parent, false);
        Vector3 P(float t)
        {
            var p = Vector3.Lerp(a, b, t);
            p.y -= sag * 4f * t * (1f - t);       // parabolic sag, zero at the supports
            return p;
        }
        for (int i = 0; i < count; i++)
        {
            var p0 = P(i / (float)count); var p1 = P((i + 1) / (float)count);
            var seg = new GameObject($"s{i}");
            seg.transform.SetParent(holder.transform, false);
            seg.transform.localPosition = (p0 + p1) * 0.5f;
            seg.transform.localRotation = Quaternion.FromToRotation(Vector3.up, (p1 - p0).normalized);
            float l = Vector3.Distance(p0, p1);

            var vis = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            vis.name = "vis"; vis.transform.SetParent(seg.transform, false);
            vis.transform.localScale = new Vector3(radius * 2f, l * 0.5f, radius * 2f);
            UnityEngine.Object.DestroyImmediate(vis.GetComponent<Collider>());
            var mat = AssetDatabase.LoadAssetAtPath<Material>(MatDir + "/RopeLink.mat")
                   ?? AssetDatabase.LoadAssetAtPath<Material>(MatDir + "/Default.mat");
            if (mat != null) vis.GetComponent<MeshRenderer>().sharedMaterial = mat;

            var col = seg.AddComponent<CapsuleCollider>();
            col.direction = 1;                 // Y
            col.radius = radius;
            col.height = l + radius * 2f;
            col.sharedMaterial = phys;
        }
        return count;
    }

    /// <summary>
    /// Saccharina blades hanging from a culture line. One thin BOX COLLIDER PER CLUSTER, not
    /// per blade — per-blade colliders would be tens of thousands and will not survive the
    /// frame budget. The cluster collider is what the sonar sees; the blades are what the eye
    /// sees.
    /// </summary>
    static int MakeAlgae(Transform parent, string[] nodes, Func<string, Vector3> pos,
                         float depth, float bladeLen, float perMetre, PhysicsMaterial phys)
    {
        int clusters = 0;
        var go = new GameObject($"algae_{nodes[0]}_{nodes[nodes.Length - 1]}");
        go.transform.SetParent(parent, false);
        for (int i = 0; i < nodes.Length - 1; i++)
        {
            var a = pos(nodes[i]); var b = pos(nodes[i + 1]);
            a.y = depth; b.y = depth;
            float len = Vector3.Distance(a, b);
            int nc = Mathf.Max(1, Mathf.RoundToInt(len / 2f));       // one cluster per 2 m
            for (int c = 0; c < nc; c++)
            {
                float t0 = c / (float)nc, t1 = (c + 1) / (float)nc;
                var mid = Vector3.Lerp(a, b, (t0 + t1) * 0.5f);
                var cl = new GameObject($"cluster{i}_{c}");
                cl.transform.SetParent(go.transform, false);
                cl.transform.localPosition = mid + new Vector3(0, -bladeLen * 0.5f, 0);

                // Saccharina fronds as real double-skinned geometry, NOT flat quads: the shared
                // material is opaque and single-sided, so a quad renders as a dark card with
                // its back face culled to black. Hanging DOWN from the rope, so the clump mesh
                // (which grows upward) is inverted.
                int blades = Mathf.Max(1, Mathf.RoundToInt(len / nc * perMetre));
                var seed = new System.Random(i * 977 + c);
                var kelpMat = KristinebergFlora.Tinted("Kristineberg_Kelp",
                                                       new Color(0.34f, 0.29f, 0.13f), 0.30f);
                for (int k = 0; k < blades; k++)
                {
                    float u = (k + 0.5f) / blades - 0.5f;
                    var along = (b - a).normalized * (len / nc) * u;
                    var q = new GameObject("frond");
                    q.transform.SetParent(cl.transform, false);
                    q.transform.localPosition = along + new Vector3(0, bladeLen * 0.5f, 0);
                    // 180 deg about X hangs the clump downward from the line.
                    q.transform.localRotation = Quaternion.Euler(180f, (float)seed.NextDouble() * 360f, 0f);
                    q.transform.localScale = new Vector3(1f, bladeLen, 1f);
                    q.AddComponent<MeshFilter>().sharedMesh =
                        KristinebergFlora.Blade("kelp", (i * 3 + c + k) % 6);
                    q.AddComponent<MeshRenderer>().sharedMaterial = kelpMat;
                }
                var col = cl.AddComponent<BoxCollider>();
                col.size = new Vector3(len / nc, bladeLen, 0.5f);
                col.sharedMaterial = phys;
                clusters++;
            }
        }
        return clusters;
    }
}
