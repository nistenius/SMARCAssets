using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using Force;
using Smarc.Environment;

/// <summary>
/// Instantiates the Kristineberg seabed dressing (eelgrass, kelp, rock) and the site's
/// current field, from kristineberg_dressing.json.
///
/// SONAR-VISIBLE BY CONSTRUCTION: every instance carries a collider whose PHYSICS MATERIAL is
/// named "Algae" or "Rock", which is what Sonar.cs reads to decide reflectivity and label.
/// Vegetation gets ONE collider per patch — not per blade. Per-blade colliders would be tens
/// of thousands of raycast targets for no extra information at sonar wavelengths.
///
/// Run from menu: SMARC -> Build Kristineberg Seabed
/// </summary>
public static class KristinebergSeabedBuilder
{
    const string DataDir = "Packages/com.smarc.assets/Runtime/Terrain/Kristineberg";
    const string PrefabPath = "Packages/com.smarc.assets/Runtime/Prefabs/Environment/KristinebergSeabed.prefab";
    const string PhysDir = "Packages/com.smarc.assets/Runtime/Materials/Physic";
    const string MatDir = "Packages/com.smarc.assets/Runtime/Materials";

    [Serializable] public class Item { public string t; public float x, y, z, yaw, s, hgt; }
    [Serializable] public class Dressing { public int total; public Item[] items; }

    [MenuItem("SMARC/Build Kristineberg Seabed")]
    public static void Build()
    {
        var p = KristinebergSiteBuilder.AssetPathToFullPublic(DataDir + "/kristineberg_dressing.json");
        if (!File.Exists(p))
        { Debug.LogError("[Seabed] no dressing at " + p + " — run build_seabed_dressing.py"); return; }
        var dr = JsonUtility.FromJson<Dressing>(File.ReadAllText(p));
        if (dr?.items == null || dr.items.Length == 0)
        { Debug.LogError("[Seabed] dressing manifest is empty"); return; }

        var physAlgae = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(PhysDir + "/Algae.physicMaterial");
        var physRock = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(PhysDir + "/Rock.physicMaterial");
        if (physAlgae == null || physRock == null)
        { Debug.LogError("[Seabed] missing Algae/Rock physicMaterial — without them the sonar reads these as unclassified"); return; }

        // Colour variants off the base algae material — only _BaseColor differs, so no HDRP
        // shader keywords are involved and nothing can silently fail to apply. Colours from
        // the Gullmarsfjorden literature and the live-camera imagery: eelgrass a fresh mid
        // green, Saccharina a browner olive, granite a grey-brown under water.
        var matEelgrass = KristinebergFlora.Tinted("Kristineberg_Eelgrass",
                                                   new Color(0.24f, 0.42f, 0.20f), 0.22f);
        var matKelp = KristinebergFlora.Tinted("Kristineberg_Kelp",
                                                new Color(0.34f, 0.29f, 0.13f), 0.30f);
        // Ceramium virgatum — the vivid pink-red in Kristineberg's 5 m live camera. Strong
        // colour on purpose: it is the most recognisable thing in that frame after the
        // eelgrass itself, and washing it out would lose the site's signature.
        var matCeramium = KristinebergFlora.Tinted("Kristineberg_Ceramium",
                                                    new Color(0.62f, 0.16f, 0.32f), 0.26f);
        var matRockTint = KristinebergFlora.Tinted("Kristineberg_SeabedRock",
                                                    new Color(0.40f, 0.39f, 0.36f), 0.10f);

        var root = new GameObject("KristinebergSeabed");
        var holders = new Dictionary<string, Transform>();
        Transform Holder(string k)
        {
            if (!holders.TryGetValue(k, out var t))
            {
                var g = new GameObject(k); g.transform.SetParent(root.transform, false);
                holders[k] = t = g.transform;
            }
            return t;
        }

        var counts = new Dictionary<string, int>();
        foreach (var it in dr.items)
        {
            var go = new GameObject(it.t);
            go.transform.SetParent(Holder(it.t), false);
            go.transform.localPosition = new Vector3(it.x, it.y, it.z);
            go.transform.localRotation = Quaternion.Euler(0f, it.yaw, 0f);
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);

            // Real geometry, not primitives. See KristinebergFlora for why: the shared material
            // is opaque and single-sided, so a flat Quad renders as a dark card with its back
            // face culled to black — which is exactly how the first pass looked.
            int variant = Mathf.Abs((int)(it.x * 7.13f + it.z * 3.71f)) % 6;
            if (it.t == "rock")
            {
                var vis = new GameObject("vis");
                vis.transform.SetParent(go.transform, false);
                vis.transform.localPosition = new Vector3(0, it.hgt * 0.35f, 0);
                vis.transform.localScale = new Vector3(it.s, it.hgt, it.s * 0.85f);
                vis.transform.localRotation = Quaternion.Euler(UnityEngine.Random.Range(-10f, 10f), 0f,
                                                               UnityEngine.Random.Range(-10f, 10f));
                vis.AddComponent<MeshFilter>().sharedMesh = KristinebergFlora.Rock(variant);
                vis.AddComponent<MeshRenderer>().sharedMaterial = matRockTint;
                // Collider: a sphere is plenty for a boulder at sonar wavelengths, and far
                // cheaper than a mesh collider on 700 of them.
                var rc = go.AddComponent<SphereCollider>();
                rc.center = new Vector3(0, it.hgt * 0.35f, 0);
                rc.radius = Mathf.Max(it.s, it.hgt) * 0.45f;
                rc.sharedMaterial = physRock;                              // Rock: 0.80, label 1
            }
            else
            {
                var vis = new GameObject("clump");
                vis.transform.SetParent(go.transform, false);
                vis.transform.localScale = new Vector3(it.s * 0.5f, it.hgt, it.s * 0.5f);
                vis.AddComponent<MeshFilter>().sharedMesh = KristinebergFlora.Blade(it.t, variant);
                var vm = it.t == "kelp" ? matKelp
                       : it.t == "ceramium" ? matCeramium
                       : matEelgrass;
                vis.AddComponent<MeshRenderer>().sharedMaterial = vm;
                // ONE collider per patch, not per blade — per-blade would be tens of thousands
                // of raycast targets for no extra information at sonar wavelengths.
                var col = go.AddComponent<BoxCollider>();
                col.center = new Vector3(0, it.hgt * 0.5f, 0);
                col.size = new Vector3(it.s, it.hgt, it.s);
                col.sharedMaterial = physAlgae;                            // Algae: 0.25, label 2
            }
            counts[it.t] = counts.TryGetValue(it.t, out var c) ? c + 1 : 1;
        }

        // --- the current field, off by default -------------------------------
        var cur = new GameObject("CurrentField (SCENARIO — off by default)");
        cur.transform.SetParent(root.transform, false);
        cur.transform.localPosition = new Vector3(100f, -12f, 30f);
        var box = cur.AddComponent<BoxCollider>();
        box.isTrigger = true;                       // ForceFieldBase applies via OnTriggerStay
        box.size = new Vector3(600f, 60f, 500f);
        var ffs = cur.AddComponent<ForceFieldStatic>();
        ffs.onlyUnderwater = true;                  // a current is water, not air
        var cf = cur.AddComponent<CurrentField>();
        cf.CurrentEnabled = false;
        cf.HeadingDeg = 45f;
        cf.SpeedMS = 0.15f;

        Directory.CreateDirectory(Path.GetDirectoryName(
            KristinebergSiteBuilder.AssetPathToFullPublic(PrefabPath)));
        PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        UnityEngine.Object.DestroyImmediate(root);

        var parts = new List<string>();
        foreach (var kv in counts) parts.Add($"{kv.Key}={kv.Value}");
        Debug.Log($"[Seabed] {dr.items.Length} instances ({string.Join(", ", parts)}) " +
                  $"+ current field -> {PrefabPath}");
        Debug.Log("[Seabed] sonar: vegetation carries 'Algae' (0.25, label 2), rock carries " +
                  "'Rock' (0.80, label 1). Current is OFF — turn it on deliberately, and note " +
                  "that it changes vehicle dynamics.");
    }
}
