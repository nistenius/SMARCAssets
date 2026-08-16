using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Procedural geometry for the Kristineberg seabed: vegetation blades and boulders.
///
/// WHY THIS EXISTS. The first pass scattered Unity primitives — `PrimitiveType.Quad` for
/// vegetation and `PrimitiveType.Cube` for rock — and it looked exactly like that: flat dark
/// cards standing on end and boxes in the sand. Two compounding reasons, both measured in
/// Algae.mat:
///   * `_CullMode: 2` / `_DoubleSidedEnable: 0` — the material is SINGLE-SIDED, so every quad
///     facing away from the camera renders black;
///   * `_AlphaCutoffEnable: 0` — it is OPAQUE, so a quad is a solid rectangle, not a leaf.
///
/// Rather than perform HDRP material surgery (alpha clipping + double-sided needs matching
/// shader keywords, and getting that wrong fails silently), the fix is GEOMETRY: build blades
/// as tapered, curved, DOUBLE-SKINNED ribbons and boulders as deformed low-poly spheres. Then
/// the existing opaque single-sided material is correct, because every surface genuinely has
/// an outward face.
///
/// Meshes are generated ONCE as shared assets in a small variant library and instanced ~1,800
/// times. A unique mesh per instance would be 1,800 assets and no batching.
/// </summary>
public static class KristinebergFlora
{
    const string Dir = "Packages/com.smarc.assets/Runtime/Terrain/Kristineberg/Flora";

    static readonly Dictionary<string, Mesh> cache = new Dictionary<string, Mesh>();

    static Mesh Save(Mesh mesh, string name)
    {
        var path = Dir + "/" + name + ".asset";
        var existing = AssetDatabase.LoadAssetAtPath<Mesh>(path);
        if (existing != null) { cache[name] = existing; return existing; }
        System.IO.Directory.CreateDirectory(
            KristinebergSiteBuilder.AssetPathToFullPublic(Dir));
        AssetDatabase.CreateAsset(mesh, path);
        cache[name] = mesh;
        return mesh;
    }

    /// <summary>
    /// A clump of tapered, curved blades. Each blade is a double-skinned ribbon: the strip is
    /// emitted twice with opposite winding, so it is lit and visible from both sides under an
    /// ordinary opaque material. That is what a flat Quad could never be.
    /// </summary>
    public static Mesh Blade(string kind, int variant)
    {
        string key = $"kristineberg_{kind}_{variant}";
        if (cache.TryGetValue(key, out var c)) return c;
        var existing = AssetDatabase.LoadAssetAtPath<Mesh>(Dir + "/" + key + ".asset");
        if (existing != null) { cache[key] = existing; return existing; }

        // Species geometry, from the Gullmarsfjorden literature and Kristineberg's own
        // "Fjordens öga" live camera at 5 m:
        //   eelgrass  Zostera marina — the meadow is 95 % this, up to 1.8 m tall in summer and
        //             ~500 shoots/m². A "tuft" of nine blades was an order of magnitude too
        //             sparse; the camera shows a closed canopy you cannot see the bottom
        //             through.
        //   ceramium  fine, much-branched red alga (Ceramium virgatum) — the vivid pink in the
        //             camera frame. Low and bushy, interspersed through the meadow.
        //   kelp      Saccharina latissima — broad straps, few per holdfast.
        bool kelp = kind == "kelp";
        bool red = kind == "ceramium";
        int blades = kelp ? 5 : red ? 26 : 34;
        int segs = red ? 5 : 7;
        float baseW = kelp ? 0.14f : red ? 0.012f : 0.030f;
        var rng = new System.Random(1000 + variant * 31 + (kelp ? 7 : red ? 13 : 0));

        var verts = new List<Vector3>();
        var norms = new List<Vector3>();
        var uvs = new List<Vector2>();
        var tris = new List<int>();

        for (int b = 0; b < blades; b++)
        {
            float yaw = (float)(rng.NextDouble() * Mathf.PI * 2.0);
            // Eelgrass leans further than kelp: long straps in a current lie over, which is
            // what gives a meadow its swept look rather than a bed of spikes.
            float lean = 0.25f + (float)rng.NextDouble() * (kelp ? 0.55f : red ? 0.5f : 0.75f);
            float len = (red ? 0.45f : 0.75f) + (float)rng.NextDouble() * (red ? 0.35f : 0.5f);
            var dir = new Vector3(Mathf.Cos(yaw), 0, Mathf.Sin(yaw));
            // Spread the shoots out so a clump reads as a patch of meadow, not a single tussock.
            float spread = kelp ? 0.10f : red ? 0.22f : 0.34f;
            var root = new Vector3(Mathf.Cos(yaw), 0, Mathf.Sin(yaw))
                     * (float)(rng.NextDouble() * spread);
            float twist = ((float)rng.NextDouble() - 0.5f) * 1.4f;

            int start = verts.Count;
            for (int s = 0; s <= segs; s++)
            {
                float t = s / (float)segs;
                // Curve: blades stand up then bend over — quadratic lean, and they taper.
                float y = len * t;
                var pos = root + dir * (lean * len * t * t) + Vector3.up * y;
                float w = baseW * (1f - 0.75f * t * t) * (kelp ? 1f + 0.5f * Mathf.Sin(t * 3.14f) : 1f);
                var side = Vector3.Cross(Vector3.up, dir).normalized;
                side = Quaternion.AngleAxis(twist * t * Mathf.Rad2Deg, dir) * side;
                verts.Add(pos - side * w); verts.Add(pos + side * w);
                var nrm = Vector3.Cross(side, Vector3.up).normalized;
                norms.Add(nrm); norms.Add(nrm);
                uvs.Add(new Vector2(0, t)); uvs.Add(new Vector2(1, t));
            }
            // front faces
            for (int s = 0; s < segs; s++)
            {
                int i = start + s * 2;
                tris.Add(i); tris.Add(i + 2); tris.Add(i + 1);
                tris.Add(i + 1); tris.Add(i + 2); tris.Add(i + 3);
            }
            // back skin: same strip, reversed winding and flipped normals, so the blade reads
            // correctly from behind instead of vanishing.
            int back = verts.Count;
            for (int s = 0; s <= segs; s++)
            {
                verts.Add(verts[start + s * 2]); verts.Add(verts[start + s * 2 + 1]);
                norms.Add(-norms[start + s * 2]); norms.Add(-norms[start + s * 2 + 1]);
                uvs.Add(uvs[start + s * 2]); uvs.Add(uvs[start + s * 2 + 1]);
            }
            for (int s = 0; s < segs; s++)
            {
                int i = back + s * 2;
                tris.Add(i); tris.Add(i + 1); tris.Add(i + 2);
                tris.Add(i + 1); tris.Add(i + 3); tris.Add(i + 2);
            }
        }

        var mesh = new Mesh { name = key };
        mesh.SetVertices(verts); mesh.SetNormals(norms); mesh.SetUVs(0, uvs);
        mesh.SetTriangles(tris, 0);
        mesh.RecalculateBounds();
        return Save(mesh, key);
    }

    /// <summary>
    /// A boulder: an icosphere-ish blob, radially deformed and squashed, flat-shaded.
    /// Cubes read as crates; this reads as rock at any angle.
    /// </summary>
    public static Mesh Rock(int variant)
    {
        string key = $"kristineberg_rock_{variant}";
        if (cache.TryGetValue(key, out var c)) return c;
        var existing = AssetDatabase.LoadAssetAtPath<Mesh>(Dir + "/" + key + ".asset");
        if (existing != null) { cache[key] = existing; return existing; }

        var rng = new System.Random(500 + variant * 17);
        int rings = 6, sectors = 9;
        var verts = new List<Vector3>();
        var tris = new List<int>();
        // deformation field, sampled per direction so the blob stays closed
        float Deform(float u, float v) =>
            0.72f + 0.28f * Mathf.PerlinNoise(u * 2.3f + variant * 5.1f, v * 2.7f + variant * 2.3f);

        for (int r = 0; r <= rings; r++)
        {
            float phi = Mathf.PI * r / rings;
            for (int s = 0; s <= sectors; s++)
            {
                float th = 2f * Mathf.PI * s / sectors;
                var dir = new Vector3(Mathf.Sin(phi) * Mathf.Cos(th), Mathf.Cos(phi),
                                      Mathf.Sin(phi) * Mathf.Sin(th));
                float d = Deform(s / (float)sectors, r / (float)rings);
                var p = Vector3.Scale(dir * d, new Vector3(1f, 0.62f, 0.88f));  // boulders sit low
                verts.Add(p);
            }
        }
        for (int r = 0; r < rings; r++)
            for (int s = 0; s < sectors; s++)
            {
                int a = r * (sectors + 1) + s, b = a + sectors + 1;
                tris.Add(a); tris.Add(b); tris.Add(a + 1);
                tris.Add(a + 1); tris.Add(b); tris.Add(b + 1);
            }

        var mesh = new Mesh { name = key };
        mesh.SetVertices(verts); mesh.SetTriangles(tris, 0);
        // Flat shading: split vertices so each face gets its own normal — a smooth-shaded
        // blob looks like a balloon, a faceted one looks like granite.
        var fv = new List<Vector3>(); var ft = new List<int>();
        var t2 = mesh.triangles; var v2 = mesh.vertices;
        for (int i = 0; i < t2.Length; i += 3)
        {
            fv.Add(v2[t2[i]]); fv.Add(v2[t2[i + 1]]); fv.Add(v2[t2[i + 2]]);
            ft.Add(i); ft.Add(i + 1); ft.Add(i + 2);
        }
        mesh.Clear();
        mesh.SetVertices(fv); mesh.SetTriangles(ft, 0);
        mesh.RecalculateNormals(); mesh.RecalculateBounds();
        return Save(mesh, key);
    }

    /// <summary>
    /// A colour variant of the base algae material. Only _BaseColor is touched, so no HDRP
    /// shader keywords are involved and nothing can silently fail to apply.
    /// </summary>
    public static Material Tinted(string name, Color col, float smoothness = 0.12f)
    {
        var path = Dir + "/" + name + ".mat";
        var existing = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (existing != null) return existing;
        var baseMat = AssetDatabase.LoadAssetAtPath<Material>(
            "Packages/com.smarc.assets/Runtime/Materials/Algae.mat");
        var mat = baseMat != null ? new Material(baseMat) : new Material(Shader.Find("HDRP/Lit"));
        mat.name = name;
        foreach (var p in new[] { "_BaseColor", "_Color" })
            if (mat.HasProperty(p)) mat.SetColor(p, col);
        if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", smoothness);
        System.IO.Directory.CreateDirectory(KristinebergSiteBuilder.AssetPathToFullPublic(Dir));
        AssetDatabase.CreateAsset(mat, path);
        return mat;
    }
}
