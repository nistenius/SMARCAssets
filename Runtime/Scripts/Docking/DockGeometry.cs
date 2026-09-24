// DockGeometry.cs — procedural meshes for the docking station (2026-09-23, docking work order R1).
//
// All meshes are in the DOCK-LOCAL Unity frame: origin at the THROAT ENTRANCE centre, +z along the
// axis INTO the tube, +x right, +y up (so the dock frame D of sam_docking is x_D = z, y_D = x,
// z_D = -y). Every surface is emitted with its OWN vertices and an explicitly chosen facing
// (each triangle is flipped until its normal agrees with the intended side), so the shell is a
// closed, correctly-faced solid: the FLS sees the outer skin, the hull meets the inner skin, and
// a non-convex MeshCollider built from it has no back-face holes.
//
// SHELL = inner cone (mouth -> throat) + inner tube + inner end cap, outer skin (wall thickness
// outward) + outer end cap, and the mouth rim annulus joining inner and outer.
// BOARD = the marker board: a rectangle with a round hole (the cone's outer mouth radius) in the
// mouth plane, two-sided, thickness BoardThickness.
// DISC  = one flat disc facing -z (towards an approaching vehicle), for the markers.

using System.Collections.Generic;
using UnityEngine;

namespace Docking
{
    public static class DockGeometry
    {
        class Builder
        {
            public readonly List<Vector3> V = new List<Vector3>();
            public readonly List<int> T = new List<int>();

            public void Tri(Vector3 a, Vector3 b, Vector3 c, Vector3 facing)
            {
                Vector3 n = Vector3.Cross(b - a, c - a);      // Unity: front face normal
                int i = V.Count;
                if (Vector3.Dot(n, facing) < 0f) { V.Add(a); V.Add(c); V.Add(b); }
                else { V.Add(a); V.Add(b); V.Add(c); }
                T.Add(i); T.Add(i + 1); T.Add(i + 2);
            }

            public void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 facing)
            {
                Tri(a, b, c, facing);
                Tri(a, c, d, facing);
            }

            public Mesh Build(string name)
            {
                var m = new Mesh { name = name };
                if (V.Count > 65000) m.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
                m.SetVertices(V);
                m.SetTriangles(T, 0);
                m.RecalculateNormals();
                m.RecalculateBounds();
                return m;
            }
        }

        static Vector3 P(float r, float ang, float z) => new Vector3(r * Mathf.Cos(ang), r * Mathf.Sin(ang), z);
        static Vector3 Radial(float ang) => new Vector3(Mathf.Cos(ang), Mathf.Sin(ang), 0f);

        // surface of revolution through a (r, z) polyline; facing: +1 = away from the axis, -1 = towards it
        static void Revolve(Builder b, IList<Vector2> prof, int seg, float facing)
        {
            for (int k = 0; k + 1 < prof.Count; k++)
            {
                for (int j = 0; j < seg; j++)
                {
                    float a0 = 2f * Mathf.PI * j / seg, a1 = 2f * Mathf.PI * (j + 1) / seg;
                    Vector3 f = Radial(0.5f * (a0 + a1)) * facing;
                    b.Quad(P(prof[k].x, a0, prof[k].y), P(prof[k].x, a1, prof[k].y),
                           P(prof[k + 1].x, a1, prof[k + 1].y), P(prof[k + 1].x, a0, prof[k + 1].y), f);
                }
            }
        }

        static void Annulus(Builder b, float rIn, float rOut, float z, int seg, Vector3 facing)
        {
            for (int j = 0; j < seg; j++)
            {
                float a0 = 2f * Mathf.PI * j / seg, a1 = 2f * Mathf.PI * (j + 1) / seg;
                if (rIn <= 1e-6f) b.Tri(new Vector3(0, 0, z), P(rOut, a0, z), P(rOut, a1, z), facing);
                else b.Quad(P(rIn, a0, z), P(rIn, a1, z), P(rOut, a1, z), P(rOut, a0, z), facing);
            }
        }

        public static Mesh Shell(float mouthD, float throatD, float coneLen, float tubeLen, float wall, int seg = 96)
        {
            float rm = 0.5f * mouthD, rt = 0.5f * throatD;
            var b = new Builder();
            var inner = new List<Vector2> { new Vector2(rm, -coneLen), new Vector2(rt, 0f), new Vector2(rt, tubeLen) };
            // outer skin: offset the cone by `wall` along its normal (so the wall is uniform), the tube radially
            float half = Mathf.Atan2(rm - rt, coneLen);
            float dr = wall / Mathf.Cos(half);
            var outerCone = new List<Vector2> { new Vector2(rm + dr, -coneLen), new Vector2(rt + dr, 0f) };
            var outerTube = new List<Vector2> { new Vector2(rt + wall, 0f), new Vector2(rt + wall, tubeLen + wall) };
            Revolve(b, inner, seg, -1f);
            Revolve(b, outerCone, seg, +1f);
            Revolve(b, outerTube, seg, +1f);
            Annulus(b, rm, rm + dr, -coneLen, seg, Vector3.back);          // mouth rim, facing the approach
            Annulus(b, rt + wall, rt + dr, 0f, seg, Vector3.forward);      // back face of the cone wall, outside the tube
            Annulus(b, 0f, rt, tubeLen, seg, Vector3.back);                // inner end cap, facing into the tube
            Annulus(b, 0f, rt + wall, tubeLen + wall, seg, Vector3.forward); // outer end cap
            return b.Build("DockShell");
        }

        public static Mesh Board(float holeR, float halfW, float top, float bottom, float zFront, float thickness, int seg = 96)
        {
            var b = new Builder();
            float zBack = zFront + thickness;
            for (int j = 0; j < seg; j++)
            {
                float a0 = 2f * Mathf.PI * j / seg, a1 = 2f * Mathf.PI * (j + 1) / seg;
                Vector3 i0 = P(holeR, a0, 0), i1 = P(holeR, a1, 0);
                Vector3 o0 = RectHit(a0, halfW, top, bottom), o1 = RectHit(a1, halfW, top, bottom);
                foreach (var zz in new[] { zFront, zBack })
                {
                    Vector3 face = zz == zFront ? Vector3.back : Vector3.forward;
                    b.Quad(new Vector3(i0.x, i0.y, zz), new Vector3(i1.x, i1.y, zz),
                           new Vector3(o1.x, o1.y, zz), new Vector3(o0.x, o0.y, zz), face);
                }
                // outer edge band and hole band close the slab
                Vector3 outN = new Vector3(0.5f * (o0.x + o1.x), 0.5f * (o0.y + o1.y), 0f);
                b.Quad(new Vector3(o0.x, o0.y, zFront), new Vector3(o1.x, o1.y, zFront),
                       new Vector3(o1.x, o1.y, zBack), new Vector3(o0.x, o0.y, zBack), outN);
                b.Quad(new Vector3(i0.x, i0.y, zFront), new Vector3(i1.x, i1.y, zFront),
                       new Vector3(i1.x, i1.y, zBack), new Vector3(i0.x, i0.y, zBack), -Radial(0.5f * (a0 + a1)));
            }
            return b.Build("DockMarkerBoard");
        }

        // where the ray from the axis at angle `a` leaves the rectangle [-halfW, halfW] x [-bottom, top]
        static Vector3 RectHit(float a, float halfW, float top, float bottom)
        {
            float c = Mathf.Cos(a), s = Mathf.Sin(a);
            float t = float.PositiveInfinity;
            if (c > 1e-6f) t = Mathf.Min(t, halfW / c);
            if (c < -1e-6f) t = Mathf.Min(t, -halfW / c);
            if (s > 1e-6f) t = Mathf.Min(t, top / s);
            if (s < -1e-6f) t = Mathf.Min(t, -bottom / s);
            return new Vector3(t * c, t * s, 0f);
        }

        public static Mesh Disc(float radius, int seg = 32)
        {
            var b = new Builder();
            Annulus(b, 0f, radius, 0f, seg, Vector3.back);
            return b.Build("DockMarkerDisc");
        }
    }
}
