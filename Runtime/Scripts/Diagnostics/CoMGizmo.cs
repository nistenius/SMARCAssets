// CoMGizmo.cs — draws what the physics actually uses, in the Scene view.
//
//   green  ▲  centre of buoyancy  = centroid of every ForcePoint attached to this vehicle
//   purple ▼  centre of gravity   = mass-weighted CoM over every ArticulationBody under the root,
//                                   using each body's centerOfMass at its CURRENT pose (so the
//                                   LCG battery and the VBS piston are where they are right now)
//   yellow    each ForcePoint, with a bar proportional to the buoyancy share it is applying
//   white     the CB→CG offset, labelled with BG (vertical) and the longitudinal lever in mm
//
// USAGE
//   1. Save as SMARCAssets/Runtime/Scripts/Diagnostics/CoMGizmo.cs (any Runtime folder that
//      references the Force namespace).
//   2. Select the vehicle root (the object carrying base_link's ArticulationBody, e.g. `sam_auv_v1`
//      or `sam2.2`) → Add Component → CoM Gizmo.
//   3. Scene view → Gizmos toggle ON (top-right of the Scene view).
//   4. In Play the markers move with the joints; set LCG/VBS from VC and watch the purple ▼.
//
// In Edit mode the prismatic joints are at their saved pose (LCG 0 %), so the CG you see there
// is NOT the LCG-50 % one. Press Play, command LCG 50 / VBS 5, then read it.

using System.Linq;
using Force;
using UnityEngine;

namespace Diagnostics
{
    [ExecuteAlways]
    public class CoMGizmo : MonoBehaviour
    {
        public float MarkerSize = 0.04f;
        public float ForceBarScale = 0.002f;   // metres of bar per newton
        public bool ShowLabels = true;

        void OnDrawGizmos()
        {
            var bodies = GetComponentsInChildren<ArticulationBody>(true)
                .Where(b => b.mass > 1e-3f).ToArray();
            var points = GetComponentsInChildren<ForcePoint>(true);
            if (bodies.Length == 0 || points.Length == 0) return;

            // Composite CG at the current pose.
            float m = 0f; Vector3 mx = Vector3.zero;
            foreach (var b in bodies)
            {
                Vector3 com = b.transform.TransformPoint(b.centerOfMass);
                m += b.mass; mx += b.mass * com;
            }
            Vector3 cg = mx / m;

            // CB = ForcePoint centroid. In LEGACY mode every point carries an equal share
            // (ForcePoint.ApplyForce divides by the related-point count) so the plain mean is the CB.
            // In STRIP mode (VolumeIsPerPoint, added 2026-09-13) each point carries its own volume, so
            // the CB is the VOLUME-WEIGHTED centroid - the plain mean would be several mm out, which on
            // a 165 N displacement is a visible trim moment.
            Vector3 cb = Vector3.zero;
            bool perPoint = false;
            foreach (var p in points) if (p.VolumeIsPerPoint) { perPoint = true; break; }
            if (perPoint)
            {
                float wsum = 0f;
                foreach (var p in points) { cb += p.Volume * p.transform.position; wsum += p.Volume; }
                cb = wsum > 0f ? cb / wsum : cg;
            }
            else
            {
                foreach (var p in points) cb += p.transform.position;
                cb /= points.Length;
            }

            // Points and their current buoyancy share.
            Gizmos.color = new Color(1f, 0.8f, 0.2f, 0.9f);
            foreach (var p in points)
            {
                Gizmos.DrawSphere(p.transform.position, MarkerSize * 0.5f);
                float f = p.AppliedBuoyancyForce.y;              // N, whatever it applied last step
                if (Application.isPlaying && f > 0f)
                    Gizmos.DrawLine(p.transform.position, p.transform.position + Vector3.up * f * ForceBarScale);
            }

            Gizmos.color = Color.green;
            DrawTri(cb, +1);
            Gizmos.color = new Color(0.56f, 0.27f, 0.68f);
            DrawTri(cg, -1);
            Gizmos.color = Color.white;
            Gizmos.DrawLine(cb, cg);

#if UNITY_EDITOR
            if (ShowLabels)
            {
                Vector3 d = transform.InverseTransformVector(cb - cg);   // in the vehicle frame
                string txt = $"m = {m:F3} kg\nCB−CG  long z = {d.z * 1000f:+0.0} mm   BG y = {d.y * 1000f:+0.0} mm   lat x = {d.x * 1000f:+0.0} mm";
                if (d.y > 1e-6f)
                    txt += $"\nstatic trim ≈ {Mathf.Atan2(d.z, d.y) * Mathf.Rad2Deg:+0.0}° " + (d.z > 0 ? "nose-up" : "nose-down");
                UnityEditor.Handles.Label(cb + Vector3.up * 0.15f, txt);
            }
#endif
        }

        void DrawTri(Vector3 c, int up)
        {
            float s = MarkerSize;
            Vector3 a = c + Vector3.up * s * up;
            Vector3 b = c + Vector3.forward * s * 0.7f - Vector3.up * s * 0.5f * up;
            Vector3 e = c - Vector3.forward * s * 0.7f - Vector3.up * s * 0.5f * up;
            Gizmos.DrawLine(a, b); Gizmos.DrawLine(b, e); Gizmos.DrawLine(e, a);
            Vector3 f = c + Vector3.right * s * 0.7f - Vector3.up * s * 0.5f * up;
            Vector3 g = c - Vector3.right * s * 0.7f - Vector3.up * s * 0.5f * up;
            Gizmos.DrawLine(a, f); Gizmos.DrawLine(f, g); Gizmos.DrawLine(g, a);
        }
    }
}
