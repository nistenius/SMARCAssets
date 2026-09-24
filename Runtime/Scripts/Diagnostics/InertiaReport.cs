// InertiaReport.cs — one Console block with the COMPOSITE inertia the physics actually uses.
//
// WHY (docking work order 2026-09-23, R0 step 1). The measured SAM 2.1 inertias are
// I_xx ~0.052, I_yy 2.73 +- 0.04, I_zz 2.505 +- 0.021 kg m^2, m 16.70 kg (SETTLED §3al, §4).
// Whether the Unity prefab's composite matches them is UNMEASURED; the 09-13 handover said
// child-link placeholder tensors gave roll ~1.06, pitch/yaw ~5.4 — "NOT fixed". A docking
// controller tuned against 2x the real pitch/yaw inertia is tuned for a different vehicle, so
// this number is read BEFORE any tuning.
//
// WHAT IT PRINTS (once, in Start, plus once more after `DelaySeconds` so joints have settled):
//   * every ArticulationBody under this object: name, mass, inertiaTensor (principal, kg m^2),
//     inertiaTensorRotation, centerOfMass (local) — the per-link inputs;
//   * the COMPOSITE about the whole vehicle's CoM, in the base_link axes, by the parallel-axis
//     theorem over all links at their CURRENT pose — reported as FRD-named axes
//     (roll = about the long axis, pitch = about the lateral axis, yaw = about the vertical axis);
//   * the hull collider's nose/tail along base_link's forward axis ("collider nose"), which is
//     the dock controller's `dock_nose_offset`.
//
// USAGE: select the vehicle root (`sam21` in KTHTankDock) -> Add Component -> Inertia Report ->
// Play -> read the Console block starting "[InertiaReport]" -> Stop -> REMOVE the component
// (it is diagnostic; a diagnostic left on a prefab is how silenced-not-disabled traps start).
// It changes nothing: it only reads.

using System.Collections;
using System.Text;
using UnityEngine;

namespace Diagnostics
{
    [AddComponentMenu("Smarc/Diagnostics/Inertia Report")]
    public class InertiaReport : MonoBehaviour
    {
        [Tooltip("A second report after this many seconds of Play (joints settled). 0 = only the Start report.")]
        public float DelaySeconds = 2f;

        void Start()
        {
            Report("Start");
            if (DelaySeconds > 0f) StartCoroutine(Later());
        }

        IEnumerator Later()
        {
            yield return new WaitForSeconds(DelaySeconds);
            Report($"t+{DelaySeconds:F1}s");
        }

        static Matrix4x4 Outer(Vector3 a, Vector3 b)
        {
            var m = Matrix4x4.zero;
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[i, j] = a[i] * b[j];
            return m;
        }

        void Report(string when)
        {
            var bodies = GetComponentsInChildren<ArticulationBody>(true);
            if (bodies.Length == 0) { Debug.LogError("[InertiaReport] no ArticulationBody under " + name); return; }
            ArticulationBody root = null;
            foreach (var b in bodies) if (b.isRoot) { root = b; break; }
            if (root == null) root = bodies[0];
            Transform frame = root.transform;          // base_link axes: Unity x right, y up, z forward

            var sb = new StringBuilder();
            sb.AppendLine($"[InertiaReport] {name} ({when}) — {bodies.Length} ArticulationBodies, frame = '{frame.name}' axes");
            double mTot = 0; Vector3 comW = Vector3.zero;
            foreach (var b in bodies)
            {
                mTot += b.mass;
                comW += b.mass * b.worldCenterOfMass;
                sb.AppendLine($"  {b.name,-24} m {b.mass,8:F4} kg  I {b.inertiaTensor.x:F5} {b.inertiaTensor.y:F5} {b.inertiaTensor.z:F5}" +
                              $"  Irot {b.inertiaTensorRotation.eulerAngles}  com(local) {b.centerOfMass.ToString("F4")}" +
                              $"  auto {b.automaticInertiaTensor}/{b.automaticCenterOfMass}");
            }
            comW /= (float)mTot;

            // composite about the vehicle CoM, expressed in the root (base_link) axes
            Matrix4x4 I = Matrix4x4.zero;
            foreach (var b in bodies)
            {
                Quaternion rWorld = b.transform.rotation * b.inertiaTensorRotation;
                Quaternion rInRoot = Quaternion.Inverse(frame.rotation) * rWorld;
                Matrix4x4 R = Matrix4x4.Rotate(rInRoot);
                Matrix4x4 D = Matrix4x4.Scale(b.inertiaTensor);
                Matrix4x4 Ib = R * D * R.transpose;
                Vector3 d = Quaternion.Inverse(frame.rotation) * (b.worldCenterOfMass - comW);
                float dd = Vector3.Dot(d, d);
                Matrix4x4 O = Outer(d, d);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        I[i, j] += Ib[i, j] + b.mass * ((i == j ? dd : 0f) - O[i, j]);
            }
            // Unity axes -> hull axes: z forward (roll), x right (pitch), y up (yaw)
            sb.AppendLine($"  COMPOSITE about CoM: m {mTot:F3} kg | I_roll(zz) {I[2, 2]:F4}  I_pitch(xx) {I[0, 0]:F4}  I_yaw(yy) {I[1, 1]:F4} kg m^2" +
                          $" | off-diag xy {I[0, 1]:F4} xz {I[0, 2]:F4} yz {I[1, 2]:F4}");
            sb.AppendLine($"  MEASURED (SETTLED §3al/§4): m 16.70 | I_xx(roll) ~0.052 (0.040-0.064) | I_yy(pitch) 2.73 +-0.04 | I_zz(yaw) 2.505 +-0.021");
            sb.AppendLine($"  RATIO sim/measured: roll {I[2, 2] / 0.052f:F2}  pitch {I[0, 0] / 2.73f:F2}  yaw {I[1, 1] / 2.505f:F2}" +
                          "   (09-13 placeholder state was roll ~1.06, pitch/yaw ~5.4 => ratios ~20 / ~2)");
            Vector3 comLocal = frame.InverseTransformPoint(comW);
            sb.AppendLine($"  vehicle CoM in base_link: {comLocal.ToString("F4")} m");

            // hull collider extent along base_link forward (non-trigger colliders only)
            float nose = float.NegativeInfinity, tail = float.PositiveInfinity;
            foreach (var c in GetComponentsInChildren<Collider>(true))
            {
                if (c.isTrigger || !c.enabled) continue;
                var bnd = c.bounds;
                for (int k = 0; k < 8; k++)
                {
                    Vector3 corner = new Vector3((k & 1) == 0 ? bnd.min.x : bnd.max.x,
                                                 (k & 2) == 0 ? bnd.min.y : bnd.max.y,
                                                 (k & 4) == 0 ? bnd.min.z : bnd.max.z);
                    float z = frame.InverseTransformPoint(corner).z;
                    nose = Mathf.Max(nose, z); tail = Mathf.Min(tail, z);
                }
            }
            sb.AppendLine($"  collider nose {nose:F3} m / tail {tail:F3} m along base_link forward (AABB corners — exact only when the hull is level)" +
                          "  => dock_nose_offset in config/dock_pid.yaml");
            Debug.Log(sb.ToString());
        }
    }
}
