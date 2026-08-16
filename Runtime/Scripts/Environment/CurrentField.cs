using UnityEngine;
using Force;

namespace Smarc.Environment
{
    /// <summary>
    /// A single water-current vector for a site, driving BOTH the physics and the visuals so
    /// they cannot disagree:
    ///   * the ForceFieldStatic on this object (pushes anything with ForcePoints),
    ///   * a global shader vector `_WaterCurrent` (drives vegetation and algae sway).
    ///
    /// OFF BY DEFAULT, DELIBERATELY. Vehicles carry ForcePoints, so a current changes their
    /// dynamics and every controller result measured at this site. A rig session that silently
    /// gained a current would read as a controller regression and cost a day chasing it. Same
    /// discipline as UNITY_BRIDGE_SIMULATED_LINK_MODE being named SIMULATED in full: the name
    /// says it is a scenario knob, not a measurement.
    ///
    /// Speed is a MODELLING ASSUMPTION, not survey data. Gullmarsfjorden surface currents are
    /// modest; 0.15 m/s is a plausible default and is recorded as an assumption in
    /// KRISTINEBERG_SITE.md. If a real current record turns up, put the number here and say so.
    /// </summary>
    [AddComponentMenu("Smarc/Environment/Current Field")]
    [RequireComponent(typeof(ForceFieldStatic))]
    public class CurrentField : MonoBehaviour
    {
        [Header("SCENARIO KNOB — off by default")]
        [Tooltip("Vehicles feel this. Turning it on changes vehicle dynamics and invalidates " +
                 "controller comparisons against runs made with it off.")]
        public bool CurrentEnabled = false;

        [Header("Current")]
        [Tooltip("Compass bearing the water flows TOWARDS, degrees; 0 = north (+Z), 90 = east (+X).")]
        [Range(0f, 360f)] public float HeadingDeg = 45f;
        [Tooltip("Metres per second. ASSUMPTION, not measured — see the class docs.")]
        [Min(0f)] public float SpeedMS = 0.15f;

        [Tooltip("Force applied per unit of current speed. The force field works in force, the " +
                 "world thinks in speed; this is the conversion, and it is a tuning value.")]
        public float ForcePerMS = 20f;

        [Header("Visual sway")]
        [Tooltip("Shader global that vegetation/algae sway reads. xyz = flow direction * speed, w = time scale.")]
        public string ShaderGlobalName = "_WaterCurrent";
        [Min(0f)] public float SwayFrequency = 0.35f;

        static readonly int SwayId = Shader.PropertyToID("_WaterCurrent");
        int _id;
        ForceFieldStatic _field;
        Collider _col;

        public Vector3 FlowVector => CurrentEnabled
            ? new Vector3(Mathf.Sin(HeadingDeg * Mathf.Deg2Rad), 0f, Mathf.Cos(HeadingDeg * Mathf.Deg2Rad)) * SpeedMS
            : Vector3.zero;

        void OnEnable() { Apply(); }
        void OnValidate() { Apply(); }
        void Update() { Apply(); }

        void Apply()
        {
            _id = string.IsNullOrEmpty(ShaderGlobalName) ? SwayId : Shader.PropertyToID(ShaderGlobalName);
            if (_field == null) _field = GetComponent<ForceFieldStatic>();

            var flow = FlowVector;

            if (_field != null)
            {
                _field.mode = StaticForceFieldMode.GlobalVector;
                _field.ForceVector = flow.sqrMagnitude > 1e-9f ? flow.normalized : Vector3.forward;
                _field.ForceMagnitude = CurrentEnabled ? SpeedMS * ForcePerMS : 0f;

                // OFF MEANS OUT OF THE PHYSICS SCENE, not merely zero-magnitude. Measured
                // 2026-08-16, both consequences of leaving a live trigger volume in the world:
                //   1) ForceFieldBase.OnTriggerStay still fired on every ForcePoint it
                //      overlapped, and ForcePoint.ApplyForce divides by
                //      RelatedForcePoints.Length — NullReferenceException storm from a
                //      "disabled" current;
                //   2) worse, the SONAR SAW IT. Sonar.cs casts with QueryParameters.Default,
                //      whose hitTriggers is UseGlobal, and Physics.queriesHitTriggers defaults
                //      to true — so a 600 x 60 x 500 m invisible box returned echoes across
                //      the whole farm at the 0.5 default reflectivity and label 0.
                // Disabling the component and its collider removes both.
                _field.enabled = CurrentEnabled;
                if (_col == null) _col = GetComponent<Collider>();
                if (_col != null) _col.enabled = CurrentEnabled;
            }

            // Same vector to the shaders. w carries the sway rate so one global covers it all.
            Shader.SetGlobalVector(_id, new Vector4(flow.x, flow.y, flow.z, SwayFrequency));
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.color = CurrentEnabled ? new Color(0.2f, 0.8f, 1f) : new Color(0.5f, 0.5f, 0.5f, 0.4f);
            var p = transform.position;
            var f = FlowVector;
            if (f.sqrMagnitude < 1e-9f)
                f = new Vector3(Mathf.Sin(HeadingDeg * Mathf.Deg2Rad), 0, Mathf.Cos(HeadingDeg * Mathf.Deg2Rad)) * 0.15f;
            Gizmos.DrawLine(p, p + f.normalized * 20f);
            Gizmos.DrawSphere(p + f.normalized * 20f, 1.5f);
        }
    }
}
