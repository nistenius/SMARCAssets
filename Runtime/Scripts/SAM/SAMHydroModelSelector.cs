using UnityEngine;
using VehicleComponents.Actuators;

namespace Force
{
    /// <summary>
    /// One switch on the SAM prefab root to choose the hydrodynamic model that runs:
    ///   V1  = SAMHydrodynamics (SAM.py 2021-23 coefficients, Propeller.cs thrust 0.005 N/rpm, PhysX drag 1.02/1.5)
    ///   V2  = SAMHydrodynamicsV2 (tank-identified surge/thrust, v2.1 6x6 set, PhysX drag zeroed, thrust at front_prop_link)
    /// In V2 mode the Propeller components keep spinning the props (visual + joint) but their force multiplier is set
    /// to zero, because V2 applies the measured thrust itself. Switching back restores the V1 multiplier.
    /// Works in the Inspector (OnValidate) and at runtime (Awake / SetModel).
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public class SAMHydroModelSelector : MonoBehaviour
    {
        public enum HydroModel { V1_SAMpy2021, V2_TankIdentified2026 }
        [Tooltip("Which hydrodynamic model drives this SAM")]
        public HydroModel Model = HydroModel.V2_TankIdentified2026;
        [Tooltip("Propeller.RPMToForceMultiplier to restore in V1 mode (prefab value 0.005)")]
        public float V1PropellerForceMultiplier = 0.005f;
        [Header("Resolved at Apply (read-only)")]
        public SAMHydrodynamics V1;
        public SAMHydrodynamicsV2 V2;
        public Propeller[] Propellers;

        void Awake() { Apply(); }
        void OnValidate() { if (!Application.isPlaying) Apply(); }

        public void SetModel(HydroModel m) { Model = m; Apply(); }

        [ContextMenu("Apply model selection")]
        public void Apply()
        {
            if (V1 == null) V1 = GetComponentInChildren<SAMHydrodynamics>(true);
            if (V2 == null) V2 = GetComponentInChildren<SAMHydrodynamicsV2>(true);
            if (Propellers == null || Propellers.Length == 0) Propellers = GetComponentsInChildren<Propeller>(true);
            bool v2 = Model == HydroModel.V2_TankIdentified2026;
            if (V1 != null) V1.enabled = !v2;
            if (V2 != null)
            {
                V2.enabled = v2;
                if (v2 && V2.ThrustSource == SAMHydrodynamicsV2.ThrustSourceMode.ExternalERPM && !V2.UseCommandRPM && V2.Propellers != null && V2.Propellers.Length == 0)
                    V2.ThrustSource = SAMHydrodynamicsV2.ThrustSourceMode.PropellerCommandRPM;
            }
            if (Propellers != null)
                foreach (var p in Propellers) if (p != null) p.RPMToForceMultiplier = v2 ? 0f : V1PropellerForceMultiplier;
            if (Application.isPlaying) Debug.Log($"[SAMHydroModelSelector] {name}: {(v2 ? "V2 (tank-identified)" : "V1 (SAM.py)")} — V1 {(V1 ? (V1.enabled ? "on" : "off") : "missing")}, V2 {(V2 ? (V2.enabled ? "on" : "off") : "missing")}, props {(Propellers != null ? Propellers.Length : 0)}");
        }
    }
}
