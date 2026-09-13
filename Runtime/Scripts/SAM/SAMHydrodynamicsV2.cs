using UnityEngine;
using DefaultNamespace.Water;

namespace Force
{
    /// <summary>
    /// SAM hydrodynamics, v2.1 (2026-09-11) — the full 6x6 coefficient set from
    /// data-cube/scripts/sam-sysid/sam_hydro_coeffs_v2_1.yaml, in a form you can test-run
    /// next to the existing SAMHydrodynamics (disable one, enable the other; never both).
    ///
    /// WHAT IS MEASURED (tank, validated out of sample) and therefore differs from SAMHydrodynamics:
    ///   * surge drag  F = -(Xu*u + Xuu*|u|*u)  with Xu 2.61, Xuu 8.92   (SAM.py: Xuu 3, 4.7x too little)
    ///   * thrust      per propeller, quadratic in eRPM feedback, reverse ratio 0.49 (Propeller.cs: linear, 0.6)
    ///   * neutral buoyancy: m = 16.85 kg
    /// Everything else is a PRIOR or carried from SAM.py and is exposed so it can be changed in the
    /// Inspector without recompiling. Provenance is in the field tooltips.
    ///
    /// FRAME. The body-local basis is (axisN = surge, t1 = lateral, t2 = vertical) exactly as in
    /// SAMHydrodynamics; DOF order for all 6x6 arrays is [u v w p q r] = [axisN, t1, t2, roll, pitch, yaw].
    /// Unity is LEFT-handed: a cross term whose sign was derived in Fossen's right-handed FRD frame can
    /// come out mirrored here. That is why the lift couplings (heave->pitch, sway->yaw) are produced
    /// PHYSICALLY by applying the transverse drag at the centre of pressure, and the explicit
    /// off-diagonal D entries default to zero. If you enter off-diagonal terms, check their sign in-sim.
    ///
    /// ADDED MASS is applied as the same stable per-DOF velocity-increment correction as v1 (diagonal
    /// only). Off-diagonal M_A is not applied (all zero in v2.1 anyway).
    ///
    /// ACTUATOR COUPLINGS (steering-rate -> roll, thrust x deflection -> roll) are included as an
    /// OPTIONAL block, OFF by default: the magnitudes are a regression on 30 min of STIM data (stable across
    /// four days, closed-loop skill only over ~1-2 s) and their SIGN in this frame is unverified.
    /// </summary>
    [DefaultExecutionOrder(100)]   // after ForcePoint, which rewrites the PhysX damping every step when wet
    public class SAMHydrodynamicsV2 : MonoBehaviour
    {
        [Header("Connected Body (assign the base_link ArticulationBody)")]
        public ArticulationBody ConnectedArticulationBody;
        public Rigidbody ConnectedRigidbody;

        [Header("Hull / rigid body (v0 rigid-body set, m from W=B in the tank)")]
        public float Length = 1.5f;
        public float Diameter = 0.19f;
        [Tooltip("MEASURED: neutral buoyancy in the tank => m = rho*nabla = 16.85 kg")]
        public float Mass = 16.85f;
        public Vector3 LongAxis = Vector3.forward;
        public float WaterDensity = 997f;   // tank = fresh water; 1026 at sea

        [Header("Linear damping D_L, diagonal [u v w p q r]  (N.s/m, N.m.s)")]
        [Tooltip("u: MEASURED 2.61 ± 0.38 | v,w,q,r: not identifiable (0) | p: 0.011 (regression, zeta~0.01, consistent with 0)")]
        public float[] DLin = { 2.61f, 0f, 0f, 0.011f, 0f, 0f };

        [Header("Quadratic damping D_Q, diagonal [u v w p q r]  (N.s2/m2, N.m.s2)")]
        [Tooltip("u: MEASURED 8.92 ± 0.81 | v,w,p,q,r: SAM.py 2021-23 tuning, NO measured provenance (surge suggests they are low)")]
        public float[] DQuad = { 8.92f, 50f, 50f, 40f, 200f, 10f };

        [Header("Off-diagonal linear damping (rows u..r, 6x6, row-major; default 0 — see FRAME note)")]
        public float[] DLinOffDiag = new float[36];

        [Header("Munk moment (prior): M_w = (Z_wdot - X_udot)*u, N_v = -(Y_vdot - X_udot)*u")]
        [Tooltip("Destabilising couple of a slender body at incidence. 15.40 kg = Z_wdot - X_udot from the Lamb prior. 0 disables.")]
        public float MunkCoefficient = 15.40f;
        [Tooltip("Centre-of-pressure offset [m] along the long axis where transverse drag is applied — gives heave->pitch / sway->yaw physically (SAM.py x_cp = 0.1).")]
        public float CenterOfPressureOffset = 0.1f;

        [Header("Added mass M_A, diagonal [u v w p q r]  (kg, kg.m2) — Lamb prior, spheroid a/b 7.89")]
        public bool UseAddedMass = true;
        public float[] AddedMass = { 0.503f, 15.90f, 15.90f, 0.041f, 1.611f, 1.611f };
        [Range(0f, 1f)] public float AddedMassScale = 1f;
        [Tooltip("Rigid inertia about CoM used to form the added-inertia fractions (v0: CAD radii x 16.85 kg)")]
        public Vector3 RigidInertia = new Vector3(0.135f, 2.192f, 2.102f); // roll, pitch, yaw

        [Header("Thrust (MEASURED) — set by the actuator adapter every FixedUpdate")]
        [Tooltip("eRPM feedback of propeller 1 and 2 (signed). Force_i = kT_i * (n/1000)^2 * sign")]
        public float Prop1eRPM, Prop2eRPM;
        public float kT1Fwd = 0.1031f, kT1Rev = 0.0529f, kT2Fwd = 0.1407f, kT2Rev = 0.0663f;
        public enum ThrustSourceMode { ExternalERPM, PropellerCommandRPM }
        [Tooltip("ExternalERPM: Prop1eRPM/Prop2eRPM are written by a replay/adapter (measured per-prop map). PropellerCommandRPM: read the two Propeller components' rpm (the command the controller sends) and use the command form 0.5*cQuad*rpm|rpm| per prop.")]
        public ThrustSourceMode ThrustSource = ThrustSourceMode.ExternalERPM;
        [Tooltip("Optional: the two Propeller components (front, back). Auto-found under the robot root when empty.")]
        public VehicleComponents.Actuators.Propeller[] Propellers;
        [Tooltip("Optional: transform whose position/forward is the thrust line (front_prop_link). Auto-found by name. When set, the thrust follows the articulated thrust-vector joints PHYSICALLY (steering roll via the motor-pack mass) and Delta1/Delta2 are ignored.")]
        public Transform ThrustTransform;
        public string ThrustTransformName = "front_prop_link";
        [Tooltip("If no eRPM feed is wired, use the command form: F_total = cQuad * rpm_cmd^2 (reverse x ReverseRatio)")]
        public bool UseCommandRPM = false;
        public float CommandRPM = 0f;
        public float cQuad = 7.99e-6f;
        public float ReverseRatio = 0.49f;
        [Tooltip("Nozzle deflections [rad]: channel 1 (vertical / elevator) and channel 2 (horizontal / rudder)")]
        public float Delta1, Delta2;
        [Tooltip("Thrust application point along the long axis, relative to base_link (nozzle joint is 0.677 m aft)")]
        public float ThrustArm = -0.677f;

        [Header("Actuator -> roll couplings (regression on STIM data, sign unverified in-sim; off by default)")]
        public bool UseSteeringRollCoupling = false;
        [Tooltip("N.m per rad/s of nozzle rate (fitted 0.019 = 0.109/s^2 x 0.1755 kg.m2; stable across 4 days)")]
        public float SteeringRateRoll = 0.019f;
        [Tooltip("N.m per (N x rad) of thrust x deflection (fitted 0.0093; same status)")]
        public float ThrustDeflectionRoll = 0.0093f;

        [Header("PhysX damping override")]
        [Tooltip("TEST-RUN FINDING 2026-09-11: the prefab's ForcePoints set the ArticulationBody linearDamping 1.02 / angularDamping 1.5 whenever wet. That isotropic PhysX drag (~m*1.02*u = 1.5 N at 0.12 m/s) dominated the measured Xu/Xuu and capped the surge at 0.12 m/s. With this on, V2 zeroes both every FixedUpdate so the identified D(nu) is the only resistance.")]
        public bool ZeroPhysXDamping = true;

        [Header("Water-surface coupling")]
        public float WaterQueryFrequency = 50f;
        public int StripCount = 10;

        [Header("Live telemetry (read-only)")]
        public float SubmergedFraction, SpeedThroughWater, ThrustForce;
        public Vector3 LastForceLocal, LastTorqueLocal;
        public float LogEverySeconds = 0f;

        MixedBody body; WaterQueryModel waterModel; float DLinPhysXWas, DAngPhysXWas;
        Vector3 axisN, t1, t2; Vector3[] stripLocal; float[] stripSubmerged;
        Vector3 prevLinVelBody, prevAngVelBody; bool havePrev;
        float lastQueryTime, cachedCenterLevel, lastLogTime, prevDelta1, prevDelta2; bool haveCenterLevel;

        void Awake()
        {
            body = new MixedBody(ConnectedArticulationBody, ConnectedRigidbody);
            if (body.isValid) { DLinPhysXWas = body.drag; DAngPhysXWas = body.angularDrag; }
            if (!body.isValid) { Debug.LogWarning($"{name}: SAMHydrodynamicsV2 needs a connected body."); enabled = false; return; }
            if (GetComponent<SAMHydrodynamics>() is SAMHydrodynamics v1 && v1.enabled)
                Debug.LogWarning($"{name}: both SAMHydrodynamics and SAMHydrodynamicsV2 are enabled — forces will be double-counted.");
            axisN = LongAxis.sqrMagnitude > 1e-9f ? LongAxis.normalized : Vector3.forward;
            Vector3 helper = Mathf.Abs(Vector3.Dot(axisN, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up;
            t1 = Vector3.Normalize(Vector3.Cross(axisN, helper));
            t2 = Vector3.Normalize(Vector3.Cross(axisN, t1));
            if (StripCount < 1) StripCount = 1;
            stripLocal = new Vector3[StripCount]; stripSubmerged = new float[StripCount];
            float dz = Length / StripCount;
            for (int i = 0; i < StripCount; i++) { stripLocal[i] = axisN * (-0.5f * Length + (i + 0.5f) * dz); stripSubmerged[i] = 1f; }
            if (DLin == null || DLin.Length != 6) DLin = new float[6];
            if (DQuad == null || DQuad.Length != 6) DQuad = new float[6];
            if (AddedMass == null || AddedMass.Length != 6) AddedMass = new float[6];
            if (DLinOffDiag == null || DLinOffDiag.Length != 36) DLinOffDiag = new float[36];
            waterModel = WaterQueryModel.GetWaterQueryModel(); lastQueryTime = -999f;
            Transform root = body.transform.root;
            if (ThrustTransform == null && !string.IsNullOrEmpty(ThrustTransformName))
                foreach (var tf in root.GetComponentsInChildren<Transform>(true)) if (tf.name == ThrustTransformName) { ThrustTransform = tf; break; }
            if (Propellers == null || Propellers.Length == 0) Propellers = root.GetComponentsInChildren<VehicleComponents.Actuators.Propeller>(true);
            if (ZeroPhysXDamping)
            {
                // the prefab's ForcePoints copy the body's damping into UnderwaterDrag at their Awake and write it back every
                // step; zero their copies too so no component re-applies the isotropic PhysX drag behind our back
                int n = 0;
                foreach (var fp in body.transform.root.GetComponentsInChildren<ForcePoint>(true))
                { fp.UnderwaterDrag = 0f; fp.UnderwaterAngularDrag = 0f; fp.AirDrag = 0f; fp.AirAngularDrag = 0f; n++; }
                body.drag = 0f; body.angularDrag = 0f;
                Debug.Log($"[SAMHydroV2] PhysX damping zeroed on {name} and on {n} ForcePoints (was {DLinPhysXWas:F2}/{DAngPhysXWas:F2})");
            }
        }

        /// <summary>Call after a teleport so the added-mass velocity-increment correction does not see the jump.</summary>
        public void ResetHistory() { havePrev = false; }

        float PropForce(float eRPM, float kFwd, float kRev)
        {
            float n = eRPM * 1e-3f; float f = n * Mathf.Abs(n);
            return f >= 0f ? kFwd * f : kRev * f;
        }

        void FixedUpdate()
        {
            if (body == null || !body.isValid) return;
            if (ZeroPhysXDamping) { body.drag = 0f; body.angularDrag = 0f; }
            Transform tr = body.transform;
            Vector3 linVelBody = tr.InverseTransformVector(body.velocity);
            Vector3 angVelBody = tr.InverseTransformDirection(body.angularVelocity);

            // ---- submergence (as v1) ----
            bool doQuery = WaterQueryFrequency <= 0f || (Time.time - lastQueryTime) >= 1f / WaterQueryFrequency;
            Vector3 center = tr.position;
            if (waterModel == null) waterModel = WaterQueryModel.GetWaterQueryModel();
            if (waterModel != null)
            {
                if (doQuery) { cachedCenterLevel = waterModel.GetWaterLevelAt(center); haveCenterLevel = true; lastQueryTime = Time.time; }
                float centerDepth = cachedCenterLevel - center.y; float span = 0.5f * Length + Diameter;
                if (centerDepth <= -span) { SubmergedFraction = 0f; return; }
                else if (centerDepth >= span) { for (int i = 0; i < StripCount; i++) stripSubmerged[i] = 1f; }
                else if (doQuery)
                    for (int i = 0; i < StripCount; i++)
                    { Vector3 wp = tr.TransformPoint(stripLocal[i]); stripSubmerged[i] = Mathf.Clamp01((waterModel.GetWaterLevelAt(wp) - wp.y) / Mathf.Max(Diameter, 1e-3f) + 0.5f); }
            }
            else for (int i = 0; i < StripCount; i++) stripSubmerged[i] = 1f;
            float sub = 0f; for (int i = 0; i < StripCount; i++) sub += stripSubmerged[i];
            SubmergedFraction = sub / StripCount; if (SubmergedFraction <= 0f) return;

            // ---- nu in the local basis ----
            float[] nu = {
                Vector3.Dot(linVelBody, axisN), Vector3.Dot(linVelBody, t1), Vector3.Dot(linVelBody, t2),
                Vector3.Dot(angVelBody, axisN), Vector3.Dot(angVelBody, t1), Vector3.Dot(angVelBody, t2) };
            SpeedThroughWater = linVelBody.magnitude;

            // ---- damping tau_d = -(D_L + D_Q|nu|) nu, diagonal + explicit off-diagonal ----
            float[] tau = new float[6];
            for (int i = 0; i < 6; i++)
            {
                float d = -(DLin[i] + DQuad[i] * Mathf.Abs(nu[i])) * nu[i];
                for (int j = 0; j < 6; j++) if (i != j) d -= DLinOffDiag[i * 6 + j] * nu[j];
                tau[i] = d * SubmergedFraction;
            }
            // Munk moment (prior): pitch from heave, yaw from sway, both scaled by surge speed
            if (MunkCoefficient != 0f)
            {
                tau[4] += MunkCoefficient * nu[0] * nu[2] * SubmergedFraction;
                tau[5] -= MunkCoefficient * nu[0] * nu[1] * SubmergedFraction;
            }

            Vector3 comWorld = tr.TransformPoint(body.centerOfMass);
            // surge at CoM; transverse drag at the CoP (physical lift coupling); rotational damping as torque
            body.AddForceAtPosition(tr.TransformVector(axisN * tau[0]), comWorld, ForceMode.Force);
            Vector3 fTrans = t1 * tau[1] + t2 * tau[2];
            body.AddForceAtPosition(tr.TransformVector(fTrans), tr.TransformPoint(body.centerOfMass + axisN * CenterOfPressureOffset), ForceMode.Force);
            Vector3 tRot = axisN * tau[3] + t1 * tau[4] + t2 * tau[5];
            body.AddTorque(tr.TransformDirection(tRot), ForceMode.Force);

            // ---- thrust (MEASURED) at the nozzle, deflected by delta1 (about t1) and delta2 (about t2) ----
            float T;
            if (UseCommandRPM) { float c = CommandRPM; T = cQuad * c * Mathf.Abs(c) * (c >= 0f ? 1f : ReverseRatio); }
            else if (ThrustSource == ThrustSourceMode.PropellerCommandRPM && Propellers != null && Propellers.Length > 0)
            {
                T = 0f;
                foreach (var pr in Propellers) { if (pr == null) continue; float c = pr.rpm; T += 0.5f * cQuad * c * Mathf.Abs(c) * (c >= 0f ? 1f : ReverseRatio); }
            }
            else T = PropForce(Prop1eRPM, kT1Fwd, kT1Rev) + PropForce(Prop2eRPM, kT2Fwd, kT2Rev);
            ThrustForce = T;
            if (Mathf.Abs(T) > 1e-6f && ThrustTransform != null)
            {
                // physical thrust vectoring: along the prop link's forward, at the prop link (joints + motor-pack mass do the rest)
                body.AddForceAtPosition(ThrustTransform.forward * T, ThrustTransform.position, ForceMode.Force);
            }
            else if (Mathf.Abs(T) > 1e-6f)
            {
                float c1 = Mathf.Cos(Delta1), s1 = Mathf.Sin(Delta1), c2 = Mathf.Cos(Delta2), s2 = Mathf.Sin(Delta2);
                Vector3 dirLocal = axisN * (c1 * c2) + t1 * (c1 * s2) - t2 * s1;      // check the deflection sign in-sim
                Vector3 pWorld = tr.TransformPoint(body.centerOfMass + axisN * ThrustArm);
                body.AddForceAtPosition(tr.TransformVector(dirLocal * T), pWorld, ForceMode.Force);
            }
            if (UseSteeringRollCoupling)
            {
                float d1 = (Delta1 - prevDelta1) / Time.fixedDeltaTime, d2 = (Delta2 - prevDelta2) / Time.fixedDeltaTime;
                float rollT = SteeringRateRoll * d1 + ThrustDeflectionRoll * T * Delta1;   // channel-1 terms were the ones with support
                body.AddTorque(tr.TransformDirection(axisN * rollT), ForceMode.Force);
            }
            prevDelta1 = Delta1; prevDelta2 = Delta2;

            LastForceLocal = axisN * tau[0] + fTrans; LastTorqueLocal = tRot;

            // ---- added mass: per-DOF velocity-increment correction (diagonal) ----
            if (UseAddedMass && havePrev && AddedMassScale > 0f)
            {
                Vector3 dv = linVelBody - prevLinVelBody, dw = angVelBody - prevAngVelBody;
                float[] rigid = { Mass, Mass, Mass, RigidInertia.x, RigidInertia.y, RigidInertia.z };
                float[] inc = { Vector3.Dot(dv, axisN), Vector3.Dot(dv, t1), Vector3.Dot(dv, t2), Vector3.Dot(dw, axisN), Vector3.Dot(dw, t1), Vector3.Dot(dw, t2) };
                float[] corr = new float[6];
                for (int i = 0; i < 6; i++) corr[i] = -AddedMass[i] / (rigid[i] + AddedMass[i]) * AddedMassScale * SubmergedFraction * inc[i];
                body.velocity += tr.TransformVector(axisN * corr[0] + t1 * corr[1] + t2 * corr[2]);
                body.angularVelocity += tr.TransformDirection(axisN * corr[3] + t1 * corr[4] + t2 * corr[5]);
            }
            prevLinVelBody = tr.InverseTransformVector(body.velocity); prevAngVelBody = tr.InverseTransformDirection(body.angularVelocity); havePrev = true;

            if (LogEverySeconds > 0f && Time.time - lastLogTime >= LogEverySeconds)
            { lastLogTime = Time.time; Debug.Log($"[SAMHydroV2] sub={SubmergedFraction:F2} U={SpeedThroughWater:F2} T={ThrustForce:F2}N Fd={LastForceLocal.magnitude:F1}N Td={LastTorqueLocal.magnitude:F2}Nm y={tr.position.y:F2} wl={cachedCenterLevel:F2} mTot={body.GetTotalConnectedMass():F2}kg physxDrag={body.drag:F2}/{body.angularDrag:F2} v={body.velocity.x:F3},{body.velocity.y:F3},{body.velocity.z:F3}"); }
        }
    }
}
