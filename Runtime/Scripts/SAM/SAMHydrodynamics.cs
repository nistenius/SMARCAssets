using UnityEngine;
using DefaultNamespace.Water;

namespace Force
{
    /// <summary>
    /// Hydrodynamic reaction model for SAM, folding in the coefficients of the
    /// project's validated 6-DOF model (smarc_modelling / SAM.py) and coupling them
    /// to the Unity water surface.
    ///
    /// WHY THIS EXISTS
    /// ---------------
    /// The live sam_auv_v1 prefab has NO hydrodynamic force model attached (both
    /// SAMForceModel.cs and SAMUnityArticulationModel.cs are unreferenced; the latter
    /// returns zero damping). SAM's only resistance is Unity's isotropic scalar
    /// ArticulationBody drag, which cannot represent AUV cross-flow drag, has no
    /// heave->pitch / sway->yaw coupling, no added mass, and is not tied to the water
    /// surface -- so heave/pitch are under-damped and SAM planes off wave crests.
    ///
    /// WHAT THIS APPLIES (defaults = smarc_modelling/SAM.py, water density 1026)
    ///   * Quadratic damping D(nu_r): Xuu, Yvv, Zww, Kpp, Mqq, Nrr, applied on the
    ///     velocity RELATIVE TO THE WATER (nu_r = nu - nu_water), exactly like the
    ///     validated model.
    ///   * Center-of-pressure coupling (x_cp): the transverse drag is applied at the
    ///     CoP, which reproduces the validated heave->pitch (D[4,2]) and sway->yaw
    ///     (D[5,1]) couplings automatically and with the correct sign.
    ///   * Added mass (MA = diag(m*k1, m*k2, m*k2, r44*Ix, k'*Iy, k'*Iy)); Lamb's
    ///     k-factors are computed from the hull geometry, matching SAM.py. For SAM the
    ///     transverse added mass is ~0.94*m -- large, and completely absent today.
    ///     Applied as a stable per-DOF velocity correction (implicit-like, fraction
    ///     < 1 of the last velocity increment), not an amplifying -MA*a force.
    ///   * Water-surface coupling: strips above the surface contribute nothing; a
    ///     straddling hull scales by the submerged fraction; the wave vertical orbital
    ///     velocity enters nu_r so the body is carried by the water, not only kicked.
    ///
    /// STRIP OPTION (UseDistributedCrossFlow)
    ///   The submerged fraction is always computed from StripCount (default 10) strips
    ///   distributed along the hull -- the same distributed idea as the buoyancy point
    ///   cloud. With UseDistributedCrossFlow = true, the transverse drag is applied
    ///   per strip (at each strip's position) instead of lumped at the CoP; the
    ///   distributed pitch/yaw damping then emerges from the integral, so the lumped
    ///   Mqq/Nrr are suppressed to avoid double-counting (roll Kpp stays lumped). Off
    ///   by default so the model reproduces the validated coefficients exactly.
    ///
    /// COST
    ///   No per-frame heap allocation (plain float/Vector3, preallocated buffers --
    ///   unlike the MathNet-based SAMForceModel which churned the GC). The HDRP water
    ///   query is gated: a fully submerged hull does at most one query per throttled
    ///   tick, an airborne hull does none, and per-strip queries happen only while
    ///   straddling the surface.
    ///
    /// FRAME: strips lie along the body-local long axis (SAM = local +Z; the
    /// ForcePoints span local z = +/-0.55). Surge = long axis; the transverse plane is
    /// the other two local axes. Verify LongAxis if the hull mesh is authored otherwise.
    /// </summary>
    public class SAMHydrodynamics : MonoBehaviour
    {
        [Header("Connected Body (assign the base_link ArticulationBody)")]
        public ArticulationBody ConnectedArticulationBody;
        public Rigidbody ConnectedRigidbody;

        [Header("Hull geometry (smarc_modelling SAM.py)")]
        public float Length = 1.5f;        // l_ss
        public float Diameter = 0.19f;     // d_ss
        public float Mass = 15.4f;         // ~ m_ss(14.9) + vbs + lcg; used for added mass & inertia
        public Vector3 LongAxis = Vector3.forward; // SAM = local +Z
        [Tooltip("Strips along the length. 10 mirrors the buoyancy point layout.")]
        public int StripCount = 10;

        [Header("Quadratic damping D(nu_r)  [smarc_modelling/SAM.py]")]
        public float WaterDensity = 1026f; // rho_w
        public float Xuu = 3f;             // surge
        public float Yvv = 50f;            // sway
        public float Zww = 50f;            // heave
        public float Kpp = 40f;            // roll
        public float Mqq = 200f;           // pitch
        public float Nrr = 10f;            // yaw
        [Tooltip("Center-of-pressure offset along the long axis [m] (SAM.py x_cp = 0.1). Gives the heave->pitch / sway->yaw coupling.")]
        public float CenterOfPressureOffset = 0.1f;

        [Header("Added mass (Lamb k-factors from geometry; SAM.py)")]
        public bool UseAddedMass = true;
        [Tooltip("r44: added roll inertia as a fraction of Ix (SAM.py = 0.3).")]
        public float R44 = 0.3f;
        [Tooltip("Global scale on the added-mass correction. Reduce toward 0 if you see instability at low physics rates.")]
        [Range(0f, 1f)] public float AddedMassScale = 1f;

        [Header("Strip distribution (optional)")]
        [Tooltip("Apply transverse drag distributed over the strips (surface-piercing realism) instead of lumped at the CoP. Suppresses lumped Mqq/Nrr to avoid double counting.")]
        public bool UseDistributedCrossFlow = false;
        [Tooltip("2D cross-flow drag coefficient used only in distributed mode (Hoerner cylinder ~1.1).")]
        public float CrossFlowCd = 1.1f;

        [Header("Water-surface coupling")]
        [Tooltip("Max water-surface queries per second (like ForcePoint). -1 = every FixedUpdate.")]
        public float WaterQueryFrequency = 50f;
        public bool UseWaveOrbitalVelocity = true;

        [Header("Live telemetry (read-only)")]
        public float SubmergedFraction;
        public float SpeedThroughWater;
        public Vector3 LastDampingForceWorld;
        public Vector3 LastDampingTorqueWorld;
        public bool DrawForces = false;
        [Tooltip("Log telemetry to the console every N seconds (0 = off). Handy for a quick in-sim sanity check.")]
        public float LogEverySeconds = 0f;

        // --- internals (preallocated; no per-frame allocation) ---
        MixedBody body;
        WaterQueryModel waterModel;

        Vector3[] stripLocal;
        float[] stripSubmerged;
        Vector3 axisN, t1, t2;      // long axis + two transverse unit axes (local)
        float stripAreaSide, axialArea, dz;

        // added mass per DOF
        float amSurge, amTrans, aiRoll, aiPitchYaw;

        Vector3 prevLinVelBodyAbs, prevAngVelBody;
        bool havePrev;

        float lastQueryTime, cachedCenterLevel, prevCenterLevel, lastLogTime;
        bool haveCenterLevel;

        void Awake()
        {
            body = new MixedBody(ConnectedArticulationBody, ConnectedRigidbody);
            if (!body.isValid) { Debug.LogWarning($"{name}: SAMHydrodynamics needs a connected body."); enabled = false; return; }

            axisN = LongAxis.sqrMagnitude > 1e-9f ? LongAxis.normalized : Vector3.forward;
            // build a stable transverse basis
            Vector3 helper = Mathf.Abs(Vector3.Dot(axisN, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up;
            t1 = Vector3.Normalize(Vector3.Cross(axisN, helper)); // lateral  -> pitch axis
            t2 = Vector3.Normalize(Vector3.Cross(axisN, t1));     // vertical -> yaw axis

            if (StripCount < 1) StripCount = 1;
            stripLocal = new Vector3[StripCount];
            stripSubmerged = new float[StripCount];
            dz = Length / StripCount;
            for (int i = 0; i < StripCount; i++)
            {
                float s = -0.5f * Length + (i + 0.5f) * dz;
                stripLocal[i] = axisN * s;
                stripSubmerged[i] = 1f;
            }
            float radius = 0.5f * Diameter;
            stripAreaSide = Diameter * dz;
            axialArea = Mathf.PI * radius * radius;

            // Lamb's added-mass k-factors from the prolate-spheroid geometry (SAM.py)
            float a = 0.5f * Length, b = 0.5f * Diameter;
            float e = Mathf.Sqrt(Mathf.Max(1e-6f, 1f - (b * b) / (a * a)));
            float ln = Mathf.Log((1f + e) / (1f - e));
            float alpha0 = (2f * (1f - e * e) / (e * e * e)) * (0.5f * ln - e);
            float beta0 = 1f / (e * e) - (1f - e * e) / (2f * e * e * e) * ln;
            float k1 = alpha0 / (2f - alpha0);
            float k2 = beta0 / (2f - beta0);
            float kprime = Mathf.Pow(e, 4f) * (beta0 - alpha0) /
                           ((2f - e * e) * (2f * e * e - (2f - e * e) * (beta0 - alpha0)));
            float Ix = 0.4f * Mass * b * b;           // solid prolate spheroid
            float Iy = 0.2f * Mass * (a * a + b * b);
            amSurge = Mass * k1;
            amTrans = Mass * k2;
            aiRoll = R44 * Ix;
            aiPitchYaw = kprime * Iy;

            waterModel = WaterQueryModel.GetWaterQueryModel();
            lastQueryTime = -999f;
        }

        void FixedUpdate()
        {
            if (body == null || !body.isValid) return;
            Transform tr = body.transform;

            // --- velocities in body-local frame ---
            Vector3 linVelBodyAbs = tr.InverseTransformVector(body.velocity);
            Vector3 angVelBody = tr.InverseTransformDirection(body.angularVelocity);

            // --- submergence (few water queries as possible) ---
            bool doQuery = WaterQueryFrequency <= 0f || (Time.time - lastQueryTime) >= 1f / WaterQueryFrequency;
            Vector3 center = tr.position;
            if (waterModel == null) waterModel = WaterQueryModel.GetWaterQueryModel();

            float verticalWaterVel = 0f;
            if (waterModel != null)
            {
                if (doQuery)
                {
                    float level = waterModel.GetWaterLevelAt(center);
                    if (haveCenterLevel && UseWaveOrbitalVelocity)
                    {
                        float dtq = (WaterQueryFrequency <= 0f) ? Time.fixedDeltaTime : (Time.time - lastQueryTime);
                        if (dtq > 1e-4f) verticalWaterVel = (level - prevCenterLevel) / dtq;
                    }
                    prevCenterLevel = level; cachedCenterLevel = level; haveCenterLevel = true; lastQueryTime = Time.time;
                }
                float centerDepth = cachedCenterLevel - center.y;
                float span = 0.5f * Length + Diameter;
                if (centerDepth <= -span) { SubmergedFraction = 0f; return; }        // airborne: no hydro
                else if (centerDepth >= span) { for (int i = 0; i < StripCount; i++) stripSubmerged[i] = 1f; }
                else if (doQuery)                                                     // straddling: per-strip (throttled)
                {
                    for (int i = 0; i < StripCount; i++)
                    {
                        Vector3 wp = tr.TransformPoint(stripLocal[i]);
                        float frac = (waterModel.GetWaterLevelAt(wp) - wp.y) / Mathf.Max(Diameter, 1e-3f) + 0.5f;
                        stripSubmerged[i] = Mathf.Clamp01(frac);
                    }
                }
            }
            else { for (int i = 0; i < StripCount; i++) stripSubmerged[i] = 1f; }

            float submergedSum = 0f;
            for (int i = 0; i < StripCount; i++) submergedSum += stripSubmerged[i];
            SubmergedFraction = submergedSum / StripCount;
            if (SubmergedFraction <= 0f) return;

            // water velocity in body frame (vertical orbital component, cheap & dominant)
            Vector3 waterVelBody = UseWaveOrbitalVelocity ? tr.InverseTransformVector(new Vector3(0f, verticalWaterVel, 0f)) : Vector3.zero;
            Vector3 vRel = linVelBodyAbs - waterVelBody; // nu_r linear part, local

            // decompose into surge / transverse
            float u = Vector3.Dot(vRel, axisN);
            float sway = Vector3.Dot(vRel, t1);
            float heave = Vector3.Dot(vRel, t2);
            float p = Vector3.Dot(angVelBody, axisN);   // roll
            float q = Vector3.Dot(angVelBody, t1);      // pitch
            float r = Vector3.Dot(angVelBody, t2);      // yaw
            SpeedThroughWater = vRel.magnitude;

            Vector3 comWorld = tr.TransformPoint(body.centerOfMass);
            Vector3 forceAccum = Vector3.zero, torqueAccum = Vector3.zero;

            // --- surge (axial) quadratic drag at CoM ---
            {
                Vector3 fLocal = axisN * (-Xuu * Mathf.Abs(u) * u) * SubmergedFraction;
                Vector3 fWorld = tr.TransformVector(fLocal);
                body.AddForceAtPosition(fWorld, comWorld, ForceMode.Force);
                forceAccum += fWorld;
            }

            if (!UseDistributedCrossFlow)
            {
                // ---- FAITHFUL LUMPED (default): reproduces SAM.py D exactly ----
                // transverse drag at the centre of pressure -> also yields the
                // heave->pitch / sway->yaw couplings with the correct sign.
                Vector3 fTransLocal = t1 * (-Yvv * Mathf.Abs(sway) * sway) + t2 * (-Zww * Mathf.Abs(heave) * heave);
                fTransLocal *= SubmergedFraction;
                Vector3 fTransWorld = tr.TransformVector(fTransLocal);
                Vector3 cpWorld = tr.TransformPoint(body.centerOfMass + axisN * CenterOfPressureOffset);
                body.AddForceAtPosition(fTransWorld, cpWorld, ForceMode.Force);
                forceAccum += fTransWorld;

                // pure rotational damping: roll, pitch, yaw
                Vector3 tLocal = axisN * (-Kpp * Mathf.Abs(p) * p)
                               + t1 * (-Mqq * Mathf.Abs(q) * q)
                               + t2 * (-Nrr * Mathf.Abs(r) * r);
                tLocal *= SubmergedFraction;
                Vector3 tWorld = tr.TransformDirection(tLocal);
                body.AddTorque(tWorld, ForceMode.Force);
                torqueAccum += tWorld;
            }
            else
            {
                // ---- DISTRIBUTED STRIPS (optional): transverse drag per strip ----
                // Yields translational transverse damping AND the pitch/yaw rate
                // damping from the integral, so lumped Mqq/Nrr are dropped here.
                float halfRho = 0.5f * WaterDensity;
                for (int i = 0; i < StripCount; i++)
                {
                    float subm = stripSubmerged[i];
                    if (subm <= 0f) continue;
                    Vector3 vStrip = vRel + Vector3.Cross(angVelBody, stripLocal[i]);
                    Vector3 vAx = Vector3.Dot(vStrip, axisN) * axisN;
                    Vector3 vTr = vStrip - vAx;
                    float vt = vTr.magnitude;
                    if (vt < 1e-5f) continue;
                    Vector3 fLocal = (-halfRho * CrossFlowCd * stripAreaSide * subm * vt) * vTr;
                    Vector3 fWorld = tr.TransformVector(fLocal);
                    Vector3 wp = tr.TransformPoint(stripLocal[i]);
                    body.AddForceAtPosition(fWorld, wp, ForceMode.Force);
                    forceAccum += fWorld;
                    if (DrawForces) Debug.DrawLine(wp, wp + fWorld * 0.01f, Color.cyan, 0f);
                }
                // roll still needs a lumped term (strips give ~no roll damping)
                Vector3 tWorld = tr.TransformDirection(axisN * (-Kpp * Mathf.Abs(p) * p) * SubmergedFraction);
                body.AddTorque(tWorld, ForceMode.Force);
                torqueAccum += tWorld;
            }

            LastDampingForceWorld = forceAccum;
            LastDampingTorqueWorld = torqueAccum;
            if (DrawForces) Debug.DrawLine(comWorld, comWorld + forceAccum * 0.01f, Color.yellow, 0f);

            // --- added mass: stable per-DOF velocity correction (implicit-like) ---
            // Removes a fraction f = MA/(m+MA) of the last velocity increment, which
            // emulates the added inertia without the blow-ups of an explicit -MA*a force.
            if (UseAddedMass && havePrev && AddedMassScale > 0f)
            {
                float m = Mass;
                float fSurge = amSurge / (m + amSurge) * AddedMassScale * SubmergedFraction;
                float fTrans = amTrans / (m + amTrans) * AddedMassScale * SubmergedFraction;

                Vector3 dvLocal = linVelBodyAbs - prevLinVelBodyAbs;
                float dU = Vector3.Dot(dvLocal, axisN);
                float dS = Vector3.Dot(dvLocal, t1);
                float dH = Vector3.Dot(dvLocal, t2);
                Vector3 corrLocal = axisN * (-fSurge * dU) + t1 * (-fTrans * dS) + t2 * (-fTrans * dH);
                body.velocity = body.velocity + tr.TransformVector(corrLocal);

                // rotational added inertia (need the rigid inertia to form the fraction)
                Vector3 It = ConnectedArticulationBody ? ConnectedArticulationBody.inertiaTensor
                            : (ConnectedRigidbody ? ConnectedRigidbody.inertiaTensor : Vector3.one);
                float ItransApprox = Mathf.Max(1e-4f, (It.x + It.y + It.z) / 3f);
                float fRoll = aiRoll / (ItransApprox + aiRoll) * AddedMassScale * SubmergedFraction;
                float fPitchYaw = aiPitchYaw / (ItransApprox + aiPitchYaw) * AddedMassScale * SubmergedFraction;

                Vector3 dwLocal = angVelBody - prevAngVelBody;
                float dP = Vector3.Dot(dwLocal, axisN);
                float dQ = Vector3.Dot(dwLocal, t1);
                float dR = Vector3.Dot(dwLocal, t2);
                Vector3 corrAng = axisN * (-fRoll * dP) + t1 * (-fPitchYaw * dQ) + t2 * (-fPitchYaw * dR);
                body.angularVelocity = body.angularVelocity + tr.TransformDirection(corrAng);
            }

            prevLinVelBodyAbs = linVelBodyAbs;
            prevAngVelBody = angVelBody;
            havePrev = true;

            if (LogEverySeconds > 0f && (Time.time - lastLogTime) >= LogEverySeconds)
            {
                lastLogTime = Time.time;
                Debug.Log($"[SAMHydro] sub={SubmergedFraction:F2} U={SpeedThroughWater:F2} " +
                          $"F={LastDampingForceWorld.magnitude:F1}N T={LastDampingTorqueWorld.magnitude:F1}Nm");
            }
        }
    }
}
