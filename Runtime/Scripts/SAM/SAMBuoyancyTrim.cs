using System.IO;
using System.Text;
using UnityEngine;
using VehicleComponents.Actuators;

namespace Force
{
    /// <summary>
    /// Makes SAM's rigid body and hydrostatics explicit and correct, and trims the vehicle so it is
    /// NEUTRAL at a chosen VBS setting — floating with the tank empty, sinking with it full — and level.
    ///
    /// WHY THIS EXISTS. Four things were wrong in the prefab, and they interact, so they are fixed together:
    ///   1. DISPLACEMENT. The ForcePoints left Volume = 0, so each computed its share from the HULL MESH,
    ///      which is still the 2016 concept hull: 1.334 m x 126 mm, 15.25 L. The real vehicle displaces
    ///      16.90 L (16.852 kg-f measured in fresh water). The CAD bare hull is 15.74 L; the remaining
    ///      1.16 L is the external rails, handles, transducers and penetrators.
    ///   2. WEIGHT. The links summed to 16.96 kg serialised, 17.01-17.26 kg once VBS.cs took over the tank
    ///      mass — against 15.21 kg-f of buoyancy. The vehicle was neither neutral nor consistently trimmed.
    ///   3. LONGITUDINAL BALANCE. With the 0.808 kg motor pack at 0.74 m aft the composite CG sits 47 mm
    ///      AFT of the centre of buoyancy. A submerged body hangs with its CG under its CB, so at
    ///      BG = 15 mm that is a 78 degree nose-up trim. Nothing else can be tested until this is fixed.
    ///   4. INERTIA. base_link carried (1.6202, 1.6202, 0.0293) from the 2016 concept. The measured vehicle
    ///      is I_pitch 2.192, I_yaw 2.102, I_roll 0.135 kg.m2 about its CG (CAD radii of gyration x 16.85 kg).
    ///
    /// WHAT IT DOES, in Start, after UrdfInertial has pushed its own values in (execution order 100):
    ///   a. Distributes DisplacedVolumeLitres over every ForcePoint, so buoyancy is a stated number.
    ///   b. Adjusts the trim link's MASS so total weight == buoyancy at NeutralAtVBSPercent.
    ///   c. Adjusts the trim link's CENTRE OF MASS so the composite CG lands at the target x_BG and BG.
    ///   d. Sets the trim link's INERTIA TENSOR so the composite matches the measured whole-vehicle inertia,
    ///      after removing every other link's own inertia and parallel-axis contribution.
    ///   e. Logs the whole balance, and warns about anything it had to clamp.
    ///
    /// The VBS takes water aboard from OUTSIDE, so it changes mass without changing displacement — that is
    /// its entire authority: 0.249 kg, 2.44 N end to end, +/- 1.22 N about a 50 % neutral point.
    ///
    /// KNOWN PLACEHOLDER, NOT FIXED HERE: the two 1 kg transceiver links at +0.70 and -0.80 m carry 51 %
    /// of the vehicle's pitch inertia between them. The total is made correct by (d), but the distribution
    /// is only as good as those two masses, which nobody has weighed.
    /// </summary>
    [DefaultExecutionOrder(100)]
    public class SAMBuoyancyTrim : MonoBehaviour
    {
        [Header("Hydrostatics (measured)")]
        [Tooltip("Displaced volume INCLUDING the external rails and fittings. 16.90 L = the measured 16.852 kg-f / 997. CAD bare hull alone is 15.74 L.")]
        public float DisplacedVolumeLitres = 16.90f;
        [Tooltip("997 = fresh tank water. 1026 = sea water, which adds 0.49 kg-f - twenty times the VBS authority in one direction.")]
        public float WaterDensity = 997f;

        [Header("Trim targets")]
        [Tooltip("VBS percentage at which the vehicle is exactly neutral. 50 gives symmetric authority: floats empty, sinks full.")]
        [Range(0f, 100f)] public float NeutralAtVBSPercent = 50f;
        [Tooltip("Longitudinal CG offset from the CB, positive forward [m]. Nothing has ever measured this on the real vehicle; 0 is the working assumption and E2 would settle it.")]
        public float TargetXBG = 0f;
        [Tooltip("Vertical CG-below-CB offset [m]. 0.0146 is the value implied by the measured 0.55-0.63 Hz roll band; the roll regression instead suggests 0.005. E3 decides.")]
        public float TargetBG = 0.0146f;

        [Header("Rigid body (measured, about the vehicle CG)")]
        public bool SetInertia = true;
        [Tooltip("Floor on the trim link's own inertia, as a fraction of the whole-vehicle target. A near-zero tensor on the articulation root stops PhysX integrating the body entirely.")]
        public float MinOwnInertiaFraction = 0.10f;
        [Tooltip("I_pitch, I_yaw, I_roll [kg.m2] - CAD radii of gyration 0.3603 / 0.3528 / 0.0894 m applied to the measured mass.")]
        public Vector3 WholeVehicleInertia = new Vector3(2.192f, 2.102f, 0.135f);

        [Header("Which links")]
        public string TrimLinkName = "base_link";
        public string VbsLinkName = "vbs_link";
        public bool ApplyTrim = true;

        [Header("Result (read-only)")]
        public float BuoyancyKgf, TotalMassKg, TrimMassAppliedKg;
        public Vector3 CompositeCgBody;      // x fwd, y stbd, z down, from the CB station
        public Vector3 CentreOfBuoyancyBody;   // read-only: where the ForcePoints actually push, in body axes from base_link
        public float NetForceEmptyN, NetForceFullN;
        public Vector3 BaseLinkOwnInertia;

        /// <summary>Optional overrides from StreamingAssets/SAMReplay/dof/config.txt (keys prefixed trim_).</summary>
        void ReadTrimConfig()
        {
            string cfg = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "dof", "config.txt");
            if (!File.Exists(cfg)) return;
            var ci = System.Globalization.CultureInfo.InvariantCulture;
            foreach (string line in File.ReadAllLines(cfg))
            {
                string t = line.Trim(); if (t.Length == 0 || t.StartsWith("#")) continue;
                int eq = t.IndexOf('='); if (eq < 0) continue;
                string k = t.Substring(0, eq).Trim().ToLowerInvariant(), v = t.Substring(eq + 1).Trim();
                float f; bool b;
                switch (k)
                {
                    case "trim_apply": if (bool.TryParse(v, out b)) ApplyTrim = b; break;
                    case "trim_set_inertia": if (bool.TryParse(v, out b)) SetInertia = b; break;
                    case "trim_min_inertia_fraction": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) MinOwnInertiaFraction = f; break;
                    case "trim_neutral_vbs": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) NeutralAtVBSPercent = f; break;
                    case "trim_displaced_litres": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) DisplacedVolumeLitres = f; break;
                }
            }
        }

        void Start()
        {
            ReadTrimConfig();
            Transform root = transform.root;
            var bodies = root.GetComponentsInChildren<ArticulationBody>(true);
            var points = root.GetComponentsInChildren<ForcePoint>(true);
            var vbs = root.GetComponentInChildren<VBS>(true);
            ArticulationBody trimLink = null, vbsLink = null;
            foreach (var ab in bodies)
            {
                if (ab.name == TrimLinkName) trimLink = ab;
                if (ab.name == VbsLinkName) vbsLink = ab;
            }
            if (points.Length == 0 || trimLink == null)
            {
                Debug.LogWarning($"{name}: SAMBuoyancyTrim found {points.Length} ForcePoints, trim link '{TrimLinkName}' {(trimLink ? "ok" : "MISSING")} - doing nothing.");
                enabled = false; return;
            }

            // (a) displacement, stated rather than inherited from an old mesh.
            // MEASURED 2026-09-12: ForcePoint.ApplyForce divides the force it is handed by the number of
            // points on the SAME body ("related points"), so a point's Volume field is the whole volume of
            // its GROUP, and a vehicle whose points sit on several links gets one group's worth of buoyancy
            // PER LINK. Setting every point to V_total therefore over-floats the vehicle by the number of
            // groups; dividing by the point count under-floats it by the same factor. The correct
            // assignment is: each point carries its group's share, which is V_total * n_group / n_total.
            var groupCount = new System.Collections.Generic.Dictionary<Object, int>();
            foreach (var p in points)
            {
                Object key = (Object)p.ConnectedArticulationBody ?? (Object)p.ConnectedRigidbody;
                if (key == null) continue;
                groupCount[key] = groupCount.TryGetValue(key, out int n) ? n + 1 : 1;
            }
            float vTotal = DisplacedVolumeLitres * 0.001f;
            float vEach = vTotal;               // reported value for the single-group case
            foreach (var p in points)
            {
                Object key = (Object)p.ConnectedArticulationBody ?? (Object)p.ConnectedRigidbody;
                int n = (key != null && groupCount.TryGetValue(key, out int c)) ? c : points.Length;
                p.Volume = vTotal * n / points.Length;
                p.WaterDensity = WaterDensity; p.MaxBuoyancyForce = 1000f;
                // MEASURED 2026-09-12: with WaterQueryFrequency > 0 the buoyancy is multiplied by
                // waterForceScale = 1/dt/freq to compensate for being applied only on query ticks - but the
                // tick timer runs on Clock.Now, so it fires every FixedUpdate anyway and the scale becomes a
                // straight ~12 % buoyancy surplus (a constant +19.6 N, seen identically at the surface and at
                // depth) that gravity never gets. Querying every step makes the scale exactly 1.
                p.WaterQueryFrequency = -1f;
                vEach = p.Volume;
            }
            if (groupCount.Count > 1)
                Debug.Log($"{name}: the {points.Length} ForcePoints sit on {groupCount.Count} bodies - volume split by group so the total displacement is {DisplacedVolumeLitres:F2} L.");
            BuoyancyKgf = DisplacedVolumeLitres * 0.001f * WaterDensity;

            // (b) mass
            float vbsFixed = 0.300f, vbsWater = vbs != null ? vbs.density / 1000f * vbs.maxVolume_l : 0f;
            float vbsAtNeutral = vbsFixed + vbsWater * NeutralAtVBSPercent / 100f;
            float others = 0f;
            foreach (var ab in bodies)
            {
                if (!ab.useGravity || ab == vbsLink || ab == trimLink) continue;
                others += ab.mass;
            }
            float newTrimMass = BuoyancyKgf - vbsAtNeutral - others;
            TrimMassAppliedKg = newTrimMass - trimLink.mass;
            if (ApplyTrim)
            {
                if (newTrimMass <= 0.01f)
                    Debug.LogWarning($"{name}: trim would need {TrimLinkName} mass {newTrimMass:F3} kg - the other links already outweigh the buoyancy. Not applied.");
                else trimLink.mass = newTrimMass;
            }
            TotalMassKg = others + vbsAtNeutral + trimLink.mass;

            // (c) centre of mass, so the composite CG lands on the target.
            // MEASURED 2026-09-12: the centre of buoyancy is where the ForcePoints actually push, i.e. their
            // centroid - NOT the base_link origin. Referencing the target to the origin left an 8 mm
            // longitudinal CG-CB offset and the vehicle pitched up at 0.7 deg/s with everything neutral.
            // Each point applies the same force (ForcePoint divides by the point count), so the effective
            // centre is the plain mean of the point positions.
            Vector3 cbWorld = Vector3.zero;
            foreach (var p in points) cbWorld += p.transform.position;
            cbWorld /= points.Length;
            Vector3 cbLocal = trimLink.transform.InverseTransformPoint(cbWorld);
            CentreOfBuoyancyBody = new Vector3(cbLocal.z, cbLocal.x, -cbLocal.y);
            Vector3 cgTargetWorld = cbWorld
                                  + trimLink.transform.TransformVector(new Vector3(0f, -TargetBG, TargetXBG));
            Vector3 sumOther = Vector3.zero; float mOther = 0f;
            foreach (var ab in bodies)
            {
                if (!ab.useGravity || ab == trimLink) continue;
                float m = (ab == vbsLink) ? vbsAtNeutral : ab.mass;
                if (m < 1e-5f) continue;
                sumOther += m * ab.transform.TransformPoint(ab.centerOfMass); mOther += m;
            }
            if (ApplyTrim && trimLink.mass > 0.01f)
            {
                Vector3 comWorld = (TotalMassKg * cgTargetWorld - sumOther) / trimLink.mass;
                trimLink.centerOfMass = trimLink.transform.InverseTransformPoint(comWorld);
            }

            // recompute the composite CG for reporting
            Vector3 sumAll = sumOther + trimLink.mass * trimLink.transform.TransformPoint(trimLink.centerOfMass);
            Vector3 cgWorld = sumAll / TotalMassKg;
            Vector3 cgLocal = trimLink.transform.InverseTransformPoint(cgWorld) - cbLocal;   // relative to the CB
            CompositeCgBody = new Vector3(cgLocal.z, cgLocal.x, -cgLocal.y);

            // (d) inertia, so the composite matches the measured whole-vehicle figures
            if (SetInertia && ApplyTrim && trimLink.mass > 0.01f)
            {
                Vector3 par = Vector3.zero;   // unity axes: x = pitch, y = yaw, z = roll
                foreach (var ab in bodies)
                {
                    if (!ab.useGravity || ab == trimLink) continue;
                    float m = (ab == vbsLink) ? vbsAtNeutral : ab.mass;
                    if (m < 1e-5f) continue;
                    Vector3 d = ab.transform.TransformPoint(ab.centerOfMass) - cgWorld;
                    par += ab.inertiaTensor + new Vector3(m * (d.y * d.y + d.z * d.z),
                                                          m * (d.x * d.x + d.z * d.z),
                                                          m * (d.x * d.x + d.y * d.y));
                }
                Vector3 dT = trimLink.transform.TransformPoint(trimLink.centerOfMass) - cgWorld;
                Vector3 own = WholeVehicleInertia - par - new Vector3(
                    trimLink.mass * (dT.y * dT.y + dT.z * dT.z),
                    trimLink.mass * (dT.x * dT.x + dT.z * dT.z),
                    trimLink.mass * (dT.x * dT.x + dT.y * dT.y));
                // A near-zero tensor on the articulation ROOT makes PhysX refuse to integrate the body at
                // all (measured 2026-09-12: the vehicle sat motionless under a 150 N imbalance), so the
                // infeasible case falls back to a physically sane share of the target instead of ~0.
                if (own.x <= MinOwnInertiaFraction * WholeVehicleInertia.x ||
                    own.y <= MinOwnInertiaFraction * WholeVehicleInertia.y ||
                    own.z <= MinOwnInertiaFraction * WholeVehicleInertia.z)
                    Debug.LogWarning($"{name}: the other links already account for {par} of the whole-vehicle inertia {WholeVehicleInertia} - the composite will be HIGH. {TrimLinkName} floored at {MinOwnInertiaFraction:P0} of target. Check the placeholder link masses (the two 1 kg transceivers carry about half the pitch inertia).");
                BaseLinkOwnInertia = new Vector3(
                    Mathf.Max(own.x, MinOwnInertiaFraction * WholeVehicleInertia.x),
                    Mathf.Max(own.y, MinOwnInertiaFraction * WholeVehicleInertia.y),
                    Mathf.Max(own.z, MinOwnInertiaFraction * WholeVehicleInertia.z));
                trimLink.inertiaTensor = BaseLinkOwnInertia;
                trimLink.inertiaTensorRotation = Quaternion.identity;
            }

            float g = Mathf.Abs(Physics.gravity.y);
            NetForceEmptyN = (BuoyancyKgf - (TotalMassKg - vbsWater * NeutralAtVBSPercent / 100f)) * g;
            NetForceFullN = (BuoyancyKgf - (TotalMassKg + vbsWater * (100f - NeutralAtVBSPercent) / 100f)) * g;

            var sb = new StringBuilder();
            sb.AppendLine($"[SAMBuoyancyTrim] {points.Length} ForcePoints on {groupCount.Count} bodies, {vEach * 1000f:F3} L per point = {DisplacedVolumeLitres:F2} L at {WaterDensity:F0} kg/m3 -> buoyancy {BuoyancyKgf:F3} kg-f");
            sb.AppendLine($"   {TrimLinkName}: mass {TrimMassAppliedKg:+0.000;-0.000} kg -> {trimLink.mass:F3} kg, CoM -> {trimLink.centerOfMass}, own inertia -> {BaseLinkOwnInertia}");
            sb.AppendLine($"   centre of buoyancy (ForcePoint centroid, body axes from base_link) = {CentreOfBuoyancyBody}");
            sb.AppendLine($"   total {TotalMassKg:F3} kg at VBS {NeutralAtVBSPercent:F0} %   composite CG (x fwd, y stbd, z down from the CB) = {CompositeCgBody}");
            sb.AppendLine($"   VBS   0 %: net {NetForceEmptyN:+0.00;-0.00} N  {(NetForceEmptyN > 0 ? "FLOATS" : "sinks")}");
            sb.AppendLine($"   VBS {NeutralAtVBSPercent,3:F0} %: net    0.00 N  NEUTRAL by construction");
            sb.Append($"   VBS 100 %: net {NetForceFullN:+0.00;-0.00} N  {(NetForceFullN < 0 ? "SINKS" : "floats")}");
            Debug.Log(sb.ToString());
            try {
                string rdir = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "dof");
                Directory.CreateDirectory(rdir);
                File.WriteAllText(Path.Combine(rdir, "trim_report.txt"), sb.ToString());
            } catch (System.Exception ex) { Debug.LogWarning("[SAMBuoyancyTrim] report write failed: " + ex.Message); }

            // --- file-driven bootstrap of the one-DOF test rig ---------------------------------
            // StreamingAssets/SAMReplay/dof/config.txt lets a run be configured without the Inspector.
            // Absent (or run=0) it does nothing, so normal scene play is unaffected.
            try {
                string cfg = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "dof", "config.txt");
                if (File.Exists(cfg))
                {
                    bool run = false;
                    foreach (string line in File.ReadAllLines(cfg))
                    {
                        string t = line.Trim(); if (t.Length == 0 || t.StartsWith("#")) continue;
                        int eq = t.IndexOf('='); if (eq < 0) continue;
                        if (t.Substring(0, eq).Trim() == "run" && t.Substring(eq + 1).Trim() == "1") run = true;
                    }
                    var vroot = transform.root;
                    var replay = vroot.GetComponentInChildren<SAMTankReplay>(true);
                    var dof = vroot.GetComponentInChildren<SAMDofTest>(true);
                    if (run)
                    {
                        if (replay != null && replay.enabled) { replay.enabled = false; Debug.Log("[SAMBuoyancyTrim] DOF run requested - SAMTankReplay disabled."); }
                        if (dof != null && !dof.enabled) { dof.enabled = true; Debug.Log("[SAMBuoyancyTrim] DOF run requested - SAMDofTest enabled."); }
                        else if (dof == null) Debug.LogWarning("[SAMBuoyancyTrim] config.txt asks for a DOF run but no SAMDofTest component is on the vehicle.");
                    }
                }
            } catch (System.Exception ex) { Debug.LogWarning("[SAMBuoyancyTrim] dof bootstrap failed: " + ex.Message); }
        }
    }
}
