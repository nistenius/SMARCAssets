using System.IO;
using System.Text;
using UnityEngine;
using VehicleComponents.Actuators;

namespace Force
{
    /// <summary>
    /// Trims SAM the way the real vehicle is trimmed: with small ballast weights bolted to the
    /// external side rails, port and starboard, sized per SITE because the water is per site.
    /// Ivan's decision, 2026-09-13; it replaces SAMBuoyancyTrim's "re-solve base_link's mass and
    /// centre of mass at Start" as the trim MECHANISM.
    ///
    /// TARGETS (Ivan, 2026-09-13)
    ///     pitch  = 0 at LCG 50 %
    ///     depth  = neutral (B = W) at VBS 50 %
    ///     surface: positive flotation at VBS 5 %
    ///     roll   = 0, i.e. the two weights are equal unless someone asks for a heel
    ///
    /// SITES (fresh -> ocean is 2.8 kg-f of buoyancy across the range, against 2.44 N of total VBS
    /// authority, so the ballast is not a detail: it is the difference between flying and sinking)
    ///     KTH tank                  997 kg/m3
    ///     Asko / Baltic            1005
    ///     Kristineberg / west coast 1025
    ///
    /// WHAT THIS COMPONENT DOES, and what it deliberately does NOT do.
    ///   It APPLIES a ballast specification - per-side mass and position - to the ballast links, and
    ///   stamps the site water density onto every ForcePoint. It then REPORTS the balance it produced:
    ///   B - W at VBS 5 / 50 / 100 %, the longitudinal CB-CG lever and its pitch moment, the roll
    ///   moment, and BG. It does NOT invent the specification: that is solved offline by
    ///   data-cube/scripts/sam-wave-model/ballast_trim.py from the prefab's own link masses and the
    ///   strip cloud's volume-weighted centroid, and the answer is written into the prefab's
    ///   ballast_port_link / ballast_stbd_link. Set ballast_autosolve = true only when you want the
    ///   component to redo that solve at Start (it prints what it would be either way).
    ///
    /// WHERE THE NUMBERS LIVE (state_persistence_discipline)
    ///   ballast mass + station : the PREFAB, on ballast_port_link / ballast_stbd_link
    ///   site water density     : StreamingAssets/SAMReplay/dof/config.txt, key ballast_density
    ///                            (or ballast_site = tank|asko|kristineberg)
    ///   displaced volume       : the per-point Volume fields of the strip cloud, in the PREFAB
    ///   base_link mass         : the PREFAB. Set ONCE so the VBS-empty total matches the measured
    ///                            16.7 kg. It is not a trim knob any more.
    ///
    /// UNMEASURED, and the report says so every run: the rail geometry (x +/-0.070, y -0.030 are
    /// assumptions, not measurements), the two 1 kg "transceiver" placeholder links at +0.70 and
    /// -0.80 m, and on sam_auv_v1 the 0.808 kg motor pack at -0.74 m. Those three between them set
    /// how much ballast the pitch balance asks for; on sam_auv_v1 they push it past 1.4 kg, which is
    /// more than anyone would bolt to a rail. Weighing them is the unblocker, not more solving.
    /// </summary>
    [DefaultExecutionOrder(110)]      // after UrdfInertial and after SAMBuoyancyTrim (100)
    public class SAMBallastTrim : MonoBehaviour
    {
        [Header("Site")]
        [Tooltip("Water density at the site [kg/m3]. 997 KTH tank, ~1005 Asko/Baltic, ~1025 Kristineberg.")]
        public float WaterDensity = 997f;
        [Tooltip("Stamp WaterDensity onto every ForcePoint, so the buoyancy is the site's.")]
        public bool SetForcePointDensity = true;

        [Header("Ballast specification (solved offline by ballast_trim.py)")]
        public bool ApplyBallast = true;
        [Tooltip("Total ballast [kg], split between the two rails.")]
        public float BallastMassKg = 0.500f;
        [Tooltip("Port/starboard split, 0.5 = even. Anything else is a deliberate heel.")]
        [Range(0f, 1f)] public float StarboardFraction = 0.5f;
        [Tooltip("Half-spacing of the external side rails [m]. ASSUMPTION: the hull tube radius is 0.0625.")]
        public float RailX = 0.070f;
        [Tooltip("Height of the weights relative to the hull axis [m], negative = below. ASSUMPTION.")]
        public float RailY = -0.030f;
        [Tooltip("Station along the hull [m] in base_link. Solved for zero pitch moment at LCG 50 %.")]
        public float RailZ = 0.3679f;
        [Tooltip("Move the ballast links to (RailX, RailY, RailZ) at Start. OFF by default: moving an "
                 + "articulation link's transform at runtime is not a reliable way to move its joint anchor. "
                 + "The prefab is where the station belongs; ballast_trim.py --write puts it there.")]
        public bool ApplyPosition = false;

        [Header("Re-solve at Start instead of applying the numbers above")]
        public bool AutoSolve = false;

        [Header("When the trim is EVALUATED")]
        [Tooltip("Seconds after Start at which the report is recomputed against the LIVE articulation, " +
                 "and the number that is written to ballast_report.txt. Start() is the wrong moment: the " +
                 "actuators have not been commanded yet, so battery_link sits at whatever LCG the prefab " +
                 "was serialised at rather than at the LCG the vehicle runs at. MEASURED 2026-09-14 on " +
                 "sam2.2.strips: 8.4 mm of composite CG between the two, which is a 34.8 deg nose-up " +
                 "hang against BG 11.2 mm -- while this component was reporting a 0.70 mm lever and " +
                 "3.6 deg. The masses are still APPLIED at Start (they must be); only the report moves. " +
                 "0 disables the re-report and you get the Start() numbers, which is what lied.")]
        public float ReportAtSeconds = 3f;
        [Tooltip("Read-only: true once the operating-pose report has been taken.")]
        public bool ReportedAtOperatingPose;

        [Header("Which links")]
        public string PortLinkName = "ballast_port_link";
        public string StarboardLinkName = "ballast_stbd_link";
        public string TrimLinkName = "base_link";
        public string VbsLinkName = "vbs_link";

        [Header("Result (read-only)")]
        public float DisplacedLitres, BuoyancyN, WeightAtNeutralN;
        public float NetAtVbs5N, NetAtVbs50N, NetAtVbs100N;
        public Vector3 CentreOfBuoyancyLocal, CompositeCgLocal;
        public float BGmm, LongitudinalLeverMm, PitchMomentNm, RollMomentNm;
        public float SolvedBallastKg, SolvedStationZ;

        /// <summary>Optional overrides from StreamingAssets/SAMReplay/dof/config.txt (keys prefixed ballast_).
        /// Same file and same style as SAMBuoyancyTrim/SAMDofTest read, so one run is configured in one place.</summary>
        void ReadBallastConfig()
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
                    case "ballast_site":
                        switch (v.ToLowerInvariant())
                        {
                            case "tank": case "kth": case "fresh": WaterDensity = 997f; break;
                            case "asko": case "baltic": case "brackish": WaterDensity = 1005f; break;
                            case "kristineberg": case "ocean": case "westcoast": WaterDensity = 1025f; break;
                            default: Debug.LogWarning($"{name}: unknown ballast_site '{v}', keeping {WaterDensity:F0}."); break;
                        }
                        break;
                    case "ballast_density": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) WaterDensity = f; break;
                    case "ballast_apply": if (bool.TryParse(v, out b)) ApplyBallast = b; break;
                    case "ballast_autosolve": if (bool.TryParse(v, out b)) AutoSolve = b; break;
                    case "ballast_mass": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) BallastMassKg = f; break;
                    case "ballast_stbd_fraction": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) StarboardFraction = Mathf.Clamp01(f); break;
                    case "ballast_x": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) RailX = f; break;
                    case "ballast_y": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) RailY = f; break;
                    case "ballast_z": if (float.TryParse(v, System.Globalization.NumberStyles.Float, ci, out f)) RailZ = f; break;
                    case "ballast_apply_position": if (bool.TryParse(v, out b)) ApplyPosition = b; break;
                }
            }
        }

        float _tSinceStart;
        bool _wantReport;

        /// The masses are applied in Start, but the TRIM ARITHMETIC is only meaningful once the
        /// actuators have been commanded to the pose the vehicle runs in. Re-run the report there.
        void FixedUpdate()
        {
            if (!_wantReport || ReportAtSeconds <= 0f) return;
            _tSinceStart += Time.fixedDeltaTime;
            if (_tSinceStart < ReportAtSeconds) return;
            _wantReport = false;
            ReportedAtOperatingPose = true;
            Solve(false);              // report only: never re-apply the masses
        }

        void Start()
        {
            Solve(true);
            _wantReport = ReportAtSeconds > 0f;
        }

        /// <param name="apply">true = Start: set the ballast masses. false = the operating-pose
        /// re-report: touch nothing, just read the articulation and re-derive CB, CG, BG and the
        /// VBS sweep from it.</param>
        void Solve(bool apply)
        {
            ReadBallastConfig();

            Transform root = transform.root;
            var bodies = root.GetComponentsInChildren<ArticulationBody>(true);
            var points = root.GetComponentsInChildren<ForcePoint>(true);
            var vbs = root.GetComponentInChildren<VBS>(true);
            ArticulationBody port = null, stbd = null, trimLink = null, vbsLink = null;
            foreach (var ab in bodies)
            {
                if (ab.name == PortLinkName) port = ab;
                if (ab.name == StarboardLinkName) stbd = ab;
                if (ab.name == TrimLinkName) trimLink = ab;
                if (ab.name == VbsLinkName) vbsLink = ab;
            }
            if (points.Length == 0 || trimLink == null || port == null || stbd == null)
            {
                Debug.LogWarning($"{name}: SAMBallastTrim needs ForcePoints, '{TrimLinkName}', "
                    + $"'{PortLinkName}' and '{StarboardLinkName}'. Found {points.Length} points, "
                    + $"trim {(trimLink ? "ok" : "MISSING")}, port {(port ? "ok" : "MISSING")}, "
                    + $"stbd {(stbd ? "ok" : "MISSING")} - doing nothing.");
                enabled = false; return;
            }

            // ---- displacement and CB, read out of the cloud (never written here) -------------
            float vTotal = 0f;
            bool perPoint = false;
            foreach (var p in points) if (p.VolumeIsPerPoint) { perPoint = true; break; }
            Vector3 cbWorld = Vector3.zero;
            if (perPoint)
            {
                foreach (var p in points) { vTotal += p.Volume; cbWorld += p.Volume * p.transform.position; }
                cbWorld = vTotal > 0f ? cbWorld / vTotal : trimLink.transform.position;
            }
            else
            {
                // legacy cloud: every point carries the group total and an equal share of it
                vTotal = points.Length > 0 ? points[0].Volume : 0f;
                foreach (var p in points) cbWorld += p.transform.position;
                cbWorld /= points.Length;
            }
            if (SetForcePointDensity) foreach (var p in points) p.WaterDensity = WaterDensity;
            DisplacedLitres = vTotal * 1000f;

            float g = Mathf.Abs(Physics.gravity.y);
            BuoyancyN = vTotal * WaterDensity * g;

            // ---- the VBS mass law, so the sweep is analytic and needs no Play ---------------
            float vbsWater = vbs != null ? vbs.density / 1000f * vbs.maxVolume_l : 0.24925f;
            float vbsFixed = 0.300f;

            // ---- everything except the ballast, at the pose it is in now --------------------
            float mOther = 0f; Vector3 sumOther = Vector3.zero;
            foreach (var ab in bodies)
            {
                if (!ab.useGravity || ab == port || ab == stbd) continue;
                float m = (ab == vbsLink) ? vbsFixed + vbsWater * 0.5f : ab.mass;
                if (m < 1e-5f) continue;
                sumOther += m * ab.transform.TransformPoint(ab.centerOfMass); mOther += m;
            }

            // ---- the solve, always computed, applied only if AutoSolve ----------------------
            float mNeeded = BuoyancyN / g - mOther;
            Vector3 cbLocal = trimLink.transform.InverseTransformPoint(cbWorld);
            Vector3 sumOtherLocal = trimLink.transform.InverseTransformPoint(sumOther / Mathf.Max(mOther, 1e-6f)) * mOther;
            SolvedBallastKg = mNeeded;
            SolvedStationZ = Mathf.Abs(mNeeded) > 1e-4f
                ? (cbLocal.z * (mOther + mNeeded) - sumOtherLocal.z) / mNeeded
                : float.NaN;
            if (AutoSolve && apply)   // the re-report must never rewrite the spec it is reporting on
            {
                if (mNeeded <= 0f)
                    Debug.LogWarning($"{name}: the solve wants {mNeeded:F4} kg of ballast - NEGATIVE. The "
                        + "vehicle is already heavier than it displaces at this density; ballast cannot fix "
                        + "that, mass has to come off. Keeping the configured spec.");
                else { BallastMassKg = mNeeded; RailZ = SolvedStationZ; }
            }

            // ---- apply --------------------------------------------------------------------
            float mStbd = BallastMassKg * StarboardFraction;
            float mPort = BallastMassKg - mStbd;
            if (ApplyBallast && apply)
            {
                port.mass = Mathf.Max(mPort, 1e-6f);
                stbd.mass = Mathf.Max(mStbd, 1e-6f);
                port.centerOfMass = Vector3.zero; stbd.centerOfMass = Vector3.zero;
                if (ApplyPosition)
                {
                    // The links are fixed joints on base_link, so moving the transform is enough as long
                    // as the anchors follow it. matchAnchors does that for us.
                    port.matchAnchors = true; stbd.matchAnchors = true;
                    port.transform.localPosition = new Vector3(-Mathf.Abs(RailX), RailY, RailZ);
                    stbd.transform.localPosition = new Vector3(+Mathf.Abs(RailX), RailY, RailZ);
                }
            }

            // ---- report -------------------------------------------------------------------
            float mTotal = mOther + port.mass + stbd.mass;
            Vector3 sumAll = sumOther + port.mass * port.transform.TransformPoint(port.centerOfMass)
                                      + stbd.mass * stbd.transform.TransformPoint(stbd.centerOfMass);
            Vector3 cgWorld = sumAll / mTotal;
            Vector3 cgLocal = trimLink.transform.InverseTransformPoint(cgWorld);
            CentreOfBuoyancyLocal = cbLocal;
            CompositeCgLocal = cgLocal;

            WeightAtNeutralN = mTotal * g;
            NetAtVbs50N = BuoyancyN - WeightAtNeutralN;
            NetAtVbs5N = BuoyancyN - (mTotal - vbsWater * 0.45f) * g;
            NetAtVbs100N = BuoyancyN - (mTotal + vbsWater * 0.50f) * g;

            BGmm = (cbLocal.y - cgLocal.y) * 1000f;
            LongitudinalLeverMm = (cbLocal.z - cgLocal.z) * 1000f;
            PitchMomentNm = WeightAtNeutralN * (cbLocal.z - cgLocal.z);
            RollMomentNm = WeightAtNeutralN * (cbLocal.x - cgLocal.x);

            var sb = new StringBuilder();
            string when = apply
                ? "at Start (prefab pose -- the actuators have NOT been commanded yet, so this CG is NOT the one the vehicle runs with)"
                : $"at the OPERATING pose, {ReportAtSeconds:F1} s in -- THIS is the trim that acts";
            sb.AppendLine("[SAMBallastTrim] " + when);
            sb.AppendLine($"[SAMBallastTrim] site rho {WaterDensity:F0} kg/m3, displacement {DisplacedLitres:F3} L "
                        + $"({(perPoint ? points.Length + " strips, per-point volumes" : "legacy group cloud")}) "
                        + $"-> B = {BuoyancyN:F2} N");
            sb.AppendLine($"   ballast {BallastMassKg:F4} kg total = port {mPort:F4} + stbd {mStbd:F4}, "
                        + $"at x +/-{Mathf.Abs(RailX):F3}, y {RailY:+0.000;-0.000}, z {RailZ:+0.000;-0.000} "
                        + $"{(ApplyBallast ? "APPLIED" : "not applied")}");
            sb.AppendLine($"   solve at this density wants {SolvedBallastKg:F4} kg at z {SolvedStationZ:+0.0000;-0.0000} "
                        + $"{(AutoSolve ? "(auto-solved and used)" : "(reported only)")}");
            if (SolvedBallastKg < 0f)
                sb.AppendLine("   !! NEGATIVE ballast: the vehicle is heavier than its displacement here.");
            if (SolvedBallastKg > 1.0f)
                sb.AppendLine("   !! over 1 kg of rail ballast. Check the placeholder link masses before believing it.");
            sb.AppendLine($"   total {mTotal:F4} kg, CB (base_link) {cbLocal}, CG {cgLocal}");
            sb.AppendLine($"   BG {BGmm:F2} mm, longitudinal CB-CG {LongitudinalLeverMm:F2} mm "
                        + $"-> pitch moment {PitchMomentNm:F3} N m, roll moment {RollMomentNm:F3} N m");
            sb.AppendLine($"   VBS   5 %: B-W {NetAtVbs5N:+0.00;-0.00} N  {(NetAtVbs5N > 0 ? "FLOATS" : "sinks")}   (target: floats)");
            sb.AppendLine($"   VBS  50 %: B-W {NetAtVbs50N:+0.00;-0.00} N  {(Mathf.Abs(NetAtVbs50N) < 0.05f ? "NEUTRAL" : "off neutral")}");
            sb.Append($"   VBS 100 %: B-W {NetAtVbs100N:+0.00;-0.00} N  {(NetAtVbs100N < 0 ? "SINKS" : "floats")}");
            Debug.Log(sb.ToString());
            try
            {
                string rdir = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "dof");
                Directory.CreateDirectory(rdir);
                File.WriteAllText(Path.Combine(rdir, "ballast_report.txt"), sb.ToString());
            }
            catch (System.Exception ex) { Debug.LogWarning("[SAMBallastTrim] report write failed: " + ex.Message); }
        }
    }
}
