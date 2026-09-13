using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using VehicleComponents.Actuators;

namespace Force
{
    /// <summary>
    /// Drives ONE actuator channel at a time on a scripted staircase, holds everything else neutral, and
    /// logs the response — so each degree of freedom can be checked against the tank data on its own before
    /// any combination is trusted.
    ///
    ///   Trim        every actuator neutral. Confirms float / neutral / sink against the VBS setting.
    ///   VBS         VBS stepped through a staircase, props off, LCG centred. Response: vertical velocity.
    ///   LCG         LCG stepped, props off, VBS at neutral.                  Response: pitch angle and rate.
    ///   RPM         propellers stepped forward and reverse, everything else neutral. Response: surge.
    ///   Elevator    thrust-vector pitch deflection at constant rpm.          Response: pitch rate, then dive.
    ///   Rudder      thrust-vector yaw deflection at constant rpm.            Response: yaw rate and turn radius.
    ///
    /// Output: StreamingAssets/SAMReplay/dof/<Mode>_<tag>.csv, one row per FixedUpdate, with the command
    /// and the full state, plus a per-step summary in the Console so a run can be read without leaving Unity.
    /// </summary>
    public class SAMDofTest : MonoBehaviour
    {
        public enum Mode { Trim, VBS, LCG, RPM, Elevator, Rudder }

        [Header("What to run")]
        public Mode TestMode = Mode.Trim;
        public bool RunOnStart = true;
        [Tooltip("Seconds held at each step. Long enough to reach a steady rate: 25 s is about 10 time constants in heave.")]
        public float StepSeconds = 25f;
        [Tooltip("Seconds of settling before the first step.")]
        public float SettleSeconds = 5f;
        public float TimeScale = 3f;
        [Tooltip("Free-text tag appended to the output file name.")]
        public string Tag = "";
        [Tooltip("If > 0, the vehicle is placed level at this depth below the water surface before the run. The scene spawns SAM at the surface, where the ForcePoint buoyancy ramp pins it and no heave is possible.")]
        public float StartDepth = 0f;
        [Tooltip("Diagnostic: run with the v2 hydrodynamics component switched off, so only the ForcePoints and PhysX gravity act.")]
        public bool DisableHydroV2 = false;
        [Tooltip("Hold the vehicle at StartDepth, level and still, for the whole settle phase. The VBS piston starts empty and needs several seconds to reach the neutral fill; without the hold the vehicle has already surfaced before the first step.")]
        public bool HoldDuringSettle = true;

        [Header("Neutral values held on the channels not under test")]
        [Range(0f, 100f)] public float NeutralVBS = 50f;
        [Range(0f, 100f)] public float NeutralLCG = 50f;
        public float CruiseRPM = 500f;          // used by the Elevator and Rudder modes

        [Header("Staircases (percent, percent, rpm, radians)")]
        public float[] VBSSteps = { 50f, 0f, 50f, 100f, 50f };
        public float[] LCGSteps = { 50f, 0f, 50f, 100f, 50f };
        public float[] RPMSteps = { 0f, 300f, 600f, 900f, 0f, -300f, -600f, -900f, 0f };
        public float[] DeflectionStepsDeg = { 0f, 3f, 5f, 7f, 0f, -3f, -5f, -7f, 0f };

        [Header("Resolved (read-only)")]
        public ArticulationBody BaseLink;
        public Propeller PropFront, PropBack;
        public Hinge HingeYaw, HingePitch;
        public VBS Vbs; public Prismatic Lcg;
        [Header("Live (read-only)")]
        public float TestTime; public int StepIndex; public bool Running;
        public float Depth, PitchDeg, SurgeMS, HeaveMS;

        MixedBody body; StringBuilder buf; StringBuilder sum; string outPath, sumPath; float[] steps; float stepStart; float y0;
        readonly List<float> acc = new List<float>();
        bool dumpedForces, holding; Force.ForcePoint[] fps; ArticulationBody[] abs_; SAMHydrodynamicsV2 hv2;

        void Start() { if (RunOnStart) Begin(); }

        /// <summary>
        /// Optional StreamingAssets/SAMReplay/dof/config.txt overrides, so a run can be set up from outside
        /// the editor. Keys: run, mode, tag, step, settle, timescale, neutral_vbs, neutral_lcg, cruise_rpm,
        /// steps (comma separated). Anything absent keeps the Inspector value.
        /// </summary>
        void ReadConfig()
        {
            string cfg = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "dof", "config.txt");
            if (!File.Exists(cfg)) return;
            var ci = CultureInfo.InvariantCulture;
            List<float> custom = null;
            foreach (string line in File.ReadAllLines(cfg))
            {
                string t = line.Trim(); if (t.Length == 0 || t.StartsWith("#")) continue;
                int eq = t.IndexOf('='); if (eq < 0) continue;
                string k = t.Substring(0, eq).Trim().ToLowerInvariant(), v = t.Substring(eq + 1).Trim();
                float f;
                switch (k)
                {
                    case "mode": if (System.Enum.TryParse(v, true, out Mode m)) TestMode = m; break;
                    case "tag": Tag = v; break;
                    case "start_depth": if (float.TryParse(v, NumberStyles.Float, ci, out f)) StartDepth = f; break;
                    case "disable_v2": if (bool.TryParse(v, out bool db)) DisableHydroV2 = db; break;
                    case "hold_during_settle": if (bool.TryParse(v, out bool hb)) HoldDuringSettle = hb; break;
                    case "step": if (float.TryParse(v, NumberStyles.Float, ci, out f)) StepSeconds = f; break;
                    case "settle": if (float.TryParse(v, NumberStyles.Float, ci, out f)) SettleSeconds = f; break;
                    case "timescale": if (float.TryParse(v, NumberStyles.Float, ci, out f)) TimeScale = f; break;
                    case "neutral_vbs": if (float.TryParse(v, NumberStyles.Float, ci, out f)) NeutralVBS = f; break;
                    case "neutral_lcg": if (float.TryParse(v, NumberStyles.Float, ci, out f)) NeutralLCG = f; break;
                    case "cruise_rpm": if (float.TryParse(v, NumberStyles.Float, ci, out f)) CruiseRPM = f; break;
                    case "steps":
                        custom = new List<float>();
                        foreach (string part in v.Split(','))
                            if (float.TryParse(part.Trim(), NumberStyles.Float, ci, out f)) custom.Add(f);
                        break;
                }
            }
            if (custom != null && custom.Count > 0)
            {
                float[] a = custom.ToArray();
                switch (TestMode)
                {
                    case Mode.VBS: VBSSteps = a; break;
                    case Mode.LCG: LCGSteps = a; break;
                    case Mode.RPM: RPMSteps = a; break;
                    case Mode.Elevator: case Mode.Rudder: DeflectionStepsDeg = a; break;
                }
            }
            Debug.Log($"[SAMDofTest] config.txt applied: mode {TestMode}, tag '{Tag}', step {StepSeconds} s, timescale {TimeScale}");
        }

        public void Begin()
        {
            ReadConfig();
            Transform root = transform.root;
            foreach (var ab in root.GetComponentsInChildren<ArticulationBody>(true)) if (ab.name == "base_link") { BaseLink = ab; break; }
            if (BaseLink == null) { Debug.LogError("[SAMDofTest] no base_link"); enabled = false; return; }
            body = new MixedBody(BaseLink, null);
            foreach (var p in root.GetComponentsInChildren<Propeller>(true))
            { if (p.linkName == "front_prop_link") PropFront = p; else if (p.linkName == "back_prop_link") PropBack = p; }
            foreach (var h in root.GetComponentsInChildren<Hinge>(true))
            { if (h.linkName == "thruster_yaw_link") HingeYaw = h; else if (h.linkName == "thruster_link") HingePitch = h; }
            Vbs = root.GetComponentInChildren<VBS>(true); Lcg = root.GetComponentInChildren<Prismatic>(true);

            steps = TestMode switch
            {
                Mode.VBS => VBSSteps,
                Mode.LCG => LCGSteps,
                Mode.RPM => RPMSteps,
                Mode.Elevator => DeflectionStepsDeg,
                Mode.Rudder => DeflectionStepsDeg,
                _ => new float[] { 0f },
            };
            string dir = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "dof");
            Directory.CreateDirectory(dir);
            outPath = Path.Combine(dir, $"{TestMode}{(string.IsNullOrEmpty(Tag) ? "" : "_" + Tag)}.csv");
            sumPath = Path.Combine(dir, $"{TestMode}{(string.IsNullOrEmpty(Tag) ? "" : "_" + Tag)}_summary.txt");
            sum = new StringBuilder();
            sum.AppendLine($"SAMDofTest {TestMode}  tag='{Tag}'  step={StepSeconds}s  settle={SettleSeconds}s  neutralVBS={NeutralVBS}  neutralLCG={NeutralLCG}  cruiseRPM={CruiseRPM}");
            buf = new StringBuilder();
            buf.AppendLine("t,step,cmd,x,y,z,roll_deg,pitch_deg,yaw_deg,u,v,w,p,q,r,vbs,lcg,rpm,yaw_defl,pitch_defl,depth_rate,buoy_N,weight_N,net_N,vy,sub,Fd,Td");
            if (StartDepth > 0f)
            {
                Vector3 p0 = BaseLink.transform.position; p0.y = -StartDepth;
                BaseLink.TeleportRoot(p0, Quaternion.identity);
                foreach (var ab in root.GetComponentsInChildren<ArticulationBody>(true))
                {
                    if (ab == null || ab.isRoot) continue;
                    var jv = ab.jointVelocity; for (int k = 0; k < jv.dofCount; k++) jv[k] = 0f; ab.jointVelocity = jv;
                }
                BaseLink.linearVelocity = Vector3.zero; BaseLink.angularVelocity = Vector3.zero;
                Debug.Log($"[SAMDofTest] placed level at {StartDepth:F2} m depth for the run.");
            }
            hv2 = root.GetComponentInChildren<SAMHydrodynamicsV2>(true);
            fps = root.GetComponentsInChildren<Force.ForcePoint>(true);
            abs_ = root.GetComponentsInChildren<ArticulationBody>(true);
            TestTime = -SettleSeconds; StepIndex = -1; stepStart = TestTime; Running = true;
            y0 = BaseLink.transform.position.y;
            if (TimeScale > 0f) Time.timeScale = TimeScale;
            // A one-DOF test must be the only writer. Anything else that can set the pose or the
            // actuators is switched off for the duration of the run (play mode only - the prefab
            // asset is untouched, so normal scene play is unaffected).
            foreach (var c in root.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (c == null || c == this) continue;
                string tn = c.GetType().Name;
                if (tn == "Teleporter_Sub" || tn == "SAMKeyboardControl" || tn == "SAMTankReplay" || tn == "Actuator_Sub")
                    if (c.enabled) { c.enabled = false; Debug.Log($"[SAMDofTest] disabled competing writer {tn} on '{c.name}' for this run."); }
            }
            if (DisableHydroV2)
            {
                var v2d = root.GetComponentInChildren<SAMHydrodynamicsV2>(true);
                if (v2d != null && v2d.enabled) { v2d.enabled = false; Debug.Log("[SAMDofTest] DIAGNOSTIC: SAMHydrodynamicsV2 disabled for this run."); }
            }
            {
                var sb0 = new StringBuilder();
                sb0.AppendLine($"root='{root.name}'  baseLink.isRoot={BaseLink.isRoot}  immovable={BaseLink.immovable}  useGravity={BaseLink.useGravity}  jointType={BaseLink.jointType}  drag={BaseLink.linearDamping}/{BaseLink.angularDamping}");
                var pab = BaseLink.transform.parent != null ? BaseLink.transform.parent.GetComponentInParent<ArticulationBody>() : null;
                sb0.AppendLine($"  parent transform='{(BaseLink.transform.parent ? BaseLink.transform.parent.name : "-")}'  parentArticulation='{(pab ? pab.name : "-")}'  activeSelf={BaseLink.gameObject.activeInHierarchy}");
                foreach (var c in root.GetComponents<MonoBehaviour>()) sb0.AppendLine($"  root component {c.GetType().Name} enabled={c.enabled}");
                int np = 0; foreach (var fp in root.GetComponentsInChildren<Force.ForcePoint>(true)) np++;
                sb0.AppendLine($"  ForcePoints={np}  hydroV1={(root.GetComponentInChildren<SAMHydrodynamics>(true) is var h1 && h1 != null ? h1.enabled.ToString() : "none")}  hydroV2={(root.GetComponentInChildren<SAMHydrodynamicsV2>(true) is var h2 && h2 != null ? h2.enabled.ToString() : "none")}");
                sb0.AppendLine("  --- bodies (name, mass kg, useGravity) ---");
                foreach (var ab in root.GetComponentsInChildren<ArticulationBody>(true))
                {
                    var par = ab.transform.parent != null ? ab.transform.parent.GetComponentInParent<ArticulationBody>() : null;
                    Vector3 cw = ab.transform.TransformPoint(ab.centerOfMass) - BaseLink.transform.position;
                    Vector3 cb = new Vector3(cw.z, cw.x, -cw.y);
                    sb0.AppendLine($"    {ab.name,-24} {ab.mass,8:F3}  grav={ab.useGravity,-5} isRoot={ab.isRoot,-5} parentAB={(par ? par.name : "NONE"),-20} cg_body=({cb.x,6:F3},{cb.y,6:F3},{cb.z,6:F3})");
                }
                sb0.AppendLine("  --- force point hierarchy (ForcePoint.RelatedForcePoints is built from the CONNECTED BODY's OWN GameObject children, so a point parented elsewhere is excluded from the divisor) ---");
                foreach (var fp in root.GetComponentsInChildren<Force.ForcePoint>(true))
                {
                    var chain = ""; var t = fp.transform;
                    while (t != null && t != root) { chain = t.name + (chain.Length > 0 ? "/" + chain : ""); t = t.parent; }
                    var cab = fp.ConnectedArticulationBody;
                    int related = cab != null ? cab.gameObject.GetComponentsInChildren<Force.ForcePoint>(true).Length : -1;
                    sb0.AppendLine($"    {chain}   relatedCount={related}");
                }
                sb0.AppendLine("  --- force points (name, AddGravity, Mass, Volume L, body) ---");
                foreach (var fp in root.GetComponentsInChildren<Force.ForcePoint>(true))
                    sb0.AppendLine($"    {fp.name,-24} addG={fp.AddGravity,-5} M={fp.Mass,7:F3} V={fp.Volume * 1000f,8:F3} L  body={(fp.ConnectedArticulationBody ? fp.ConnectedArticulationBody.name : "-")}  rho={fp.WaterDensity:F0}");
                Debug.Log("[SAMDofTest] rig: " + sb0.ToString());
                try { File.WriteAllText(Path.Combine(dir, "rig_report.txt"), sb0.ToString()); } catch { }
            }
            Debug.Log($"[SAMDofTest] {TestMode}: {steps.Length} steps x {StepSeconds} s (settle {SettleSeconds} s) -> {outPath}");
        }

        float Cmd => (StepIndex < 0 || StepIndex >= steps.Length) ? steps[0] : steps[Mathf.Clamp(StepIndex, 0, steps.Length - 1)];

        void FixedUpdate()
        {
            if (!Running) return;
            TestTime += Time.fixedDeltaTime;
            if (TestTime >= 0f && (StepIndex < 0 || TestTime - stepStart >= StepSeconds))
            {
                if (StepIndex >= 0) SummariseStep();
                StepIndex++; stepStart = TestTime; acc.Clear();
                if (StepIndex >= steps.Length) { Finish(); return; }
            }

            // ---- commands: the channel under test moves, every other one is held neutral ----
            float vbsCmd = NeutralVBS, lcgCmd = NeutralLCG, rpmCmd = 0f, yawDef = 0f, pitDef = 0f;
            float c = Cmd;
            switch (TestMode)
            {
                case Mode.VBS: vbsCmd = c; break;
                case Mode.LCG: lcgCmd = c; break;
                case Mode.RPM: rpmCmd = c; break;
                case Mode.Elevator: rpmCmd = CruiseRPM; pitDef = c * Mathf.Deg2Rad; break;
                case Mode.Rudder:   rpmCmd = CruiseRPM; yawDef = c * Mathf.Deg2Rad; break;
            }
            BaseLink.WakeUp();
            if (Vbs != null) Vbs.SetPercentage(vbsCmd);
            if (Lcg != null) Lcg.SetPercentage(lcgCmd);
            if (PropFront != null) PropFront.SetRpm(rpmCmd);
            if (PropBack != null) PropBack.SetRpm(rpmCmd);
            if (HingeYaw != null) HingeYaw.SetAngle(yawDef);
            if (HingePitch != null) HingePitch.SetAngle(pitDef);

            // During the settle phase the vehicle is pinned level at StartDepth so the actuators can
            // reach their neutral positions before anything is asked of the hydrodynamics.
            // Proper kinematic hold: immovable pins the articulation root without leaving any residual
            // momentum. Zeroing the velocities instead does NOT work - the forces still act inside each
            // solver step, so the body creeps and arrives at the release with real momentum (measured
            // 2026-09-12: a 0.6 m/s launch and 0.7 deg/s of pitch built up during a 12 s "hold").
            if (HoldDuringSettle && StartDepth > 0f)
            {
                bool wantHold = TestTime < 0f;
                if (wantHold != holding)
                {
                    holding = wantHold;
                    BaseLink.immovable = wantHold;
                    if (!wantHold)
                    {
                        BaseLink.linearVelocity = Vector3.zero; BaseLink.angularVelocity = Vector3.zero;
                        var v2 = BaseLink.transform.root.GetComponentInChildren<SAMHydrodynamicsV2>(true);
                        if (v2 != null) v2.ResetHistory();
                        Debug.Log($"[SAMDofTest] hold released at {BaseLink.transform.position.y:F3} m, pitch {PitchDeg:F2} deg.");
                    }
                }
            }

            // ---- state ----
            Transform tr = BaseLink.transform;
            Vector3 pos = tr.position; Vector3 e = tr.rotation.eulerAngles;
            Vector3 vl = tr.InverseTransformVector(body.velocity), wl = tr.InverseTransformDirection(body.angularVelocity);
            float u = vl.z, v = vl.x, w = -vl.y, pr = wl.z, q = wl.x, r = -wl.y;
            float roll = Mathf.DeltaAngle(0f, e.z), pitch = -Mathf.DeltaAngle(0f, e.x), yaw = Mathf.DeltaAngle(0f, e.y);
            Depth = y0 - pos.y; PitchDeg = pitch; SurgeMS = u; HeaveMS = -body.velocity.y;
            if (!dumpedForces && TestTime > 0.2f)
            {
                dumpedForces = true;
                var sb1 = new StringBuilder();
                Vector3 bTot = Vector3.zero, gTot = Vector3.zero;
                foreach (var fp in BaseLink.transform.root.GetComponentsInChildren<Force.ForcePoint>(true))
                { bTot += fp.AppliedBuoyancyForce; gTot += fp.AppliedGravityForce;
                  sb1.AppendLine($"    {fp.name,-24} buoy={fp.AppliedBuoyancyForce.y,9:F3} N grav={fp.AppliedGravityForce.y,9:F3} N  under={fp.IsUnderwater} sub={fp.IsSubmerged} d={fp.CurrentDepth:F3}"); }
                float wsum = 0f; foreach (var ab in BaseLink.transform.root.GetComponentsInChildren<ArticulationBody>(true)) if (ab.useGravity) wsum += ab.mass;
                sb1.AppendLine($"  SUM forcepoint buoyancy {bTot.y:F3} N, forcepoint gravity {gTot.y:F3} N, PhysX-gravity mass {wsum:F3} kg = {wsum * 9.81f:F3} N");
                sb1.AppendLine($"  NET (up positive) = {bTot.y + gTot.y - wsum * 9.81f:F3} N");
                float mTot = 0f; foreach (var ab in BaseLink.transform.root.GetComponentsInChildren<ArticulationBody>(true)) mTot += ab.mass;
                sb1.AppendLine($"  total articulation mass = {mTot:F3} kg   Physics.gravity = {Physics.gravity}   body v = {BaseLink.linearVelocity}");
                sb1.AppendLine($"  water level at hull = {(DefaultNamespace.Water.WaterQueryModel.GetWaterQueryModel() != null ? DefaultNamespace.Water.WaterQueryModel.GetWaterQueryModel().GetWaterLevelAt(BaseLink.transform.position).ToString("F3") : "n/a")}   hull y = {BaseLink.transform.position.y:F3}");
                Debug.Log("[SAMDofTest] forces:\n" + sb1);
                try { File.WriteAllText(Path.Combine(Path.GetDirectoryName(outPath), "force_report.txt"), sb1.ToString()); } catch { }
            }
            if (TestTime >= 0f) acc.Add(TestMode switch
            {
                Mode.VBS => HeaveMS, Mode.LCG => pitch, Mode.RPM => u,
                Mode.Elevator => q, Mode.Rudder => r, _ => HeaveMS });

            var ci = CultureInfo.InvariantCulture;
            buf.Append(TestTime.ToString("F3", ci)).Append(',').Append(StepIndex).Append(',').Append(c.ToString("F3", ci)).Append(',')
               .Append(pos.z.ToString("F4", ci)).Append(',').Append(pos.x.ToString("F4", ci)).Append(',').Append((-pos.y).ToString("F4", ci)).Append(',')
               .Append(roll.ToString("F3", ci)).Append(',').Append(pitch.ToString("F3", ci)).Append(',').Append(yaw.ToString("F3", ci)).Append(',')
               .Append(u.ToString("F4", ci)).Append(',').Append(v.ToString("F4", ci)).Append(',').Append(w.ToString("F4", ci)).Append(',')
               .Append(pr.ToString("F4", ci)).Append(',').Append(q.ToString("F4", ci)).Append(',').Append(r.ToString("F4", ci)).Append(',')
               .Append(vbsCmd.ToString("F1", ci)).Append(',').Append(lcgCmd.ToString("F1", ci)).Append(',').Append(rpmCmd.ToString("F0", ci)).Append(',')
               .Append(yawDef.ToString("F4", ci)).Append(',').Append(pitDef.ToString("F4", ci)).Append(',')
               .Append(HeaveMS.ToString("F4", ci)).Append(',');
            float bN = 0f; for (int i = 0; i < fps.Length; i++) bN += fps[i].AppliedBuoyancyForce.y + fps[i].AppliedGravityForce.y;
            float mN = 0f; for (int i = 0; i < abs_.Length; i++) if (abs_[i].useGravity) mN += abs_[i].mass;
            mN *= Mathf.Abs(Physics.gravity.y);
            buf.Append(bN.ToString("F3", ci)).Append(',').Append(mN.ToString("F3", ci)).Append(',')
               .Append((bN - mN).ToString("F3", ci)).Append(',')
               .Append(BaseLink.linearVelocity.y.ToString("F4", ci)).Append(',')
               .Append((hv2 != null ? hv2.SubmergedFraction : -1f).ToString("F3", ci)).Append(',')
               .Append((hv2 != null ? hv2.LastForceLocal.magnitude : -1f).ToString("F3", ci)).Append(',')
               .Append((hv2 != null ? hv2.LastTorqueLocal.magnitude : -1f).ToString("F3", ci)).Append('\n');
        }

        void SummariseStep()
        {
            if (acc.Count < 10) return;
            int n = acc.Count, tail = Mathf.Max(10, n / 3);
            float s = 0f; for (int i = n - tail; i < n; i++) s += acc[i];
            float steady = s / tail;
            string what = TestMode switch
            {
                Mode.VBS => $"steady heave {steady:F4} m/s (down +)",
                Mode.LCG => $"steady pitch {steady:F2} deg",
                Mode.RPM => $"steady surge {steady:F4} m/s",
                Mode.Elevator => $"steady pitch rate {steady:F4} rad/s, pitch now {PitchDeg:F1} deg",
                Mode.Rudder => $"steady yaw rate {steady:F4} rad/s",
                _ => $"steady {steady:F4}",
            };
            float sysMass = 0f; foreach (var ab in BaseLink.transform.root.GetComponentsInChildren<ArticulationBody>(true)) sysMass += ab.mass;
            string diag = $"  [m_base {BaseLink.mass:F3} m_sys {sysMass:F3} I {BaseLink.inertiaTensor} sleep {BaseLink.IsSleeping()} vbs% {(Vbs != null ? Vbs.GetCurrentValue() : -1f):F1} lcg% {(Lcg != null ? Lcg.GetCurrentValue() : -1f):F1}]";
            string row = $"step {StepIndex,2}  cmd {Cmd,8:F1}: {what}   depth {Depth:F2} m  pitch {PitchDeg:F1} deg  surge {SurgeMS:F3} m/s" + diag;
            Debug.Log($"[SAMDofTest] {TestMode} {row}");
            if (sum != null) { sum.AppendLine(row); try { File.WriteAllText(sumPath, sum.ToString()); } catch { } }
        }

        public void Finish()
        {
            if (!Running) return; Running = false; Time.timeScale = 1f;
            if (BaseLink != null && BaseLink.immovable) BaseLink.immovable = false;
            File.WriteAllText(outPath, buf.ToString());
            if (sum != null) { sum.AppendLine("DONE"); File.WriteAllText(sumPath, sum.ToString()); }
            Debug.Log($"[SAMDofTest] {TestMode} done, wrote {outPath}");
        }
        void OnDisable() { if (Running) Finish(); }
    }
}
