using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using VehicleComponents.Actuators;

namespace Force
{
    /// <summary>
    /// Replays a tank recording into the SAM prefab: EXACTLY the recorded actuator commands (rpm command to the
    /// Propellers, eRPM feedback to SAMHydrodynamicsV2, thrust-vector angles to the two Hinges, VBS and LCG
    /// percentages) while the chosen hydrodynamic model (SAMHydroModelSelector) moves the vehicle. The simulated
    /// track is written next to the input so it can be scored against the MoCap track offline.
    ///
    /// Input: StreamingAssets/SAMReplay/<CsvName>  (built by data-cube/scripts/sam-sysid/build_replay_csv.py)
    ///   columns: t,valid,cmd_rpm1,cmd_rpm2,fb_rpm1,fb_rpm2,d1,d2,vbs,lcg, ux,uy,uz, uqx,uqy,uqz,uqw (Unity RUF pose),
    ///            rx,ry,rz, vu,vv,vw (body FRD velocity from the pose track), bu..br, bp..br, sp,sq,sr (STIM), wp,wq,wr
    /// Output: StreamingAssets/SAMReplay/out/<CsvName without .csv>_<tag>.csv
    ///   t,restart,px,py,pz,qx,qy,qz,qw,su,sv,sw,sp,sq,sr,rpm1,rpm2,yaw,pitch,vbs,lcg,thrust
    ///
    /// Every RestartEverySeconds the vehicle is teleported back onto the recorded pose with the recorded velocity
    /// (open-loop rollouts of that horizon, like the identification gates); 0 = one continuous open-loop run.
    /// </summary>
    public class SAMTankReplay : MonoBehaviour
    {
        [Header("Input")]
        public string CsvName = "rosbag2_2025_06_10-17_57_49.csv";
        public bool RunOnStart = true;
        [Tooltip("Seconds into the recording to start from")] public float StartTime = 0f;
        [Tooltip("Seconds to replay (0 = to the end)")] public float MaxDuration = 0f;
        [Tooltip("Teleport onto the recorded state every N s (0 = never)")] public float RestartEverySeconds = 10f;
        [Tooltip("Time.timeScale while replaying (physics dt unchanged; >1 runs faster than real time if the machine keeps up)")] public float TimeScale = 1f;
        [Header("Placement of the MoCap frame in the scene")]
        public Vector3 WorldOffset = Vector3.zero;
        public float YawOffsetDeg = 0f;
        [Header("Actuator mapping")]
        [Tooltip("d1 = thruster_horizontal_radians -> ThrusterYaw hinge; d2 = thruster_vertical_radians -> ThrusterPitch hinge. Flip the signs here if the replayed yaw/pitch rates come out mirrored.")]
        public float YawSign = -1f;   // MEASURED 2026-09-11 on the June-10 replay: +1 gave corr(yaw cmd, yaw rate) = -0.4..-0.56 vs +0.30 in the recording; -1 gives +0.40
        public float PitchSign = 1f;
        public bool FeedERPMToV2 = true;
        [Header("Debug switches")]
        public bool DriveActuators = true;
        public bool DriveProps = true, DriveHinges = true, DriveVbsLcg = true;
        public bool SetVelocityOnRestart = true;
        [Header("Resolved (read-only)")]
        public ArticulationBody BaseLink;
        public SAMHydroModelSelector Selector;
        public SAMHydrodynamicsV2 V2;
        public Propeller PropFront, PropBack;
        public Hinge HingeYaw, HingePitch;
        public VBS Vbs;
        public Prismatic Lcg;
        [Header("Live (read-only)")]
        public float ReplayTime; public int SampleIndex; public bool Running; public int Restarts;
        public float PosError;   // |sim - recorded| now

        float[][] rows; ArticulationBody[] allBodies; Vector3 pendingVel, pendingAngVel; bool havePendingVel; Dictionary<string, int> col; float dtRec; float lastRestart = -1e9f; StringBuilder outBuf; string outPath; MixedBody body; float tEnd;

        void Start() { if (RunOnStart) Begin(); }

        public void Begin()
        {
            string path = Path.Combine(Application.streamingAssetsPath, "SAMReplay", CsvName);
            if (!File.Exists(path)) { Debug.LogError($"[SAMReplay] not found: {path}"); enabled = false; return; }
            var lines = File.ReadAllLines(path);
            var hdr = lines[0].Split(','); col = new Dictionary<string, int>(); for (int i = 0; i < hdr.Length; i++) col[hdr[i].Trim()] = i;
            var list = new List<float[]>(lines.Length);
            for (int i = 1; i < lines.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(lines[i])) continue;
                var f = lines[i].Split(','); var r = new float[f.Length];
                for (int j = 0; j < f.Length; j++) r[j] = float.Parse(f[j], CultureInfo.InvariantCulture);
                list.Add(r);
            }
            rows = list.ToArray(); dtRec = rows[1][col["t"]] - rows[0][col["t"]];
            tEnd = MaxDuration > 0 ? Mathf.Min(StartTime + MaxDuration, rows[rows.Length - 1][col["t"]]) : rows[rows.Length - 1][col["t"]];

            Transform root = transform.root;
            foreach (var ab in root.GetComponentsInChildren<ArticulationBody>(true)) if (ab.name == "base_link") { BaseLink = ab; break; }
            if (BaseLink == null) { Debug.LogError("[SAMReplay] no base_link ArticulationBody"); enabled = false; return; }
            body = new MixedBody(BaseLink, null);
            Selector = root.GetComponentInChildren<SAMHydroModelSelector>(true);
            V2 = root.GetComponentInChildren<SAMHydrodynamicsV2>(true);
            foreach (var p in root.GetComponentsInChildren<Propeller>(true)) { if (p.linkName == "front_prop_link") PropFront = p; else if (p.linkName == "back_prop_link") PropBack = p; }
            foreach (var h in root.GetComponentsInChildren<Hinge>(true)) { if (h.linkName == "thruster_yaw_link") HingeYaw = h; else if (h.linkName == "thruster_link") HingePitch = h; }
            Vbs = root.GetComponentInChildren<VBS>(true); Lcg = root.GetComponentInChildren<Prismatic>(true);
            if (V2 != null && FeedERPMToV2) { V2.ThrustSource = SAMHydrodynamicsV2.ThrustSourceMode.ExternalERPM; V2.UseCommandRPM = false; }

            string tag = Selector != null ? (Selector.Model == SAMHydroModelSelector.HydroModel.V2_TankIdentified2026 ? "v2" : "v1") : "na";
            string outDir = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "out"); Directory.CreateDirectory(outDir);
            outPath = Path.Combine(outDir, Path.GetFileNameWithoutExtension(CsvName) + $"_{tag}_H{RestartEverySeconds:F0}.csv");
            outBuf = new StringBuilder(); outBuf.AppendLine("t,restart,px,py,pz,qx,qy,qz,qw,su,sv,sw,sp,sq,sr,rpm1,rpm2,yaw,pitch,vbs,lcg,thrust,perr");
            ReplayTime = StartTime; SampleIndex = 0; Restarts = 0; lastRestart = -1e9f; Running = true; if (TimeScale > 0f) Time.timeScale = TimeScale;
            Debug.Log($"[SAMReplay] {CsvName}: {rows.Length} rows at {1f / dtRec:F0} Hz, replaying {StartTime:F0}..{tEnd:F0} s, model {tag}, restart every {RestartEverySeconds} s -> {outPath}");
        }

        float C(float[] r, string k) => r[col[k]];

        void FixedUpdate()
        {
            if (!Running) return;
            if (havePendingVel) { havePendingVel = false; body.velocity = pendingVel; body.angularVelocity = pendingAngVel; if (V2 != null) V2.ResetHistory(); }
            ReplayTime += Time.fixedDeltaTime;
            if (ReplayTime > tEnd) { Finish(); return; }
            int i = Mathf.Clamp(Mathf.RoundToInt((ReplayTime - rows[0][col["t"]]) / dtRec), 0, rows.Length - 1); SampleIndex = i; var r = rows[i];

            // ---- restart onto the recorded state ----
            bool restart = false;
            if (RestartEverySeconds > 0 && ReplayTime - lastRestart >= RestartEverySeconds && C(r, "valid") > 0.5f || lastRestart < -1e8f)
            {
                Quaternion yawOff = Quaternion.Euler(0f, YawOffsetDeg, 0f);
                Vector3 pos = WorldOffset + yawOff * new Vector3(C(r, "ux"), C(r, "uy"), C(r, "uz"));
                Quaternion rot = yawOff * new Quaternion(C(r, "uqx"), C(r, "uqy"), C(r, "uqz"), C(r, "uqw"));
                BaseLink.TeleportRoot(pos, rot);
                // TeleportRoot keeps stale joint velocities; combined with a root-velocity set they blow the articulation up
                // (measured 2026-09-11: p 2 -> 4 -> 78 -> 1e7 rad/s within 5 steps). Zero every joint velocity first.
                if (allBodies == null) allBodies = BaseLink.transform.root.GetComponentsInChildren<ArticulationBody>(true);
                foreach (var ab in allBodies)
                {
                    if (ab == null || ab.isRoot) continue;
                    var jv = ab.jointVelocity; for (int k = 0; k < jv.dofCount; k++) jv[k] = 0f; ab.jointVelocity = jv;
                }
                // recorded body velocity is FRD (vu fwd, vv right, vw down) -> RUF local (vv, -vw, vu)
                if (SetVelocityOnRestart)
                {
                    // applied on the NEXT FixedUpdate: setting root velocities in the same step as TeleportRoot blew the
                    // articulation up (roll rate 2 -> 78 -> 1e7 rad/s in five steps, measured 2026-09-11)
                    Vector3 vLocal = new Vector3(C(r, "vv"), -C(r, "vw"), C(r, "vu"));
                    Vector3 wLocal = col.ContainsKey("wp") ? new Vector3(C(r, "wq"), -C(r, "wr"), C(r, "wp")) : Vector3.zero; // FRD rates (p,q,r) -> RUF (q, -r, p)
                    pendingVel = rot * vLocal; pendingAngVel = rot * wLocal; havePendingVel = true;
                }
                if (V2 != null) V2.ResetHistory();
                lastRestart = ReplayTime; Restarts++; restart = true;
            }

            // ---- identical actuator commands ----
            float rpm1 = C(r, "cmd_rpm1"), rpm2 = C(r, "cmd_rpm2");
            float yaw = YawSign * C(r, "d1"), pitch = PitchSign * C(r, "d2");
            if (DriveActuators)
            {
                if (DriveProps)
                {
                    if (PropFront != null) PropFront.SetRpm(rpm1);
                    if (PropBack != null) PropBack.SetRpm(rpm2);
                    if (V2 != null && FeedERPMToV2) { V2.Prop1eRPM = C(r, "fb_rpm1"); V2.Prop2eRPM = C(r, "fb_rpm2"); }
                }
                if (DriveHinges)
                {
                    if (HingeYaw != null) HingeYaw.SetAngle(yaw);
                    if (HingePitch != null) HingePitch.SetAngle(pitch);
                }
                if (DriveVbsLcg)
                {
                    if (Vbs != null) Vbs.SetPercentage(C(r, "vbs"));
                    if (Lcg != null) Lcg.SetPercentage(C(r, "lcg"));
                }
            }

            // ---- record ----
            Transform tr = BaseLink.transform; Vector3 p = tr.position; Quaternion q = tr.rotation;
            Vector3 vl = tr.InverseTransformVector(body.velocity), wl = tr.InverseTransformDirection(body.angularVelocity);
            Vector3 rec = WorldOffset + Quaternion.Euler(0f, YawOffsetDeg, 0f) * new Vector3(C(r, "ux"), C(r, "uy"), C(r, "uz")); PosError = (p - rec).magnitude;
            var ci = CultureInfo.InvariantCulture;
            outBuf.Append(ReplayTime.ToString("F3", ci)).Append(',').Append(restart ? 1 : 0).Append(',')
                .Append(p.x.ToString("F4", ci)).Append(',').Append(p.y.ToString("F4", ci)).Append(',').Append(p.z.ToString("F4", ci)).Append(',')
                .Append(q.x.ToString("F5", ci)).Append(',').Append(q.y.ToString("F5", ci)).Append(',').Append(q.z.ToString("F5", ci)).Append(',').Append(q.w.ToString("F5", ci)).Append(',')
                .Append(vl.z.ToString("F4", ci)).Append(',').Append(vl.x.ToString("F4", ci)).Append(',').Append((-vl.y).ToString("F4", ci)).Append(',')      // FRD u,v,w
                .Append(wl.z.ToString("F4", ci)).Append(',').Append(wl.x.ToString("F4", ci)).Append(',').Append((-wl.y).ToString("F4", ci)).Append(',')      // FRD p,q,r
                .Append(rpm1.ToString("F0", ci)).Append(',').Append(rpm2.ToString("F0", ci)).Append(',').Append(yaw.ToString("F4", ci)).Append(',').Append(pitch.ToString("F4", ci)).Append(',')
                .Append(C(r, "vbs").ToString("F1", ci)).Append(',').Append(C(r, "lcg").ToString("F1", ci)).Append(',').Append((V2 != null && V2.enabled ? V2.ThrustForce : 0f).ToString("F3", ci)).Append(',')
                .Append(PosError.ToString("F4", ci)).Append('\n');
        }

        public void Finish()
        {
            if (!Running) return; Running = false; Time.timeScale = 1f;
            File.WriteAllText(outPath, outBuf.ToString());
            Debug.Log($"[SAMReplay] done: {Restarts} restarts, wrote {outPath}");
        }
        void OnDisable() { if (Running) Finish(); }
    }
}
