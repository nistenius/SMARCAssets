// WaveSweepRig.cs — drives a whole wave-response matrix in ONE Play session.
//
// WHY A RIG AND NOT WaveTestLogger ALONE. WaveTestLogger and WaterSurfaceProbe are one-shot:
// one Play, one file, one condition. The question "how does SAM respond across sea states AND
// depths" is a matrix, and pressing Play twenty times by hand is how a measurement campaign
// silently drifts (different settle, different start pose, different water phase). This rig
// owns the whole matrix: it sets the sea state, places the vehicle, trims it, records, and
// advances, writing one CSV per case plus a manifest, then exits Play mode by itself.
//
// HOW A CASE RUNS (four phases, all logged, `phase` column says which):
//   0 SET        teleport to (home.x, stillWater - depth, home.z), identity rotation,
//                immovable = true, push the sea state onto the HDRP WaterSurface.
//   1 WATERSET   still pinned, let HDRP regenerate its spectrum and the surface settle.
//   2 TRIMFIND   released; a strong integral+damping controller finds the vertical force that
//                holds the target depth. NOT recorded as response data.
//   3 RECORD     the trim force is FROZEN to a constant and the response is logged.
//
// TWO THINGS THE FIRST SMOKE RUN TAUGHT (2026-09-13, both measured, both fixed here):
//   * THE ACTUATORS MUST BE COMMANDED. VBS sat at its serialized 0 % and the vehicle settled
//     25° nose-up at depth. The ballast is solved for VBS 50 %; at 0 % the tank is 0.176 kg
//     light and the piston is at the end of its stroke, which is ~0.79 N.m of pitch moment
//     against the 1.83 N.m/rad that BG = 11.2 mm actually provides — 25°, exactly what was
//     measured. The rig now commands VBS and LCG every step and will not leave SETTLE until
//     the piston has physically arrived.
//   * THE HOLD FORCE MUST BE CALIBRATED, NOT SEARCHED. The analytic seed uses the COUNTED mass,
//     and the counted mass is wrong: measured at the surface on 2026-09-13, the vehicle floats
//     as if it weighs 15.0 kg while its links sum to 16.85 kg — ~17.7 N of weight never reaches
//     the solver (the 2026-09-12 blocker, reproduced statically). Seeded from the counted mass
//     the integrator starts 17.7 N out, and at any gain slow enough to be stable it cannot
//     recover before the vehicle has surfaced: calm_d3 was ordered to 3 m and spent its trim
//     window between 4.65 m and 0.86 m before ending up afloat. So the rig now MEASURES the
//     felt weight — a freely floating body's mean buoyancy IS its felt weight — during the
//     depth-0 case, and every later depth case is seeded from that instead.
//   * THE HOLD FORCE IS NOT A CONTROL PROBLEM. A depth controller chasing a vehicle that was
//     slowly pitching wound its integral to -59.5 N against a true imbalance of -1.7 N, froze
//     that, drove the vehicle to -5.5 m and burst the solver. The force needed is known in
//     closed form — weight minus the buoyancy the strips actually report — so it is now
//     COMPUTED, not searched, and the controller is only a small trim around it.
//
// WHY FREEZE THE HOLD FORCE. A vehicle that is not exactly neutral drifts out of its depth
// band during a 75 s record, and a depth CONTROLLER running during the record would inject
// force at wave frequencies and corrupt the very transfer function we are measuring. A
// constant force cannot: it is a static trim weight, flat at every frequency, zero phase. The
// frozen value is logged (F_hold) and is itself the measurement of the residual imbalance —
// this is the channel that will say whether the 2026-09-12 ~20 N ascent survives on the strip
// cloud. Depth 0 cases run with no hold force at all (free-floating surface case).
//
// WHAT IS LOGGED FOR THE WAVE ITSELF. Elevation is sampled every FixedUpdate at N fixed world
// points on a line through the vehicle's home station, through the SAME WaterQueryModel the
// buoyancy reads — not HDRP's rendered surface. That is deliberate: the RAO must be formed
// against the water the physics actually saw. The spatial line lets the analysis recover the
// wavelength by cross-correlation, so k is MEASURED and the exp(-kd) gate becomes
// parameter-free (ADR-011 acceptance gate 1).
//
// Config: <project>/../_logs/wave/config.txt. Output: the same folder. Nothing is written
// inside Assets/, so no asset reimport churn between cases.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using DefaultNamespace.Water;
using Force;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using VehicleComponents.Actuators;

namespace Diagnostics
{
    public class WaveSweepRig : MonoBehaviour
    {
        [Serializable]
        public class Case
        {
            public string Label = "case";
            public float WindKmh = 30f;
            public float RepetitionSize = 500f;
            public float Chaos = 0.8f;
            public bool Ripples = true;
            public float DepthM = 0f;
            /// Per-case A/B switch for the HDRPWaterQueryModel shared-seed defect, so the
            /// before/after sits in ONE run against identical water instead of two sessions.
            public bool QueryDefect = false;
            /// Links to force to ~0 kg for THIS case only ('|'-separated). The mass audit needs
            /// baseline and modified in one run against the same water, so it cannot be global.
            public string ZeroLinks = "";
            /// Per-case VBS command, NaN = use the global neutral_vbs. The buoyancy gates want
            /// VBS 5 / 24 / 100 % against ONE ballast in ONE run; a global-only VBS made that
            /// three Play sessions and three chances for the water to differ.
            public float VbsPercent = float.NaN;
        }

        [Header("Run")]
        public bool Run = true;
        public float SettleWaterSeconds = 6f;
        public float TrimFindSeconds = 20f;
        public float RecordSeconds = 75f;
        [Tooltip("Stop Play mode when the matrix finishes.")]
        public bool ExitPlayWhenDone = true;

        [Header("Wave probe line (fixed world points, queried through WaterQueryModel)")]
        public int ProbeCount = 5;
        public float ProbeSpan = 40f;
        public Vector3 ProbeDirection = Vector3.forward;

        [Header("Actuators — commanded every step, never left to the serialized value")]
        public float NeutralVbsPercent = 50f;
        public float NeutralLcgPercent = 50f;
        [Tooltip("SETTLE will not end until the VBS piston is within this many percent of target.")]
        public float VbsArrivedTolerance = 1.0f;

        [Header("Trim (seeded analytically, gently corrected, then frozen)")]
        public float TrimKi = 1.5f;
        public float TrimKd = 15f;
        public float MaxHoldForce = 200f;

        [Tooltip("How far the integrator may move away from the analytic seed, in newtons. The " +
                 "seed is physically grounded (measured felt weight minus reported buoyancy), so " +
                 "the correction should be small. Without this clamp a vehicle being thrown " +
                 "around near the surface winds the integral to the rail — moderate_d1 froze at " +
                 "-200 N against a true imbalance of -19 N and was never recoverable.")]
        public float TrimAuthority = 25f;

        [Header("Mass audit — the 2026-09-12 blocker, measured instead of guessed")]
        [Tooltip("Comma-separated link names whose ArticulationBody mass is forced to ~0 at case " +
                 "start. A link the solver never felt cannot change anything when it is removed; " +
                 "one it did felt shows up immediately as a change of draft.")]
        public string ZeroLinkMass = "";

        [Header("Safety — abort the case instead of letting PhysX diverge")]
        public float AbortDepthError = 3f;
        public float AbortPitchDeg = 60f;
        [Tooltip("Vertical slams of 4-5 m/s do occur at the surface in the bigger seas. They are " +
                 "not physical (orbital velocity in a 1.5 m / 5 s sea is 0.94 m/s) but they are " +
                 "transient, and aborting on them throws away the record that quantifies them.")]
        public float AbortSpeed = 12f;

        [Header("Targets (auto-found if left empty)")]
        public GameObject Vehicle;
        public WaterSurface Surface;

        [Header("Read-only")]
        public int CaseIndex;
        public string Phase = "idle";
        public float HoldForceN;
        public string LastAbort = "";

        readonly List<Case> _cases = new List<Case>();
        ArticulationBody _root;
        ForcePoint[] _points;
        float[] _pointBmax;
        VBS _vbs; Prismatic _lcg;
        WaterQueryModel _water;
        Vector3[] _probe;
        Vector3 _home;
        float _stillWaterY;

        string _dir;
        StreamWriter _w; readonly StringBuilder _sb = new StringBuilder(1 << 16);
        int _phase; float _tPhase; float _tRec; int _rows;
        float _holdIntegral;
        float _freezeSum; int _freezeN; float _seedN;
        float _totalMassKg;
        float _feltWeightN = float.NaN;   // measured from a free float, not counted from masses
        float _floatBsum; int _floatN; int _nonFinite;
        readonly Dictionary<ArticulationBody, float> _origMass = new Dictionary<ArticulationBody, float>();
        bool _done;
        bool _pendingFirstCase;
        string _abort;
        readonly List<string> _summary = new List<string>();

        // ---------------------------------------------------------------- setup

        void Start()
        {
            _dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "_logs", "wave"));
            Directory.CreateDirectory(_dir);

            if (!ReadConfig()) { enabled = false; return; }
            if (!Run) { Debug.Log("[WaveSweepRig] run=0, standing down."); enabled = false; return; }

            if (Vehicle == null)
            {
                var fp = FindObjectsByType<ForcePoint>(FindObjectsSortMode.None).FirstOrDefault();
                if (fp != null) Vehicle = fp.GetComponentInParent<ArticulationBody>()?.transform.root.gameObject;
            }
            if (Vehicle == null) { Debug.LogError("[WaveSweepRig] no vehicle found."); enabled = false; return; }

            var bodies = Vehicle.GetComponentsInChildren<ArticulationBody>(true);
            _root = bodies.FirstOrDefault(b => b.isRoot) ?? bodies.OrderByDescending(b => b.mass).First();
            _points = Vehicle.GetComponentsInChildren<ForcePoint>(true);
            _pointBmax = _points.Select(p => p.Volume * p.WaterDensity * Mathf.Abs(Physics.gravity.y)).ToArray();
            _vbs = Vehicle.GetComponentsInChildren<VBS>(true).FirstOrDefault();
            _lcg = Vehicle.GetComponentsInChildren<Prismatic>(true).FirstOrDefault();
            _water = WaterQueryModel.GetWaterQueryModel();
            if (Surface == null) Surface = FindObjectsByType<WaterSurface>(FindObjectsSortMode.None).FirstOrDefault();

            if (_water == null) { Debug.LogError("[WaveSweepRig] no WaterQueryModel in the scene."); enabled = false; return; }
            if (Surface == null) { Debug.LogError("[WaveSweepRig] no HDRP WaterSurface in the scene."); enabled = false; return; }
            if (!Surface.scriptInteractions)
            {
                Surface.scriptInteractions = true;
                Debug.LogWarning("[WaveSweepRig] WaterSurface.scriptInteractions was OFF — turned on, the query model needs it.");
            }

            SilenceCompetingWriters();

            _totalMassKg = _root.GetComponentsInChildren<ArticulationBody>(true).Sum(b => b.mass);
            _home = _root.transform.position;
            _stillWaterY = Surface.transform.position.y;

            _probe = new Vector3[Mathf.Max(1, ProbeCount)];
            Vector3 dir = ProbeDirection.sqrMagnitude < 1e-6f ? Vector3.forward : ProbeDirection.normalized;
            for (int i = 0; i < _probe.Length; ++i)
            {
                float s = _probe.Length == 1 ? 0f : (i / (float)(_probe.Length - 1) - 0.5f) * ProbeSpan;
                _probe[i] = new Vector3(_home.x, _stillWaterY, _home.z) + dir * s;
            }

            Debug.Log($"[WaveSweepRig] {_cases.Count} cases | vehicle {_root.name} " +
                      $"({_points.Length} ForcePoints, {_points.Count(p => p.SectionRadius > 0f)} strips, " +
                      $"{(_points.Sum(p => p.Volume) * 1000f):F2} L) | water {_water.GetType().Name} | " +
                      $"surface '{Surface.name}' | still water y={_stillWaterY:F3} | out {_dir}");

            Debug.Log($"[WaveSweepRig] effective: settle {SettleWaterSeconds}s, trim {TrimFindSeconds}s " +
                      $"(Ki {TrimKi}, Kd {TrimKd}), record {RecordSeconds}s, VBS {NeutralVbsPercent}%, LCG {NeutralLcgPercent}%");
            WriteManifestHeader();
            // NOT BeginCase(0) here.  SAMBallastTrim has execution order 110, so its Start() -- which
            // applies the ballast masses from config.txt -- has NOT run yet when ours does.  Starting
            // case 0 from Start() therefore wrote a mass audit of the PRE-ballast prefab and (once the
            // mass snapshot below existed) restored those pre-ballast masses at every later case,
            // silently undoing SAMBallastTrim.  MEASURED 2026-09-13.  Defer to the first FixedUpdate,
            // by which time every Start() in the scene has run.
            _pendingFirstCase = true;
        }

        /// A second writer during an identification run is how 2026-08-14 lost two hours and
        /// 2026-09-12 lost an evening. Disable by type NAME so this file needs no reference to
        /// the SAM assembly and keeps working if those components are renamed away.
        void SilenceCompetingWriters()
        {
            string[] kill = { "Teleporter_Sub", "SAMKeyboardControl", "SAMTankReplay", "SAMDofTest", "Actuator_Sub" };
            int n = 0;
            foreach (var mb in Vehicle.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb == null || mb == this) continue;
                if (kill.Contains(mb.GetType().Name) && mb.enabled) { mb.enabled = false; n++; }
            }
            if (n > 0) Debug.Log($"[WaveSweepRig] disabled {n} competing writer(s) for the run.");
        }

        bool ReadConfig()
        {
            string path = Path.Combine(_dir, "config.txt");
            if (!File.Exists(path)) { Debug.LogError($"[WaveSweepRig] no config at {path}"); return false; }
            var ci = CultureInfo.InvariantCulture;
            foreach (var raw in File.ReadAllLines(path))
            {
                string line = raw.Trim();
                if (line.Length == 0 || line.StartsWith("#")) continue;
                int eq = line.IndexOf('=');
                if (eq < 0) continue;
                string k = line.Substring(0, eq).Trim().ToLowerInvariant();
                string v = line.Substring(eq + 1).Trim();
                float f; bool b; int i;
                switch (k)
                {
                    case "run": if (bool.TryParse(v, out b)) Run = b; else if (int.TryParse(v, out i)) Run = i != 0; break;
                    case "settle_water": if (float.TryParse(v, NumberStyles.Float, ci, out f)) SettleWaterSeconds = f; break;
                    case "trim_find": if (float.TryParse(v, NumberStyles.Float, ci, out f)) TrimFindSeconds = f; break;
                    case "record": if (float.TryParse(v, NumberStyles.Float, ci, out f)) RecordSeconds = f; break;
                    case "probe_count": if (int.TryParse(v, out i)) ProbeCount = i; break;
                    case "probe_span": if (float.TryParse(v, NumberStyles.Float, ci, out f)) ProbeSpan = f; break;
                    case "exit_play": if (bool.TryParse(v, out b)) ExitPlayWhenDone = b; break;
                    case "neutral_vbs": if (float.TryParse(v, NumberStyles.Float, ci, out f)) NeutralVbsPercent = f; break;
                    case "neutral_lcg": if (float.TryParse(v, NumberStyles.Float, ci, out f)) NeutralLcgPercent = f; break;
                    case "zero_link_mass": ZeroLinkMass = v; break;
                    // Gains live in the config, not the scene: a serialized gain from an older
                    // build of this component silently outlives the fix that changed its default.
                    case "trim_ki": if (float.TryParse(v, NumberStyles.Float, ci, out f)) TrimKi = f; break;
                    case "trim_kd": if (float.TryParse(v, NumberStyles.Float, ci, out f)) TrimKd = f; break;
                    case "abort_depth_error": if (float.TryParse(v, NumberStyles.Float, ci, out f)) AbortDepthError = f; break;
                    case "abort_pitch_deg": if (float.TryParse(v, NumberStyles.Float, ci, out f)) AbortPitchDeg = f; break;
                    case "abort_speed": if (float.TryParse(v, NumberStyles.Float, ci, out f)) AbortSpeed = f; break;
                    case "trim_authority": if (float.TryParse(v, NumberStyles.Float, ci, out f)) TrimAuthority = f; break;
                    case "case":
                        var p = v.Split(',');
                        if (p.Length < 6) { Debug.LogWarning($"[WaveSweepRig] bad case line: {v}"); break; }
                        _cases.Add(new Case
                        {
                            Label = p[0].Trim(),
                            WindKmh = float.Parse(p[1], ci),
                            RepetitionSize = float.Parse(p[2], ci),
                            Chaos = float.Parse(p[3], ci),
                            Ripples = p[4].Trim() != "0",
                            DepthM = float.Parse(p[5], ci),
                            QueryDefect = p.Length > 6 && p[6].Trim() != "0",
                            ZeroLinks = p.Length > 7 ? p[7].Trim() : "",
                            VbsPercent = p.Length > 8 && p[8].Trim().Length > 0
                                         ? float.Parse(p[8], ci) : float.NaN,
                        });
                        break;
                }
            }
            if (_cases.Count == 0) { Debug.LogError("[WaveSweepRig] config has no case= lines."); return false; }
            return true;
        }

        // ---------------------------------------------------------------- cases

        void BeginCase(int idx)
        {
            CaseIndex = idx;
            var c = _cases[idx];

            ApplyMassOverrides(c);

            var hq = _water as HDRPWaterQueryModel;
            if (hq != null) { hq.SeedFromPreviousResult = c.QueryDefect; hq.ResetStats(); }
            else if (c.QueryDefect) Debug.LogWarning("[WaveSweepRig] query_defect asked for but the scene's query model is not HDRPWaterQueryModel.");

            Surface.repetitionSize = c.RepetitionSize;
            Surface.largeWindSpeed = c.WindKmh;
            Surface.largeChaos = c.Chaos;
            Surface.ripples = c.Ripples;

            _root.immovable = true;
            _root.TeleportRoot(new Vector3(_home.x, _stillWaterY - c.DepthM, _home.z), Quaternion.identity);
            _root.linearVelocity = Vector3.zero;
            _root.angularVelocity = Vector3.zero;

            if (!string.IsNullOrWhiteSpace(_cases[idx].ZeroLinks)) _feltWeightN = float.NaN;
            HoldForceN = 0f; _holdIntegral = 0f; _abort = "";
            _freezeSum = 0f; _freezeN = 0;
            _floatBsum = 0f; _floatN = 0; _nonFinite = 0;
            _phase = 0; _tPhase = 0f; _tRec = 0f; _rows = 0;
            CommandActuators();

            OpenWriter(c);
            Debug.Log($"[WaveSweepRig] case {idx + 1}/{_cases.Count} '{c.Label}': wind {c.WindKmh} km/h, " +
                      $"patch {c.RepetitionSize} m, chaos {c.Chaos}, ripples {c.Ripples}, depth {c.DepthM} m" +
                      (c.QueryDefect ? "  [LEGACY SHARED-SEED QUERY]" : ""));
        }

        void OpenWriter(Case c)
        {
            var ci = CultureInfo.InvariantCulture;
            string path = Path.Combine(_dir, $"wave_{c.Label}.csv");
            _w = new StreamWriter(path, false, Encoding.UTF8);
            _w.WriteLine("# WaveSweepRig");
            _w.WriteLine("# utc," + DateTime.UtcNow.ToString("o"));
            _w.WriteLine("# scene," + UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
            _w.WriteLine("# case," + c.Label);
            _w.WriteLine("# wind_kmh," + c.WindKmh.ToString("R", ci));
            _w.WriteLine("# repetition_m," + c.RepetitionSize.ToString("R", ci));
            _w.WriteLine("# chaos," + c.Chaos.ToString("R", ci));
            _w.WriteLine("# ripples," + (c.Ripples ? 1 : 0));
            _w.WriteLine("# target_depth_m," + c.DepthM.ToString("R", ci));
            _w.WriteLine("# fixedDeltaTime," + Time.fixedDeltaTime.ToString("R", ci));
            _w.WriteLine("# still_water_y," + _stillWaterY.ToString("R", ci));
            _w.WriteLine("# vehicle," + _root.name + ",mass_root," + _root.mass.ToString("R", ci));
            _w.WriteLine("# total_mass_at_case_start," + LiveTotalMassKg().ToString("R", ci));
            _w.WriteLine("# force_points," + _points.Length + ",strips," + _points.Count(p => p.SectionRadius > 0f)
                         + ",sum_volume_L," + (_points.Sum(p => p.Volume) * 1000f).ToString("R", ci));
            _w.WriteLine("# water_model," + _water.GetType().Name + ",shared_seed_defect," + (c.QueryDefect ? 1 : 0));
            _w.WriteLine("# zero_link_mass," + (string.IsNullOrWhiteSpace(c.ZeroLinks)
                         ? (string.IsNullOrWhiteSpace(ZeroLinkMass) ? "none" : ZeroLinkMass.Replace(',', '|'))
                         : c.ZeroLinks));
            _w.WriteLine("# commanded_vbs_pct," + CaseVbsPercent().ToString("R", ci)
                         + ",commanded_lcg_pct," + NeutralLcgPercent.ToString("R", ci));
            WriteMassAudit(_w, ci);
            for (int i = 0; i < _probe.Length; ++i)
                _w.WriteLine($"# probe{i}," + _probe[i].x.ToString("R", ci) + "," + _probe[i].z.ToString("R", ci));
            var cols = new List<string> { "t", "phase", "y", "depth", "roll", "pitch", "yaw", "vy", "vx", "vz" };
            for (int i = 0; i < _probe.Length; ++i) cols.Add("eta" + i);
            // mass_live is NOT the case-start audit: VBS rewrites vbs_link's mass every step, so the
            // only weight that can be compared with B_sum is the one summed on the same step.
            // MEASURED 2026-09-13: without it, "felt weight exceeds counted mass" is unreadable.
            // B_call is the force actually handed to AddForceAtPosition THIS step, summed over the
            // cloud. B_sum is the same sum taken from ForcePoint.AppliedBuoyancyForce, which is
            // STICKY -- it keeps its last value on a step where nothing was applied. If the two
            // differ, the reported buoyancy is not the buoyancy the solver got. drag_lin is the
            // articulation's own linear damping, which the ForcePoints WRITE at runtime
            // (UnderwaterDrag), so it is live even with SAMHydrodynamics disabled.
            cols.AddRange(new[] { "eta_cb", "B_sum", "f_mean", "f_min", "f_max", "F_hold", "vbs_pct", "lcg_pct",
                                  "mass_live", "B_call", "drag_lin", "adrag_lin", "lcg_actual" });
            _w.WriteLine(string.Join(",", cols));
        }

        void CloseWriter(Case c)
        {
            if (_w == null) return;
            _w.Write(_sb.ToString()); _sb.Clear();
            _w.WriteLine("# rows," + _rows);
            _w.WriteLine("# frozen_hold_force_N," + HoldForceN.ToString("R", CultureInfo.InvariantCulture));
            _w.WriteLine("# abort," + (_abort == "" ? "none" : _abort));
            if (_floatN > 200 && _abort == "" && !float.IsNaN(_floatBsum) && !float.IsInfinity(_floatBsum))
            {
                float felt = _floatBsum / _floatN;
                // Only adopt a weighing from a case that actually floated freely and settled.
                if (felt > 1f && felt < 1000f)
                {
                    bool first = float.IsNaN(_feltWeightN);
                    _feltWeightN = felt;
                    Debug.Log($"[WaveSweepRig] felt weight measured from the free float: {felt:F2} N " +
                              $"({felt / Mathf.Abs(Physics.gravity.y):F3} kg) vs {LiveTotalMassKg():F3} kg counted " +
                              $"-> deficit {LiveTotalMassKg() - felt / Mathf.Abs(Physics.gravity.y):F3} kg" +
                              (first ? "  [this is what every depth case below is trimmed against]" : ""));
                }
            }
            WriteStripAudit(_w, CultureInfo.InvariantCulture);
            WriteCgAudit(_w, CultureInfo.InvariantCulture);
            _w.WriteLine("# felt_weight_N," + (float.IsNaN(_feltWeightN) ? "unmeasured" : _feltWeightN.ToString("R", CultureInfo.InvariantCulture))
                         + ",float_samples," + _floatN + ",non_finite_point_samples," + _nonFinite);
            var hqc = _water as HDRPWaterQueryModel;
            if (hqc != null)
                _w.WriteLine($"# water_queries,{hqc.Queries},failed,{hqc.Failed},non_converged,{hqc.NonConverged}," +
                             $"worst_error_m,{hqc.WorstErrorM.ToString("R", CultureInfo.InvariantCulture)}," +
                             $"worst_iterations,{hqc.WorstIterations}");
            _w.Flush(); _w.Close(); _w = null;
            string line = $"{c.Label},{c.WindKmh},{c.RepetitionSize},{c.Chaos},{(c.Ripples ? 1 : 0)},{c.DepthM}," +
                          $"{HoldForceN.ToString("F3", CultureInfo.InvariantCulture)},{_rows}," +
                          (_abort == "" ? "ok" : "\"" + _abort + "\"") +
                          $",{(c.QueryDefect ? 1 : 0)}" +
                          (_water is HDRPWaterQueryModel h2
                              ? $",{h2.NonConverged},{h2.Failed}"
                              : ",,");
            _summary.Add(line);
            Debug.Log($"[WaveSweepRig] case '{c.Label}' done: {_rows} rows, frozen hold force {HoldForceN:F2} N");
        }

        // ---------------------------------------------------------------- loop

        void FixedUpdate()
        {
            if (_done) return;
            if (_pendingFirstCase)
            {
                _pendingFirstCase = false;
                SnapshotMasses();                                   // AFTER every Start(), SAMBallastTrim included
                _totalMassKg = _root.GetComponentsInChildren<ArticulationBody>(true).Sum(b => b.mass);
                BeginCase(0);
                return;
            }
            if (_w == null) return;
            var c = _cases[CaseIndex];
            float dt = Time.fixedDeltaTime;
            _tPhase += dt;

            CommandActuators();

            float yTarget = _stillWaterY - c.DepthM;
            bool holding = c.DepthM > 0.01f;

            switch (_phase)
            {
                case 0:
                    Phase = "SET";
                    if (_tPhase >= 0.5f) { _phase = 1; _tPhase = 0f; }
                    break;

                case 1:
                {
                    Phase = "SETTLE";
                    // Leave only when the water has settled AND the piston has physically arrived:
                    // a commanded VBS is not an arrived VBS, and the first smoke run trimmed
                    // against a tank that was still travelling.
                    float vbsNow = VbsActualPercent();
                    float lcgNow = LcgActualPercent();
                    bool lcgThere = float.IsNaN(lcgNow) || Mathf.Abs(lcgNow - NeutralLcgPercent) <= VbsArrivedTolerance;
            bool vbsThere = (float.IsNaN(vbsNow) || Mathf.Abs(vbsNow - CaseVbsPercent()) <= VbsArrivedTolerance)
                                    && lcgThere;
                    if (_tPhase >= SettleWaterSeconds && (vbsThere || _tPhase >= SettleWaterSeconds + 30f))
                    {
                        if (!vbsThere)
                            Debug.LogWarning($"[WaveSweepRig] '{c.Label}': LCG at {lcgNow:F2}% (want {NeutralLcgPercent}%); VBS never reached {CaseVbsPercent()}% " +
                                             $"(stuck at {vbsNow:F1}%) — continuing anyway.");

                        // Analytic seed: the static imbalance the strips and the masses actually report.
                        // Positive HoldForceN is up, so it must cancel (B - W): F = W - B.
                        _totalMassKg = LiveTotalMassKg();   // VBS rewrites vbs_link's mass every step
                        // Seed from the MEASURED felt weight when a float case has supplied one;
                        // fall back to the counted mass only if nothing has been weighed yet.
                        float wRef = float.IsNaN(_feltWeightN) ? _totalMassKg * Mathf.Abs(Physics.gravity.y) : _feltWeightN;
                        _holdIntegral = holding ? (wRef - LastBuoyancySum()) : 0f;
                        _holdIntegral = Mathf.Clamp(_holdIntegral, -MaxHoldForce, MaxHoldForce);
                        _seedN = _holdIntegral;
                        HoldForceN = _holdIntegral;
                        _root.immovable = false;
                        _phase = holding ? 2 : 3;
                        _tPhase = 0f;
                        if (holding)
                            Debug.Log($"[WaveSweepRig] '{c.Label}': VBS {vbsNow:F1}%, " +
                                      $"m = {_totalMassKg:F3} kg, B = {LastBuoyancySum():F2} N, " +
                                      $"W counted = {_totalMassKg * Mathf.Abs(Physics.gravity.y):F2} N, " +
                                      $"W felt = {(float.IsNaN(_feltWeightN) ? float.NaN : _feltWeightN):F2} N " +
                                      $"-> seed {HoldForceN:F2} N");
                    }
                    break;
                }

                case 2:
                {
                    Phase = "TRIM";
                    float e = yTarget - _root.transform.position.y;
                    // Bounded around the seed, not around zero: the seed is the physics, the
                    // integrator is only a trim on it.
                    _holdIntegral = Mathf.Clamp(_holdIntegral + TrimKi * e * dt,
                                                _seedN - TrimAuthority, _seedN + TrimAuthority);
                    HoldForceN = Mathf.Clamp(_holdIntegral - TrimKd * _root.linearVelocity.y, -MaxHoldForce, MaxHoldForce);
                    _root.AddForce(Vector3.up * HoldForceN);
                    // Freeze the MEAN of the last 40 % of the window, not the last sample: the
                    // integral still ripples at wave frequency and whichever instant we happened
                    // to stop on would be baked into the whole recording as a bias.
                    if (_tPhase >= 0.6f * TrimFindSeconds) { _freezeSum += _holdIntegral; _freezeN++; }
                    if (_tPhase >= TrimFindSeconds)
                    {
                        HoldForceN = _freezeN > 0 ? _freezeSum / _freezeN : _holdIntegral;
                        _phase = 3; _tPhase = 0f;
                        Debug.Log($"[WaveSweepRig] '{c.Label}' trim frozen at {HoldForceN:F2} N " +
                                  $"(depth error {yTarget - _root.transform.position.y:+0.000;-0.000} m).");
                    }
                    break;
                }

                case 3:
                    Phase = "RECORD";
                    if (holding) _root.AddForce(Vector3.up * HoldForceN);
                    else
                    {
                        float bNow = LastBuoyancySum();     // free float: mean B IS the felt weight
                        if (!float.IsNaN(bNow) && !float.IsInfinity(bNow)) { _floatBsum += bNow; _floatN++; }
                    }
                    _tRec += dt;
                    break;
            }

            CheckSanity(c, yTarget, holding);

            LogRow();

            if (_abort != "" || (_phase == 3 && _tRec >= RecordSeconds))
            {
                CloseWriter(c);
                if (CaseIndex + 1 < _cases.Count) BeginCase(CaseIndex + 1);
                else Finish();
            }
        }

        /// Snapshot the mass of EVERY ArticulationBody under the root, once. This is the only
        /// source the per-case restore reads from, so a case can zero any link and the next case
        /// still starts from the prefab's masses.
        void SnapshotMasses()
        {
            _origMass.Clear();
            if (_root == null) return;
            foreach (var b in _root.GetComponentsInChildren<ArticulationBody>(true))
                if (b != null) _origMass[b] = b.mass;
            Debug.Log($"[WaveSweepRig] mass snapshot: {_origMass.Count} links, {_origMass.Values.Sum():F4} kg total.");
        }

        /// Force named links to ~0 kg. PhysX rejects a zero-mass articulation link, so 1e-7 is
        /// the practical zero — the same value the prefab already uses for its sensor links.
        void ApplyMassOverrides(Case c)
        {
            // Always restore first: an audit case must not leak its zeroed masses into the next one.
            // _origMass is a snapshot of EVERY link taken once in the first FixedUpdate (SnapshotMasses),
            // not just of links a previous case zeroed -- the old code only ever recorded links it was about to
            // zero, so the first zeroing case had nothing to restore and audit_base2 came back
            // identical to audit_no_battery.  MEASURED 2026-09-13, fixed here.
            if (_origMass.Count == 0) SnapshotMasses();
            foreach (var kv in _origMass) if (kv.Key != null) kv.Key.mass = kv.Value;

            string spec = string.IsNullOrWhiteSpace(c.ZeroLinks) ? ZeroLinkMass : c.ZeroLinks;
            if (string.IsNullOrWhiteSpace(spec)) return;
            var names = spec.Split('|', ',').Select(x => x.Trim()).Where(x => x.Length > 0).ToArray();
            foreach (var b in _root.GetComponentsInChildren<ArticulationBody>(true))
                if (names.Contains(b.name))
                {
                    Debug.Log($"[WaveSweepRig] mass audit '{c.Label}': {b.name} {b.mass:F4} kg -> 1e-7 kg");
                    b.mass = 1e-7f;   // PhysX rejects a genuinely zero-mass articulation link
                }
        }

        /// One line per ForcePoint at the END of a case: its station on the hull, what it computed
        /// this step, what it handed to the solver this step, and how many of its DoUpdate calls
        /// actually applied a force. This is what answers "is the loss before or after the call,
        /// and is it symmetric fore-and-aft" without another Play session.
        void WriteStripAudit(StreamWriter w, CultureInfo ci)
        {
            w.WriteLine("# strip,index,local_z,radius,volume_L,computed_N,applied_N,clamped_off_N," +
                        "scaled_off_N,apply_steps,steps,underwater");
            var baseT = _root.transform;
            for (int i = 0; i < _points.Length; ++i)
            {
                var p = _points[i];
                Vector3 loc = baseT.InverseTransformPoint(p.transform.position);
                w.WriteLine($"# strip,{i},{loc.z.ToString("F5", ci)},{p.SectionRadius.ToString("F5", ci)}," +
                            $"{(p.Volume * 1000f).ToString("F4", ci)},{p.ComputedBuoyancyN.ToString("F5", ci)}," +
                            $"{p.AppliedBuoyancyThisStep.y.ToString("F5", ci)}," +
                            $"{p.ClampedOffN.ToString("F5", ci)},{p.ScaledOffN.ToString("F5", ci)}," +
                            $"{p.BuoyancyApplyCount},{p.StepCount},{(p.IsUnderwater ? 1 : 0)}");
            }
        }

        /// The SOLVER's own mass distribution, link by link, at the end of a case: the CoM PhysX is
        /// actually using, where it is in the base_link frame, and the composite it adds up to.
        /// SAMBallastTrim computes its CG from ab.transform.TransformPoint(ab.centerOfMass) once at
        /// Start; this is the same quantity read from the running articulation, so the two can be
        /// differenced instead of argued about. Includes automaticCenterOfMass, because a link left
        /// on "Automatic Center Of Mass" has its CoM derived from colliders and ignores whatever a
        /// prefab edit or a solver wrote into it.
        void WriteCgAudit(StreamWriter w, CultureInfo ci)
        {
            var baseT = _root.transform;
            w.WriteLine("# cg,link,mass,useGravity,autoCom,com_local_x,com_local_y,com_local_z," +
                        "world_com_in_base_x,world_com_in_base_y,world_com_in_base_z");
            float mAll = 0f; Vector3 sAll = Vector3.zero;
            float mGrav = 0f; Vector3 sGrav = Vector3.zero;
            foreach (var ab in _root.GetComponentsInChildren<ArticulationBody>(true))
            {
                Vector3 wc = ab.worldCenterOfMass;
                Vector3 lc = baseT.InverseTransformPoint(wc);
                w.WriteLine($"# cg,{ab.name},{ab.mass.ToString("R", ci)},{(ab.useGravity ? 1 : 0)}," +
                            $"{(ab.automaticCenterOfMass ? 1 : 0)}," +
                            $"{ab.centerOfMass.x.ToString("F6", ci)},{ab.centerOfMass.y.ToString("F6", ci)}," +
                            $"{ab.centerOfMass.z.ToString("F6", ci)}," +
                            $"{lc.x.ToString("F6", ci)},{lc.y.ToString("F6", ci)},{lc.z.ToString("F6", ci)}");
                if (ab.mass < 1e-5f) continue;
                mAll += ab.mass; sAll += ab.mass * lc;
                if (ab.useGravity) { mGrav += ab.mass; sGrav += ab.mass * lc; }
            }
            Vector3 cgAll = mAll > 0f ? sAll / mAll : Vector3.zero;
            Vector3 cgGrav = mGrav > 0f ? sGrav / mGrav : Vector3.zero;
            w.WriteLine($"# cg_composite_all,{mAll.ToString("R", ci)},{cgAll.x.ToString("F6", ci)}," +
                        $"{cgAll.y.ToString("F6", ci)},{cgAll.z.ToString("F6", ci)}");
            w.WriteLine($"# cg_composite_gravity_only,{mGrav.ToString("R", ci)},{cgGrav.x.ToString("F6", ci)}," +
                        $"{cgGrav.y.ToString("F6", ci)},{cgGrav.z.ToString("F6", ci)}");
            // the cloud's CB in the same frame, so CB-CG is one subtraction and not a second tool
            Vector3 cb = Vector3.zero; float v = 0f;
            for (int i = 0; i < _points.Length; ++i) { cb += _points[i].transform.position * _points[i].Volume; v += _points[i].Volume; }
            cb = baseT.InverseTransformPoint(cb / Mathf.Max(v, 1e-9f));
            w.WriteLine($"# cb_in_base,{cb.x.ToString("F6", ci)},{cb.y.ToString("F6", ci)},{cb.z.ToString("F6", ci)}");
            w.WriteLine($"# cb_minus_cg_mm,long,{((cb.z - cgAll.z) * 1000f).ToString("F3", ci)}," +
                        $"BG,{((cb.y - cgAll.y) * 1000f).ToString("F3", ci)}," +
                        $"lat,{((cb.x - cgAll.x) * 1000f).ToString("F3", ci)}");
            Debug.Log($"[WaveSweepRig] SOLVER composite CG in base_link: ({cgAll.x:F5}, {cgAll.y:F5}, {cgAll.z:F5}) " +
                      $"of {mAll:F4} kg | CB ({cb.x:F5}, {cb.y:F5}, {cb.z:F5}) | " +
                      $"CB-CG long {(cb.z - cgAll.z) * 1000f:F2} mm, BG {(cb.y - cgAll.y) * 1000f:F2} mm");
        }

        /// One line per link: what the scene says it weighs and whether gravity is on for it.
        /// Written into every case header so a draft can always be reconciled against a mass.
        void WriteMassAudit(StreamWriter w, CultureInfo ci)
        {
            foreach (var b in _root.GetComponentsInChildren<ArticulationBody>(true))
                w.WriteLine($"# link,{b.name},{b.mass.ToString("R", ci)}," +
                            $"gravity,{(b.useGravity ? 1 : 0)},root,{(b.isRoot ? 1 : 0)}");
        }

        float LiveTotalMassKg()
        {
            float m = 0f;
            foreach (var b in _root.GetComponentsInChildren<ArticulationBody>(true)) m += b.mass;
            return m;
        }

        /// The strips' own reported buoyancy this step — the number the hold force is built on.
        ///
        /// NON-FINITE SAMPLES ARE REAL AND MUST BE DROPPED, NOT SUMMED. A handful of ForcePoint
        /// samples per case come back NaN or infinite — six rows in 3001 on the still-water case,
        /// from water queries made before HDRP's CPU simulation is available, which return
        /// error = FLT_MAX and a height of zero. One of them is enough to turn a 60 s average
        /// into NaN, and on the first attempt at this exactly that silently threw away the felt-
        /// weight calibration the whole depth sweep depends on. Count them instead.
        float LastBuoyancySum()
        {
            float b = 0f;
            for (int i = 0; i < _points.Length; ++i)
            {
                if (!_points[i].IsUnderwater) continue;
                float v = _points[i].AppliedBuoyancyForce.y;
                if (float.IsNaN(v) || float.IsInfinity(v)) { _nonFinite++; continue; }
                b += v;
            }
            return b;
        }

        /// The VBS this case asks for: its own if the config line carried a 9th field, else global.
        float CaseVbsPercent()
        {
            if (_cases != null && CaseIndex >= 0 && CaseIndex < _cases.Count
                && !float.IsNaN(_cases[CaseIndex].VbsPercent)) return _cases[CaseIndex].VbsPercent;
            return NeutralVbsPercent;
        }

        void CommandActuators()
        {
            if (_vbs != null) _vbs.percentage = CaseVbsPercent();
            if (_lcg != null) _lcg.percentage = NeutralLcgPercent;
        }

        /// VBS.GetCurrentValue() reads the piston's joint position, which does not exist until
        /// LinkAttachment has attached. Never let that throw — it would kill the whole matrix.
        float VbsActualPercent()
        {
            if (_vbs == null) return CaseVbsPercent();
            try { return _vbs.GetCurrentValue(); } catch { return float.NaN; }
        }

        /// The LCG piston's ACTUAL position. A commanded LCG is not an arrived LCG, and this one
        /// takes its time: MEASURED 2026-09-14, battery_link sat at z +0.0858, +0.0630 and +0.0694
        /// in three runs that all commanded LCG 50 %. 2.7 kg over 23 mm is 3.7 mm of composite CG,
        /// which is most of the residual pitch the trim solve was chasing.
        float LcgActualPercent()
        {
            if (_lcg == null) return NeutralLcgPercent;
            try { return _lcg.GetCurrentValue(); } catch { return float.NaN; }
        }

        /// A diverging articulation poisons every LATER case too, because PhysX keeps the bad
        /// state. Catch it early, pin the body, and move on with the case marked FAILED rather
        /// than writing 60 s of 1e12 and losing the rest of the matrix.
        void CheckSanity(Case c, float yTarget, bool holding)
        {
            if (_phase < 2 || _abort != "") return;
            Vector3 p = _root.transform.position, v = _root.linearVelocity;
            float pitch = Mathf.Abs(Mathf.DeltaAngle(0f, _root.transform.rotation.eulerAngles.x));

            string why = null;
            if (!IsFinite(p) || !IsFinite(v)) why = "non-finite state";
            else if (holding && Mathf.Abs(p.y - yTarget) > AbortDepthError) why = $"depth error {p.y - yTarget:F2} m";
            else if (pitch > AbortPitchDeg) why = $"pitch {pitch:F1} deg";
            else if (v.magnitude > AbortSpeed) why = $"speed {v.magnitude:F2} m/s";
            if (why == null) return;

            _abort = why; LastAbort = $"{c.Label}: {why}";
            _root.immovable = true;
            _root.linearVelocity = Vector3.zero;
            _root.angularVelocity = Vector3.zero;
            Debug.LogWarning($"[WaveSweepRig] case '{c.Label}' ABORTED in {Phase} at t={_tRec:F1}s: {why}");
        }

        static bool IsFinite(Vector3 q) =>
            !(float.IsNaN(q.x) || float.IsNaN(q.y) || float.IsNaN(q.z) ||
              float.IsInfinity(q.x) || float.IsInfinity(q.y) || float.IsInfinity(q.z));

        void LogRow()
        {
            var ci = CultureInfo.InvariantCulture;
            var c = _cases[CaseIndex];

            Vector3 cb = Vector3.zero; float vsum = 0f;
            for (int i = 0; i < _points.Length; ++i) { cb += _points[i].transform.position * _points[i].Volume; vsum += _points[i].Volume; }
            cb /= Mathf.Max(vsum, 1e-9f);

            float bsum = 0f, fmin = 1f, fmax = 0f, fsum = 0f, bcall = 0f;
            for (int i = 0; i < _points.Length; ++i)
            {
                float bc = _points[i].AppliedBuoyancyThisStep.y;
                if (!float.IsNaN(bc) && !float.IsInfinity(bc)) bcall += bc;
                float b = _points[i].IsUnderwater ? _points[i].AppliedBuoyancyForce.y : 0f;
                if (float.IsNaN(b) || float.IsInfinity(b)) b = 0f;   // see LastBuoyancySum
                bsum += b;
                float fr = _pointBmax[i] > 0f ? b / _pointBmax[i] : 0f;
                fsum += fr; fmin = Mathf.Min(fmin, fr); fmax = Mathf.Max(fmax, fr);
            }

            Vector3 p = _root.transform.position, v = _root.linearVelocity;
            Vector3 e = _root.transform.rotation.eulerAngles;
            float Wrap(float a) => a > 180f ? a - 360f : a;

            _sb.Append((_phase == 3 ? _tRec : -_tPhase).ToString("F4", ci))
               .Append(',').Append(_phase)
               .Append(',').Append(p.y.ToString("F5", ci))
               .Append(',').Append((_stillWaterY - p.y).ToString("F5", ci))
               .Append(',').Append(Wrap(e.z).ToString("F3", ci))
               .Append(',').Append(Wrap(e.x).ToString("F3", ci))
               .Append(',').Append(Wrap(e.y).ToString("F3", ci))
               .Append(',').Append(v.y.ToString("F5", ci))
               .Append(',').Append(v.x.ToString("F5", ci))
               .Append(',').Append(v.z.ToString("F5", ci));

            for (int i = 0; i < _probe.Length; ++i)
                _sb.Append(',').Append(_water.GetWaterLevelAt(_probe[i]).ToString("F5", ci));

            _sb.Append(',').Append(_water.GetWaterLevelAt(cb).ToString("F5", ci))
               .Append(',').Append(bsum.ToString("F3", ci))
               .Append(',').Append((_points.Length > 0 ? fsum / _points.Length : 0f).ToString("F4", ci))
               .Append(',').Append(fmin.ToString("F4", ci))
               .Append(',').Append(fmax.ToString("F4", ci))
               .Append(',').Append(HoldForceN.ToString("F3", ci))
               .Append(',').Append(VbsActualPercent().ToString("F1", ci))
               .Append(',').Append((_lcg != null ? _lcg.percentage : float.NaN).ToString("F1", ci))
               .Append(',').Append(LiveTotalMassKg().ToString("F5", ci))
               .Append(',').Append(bcall.ToString("F4", ci))
               .Append(',').Append(_root.linearDamping.ToString("F4", ci))
               .Append(',').Append(_root.angularDamping.ToString("F4", ci))
               .Append(',').Append(LcgActualPercent().ToString("F3", ci))
               .Append('\n');

            if (++_rows % 250 == 0) { _w.Write(_sb.ToString()); _sb.Clear(); _w.Flush(); }
        }

        // ---------------------------------------------------------------- end

        void WriteManifestHeader()
        {
            var ci = CultureInfo.InvariantCulture;
            using (var m = new StreamWriter(Path.Combine(_dir, "manifest.csv"), false, Encoding.UTF8))
            {
                m.WriteLine("# WaveSweepRig manifest, " + DateTime.UtcNow.ToString("o"));
                m.WriteLine("# scene," + UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
                m.WriteLine("# settle_water," + SettleWaterSeconds.ToString("R", ci) +
                            ",trim_find," + TrimFindSeconds.ToString("R", ci) +
                            ",record," + RecordSeconds.ToString("R", ci) +
                            ",trim_ki," + TrimKi.ToString("R", ci) +
                            ",trim_kd," + TrimKd.ToString("R", ci) +
                            ",neutral_vbs," + NeutralVbsPercent.ToString("R", ci) +
                            ",neutral_lcg," + NeutralLcgPercent.ToString("R", ci));
                m.WriteLine("label,wind_kmh,repetition_m,chaos,ripples,depth_m,hold_force_N,rows,status," +
                            "shared_seed_defect,query_non_converged,query_failed");
            }
        }

        void Finish()
        {
            _done = true; Phase = "done";
            using (var m = new StreamWriter(Path.Combine(_dir, "manifest.csv"), true, Encoding.UTF8))
                foreach (var s in _summary) m.WriteLine(s);
            Debug.Log($"[WaveSweepRig] MATRIX COMPLETE — {_summary.Count} cases -> {_dir}");
#if UNITY_EDITOR
            if (ExitPlayWhenDone) UnityEditor.EditorApplication.isPlaying = false;
#endif
        }

        void OnDisable()
        {
            if (_w != null && !_done) { CloseWriter(_cases[CaseIndex]); }
        }

        void OnDrawGizmosSelected()
        {
            if (_probe == null) return;
            Gizmos.color = Color.cyan;
            foreach (var q in _probe) Gizmos.DrawSphere(q, 0.5f);
        }
    }
}
