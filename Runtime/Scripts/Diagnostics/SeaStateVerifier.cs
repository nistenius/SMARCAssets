// SeaStateVerifier — measures the sea state the PHYSICS actually reads, and what the vehicle does in
// it, case by case, in ONE Play session (2026-09-23).
//
// It samples the scene's WaterQueryModel — the exact call ForcePoint makes, fallbacks and all — at a
// small cross of points every FixedUpdate, so what it measures is what the hull floats on, not HDRP's
// rendered surface and not a model. For each case it drives SiteEnvironment, waits for the sea to
// settle, records, and reduces:
//     Hs = 4 sigma(eta)          Tp = peak of the band-averaged DFT      Tz = zero up-crossings
//     travel bearing from the phase lag across the cross
//     VEHICLE: heave RAO = sigma(z) / sigma(eta), correlation and phase lag against eta at the CB
// and writes one summary row per case beside the SiteEnvironment prediction, plus the raw series.
//
// TWO SECOND-WRITERS IT TAKES OWNERSHIP OF, because both defeat the measurement silently:
//   * EnvironmentTimeline.Update() re-applies the site's Hs/Tp to SiteEnvironment EVERY FRAME. Left
//     running it would overwrite each case's sea state a frame after it was set, and every case would
//     read the same sea. It is disabled for the run and restored afterwards.
//   * the actuators. A vehicle nobody commands sits at VBS 0 with the watchdog value, which is not a
//     state anything was solved for (WaveSweepRig learned this the hard way on 2026-09-13). VBS and
//     LCG are commanded every FixedUpdate to VbsPercent / LcgPercent.
//
// CASES (Cases, one per line; blank/# ignored):
//     label,seastate,<Hs m>,<Tp s>[,<record s>]        SiteEnvironment drive = SeaState at that Hs/Tp
//     label,wind,<swell wind m/s>,<band multiplier>[,<record s>]   drive = Wind, multipliers set
// Record length: at least ~40 peak periods for a 10 % Hs estimate.
//
// OUTPUT: <repo>/_logs/wave/seastate/<utc>/ summary.csv, summary.md, case_<label>.csv
// The wind cases' "meas/model" column is the HsCorrection to write back into SiteEnvironment.

using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using DefaultNamespace.Water;
using Smarc.Environment;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using VehicleComponents.Actuators;

namespace Diagnostics
{
    [AddComponentMenu("Smarc/Diagnostics/Sea State Verifier")]
    public class SeaStateVerifier : MonoBehaviour
    {
        public SiteEnvironment Environment;
        [Tooltip("Optional: the vehicle whose response is measured. Its base_link is tracked against the " +
                 "surface at the SAME world x,z, so the heave RAO is against the wave the hull is in.")]
        public GameObject Vehicle;

        [TextArea(4, 16)]
        public string Cases =
            "still,seastate,0,4,60\n" +
            "hs025_tp3,seastate,0.25,3,180\n" +
            "hs05_tp4,seastate,0.5,4,180\n" +
            "hs10_tp6,seastate,1.0,6,240\n" +
            "hs18_tp13,seastate,1.8,13,300\n" +
            "w10,wind,2.78,1,180\n" +
            "w30,wind,8.33,1,240";

        public float SettleSeconds = 12f;
        [Tooltip("Default record length when a case does not give its own.")]
        public float RecordSeconds = 180f;
        [Tooltip("Cross arm length, m. 0 = automatic: lambda_p / 8, clamped to [0.5, 8] m.")]
        public float ArmLength_m = 0f;

        [Header("Actuators (commanded every step — an uncommanded vehicle is not a trim state)")]
        public bool CommandActuators = true;
        [Range(0, 100)] public float VbsPercent = 24f;
        [Range(0, 100)] public float LcgPercent = 50f;

        [Header("Scene hygiene")]
        [Tooltip("Disable EnvironmentTimeline while the sweep runs. It re-applies the site's sea state every " +
                 "frame and would overwrite every case. Restored on exit.")]
        public bool OwnTheSeaState = true;
        [Tooltip("Disable live AIS traffic for the run (network + moving hulls, neither wanted in a measurement).")]
        public bool QuietScene = true;

        [Header("Abort")]
        [Tooltip("Vertical speed that means the surface model has diverged (2026-09-13: 40.6 m/s at 60 km/h). " +
                 "The case is marked diverged and the sweep carries on.")]
        public float AbortVerticalSpeed = 20f;

        public bool ExitPlayWhenDone = false;
        public string OutputDir = "";

        [Header("Progress (read-only)")]
        public string Status = "";
        public int CaseIndex, CaseCount;

        WaterQueryModel _water;
        HDRPWaterQueryModel _hq;
        ArticulationBody _base;
        VBS _vbs; Prismatic _lcg;
        readonly List<float> _t = new List<float>();
        readonly List<float>[] _eta = { new List<float>(), new List<float>(), new List<float>() };
        readonly List<float> _vz = new List<float>(), _vpitch = new List<float>(), _etaAtVehicle = new List<float>();
        Vector3[] _pts = new Vector3[3];
        bool _recording, _diverged;
        float _worstVz;
        string _dir;
        readonly List<Behaviour> _suspended = new List<Behaviour>();

        IEnumerator Start()
        {
            if (Environment == null) Environment = FindFirstObjectByType<SiteEnvironment>();
            _water = WaterQueryModel.GetWaterQueryModel();
            _hq = _water as HDRPWaterQueryModel;
            if (Environment == null || _water == null)
            {
                Debug.LogError("[SeaStateVerifier] needs a SiteEnvironment and exactly one WaterQueryModel in the scene.");
                yield break;
            }
            ResolveVehicle();
            SuspendSecondWriters();

            string root = string.IsNullOrEmpty(OutputDir)
                ? Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "_logs", "wave", "seastate"))
                : OutputDir;
            _dir = Path.Combine(root, DateTime.UtcNow.ToString("yyyyMMdd'T'HHmmss'Z'"));
            Directory.CreateDirectory(_dir);

            var lines = new List<string>();
            foreach (string raw in Cases.Split('\n'))
            { string l = raw.Trim(); if (l.Length > 0 && !l.StartsWith("#")) lines.Add(l); }
            CaseCount = lines.Count;

            var summary = new StringBuilder();
            summary.AppendLine("label,mode,a,b,swell_wind_ms,band_mult,model_hs_phys_m,model_hs_vis_m,model_tp_s,meas_hs_m,meas_tp_s,meas_tz_s," +
                               "meas_bearing_deg,commanded_toward_deg,meas_over_model,veh_heave_rao,veh_heave_sd_m,veh_pitch_rms_deg,veh_corr," +
                               "veh_mean_y_m,diverged,worst_vz_ms,query_failed,query_total,records");
            var md = new StringBuilder();
            md.AppendLine($"# Sea state + vehicle response, measured {DateTime.UtcNow:o}");
            md.AppendLine($"scene {UnityEngine.SceneManagement.SceneManager.GetActiveScene().name}; " +
                          $"surface {(Environment.Ocean != null ? Environment.Ocean.name : "?")}, repetition {(Environment.Ocean != null ? Environment.Ocean.repetitionSize : 0):F0} m, " +
                          $"ripples in physics {(Environment.Ocean != null && Environment.Ocean.cpuEvaluateRipples)}, script interactions {(_hq != null ? _hq.ScriptInteractionsMode : "?")}; " +
                          $"HsCorrection {Environment.HsCorrection:F3}; water {SiteWater.Density():F1} kg/m3; " +
                          $"vehicle {(Vehicle != null ? Vehicle.name : "none")} at VBS {VbsPercent:F0} / LCG {LcgPercent:F0}; fixed dt {Time.fixedDeltaTime:F4} s\n");
            md.AppendLine("| case | asked | model Hs | model Tp | MEAS Hs | MEAS Tp | Tz | bearing | meas/model | heave RAO | pitch rms | failed |");
            md.AppendLine("|---|---|---|---|---|---|---|---|---|---|---|---|");

            foreach (string line in lines)
            {
                var f = line.Split(',');
                if (f.Length < 4) { Debug.LogWarning($"[SeaStateVerifier] bad case '{line}'"); continue; }
                CaseIndex++;
                string label = f[0].Trim(), mode = f[1].Trim().ToLowerInvariant();
                float a = P(f[2]), b = P(f[3]);
                float rec = f.Length > 4 ? P(f[4]) : RecordSeconds;

                var env = Environment;
                env.UseMeasuredWaves = false;
                if (mode == "seastate") { env.Drive = SiteEnvironment.OceanDrive.SeaState; env.TargetHs_m = a; env.TargetTp_s = b; }
                else { env.Drive = SiteEnvironment.OceanDrive.Wind; env.SwellWind_ms = a; if (env.Ocean != null) { env.Ocean.largeBand0Multiplier = b; env.Ocean.largeBand1Multiplier = b; } }
                env.Apply();
                float corr = Mathf.Max(env.HsCorrection, 1e-6f);
                float modelHs = env.RealisedHsPhysics_m / corr, modelHsVis = env.RealisedHsVisual_m / corr, modelTp = env.RealisedTp_s;

                float lp = 9.81f * modelTp * modelTp / (2f * Mathf.PI);
                float arm = ArmLength_m > 0f ? ArmLength_m : Mathf.Clamp(lp / 8f, 0.5f, 8f);
                Vector3 c0 = new Vector3(transform.position.x, 0f, transform.position.z);
                _pts[0] = c0; _pts[1] = c0 + Vector3.right * arm; _pts[2] = c0 + Vector3.forward * arm;

                Status = $"{CaseIndex}/{CaseCount} {label}: settling {SettleSeconds:F0} s";
                yield return new WaitForSeconds(SettleSeconds);

                if (_hq != null) _hq.ResetStats();
                _t.Clear(); foreach (var e in _eta) e.Clear();
                _vz.Clear(); _vpitch.Clear(); _etaAtVehicle.Clear();
                _diverged = false; _worstVz = 0f;
                _recording = true;
                Status = $"{CaseIndex}/{CaseCount} {label}: recording {rec:F0} s";
                yield return new WaitForSeconds(rec);
                _recording = false;

                int n = _t.Count;
                float dt = n > 1 ? (_t[n - 1] - _t[0]) / (n - 1) : Time.fixedDeltaTime;
                float hs = 0f; for (int p = 0; p < 3; ++p) hs += 4f * Std(_eta[p]); hs /= 3f;
                Spectrum(_eta[0], dt, out float fp, out _, out _);
                float tp = fp > 0 ? 1f / fp : float.NaN;
                float tz = ZeroUpcrossPeriod(_eta[0], dt);
                float bearing = Bearing(fp, dt, arm);
                float ratio = modelHs > 0 ? hs / modelHs : float.NaN;

                float vsd = Std(_vz), esd = Std(_etaAtVehicle);
                float rao = esd > 1e-4f ? vsd / esd : float.NaN;
                float vcorr = Corr(_vz, _etaAtVehicle);
                float prms = Std(_vpitch);
                float vmean = Mean(_vz);
                int failed = _hq != null ? _hq.Failed : -1, total = _hq != null ? _hq.Queries : -1;

                summary.AppendLine(string.Join(",", label, mode, I(a), I(b), I(env.SwellWind_ms), I(env.SwellBandMultiplier),
                    I(modelHs), I(modelHsVis), I(modelTp), I(hs), I(tp), I(tz), I(bearing), I(env.SwellToward_deg), I(ratio),
                    I(rao), I(vsd), I(prms), I(vcorr), I(vmean), _diverged ? "1" : "0", I(_worstVz), failed.ToString(), total.ToString(), n.ToString()));
                md.AppendLine($"| {label} | {mode} {a:0.##}, {b:0.##} | {modelHs:F3} m | {modelTp:F2} s | **{hs:F3} m** | **{tp:F2} s** | {tz:F2} s | " +
                              $"{bearing:F0}°/{env.SwellToward_deg:F0}° | {ratio:F3} | **{rao:F2}** | {prms:F2}° | {failed}/{total}{(_diverged ? " **DIVERGED**" : "")} |");

                using (var w = new StreamWriter(Path.Combine(_dir, $"case_{label}.csv")))
                {
                    w.WriteLine($"# {line}; arm {arm:F3} m; wind {env.SwellWind_ms:F3} m/s; mult {env.SwellBandMultiplier:F4}; " +
                                $"model Hs {modelHs:F4} Tp {modelTp:F3}; rho {SiteWater.Density():F1}");
                    w.WriteLine("t,eta0,eta_x,eta_z,veh_y,veh_pitch_deg,eta_at_veh");
                    for (int i = 0; i < n; ++i)
                        w.WriteLine(FormattableString.Invariant(
                            $"{_t[i]:F4},{_eta[0][i]:F5},{_eta[1][i]:F5},{_eta[2][i]:F5},{V(_vz, i)},{V(_vpitch, i)},{V(_etaAtVehicle, i)}"));
                }
                Debug.Log($"[SeaStateVerifier] {label}: model Hs {modelHs:F3} Tp {modelTp:F2} | MEASURED Hs {hs:F3} Tp {tp:F2} Tz {tz:F2} " +
                          $"bearing {bearing:F0} (cmd {env.SwellToward_deg:F0}) | meas/model {ratio:F3} | vehicle heave RAO {rao:F2} " +
                          $"(sd {vsd:F3} m vs eta {esd:F3} m, corr {vcorr:F2}), pitch rms {prms:F2} deg{(_diverged ? " — DIVERGED" : "")} | failed {failed}/{total}");
                File.WriteAllText(Path.Combine(_dir, "summary.csv"), summary.ToString());
                File.WriteAllText(Path.Combine(_dir, "summary.md"), md.ToString());
            }

            RestoreSecondWriters();
            Status = "done -> " + _dir;
            Debug.Log("[SeaStateVerifier] done -> " + _dir + "\n" + md);
#if UNITY_EDITOR
            if (ExitPlayWhenDone) UnityEditor.EditorApplication.isPlaying = false;
#endif
        }

        void OnDisable() { RestoreSecondWriters(); }

        // ---- scene ownership ---------------------------------------------------------------

        void SuspendSecondWriters()
        {
            if (OwnTheSeaState)
                foreach (var tl in FindObjectsByType<EnvironmentTimeline>(FindObjectsSortMode.None))
                    if (tl.enabled) { tl.enabled = false; _suspended.Add(tl); }
            if (QuietScene)
                foreach (var ais in FindObjectsByType<AisTrafficLive>(FindObjectsSortMode.None))
                    if (ais.enabled) { ais.enabled = false; _suspended.Add(ais); }
            if (_suspended.Count > 0)
                Debug.LogWarning("[SeaStateVerifier] suspended for the run (restored at the end): " +
                                 string.Join(", ", _suspended.ConvertAll(b => $"{b.GetType().Name} on '{b.name}'")));
        }

        void RestoreSecondWriters()
        {
            foreach (var b in _suspended) if (b != null) b.enabled = true;
            _suspended.Clear();
        }

        void ResolveVehicle()
        {
            if (Vehicle == null) return;
            foreach (var ab in Vehicle.GetComponentsInChildren<ArticulationBody>(true))
                if (ab.name == "base_link") { _base = ab; break; }
            if (_base == null) _base = Vehicle.GetComponentInChildren<ArticulationBody>(true);
            _vbs = Vehicle.GetComponentInChildren<VBS>(true);
            _lcg = null;
            foreach (var pr in Vehicle.GetComponentsInChildren<Prismatic>(true))
                if (pr.name.ToLowerInvariant().Contains("lcg")) { _lcg = pr; break; }
            Debug.Log($"[SeaStateVerifier] vehicle '{Vehicle.name}': base_link {(_base != null ? "ok" : "NOT FOUND")}, " +
                      $"VBS {(_vbs != null ? "ok" : "none")}, LCG {(_lcg != null ? _lcg.name : "none")}.");
        }

        void FixedUpdate()
        {
            if (CommandActuators)
            {
                if (_vbs != null) _vbs.SetPercentage(VbsPercent);
                if (_lcg != null) _lcg.SetPercentage(LcgPercent);
            }
            if (!_recording || _water == null) return;
            _t.Add(Time.time);
            for (int p = 0; p < 3; ++p) _eta[p].Add(_water.GetWaterLevelAt(_pts[p]));
            if (_base != null)
            {
                Vector3 pos = _base.transform.position;
                _vz.Add(pos.y);
                _etaAtVehicle.Add(_water.GetWaterLevelAt(new Vector3(pos.x, 0f, pos.z)));
                float pitch = _base.transform.eulerAngles.x; if (pitch > 180f) pitch -= 360f;
                _vpitch.Add(-pitch);
                float vz = Mathf.Abs(_base.linearVelocity.y);
                if (vz > _worstVz) _worstVz = vz;
                if (vz > AbortVerticalSpeed) _diverged = true;
            }
        }

        // ---- reduction ----------------------------------------------------------------------

        static float P(string s) => float.Parse(s.Trim(), CultureInfo.InvariantCulture);
        static string I(float v) => float.IsNaN(v) || float.IsInfinity(v) ? "" : v.ToString("0.#####", CultureInfo.InvariantCulture);
        static string V(List<float> l, int i) => i < l.Count ? l[i].ToString("F5", CultureInfo.InvariantCulture) : "";

        static float Mean(List<float> x) { if (x.Count == 0) return float.NaN; double m = 0; foreach (var v in x) m += v; return (float)(m / x.Count); }

        static float Std(List<float> x)
        {
            if (x.Count < 2) return 0f;
            double m = 0; foreach (var v in x) m += v; m /= x.Count;
            double s = 0; foreach (var v in x) s += (v - m) * (v - m);
            return (float)Math.Sqrt(s / (x.Count - 1));
        }

        static float Corr(List<float> a, List<float> b)
        {
            int n = Math.Min(a.Count, b.Count); if (n < 2) return float.NaN;
            double ma = 0, mb = 0; for (int i = 0; i < n; ++i) { ma += a[i]; mb += b[i]; } ma /= n; mb /= n;
            double sa = 0, sb = 0, sab = 0;
            for (int i = 0; i < n; ++i) { double da = a[i] - ma, db = b[i] - mb; sa += da * da; sb += db * db; sab += da * db; }
            return (sa <= 0 || sb <= 0) ? float.NaN : (float)(sab / Math.Sqrt(sa * sb));
        }

        static float ZeroUpcrossPeriod(List<float> x, float dt)
        {
            if (x.Count < 4) return float.NaN;
            double m = 0; foreach (var v in x) m += v; m /= x.Count;
            int first = -1, last = -1, count = 0;
            for (int i = 1; i < x.Count; ++i)
                if (x[i - 1] - m < 0 && x[i] - m >= 0) { if (first < 0) first = i; last = i; count++; }
            return count > 1 ? (last - first) * dt / (count - 1) : float.NaN;
        }

        // Plain DFT on a frequency grid (records are short), band-averaged over 5 bins; the peak frequency
        // is then the strongest RAW bin inside that band, so the phase used for the bearing sits ON a line.
        static void Spectrum(List<float> x, float dt, out float fPeak, out float[] freqs, out double[] psd)
        {
            int n = x.Count; double m = 0; foreach (var v in x) m += v; m /= Math.Max(n, 1);
            double T = n * dt; double df = T > 0 ? 1.0 / T : 1.0; int nf = (int)Math.Min(2.0 / df, 4000);
            freqs = new float[Math.Max(nf, 1)]; psd = new double[Math.Max(nf, 1)]; fPeak = 0;
            if (nf < 8) return;
            for (int k = 1; k < nf; ++k)
            {
                double f = k * df, re = 0, im = 0, w = 2 * Math.PI * f * dt;
                for (int i = 0; i < n; ++i) { double v = x[i] - m; re += v * Math.Cos(w * i); im -= v * Math.Sin(w * i); }
                freqs[k] = (float)f; psd[k] = (re * re + im * im);
            }
            double best = -1; int kb = 0;
            for (int k = 3; k < nf - 2; ++k)
            {
                double s = psd[k - 2] + psd[k - 1] + psd[k] + psd[k + 1] + psd[k + 2];
                if (s > best) { best = s; kb = k; }
            }
            if (kb == 0) return;
            int kr = kb; for (int k = Math.Max(1, kb - 2); k <= Math.Min(nf - 1, kb + 2); ++k) if (psd[k] > psd[kr]) kr = k;
            fPeak = freqs[kr];
        }

        // Travel direction at the peak frequency from the phase lag between the centre and the two arms.
        // eta = A cos(wt - k.r): phase(r) - phase(0) = -k.r  =>  k_x = -dphi_x / arm, k_z = -dphi_z / arm.
        float Bearing(float f, float dt, float arm)
        {
            if (f <= 0 || _t.Count < 16) return float.NaN;
            double w = 2 * Math.PI * f * dt;
            double p0 = Phase(_eta[0], w);
            double dx = Wrap(Phase(_eta[1], w) - p0), dz = Wrap(Phase(_eta[2], w) - p0);
            double kx = -dx / arm, kz = -dz / arm;          // Unity +x = grid east, +z = grid north
            double b = Math.Atan2(kx, kz) * 180.0 / Math.PI;
            return (float)((b + 360.0) % 360.0);
        }

        static double Phase(List<float> x, double w)
        {
            double re = 0, im = 0, m = 0; foreach (var v in x) m += v; m /= Math.Max(x.Count, 1);
            for (int i = 0; i < x.Count; ++i) { double v = x[i] - m; re += v * Math.Cos(w * i); im -= v * Math.Sin(w * i); }
            return Math.Atan2(im, re);
        }
        static double Wrap(double a) { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; }
    }
}
