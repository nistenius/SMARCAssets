using System;
using UnityEngine;

namespace Smarc.Environment
{
    /// <summary>
    /// The SEA and the WEATHER over time, played from the scene's ScenarioClock (it owns no time of
    /// its own — Ivan, 2026-09-23: time is its own object). Holds the site package's environment
    /// TIMELINE (every sample tagged measured / predicted tide / forecast / model / reanalysis) and
    /// applies the sample at the clock's time:
    ///
    ///   * SiteEnvironment (the Ocean's sea state, SI) — swell / ripple wind and direction, and its
    ///     MEASURED record fields, so the Inspector always shows the sea "now";
    ///   * CurrentField — speed and heading only; CurrentEnabled is never touched (scenario knob);
    ///   * the TIDE: the Ocean transform must stay at the origin (HDRPWaterQueryModel, SETTLED
    ///     2026-08-18), so a rising sea moves the terrain, structures and hazards DOWN by the same
    ///     amount (TideMovers), relative to where the bridge built them (sea surface at WindowStart);
    ///   * the WEATHER: cloud cover (read by SunFromClock), visibility -> the HDRP Fog of the scene's
    ///     Volume (Play mode only: Volume.profile is a per-play copy, the asset is never edited),
    ///     precipitation and air temperature as state.
    ///
    /// Outside the window the ends are held. Gaps (-999 = no provider at that time): the last valid
    /// value is HELD and StateNow says so. Disable this component to edit the sea by hand.
    /// </summary>
    [ExecuteAlways]
    [AddComponentMenu("Smarc/Environment/Environment Timeline (sea + weather)")]
    public class EnvironmentTimeline : MonoBehaviour
    {
        public const float Missing = -999f;

        [Header("WINDOW (from OCEANVERSE environment.json timeline)")]
        public string WindowStartUtc = "";
        public string WindowEndUtc = "";
        [Tooltip("When the providers were asked: 'forecast' means after this time, 'measured' before it.")]
        public string GeneratedUtc = "";
        public string Epochs = "";
        [TextArea(2, 8)] public string Sources = "";
        [TextArea(1, 4)] public string Gaps = "";

        [Header("Time comes from the ScenarioClock")]
        [Tooltip("The scene's clock (the builder sets it). Scrub / advance time THERE, not here.")]
        public ScenarioClock Clock;
        [Tooltip("Move terrain / structures / hazards with the tide (the Ocean stays at y = 0).")]
        public bool ApplyTideToGeometry = true;
        [Tooltip("Set the HDRP Fog mean free path from the visibility (Play mode; Koschmieder: mfp = visibility / 3.912).")]
        public bool ApplyVisibilityToFog = true;

        [Header("NOW (read-only)")]
        public string ScenarioUtc = "";
        public float ScenarioOffset_h;
        [Tooltip("Cloud cover now, % (NaN = no data). SunFromClock dims the sun with it.")]
        public float CloudCoverNow_pct = float.NaN;
        public float VisibilityNow_m = float.NaN;
        [TextArea(3, 10)] public string StateNow = "";

        [Header("Targets")]
        public SiteEnvironment Environment;
        public CurrentField Current;
        [Tooltip("Moved in y with the tide. The builder fills this with the terrain tiles, the structures and hazards roots.")]
        public Transform[] TideMovers = new Transform[0];
        [Tooltip("Their y as built (sea surface at WindowStart). Written by the builder; do not edit.")]
        public float[] TideMoverBaseY = new float[0];

        [Header("Series (SI; written by the builder)")]
        public float StepSeconds = 600f;
        public float BuildSeaLevelMsl_m;
        public float[] T_s = new float[0];
        public float[] SeaLevelMsl_m = new float[0], TideDy_m = new float[0];
        public float[] SwellWind_ms = new float[0], RippleWind_ms = new float[0], SwellToward_deg = new float[0];
        public float[] WaveHs_m = new float[0], WaveTp_s = new float[0], WaveFrom_deg = new float[0];
        public float[] WindSpeed_ms = new float[0], WindFrom_deg = new float[0];
        public float[] CurrentSpeed_ms = new float[0], CurrentToward_deg = new float[0];
        public float[] CloudCover_pct = new float[0], Visibility_m = new float[0], Precipitation_mm_h = new float[0], AirTemperature_c = new float[0];
        public string KindSeaLevel = "", KindWaves = "", KindWind = "", KindCurrent = "", KindWeather = "", KindAirTemperature = "";

        float lastEvaluated = float.NaN;
        float lastDy = float.NaN;
        UnityEngine.Rendering.HighDefinition.Fog fog;
        bool fogLooked;

        public float Duration_s => T_s != null && T_s.Length > 0 ? T_s[T_s.Length - 1] : 0f;

        void OnEnable() { lastEvaluated = float.NaN; fogLooked = false; Tick(); }
        void OnValidate() { lastEvaluated = float.NaN; Tick(); }
        void Update() { Tick(); }

        void Tick()
        {
            if (Clock == null) Clock = ScenarioClock.Find();
            if (Clock == null) { StateNow = "no ScenarioClock in the scene — rebuild the site (SMARC → Rebuild Last Curated Bundle)"; return; }
            if (!ScenarioClock.TryUtc(WindowStartUtc, out var t0)) return;
            float t = (float)Math.Max(0, Math.Min((Clock.Utc - t0).TotalSeconds, Duration_s));
            if (!float.IsNaN(lastEvaluated) && Mathf.Abs(t - lastEvaluated) < 1f) return;   // at most once per scenario second
            Evaluate(t);
        }

        // ---- interpolation ---------------------------------------------------------------
        bool Sample(float[] a, float t, bool circular, out float v, out bool held)
        {
            v = 0f; held = false;
            if (a == null || a.Length == 0 || T_s == null || T_s.Length != a.Length) return false;
            int n = a.Length;
            float step = StepSeconds > 0 ? StepSeconds : 1f;
            int i = Mathf.Clamp(Mathf.FloorToInt(t / step), 0, n - 1);
            int j = Mathf.Min(i + 1, n - 1);
            float f = j == i ? 0f : Mathf.Clamp01((t - T_s[i]) / Mathf.Max(1e-3f, T_s[j] - T_s[i]));
            bool vi = a[i] > Missing + 1f, vj = a[j] > Missing + 1f;
            // outside the window the nearest sample is HELD — say so (2026-09-24: a Live clock 54 min past the
            // window end showed the last tide, +0.82 m, as if it were now)
            held = t < T_s[0] - 0.5f * step || t > T_s[n - 1] + 0.5f * step;
            if (vi && vj) { v = circular ? Mathf.Repeat(a[i] + Mathf.DeltaAngle(a[i], a[j]) * f, 360f) : Mathf.Lerp(a[i], a[j], f); return true; }
            if (vi && f < 0.5f) { v = a[i]; return true; }
            if (vj && f >= 0.5f) { v = a[j]; return true; }
            for (int k = i; k >= 0; k--) if (a[k] > Missing + 1f) { v = a[k]; held = true; return true; }   // HOLD across a gap
            return false;
        }

        static char KindAt(string codes, float t, float step)
        {
            if (string.IsNullOrEmpty(codes)) return '-';
            int i = Mathf.Clamp(Mathf.RoundToInt(t / Mathf.Max(1f, step)), 0, codes.Length - 1);
            return codes[i];
        }

        static string KindWord(char c)
        {
            switch (c)
            {
                case 'M': return "measured";
                case 'P': return "predicted tide";
                case 'Q': return "predicted tide + measured surge residual";
                case 'F': return "forecast";
                case 'A': return "model";
                case 'R': return "reanalysis";
                case 'C': return "computed";
                default: return "no data";
            }
        }

        // ---- apply ----------------------------------------------------------------------
        public void Evaluate(float t)
        {
            if (T_s == null || T_s.Length == 0) { StateNow = "no timeline (the site package has none — attach one with ovsite.environment --bundle … --for 6h)"; return; }
            lastEvaluated = t;
            ScenarioOffset_h = t / 3600f;
            if (ScenarioClock.TryUtc(WindowStartUtc, out var t0))
                ScenarioUtc = t0.AddSeconds(t).ToString("yyyy-MM-ddTHH:mm:ssZ");
            var sb = new System.Text.StringBuilder();
            bool h;
            float stepW = Mathf.Max(1f, StepSeconds), tEnd = T_s[T_s.Length - 1];
            if (t > tEnd + 0.5f * stepW || t < T_s[0] - 0.5f * stepW)
                sb.AppendLine($"OUTSIDE THE WINDOW ({(t < T_s[0] ? $"{(T_s[0] - t) / 60f:F0} min before its start" : $"{(t - tEnd) / 60f:F0} min past its end")}): " +
                              "every value below is HELD at the nearest sample, not the sea at this time — re-time the site " +
                              "(console `time at now for 6h`, then `export smarc_unity`).");

            if (Environment != null)
            {
                if (Sample(SwellWind_ms, t, false, out var sw, out h)) Environment.SwellWind_ms = sw;
                if (Sample(RippleWind_ms, t, false, out var rw, out _)) Environment.RippleWind_ms = Mathf.Min(rw, 13.9f);
                if (Sample(SwellToward_deg, t, true, out var tw, out _)) { Environment.SwellToward_deg = tw; Environment.RippleToward_deg = tw; }
                if (Sample(WaveHs_m, t, false, out var hs, out bool hh)) Environment.WaveHs_m = hs;
                if (Sample(WaveTp_s, t, false, out var tp, out _)) Environment.WaveTp_s = tp;
                if (Sample(WaveFrom_deg, t, true, out var wf, out _)) Environment.WaveFrom_deg = wf;
                if (Sample(WindSpeed_ms, t, false, out var ws, out bool hw)) Environment.WindSpeed_ms = ws;
                if (Sample(WindFrom_deg, t, true, out var wd, out _)) Environment.WindFrom_deg = wd;
                if (Sample(SeaLevelMsl_m, t, false, out var sl, out _)) Environment.SeaLevelMsl_m = sl;
                Environment.MeasuredAtUtc = ScenarioUtc + " (timeline)";
                Environment.Apply();
                sb.AppendLine($"waves Hs {Environment.WaveHs_m:F2} m, Tp {Environment.WaveTp_s:F1} s from {Environment.WaveFrom_deg:F0}° — {KindWord(KindAt(KindWaves, t, StepSeconds))}{(hh ? " (HELD: no data now)" : "")}");
                sb.AppendLine($"wind {Environment.WindSpeed_ms:F1} m/s from {Environment.WindFrom_deg:F0}° — {KindWord(KindAt(KindWind, t, StepSeconds))}{(hw ? " (HELD)" : "")}");
            }
            if (Current != null && Sample(CurrentSpeed_ms, t, false, out var cs, out bool hc) && Sample(CurrentToward_deg, t, true, out var cd, out _))
            {
                Current.SpeedMS = cs; Current.HeadingDeg = cd;
                sb.AppendLine($"current {cs:F2} m/s toward {cd:F0}° — {KindWord(KindAt(KindCurrent, t, StepSeconds))}{(hc ? " (HELD)" : "")}" +
                              (Current.CurrentEnabled ? "" : " [CurrentField OFF: vehicles do not feel it]"));
            }
            if (Sample(TideDy_m, t, false, out var dy, out bool ht) && Sample(SeaLevelMsl_m, t, false, out var eta, out _))
            {
                sb.AppendLine($"sea level {eta:+0.000;-0.000} m MSL — {KindWord(KindAt(KindSeaLevel, t, StepSeconds))}{(ht ? " (HELD)" : "")}; " +
                              (ApplyTideToGeometry ? $"terrain moved {dy:+0.000;-0.000} m" : "tide NOT applied to geometry"));
                ApplyTide(ApplyTideToGeometry ? dy : 0f);
            }
            bool hcl = false;
            CloudCoverNow_pct = Sample(CloudCover_pct, t, false, out var cc, out hcl) ? cc : float.NaN;
            VisibilityNow_m = Sample(Visibility_m, t, false, out var vis, out bool hv) ? vis : float.NaN;
            string pr = Sample(Precipitation_mm_h, t, false, out var pp, out _) ? $", precipitation {pp:F1} mm/h" : "";
            string at = Sample(AirTemperature_c, t, false, out var ta, out _) ? $", air {ta:F1} °C ({KindWord(KindAt(KindAirTemperature, t, StepSeconds))})" : "";
            if (!float.IsNaN(CloudCoverNow_pct) || !float.IsNaN(VisibilityNow_m) || pr.Length > 0 || at.Length > 0)
                sb.AppendLine($"weather: cloud {(float.IsNaN(CloudCoverNow_pct) ? "?" : CloudCoverNow_pct.ToString("F0"))} %" +
                              $", visibility {(float.IsNaN(VisibilityNow_m) ? "?" : (VisibilityNow_m / 1000f).ToString("F1") + " km")}{pr}{at}" +
                              $" — {KindWord(KindAt(KindWeather, t, StepSeconds))}{(hcl || hv ? " (HELD)" : "")}");
            if (ApplyVisibilityToFog && Application.isPlaying && !float.IsNaN(VisibilityNow_m)) ApplyFog(VisibilityNow_m, sb);
            StateNow = sb.ToString().TrimEnd();
        }

        void ApplyFog(float visibility_m, System.Text.StringBuilder sb)
        {
            if (!fogLooked)
            {
                fogLooked = true;
                foreach (var vol in FindObjectsByType<UnityEngine.Rendering.Volume>(FindObjectsSortMode.None))
                {
                    if (!vol.isGlobal || vol.sharedProfile == null) continue;
                    if (vol.profile.TryGet(out fog)) break;       // .profile: this Play's copy — the asset is never edited
                }
            }
            if (fog == null) { sb.AppendLine("fog: no global Volume with a Fog override — visibility not applied"); return; }
            fog.enabled.overrideState = true; fog.enabled.value = true;
            fog.meanFreePath.overrideState = true;
            fog.meanFreePath.value = Mathf.Max(1f, visibility_m / 3.912f);
        }

        void ApplyTide(float dy)
        {
            if (!float.IsNaN(lastDy) && Mathf.Abs(dy - lastDy) < 0.002f) return;   // < 2 mm: do not touch static colliders
            lastDy = dy;
            if (TideMovers == null || TideMoverBaseY == null) return;
            for (int k = 0; k < TideMovers.Length && k < TideMoverBaseY.Length; k++)
            {
                var tr = TideMovers[k];
                if (tr == null) continue;
                var p = tr.position;
                p.y = TideMoverBaseY[k] + dy;
                tr.position = p;
            }
        }

        [ContextMenu("Log scenario time now")]
        void LogNow() => Debug.Log($"[EnvironmentTimeline] scenario {ScenarioUtc} (+{ScenarioOffset_h:F2} h of {Duration_s / 3600f:F1} h)\n{StateNow}");
    }
}
