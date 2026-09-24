using System;
using System.Globalization;
using UnityEngine;

namespace Smarc.Environment
{
    /// <summary>
    /// THE SCENARIO CLOCK — time as its own object (Ivan, 2026-09-23: "shouldn't time be its own
    /// object, as this should drive marine traffic, sun, weather, and more"). ONE per scene; every
    /// time-dependent thing READS it and owns no time of its own:
    ///
    ///   EnvironmentTimeline — the sea (tide, waves, current) and the weather (wind, cloud, fog)
    ///   SunFromClock        — the sun's direction and strength, computed from time + place
    ///   AisTrafficLive      — live ships, shown only while the clock is at NOW
    ///
    /// Modes:
    ///   Window — scenario time = WindowStart + Offset. Frozen at Offset (scrub it, edit mode too),
    ///            or advancing with Play time × TimeScale when AdvanceDuringPlay is on.
    ///   Live   — scenario time = wall-clock UTC now (what live AIS needs).
    ///
    /// NOT THE ROS CLOCK. Unity.Robotics.Core.Clock stays anchored to wall time (monotonic across
    /// Play sessions — the 2026-08-07 TF_OLD_DATA fix). This clock only selects what the world looks
    /// like; the vehicle stack never sees it.
    ///
    /// Written by BundleSiteBuilder from unity_build.json "scenario_clock" (OCEANVERSE time.json).
    /// </summary>
    [ExecuteAlways]
    [DefaultExecutionOrder(-100)]          // tick before its readers
    [AddComponentMenu("Smarc/Environment/Scenario Clock")]
    public class ScenarioClock : MonoBehaviour
    {
        public enum Mode { Window, Live }

        [Header("THE CLOCK — one per scene")]
        public Mode ClockMode = Mode.Window;
        public string WindowStartUtc = "";
        public string WindowEndUtc = "";
        [Tooltip("Window mode: hours after WindowStart. Scrub it (edit mode too) to see the world at that time.")]
        [Min(0f)] public float Offset_h = 0f;
        [Tooltip("Window mode: OFF = frozen at Offset for the whole run; ON = advances with Play time × TimeScale.")]
        public bool AdvanceDuringPlay = false;
        [Tooltip("Scenario seconds per Play second (1 = real time; 60 = a minute per second).")]
        [Min(0f)] public float TimeScale = 1f;

        [Header("Place (the sun is computed for it)")]
        public double Latitude;
        public double Longitude;
        [Tooltip("True north -> grid north at the site, deg (Unity +z is UTM GRID north).")]
        public float GridConvergence_deg;

        [Header("NOW (read-only)")]
        public string ScenarioUtc = "";
        [Tooltip("past / present / future relative to the wall clock (±30 min = present)")]
        public string Epoch = "";
        public float OffsetNow_h;
        public string Note = "";

        public static ScenarioClock Active { get; private set; }

        /// Scenario time now (UTC). Every reader uses this — never its own clock.
        public DateTime Utc { get; private set; } = DateTime.UtcNow;
        public bool HasWindow => TryUtc(WindowStartUtc, out _) && TryUtc(WindowEndUtc, out _);
        public double WindowDuration_s => TryUtc(WindowStartUtc, out var a) && TryUtc(WindowEndUtc, out var b) ? (b - a).TotalSeconds : 0;
        /// Seconds after WindowStart (negative before it); NaN without a window.
        public double SecondsIntoWindow => TryUtc(WindowStartUtc, out var a) ? (Utc - a).TotalSeconds : double.NaN;
        /// |scenario time - wall clock| ≤ 30 min: what live data (AIS) may be shown against.
        public bool IsNow => ClockMode == Mode.Live || Math.Abs((Utc - DateTime.UtcNow).TotalMinutes) <= 30;   // Live is now by definition (edit mode may not have ticked lately)

        /// PHASE TIME for anything that oscillates (the analytic wave field, ADR-011 C): seconds that
        /// ALWAYS advance with Play at 1 s per s — frozen clock or not, whatever TimeScale — offset by
        /// where in the window the run starts, so a run is reproducible from (Offset_h, Play time).
        /// Do NOT drive wave phase from Utc / SecondsIntoWindow: a frozen clock would freeze the sea,
        /// and TimeScale 60 would make the waves 60× too fast. The clock selects the SEA STATE (Hs, Tp,
        /// direction); phase time only moves it.
        public double PhaseSeconds => Offset_h * 3600.0 + (Application.isPlaying && playStart >= 0 ? Time.timeAsDouble - playStart : 0.0);

        double playStart = -1;

        public static bool TryUtc(string s, out DateTime t) =>
            DateTime.TryParse(s, CultureInfo.InvariantCulture, DateTimeStyles.AdjustToUniversal | DateTimeStyles.AssumeUniversal, out t);

        /// The clock of the scene (the first enabled one).
        public static ScenarioClock Find()
        {
            if (Active != null && Active.isActiveAndEnabled) return Active;
            Active = FindFirstObjectByType<ScenarioClock>();
            return Active;
        }

        void OnEnable() { Active = this; playStart = -1; Tick(); }
        void OnDisable() { if (Active == this) Active = null; }
        void OnValidate()
        {
            Offset_h = Mathf.Clamp(Offset_h, 0f, (float)(WindowDuration_s / 3600.0));
            Tick();
#if UNITY_EDITOR
            UnityEditor.EditorApplication.QueuePlayerLoopUpdate();   // scrubbing in edit mode: let the readers (sea, sun) follow now
#endif
        }
        void Update() { Tick(); }

        void Tick()
        {
            if (Application.isPlaying && playStart < 0) playStart = Time.timeAsDouble;
            if (ClockMode == Mode.Live)
            {
                Utc = DateTime.UtcNow;
                Note = HasWindow && TryUtc(WindowEndUtc, out var e) && Utc > e
                    ? "LIVE: now is after the data window — the sea holds its last sample; re-time the site (console `time at now`)"
                    : "LIVE: wall-clock UTC";
            }
            else if (TryUtc(WindowStartUtc, out var t0))
            {
                double t = Offset_h * 3600.0;
                if (Application.isPlaying && AdvanceDuringPlay)
                    t += (Time.timeAsDouble - playStart) * TimeScale;
                double dur = WindowDuration_s;
                Note = dur > 0 && t > dur ? "END OF WINDOW — the world holds the last sample" : "";
                Utc = t0.AddSeconds(Math.Max(0, Math.Min(t, Math.Max(dur, 0))));
            }
            else
            {
                Utc = DateTime.UtcNow;
                Note = "no window (re-export the site with a timeline) — showing wall-clock now";
            }
            ScenarioUtc = Utc.ToString("yyyy-MM-ddTHH:mm:ssZ", CultureInfo.InvariantCulture);
            double dm = (Utc - DateTime.UtcNow).TotalMinutes;
            Epoch = Math.Abs(dm) <= 30 ? "present" : dm < 0 ? "past" : "future";
            OffsetNow_h = HasWindow ? (float)(SecondsIntoWindow / 3600.0) : 0f;
        }

        [ContextMenu("Log the clock")]
        void LogNow() => Debug.Log($"[ScenarioClock] {ClockMode}: {ScenarioUtc} ({Epoch}) window {WindowStartUtc} → {WindowEndUtc}. {Note}");
    }
}
