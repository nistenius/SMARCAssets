using UnityEngine;
using ROS.Core;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// Water Linked Sonar 3D-15 operating modes (as on SAM 2.2).
    ///
    /// The real unit runs at two frequencies with different range/rate/beam width:
    ///   NAVIGATION  1.2 MHz : 15 m,  5 Hz, 0.6 deg H x 2.4 deg V  — transit, long look
    ///   INSPECTION  2.4 MHz :  4 m, 20 Hz, ~0.3 deg H x 1.2 deg V — close work, fine detail
    /// Field of view stays 90 deg x 40 deg in both; what changes is range, ping rate
    /// and angular resolution.
    ///
    /// WHAT THE SIM SWITCH ACTUALLY CHANGES: range and ping rate only. Ray counts
    /// are fixed at Awake — see Apply() for why (live re-allocation crashes every
    /// consumer holding a sized buffer, RayViewer first). At 4 m the same 2550 rays
    /// land ~4x denser on target, which is the detail inspection mode is for; the
    /// spec's finer beams (0.3 x 1.2 deg) are NOT modelled, so do not quote sim
    /// inspection resolution as spec-faithful.
    ///
    /// AutoSwitch: drop to inspection when the nearest return comes inside
    /// EnterInspectionRange, return to navigation only past ExitInspectionRange
    /// (hysteresis, so a wall at the boundary does not flap the sensor), rate-limited
    /// by MinSecondsBetweenSwitches.
    /// </summary>
    [DefaultExecutionOrder(-100)] // apply before Sonar.Awake sizes its arrays
    [RequireComponent(typeof(Sonar))]
    [AddComponentMenu("Smarc/Sensor/WaterLinkedSonar3DModes")]
    public class WaterLinkedSonar3DModes : MonoBehaviour
    {
        public enum Mode { Navigation, Inspection }

        [Tooltip("Current operating mode. Applied at Play, and at runtime if AutoSwitch is on.")]
        public Mode CurrentMode = Mode.Navigation;

        [Header("Automatic mode switching")]
        [Tooltip("Switch to Inspection when structure comes close, back out again when it recedes.")]
        public bool AutoSwitch = true;
        [Tooltip("Nearest return closer than this [m] -> Inspection. Keep below the navigation-mode stop envelope so the switch happens BEFORE the vehicle is committed.")]
        public float EnterInspectionRange = 3.0f;
        [Tooltip("Nearest return beyond this [m] -> Navigation. MUST be < InsRange or the exit can never be observed (inspection mode cannot see past its own MaxRange).")]
        public float ExitInspectionRange = 3.5f;
        [Tooltip("Seconds with NO returns at all in inspection mode before falling back to navigation. Empty water is the normal way an inspection ends.")]
        public float NoReturnFallbackSeconds = 2f;
        [Tooltip("Minimum seconds between mode changes (hysteresis in time as well as range).")]
        public float MinSecondsBetweenSwitches = 3f;

        [Header("Navigation preset (1.2 MHz, WL spec)")]
        public float NavRange = 15f;
        public int NavBeams = 150;        // 90 / 150 = 0.60 deg H
        public int NavRaysPerBeam = 17;   // 40 / 17  = 2.35 deg V
        public float NavPingHz = 5f;

        [Header("Inspection preset (2.4 MHz — range/rate only, see class summary)")]
        public float InsRange = 4f;
        public int InsBeams = 150;        // keep == NavBeams: see Apply()
        public int InsRaysPerBeam = 17;   // keep == NavRaysPerBeam: see Apply()
        public float InsPingHz = 10f;     // spec 20 Hz

        float lastSwitchTime = -999f;
        float noReturnSince = -1f;
        Sonar sonar;

        void OnValidate()
        {
            // Exit must be observable FROM inspection mode: its MaxRange is the
            // horizon there. A threshold beyond InsRange deadlocks the sensor in
            // inspection forever (found on the rig 2026-08-12 — the vehicle passed
            // a wreck, switched in, then reported "no data" for the rest of the run).
            float maxObservable = InsRange - 0.5f;
            if (ExitInspectionRange > maxObservable) ExitInspectionRange = maxObservable;
            if (ExitInspectionRange <= EnterInspectionRange)
                EnterInspectionRange = Mathf.Max(0.5f, ExitInspectionRange - 0.5f);
            Apply(CurrentMode, reinit: false);
        }

        void Awake()
        {
            sonar = GetComponent<Sonar>();
            Apply(CurrentMode, reinit: false);   // before Sonar.Awake sizes arrays
        }

        void Update()
        {
            if (!AutoSwitch || sonar == null) return;
            if (Time.time - lastSwitchTime < MinSecondsBetweenSwitches) return;

            float nearest = NearestHitRange();

            if (float.IsInfinity(nearest))
            {
                // NO RETURNS. In navigation mode that is just open water — stay put.
                // In inspection mode it is the normal END of an inspection: the
                // sensor's 4 m horizon means "nothing in range" is the ONLY way to
                // observe that we have moved away. Treating infinity as "no
                // information" here deadlocked the sonar in inspection mode for a
                // whole mission (rig, 2026-08-12) — the exit threshold sat beyond
                // the mode's own MaxRange and could never be seen.
                if (CurrentMode == Mode.Inspection)
                {
                    if (noReturnSince < 0f) noReturnSince = Time.time;
                    if (Time.time - noReturnSince >= NoReturnFallbackSeconds)
                        Switch(Mode.Navigation);
                }
                return;
            }
            noReturnSince = -1f;

            if (CurrentMode == Mode.Navigation && nearest < EnterInspectionRange)
                Switch(Mode.Inspection);
            else if (CurrentMode == Mode.Inspection && nearest > ExitInspectionRange)
                Switch(Mode.Navigation);
        }

        /// <summary>Closest real return this frame, or +inf if the sonar sees nothing.</summary>
        public float NearestHitRange()
        {
            if (sonar == null || sonar.SonarHits == null) return Mathf.Infinity;
            float best = Mathf.Infinity;
            foreach (var h in sonar.SonarHits)
            {
                if (h == null || h.ReturnIntensity <= 0f) continue;
                float d = h.Hit.distance;
                if (d > 0.05f && d < best) best = d;
            }
            return best;
        }

        void Switch(Mode m)
        {
            lastSwitchTime = Time.time;
            noReturnSince = -1f;
            CurrentMode = m;
            Apply(m, reinit: true);
            Debug.Log($"[Sonar3D-15] mode -> {m} " +
                      $"({(m == Mode.Navigation ? NavRange : InsRange)} m, " +
                      $"{(m == Mode.Navigation ? NavPingHz : InsPingHz)} Hz)");
        }

        public void Apply(Mode m, bool reinit)
        {
            if (sonar == null) sonar = GetComponent<Sonar>();
            if (sonar == null) return;

            bool nav = m == Mode.Navigation;
            sonar.MaxRange = nav ? NavRange : InsRange;
            sonar.frequency = nav ? NavPingHz : InsPingHz;
            // RAY COUNTS ARE SET ONCE, AT AWAKE — never at runtime.
            // Changing them live re-allocates Sonar.SonarHits, but RayViewer (and
            // anything else holding a sized buffer) allocated ITS arrays at startup
            // for the old count: the result is a per-frame IndexOutOfRangeException
            // storm out of RayViewer.UpdateHits (seen on the rig 2026-08-12).
            // The physically meaningful part of inspection mode is the shorter
            // range and faster ping anyway — at 4 m the SAME 2550 rays land ~4x
            // denser on target, which is the detail the mode is for.
            if (!Application.isPlaying)
            {
                sonar.NumBeams = nav ? NavBeams : InsBeams;
                sonar.NumRaysPerBeam = nav ? NavRaysPerBeam : InsRaysPerBeam;
            }

            // keep the point-cloud publisher in step with the ping rate
            foreach (var mb in GetComponents<MonoBehaviour>())
            {
                if (mb == null || mb == this || !(mb is ROSBehaviour rosb)) continue;
                if (rosb.topic == null || !rosb.topic.Contains("sonar3d")) continue;
                var f = mb.GetType().GetField("frequency", BindingFlags.Public | BindingFlags.Instance);
                if (f != null && f.FieldType == typeof(float))
                    f.SetValue(mb, nav ? NavPingHz : InsPingHz);
            }

            // No array re-allocation, so nothing to re-init: the switch is now
            // just two scalars and is safe mid-flight.
        }
    }
}
