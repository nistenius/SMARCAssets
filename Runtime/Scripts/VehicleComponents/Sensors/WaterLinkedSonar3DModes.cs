using System.Reflection;   // publisher rate is set via reflection (see Apply)
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;      // StringMsg
using DefaultNamespace;         // Utils.FindParentWithTag
using ROS.Core;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// Water Linked Sonar 3D-15 operating modes (as on SAM 2.2).
    ///
    /// The real unit runs at two frequencies with different FOV/range/rate/beam width.
    /// CORRECTED 2026-09-09 against the Water Linked datasheet (waterlinked.com/datasheets/
    /// sonar-3D-15, read that day; SETTLED §3ad):
    ///   NAVIGATION  1.2 MHz : 90 deg x 40 deg, 15 m, 5 Hz, beams 0.85 deg H x 1.60 deg V
    ///   INSPECTION  2.4 MHz : 40 deg x 40 deg,  4 m, 20 Hz, beams 0.45 deg H x 0.85 deg V
    /// Range resolution 1.5 mm; minimum range 20 cm; 20 W.
    ///
    /// THE FIELD OF VIEW DOES **NOT** STAY 90 x 40. This summary said it did, and that was an
    /// over-claim: the high-frequency mode is 40 deg x 40 deg, i.e. it gives up more than half
    /// its azimuth fan to buy the finer beams. The sim does not model either FOV change or the
    /// finer beams (see below), so nothing in the scene changes with this correction — but a
    /// mission planned against "90 deg in both" would expect an inspection-mode fan that does
    /// not exist on the real unit.
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

        [Header("ROS: the mode is MISSION-COMMANDED, and always announced")]
        [Tooltip("Robot namespace for the two topics. Empty = resolve from the parent tagged 'robot'.")]
        public string RobotName = "";
        [Tooltip("Publish payload/sonar3d/mode as 'Mode|range_m|ping_hz'. Consumers (obstacle " +
                 "detector, margin rose) MUST bound themselves by this range instead of assuming " +
                 "15 m — otherwise they keep asserting a horizon the sensor no longer has.")]
        public bool PublishMode = true;
        [Tooltip("Accept payload/sonar3d/set_mode ('navigation' | 'inspection').")]
        public bool AcceptModeCommands = true;

        [Header("Automatic mode switching (OFF by default — see class summary)")]
        [Tooltip("Range-triggered switching. DEFAULT OFF as of 2026-08-12: changing the sensor's " +
                 "horizon changes the evidence base of the protective stop and the speed governor, " +
                 "and neither of them asked for it. Inspection is a deliberate act; the mission " +
                 "knows when it is inspecting, the sonar does not.")]
        public bool AutoSwitch = false;
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
        // 2026-09-09: was 10 f. The datasheet says 20 Hz and the previous comment said so
        // while the value did not — a number that documents its own disagreement is the shape
        // SETTLED §3k warns about. Corrected to the spec. NOT COMPILED: this session has no
        // Unity, so the effect on the scene is unverified and the prefab's own serialized
        // InsPingHz (also 10, also corrected) is what the built scene actually reads.
        public float InsPingHz = 20f;     // datasheet: 20 Hz

        float lastSwitchTime = -999f;
        float noReturnSince = -1f;
        Sonar sonar;
        ROSConnection ros;
        string modeTopic, setModeTopic;
        float lastAnnounce = -999f;

        void Start()
        {
            if (!PublishMode && !AcceptModeCommands) return;
            if (string.IsNullOrEmpty(RobotName))
            {
                var robot = Utils.FindParentWithTag(gameObject, "robot", false);
                RobotName = robot != null ? robot.name : "sam_auv_v1";
            }
            ros = ROSConnection.GetOrCreateInstance();
            modeTopic = $"/{RobotName}/payload/sonar3d/mode";
            setModeTopic = $"/{RobotName}/payload/sonar3d/set_mode";
            if (PublishMode) ros.RegisterPublisher<StringMsg>(modeTopic);
            if (AcceptModeCommands)
                ros.Subscribe<StringMsg>(setModeTopic, m =>
                {
                    var s = (m.data ?? "").Trim().ToLowerInvariant();
                    if (s.StartsWith("nav")) Switch(Mode.Navigation);
                    else if (s.StartsWith("ins")) Switch(Mode.Inspection);
                    else Debug.LogWarning($"[Sonar3D-15] set_mode '{m.data}' not understood " +
                                          "(expected 'navigation' or 'inspection') — mode unchanged.");
                });
            Announce();
        }

        /// <summary>Tell the world what horizon it is actually looking through.
        ///
        /// Range and ping rate are not cosmetic: the protective stop's envelope
        /// R_stop(u) = margin + u*t_react + u^2/(2a) reaches 4.25 m at 0.5 m/s,
        /// which EXCEEDS the 4 m inspection horizon. A consumer that assumes 15 m
        /// while the sensor sees 4 m is computing a stop it cannot observe the
        /// trigger for, and the margin rose would report free space out to 14.5 m
        /// on a claim the sensor never made.</summary>
        void Announce()
        {
            if (!PublishMode || ros == null) return;
            bool nav = CurrentMode == Mode.Navigation;
            ros.Publish(modeTopic, new StringMsg(
                $"{CurrentMode}|{(nav ? NavRange : InsRange):F1}|{(nav ? NavPingHz : InsPingHz):F1}"));
            lastAnnounce = Time.time;
        }

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
            // Re-announce at 1 Hz, not only on change: a consumer that starts late
            // (or restarts with the stack, which happens every arm) would otherwise
            // never learn the horizon and would fall back to assuming 15 m.
            if (PublishMode && Time.time - lastAnnounce > 1f) Announce();

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
            Announce();   // the horizon just changed — say so immediately
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
