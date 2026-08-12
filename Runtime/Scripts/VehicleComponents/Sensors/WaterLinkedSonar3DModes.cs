using System.Reflection;
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
    /// SIM DERATING (read before trusting inspection-mode numbers): the literal spec
    /// would be 300 x 33 = 9900 rays at 20 Hz — about 16x the navigation-mode load,
    /// which the 2-core rig VM cannot carry. The inspection defaults below are
    /// deliberately derated (200 x 25 at 10 Hz ~ 4x nav load) and are marked in the
    /// inspector. Raise them on the Orin, not on the VM, and never quote
    /// inspection-mode resolution as if it were spec-faithful.
    ///
    /// AutoSwitch: drop to inspection when the nearest return comes inside
    /// EnterInspectionRange, return to navigation only past ExitInspectionRange
    /// (hysteresis, so a wall at the boundary does not flap the sensor). Mode changes
    /// re-allocate the sonar's ray/bucket arrays, so they are rate-limited by
    /// MinSecondsBetweenSwitches.
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
        [Tooltip("Minimum seconds between mode changes; each change re-allocates arrays.")]
        public float MinSecondsBetweenSwitches = 3f;

        [Header("Navigation preset (1.2 MHz, WL spec)")]
        public float NavRange = 15f;
        public int NavBeams = 150;        // 90 / 150 = 0.60 deg H
        public int NavRaysPerBeam = 17;   // 40 / 17  = 2.35 deg V
        public float NavPingHz = 5f;

        [Header("Inspection preset (2.4 MHz, SIM-DERATED — see class summary)")]
        public float InsRange = 4f;
        public int InsBeams = 200;        // 0.45 deg H  (spec would be 300 = 0.30)
        public int InsRaysPerBeam = 25;   // 1.60 deg V  (spec would be  33 = 1.21)
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
            sonar.NumBeams = nav ? NavBeams : InsBeams;
            sonar.NumRaysPerBeam = nav ? NavRaysPerBeam : InsRaysPerBeam;
            sonar.frequency = nav ? NavPingHz : InsPingHz;

            // keep the point-cloud publisher in step with the ping rate
            foreach (var mb in GetComponents<MonoBehaviour>())
            {
                if (mb == null || mb == this || !(mb is ROSBehaviour rosb)) continue;
                if (rosb.topic == null || !rosb.topic.Contains("sonar3d")) continue;
                var f = mb.GetType().GetField("frequency", BindingFlags.Public | BindingFlags.Instance);
                if (f != null && f.FieldType == typeof(float))
                    f.SetValue(mb, nav ? NavPingHz : InsPingHz);
            }

            if (!reinit || !Application.isPlaying) return;

            // Re-allocate the ray/profile arrays for the new beam counts. These are
            // private in Sonar (they are called from its Awake), so reflection —
            // same pattern DeepVisionSSS already uses for the publisher rate.
            var t = sonar.GetType();
            foreach (var name in new[] { "InitHits", "InitBeamProfileSimple" })
            {
                var mi = t.GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance);
                if (mi != null) mi.Invoke(sonar, null);
                else Debug.LogWarning($"[Sonar3D-15] could not re-init '{name}' — " +
                                      "mode change may leave stale arrays; " +
                                      "restart Play if the cloud looks wrong.");
            }
        }
    }
}
