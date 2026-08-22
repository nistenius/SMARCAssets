using UnityEngine;
using GeoRef;

using DefaultNamespace.Water;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// How the receiver behaves after the antenna comes out of the water.
    /// A real GPS does not produce a trustworthy fix the instant it surfaces: it has to
    /// re-acquire satellites (time-to-first-fix, longer the longer it was blacked out),
    /// and the first fixes after that are noticeably worse than the settled solution.
    /// Ivan, 2026-08-08: "a GPS that has been disconnected for a longer time takes a
    /// while to settle once surfaced, so we can't just rely on the first fix we get."
    /// </summary>
    public enum GPSFixState
    {
        Submerged,   // antenna underwater, receiver blacked out
        Acquiring,   // surfaced, counting down time-to-first-fix, NO fix published
        Settling,    // fix published, but covariance inflated and decaying
        Tracking     // settled, nominal accuracy
    }

    [AddComponentMenu("Smarc/Sensor/GPS")]
    public class GPS: Sensor
    {
        [Header("GPS")]
        public double easting;
        public double northing;
        public double lat;
        public double lon;
        public double alt;
        [Tooltip("True only when a usable fix is being published. False while submerged AND while re-acquiring.")]
        public bool fix;

        [Header("Noise (u-blox standalone; SAM tank bags showed 0.1-5.8 m E/N scatter, median ~1.2 m)")]
        [Tooltip("Add noise to the published values. Covariances are populated either way. Disable for deterministic regression runs.")]
        public bool enableNoise = true;
        [Tooltip("0 = new random seed every run. Any other value = repeatable noise sequence.")]
        public int noiseSeed = 0;
        [Tooltip("Horizontal sigma per axis (east/north), m, once settled, STANDALONE (no RTK corrections). ZED-F9P autonomous ~1.5 m CEP.")]
        public double sigmaHorizontal = 1.5;
        [Tooltip("Vertical sigma, m, once settled, standalone.")]
        public double sigmaVertical = 2.0;

        [Header("RTK (SparkFun GPS-RTK2 / u-blox ZED-F9P, as fitted to SAM)")]
        [Tooltip("Receiver is RTK-capable AND a correction stream is configured. Turn OFF to model a plain GNSS receiver or a dead NTRIP link.")]
        public bool rtkEnabled = true;
        [Tooltip("Corrections are actually arriving right now. Drop this to watch the solution fall back to float, then standalone — that is what a lost link looks like.")]
        public bool rtkCorrectionsAvailable = true;
        [Tooltip("Horizontal sigma with an RTK FIXED solution [m]. Spec is 10 mm + 1 ppm of baseline; 0.014 m ~ 10 mm at a few km from the base.")]
        public double sigmaHorizontalRtkFixed = 0.014;
        [Tooltip("Vertical sigma with RTK FIXED [m] — roughly 1.5-2x horizontal, as usual for GNSS.")]
        public double sigmaVerticalRtkFixed = 0.025;
        [Tooltip("Horizontal sigma while the ambiguities are still FLOAT [m].")]
        public double sigmaHorizontalRtkFloat = 0.35;
        [Tooltip("Vertical sigma while FLOAT [m].")]
        public double sigmaVerticalRtkFloat = 0.6;
        [Tooltip("Seconds of continuous corrections after the first fix before the solution goes FIXED. Short baselines converge in seconds; this is the honest 'RTK is not instant' term.")]
        public float rtkFixSeconds = 12f;

        [Header("Re-acquisition after surfacing")]
        [Tooltip("Model time-to-first-fix and settling. Disable for the old instant-perfect-fix behaviour.")]
        public bool modelReacquisition = true;
        [Tooltip("TTFF when the blackout was short (hot start): ephemeris still valid.")]
        public float hotStartSeconds = 2f;
        [Tooltip("Blackouts shorter than this count as a hot start.")]
        public float hotStartMaxBlackout = 120f;
        [Tooltip("TTFF after a medium blackout (warm start).")]
        public float warmStartSeconds = 20f;
        [Tooltip("Blackouts shorter than this count as a warm start; longer is a cold start.")]
        public float warmStartMaxBlackout = 7200f;
        [Tooltip("TTFF after a long blackout (cold start): almanac stale, full sky search.")]
        public float coldStartSeconds = 60f;
        [Tooltip("Time constant of the accuracy improvement after the first fix. Sigma decays from settleSigmaMultiplier*sigma to sigma.")]
        public float settleTimeConstant = 15f;
        [Tooltip("How much worse the first fixes are than the settled solution.")]
        public float settleSigmaMultiplier = 6f;
        [Tooltip("Stop reporting Settling once the inflation has decayed below this factor.")]
        public float settledThreshold = 1.15f;

        [Header("Splash tolerance (the receiver debounce)")]
        [Tooltip("The antenna must be CONTINUOUSLY wet for longer than this before the receiver "
                 + "declares a blackout and resets its state machine. Splashes shorter than this "
                 + "hold the current state and keep the settling clocks running. "
                 + "0 = the old behaviour: any wet frame is a blackout. "
                 + "THE VALUE IS THE PI'S CALL — 0.5 s is a placeholder, not a measurement.")]
        public float wetGraceSeconds = 0.5f;
        // Why this exists at all (2026-08-17): the antenna sits 0.071 m above base_link and
        // pitch dunks it at the surface, so the fix flickers in any sea state. A real receiver
        // does not lose carrier-phase continuity over a sub-second splash — it keeps tracking
        // and the solution survives. Without the grace, every dunk sent the state machine back
        // to Submerged, which restarts acquisition and zeroes settlingSeconds — and
        // settlingSeconds is what rtkFixSeconds (12 s) counts, so RTK FIXED (the +-0.1 m the
        // nav_ready latch wants) would be UNREACHABLE in any sea state, for a reason nothing
        // in the model would name. Splashes longer than the grace are still real blackouts and
        // the submerged time already counted is carried into blackoutSeconds, so the hot/warm/
        // cold start choice is not made cheaper by the debounce.

        /// <summary>Which solution the receiver is currently producing. Exposed so the
        /// HUD and the estimator can tell a 1.5 m fix from a 1.4 cm one — they are the
        /// same message type and wildly different information.</summary>
        public enum RtkSolution { Standalone, RtkFloat, RtkFixed }

        [Header("Current receiver state")]
        public GPSFixState state = GPSFixState.Submerged;
        [Tooltip("Current RTK solution quality (read-only).")]
        public RtkSolution rtkSolution = RtkSolution.Standalone;
        [Tooltip("Seconds the antenna has been continuously underwater (drives hot/warm/cold start).")]
        public float blackoutSeconds;
        [Tooltip("Seconds since the receiver started acquiring.")]
        public float acquiringSeconds;
        [Tooltip("Seconds since the first fix of this surfacing.")]
        public float settlingSeconds;
        [Tooltip("Current sigma inflation vs the settled value. 1 = fully settled.")]
        public float currentInflation = 1f;
        [Tooltip("TTFF chosen for this surfacing, from the blackout duration.")]
        public float currentTTFF;
        [Tooltip("Seconds the antenna has been continuously wet WITHOUT a blackout being declared "
                 + "yet. Non-zero here with state != Submerged is a splash being ridden out.")]
        public float wetSeconds;

        [Tooltip("ENU diagonal, m^2, for NavSatFix.position_covariance. Reflects the inflation while settling.")]
        public double[] positionCovariance = new double[9];

        private GaussianNoise noise;
        private GlobalReferencePoint _gpsRef;
        private WaterQueryModel _waterModel;
        private bool restartRequested = false;

        void Start()
        {
            noise = new GaussianNoise(noiseSeed);
            UpdateCovariance();

            var gpsRefs = FindObjectsByType<GlobalReferencePoint>(FindObjectsSortMode.None);
            if(gpsRefs.Length < 1)
            {
                Debug.Log("No GPS Reference found in the scene. Setting values to 0");
                easting = 0.0;
                northing = 0.0;
                lat = 0.0;
                lon = 0.0;
                fix = true;
            }
            else _gpsRef = gpsRefs[0];

            var waterModels = FindObjectsByType<WaterQueryModel>(FindObjectsSortMode.None);
            if(waterModels.Length < 1) Debug.Log("No water query model found. GPS will always run.");
            else _waterModel = waterModels[0];
        }

        /// <summary>
        /// Command the receiver to restart and re-acquire. Operationally this is what you do
        /// when a receiver comes up after a long dive and sits on a bad multipath solution:
        /// a clean hot restart often reaches a trustworthy fix sooner than waiting for the
        /// stale solution to converge. Wired to ROS by GPSRestart_Sub.
        /// </summary>
        public void RestartReceiver()
        {
            restartRequested = true;
        }

        public (double, double, double, double) GetUTMLatLon()
        {
            return _gpsRef.GetUTMLatLonOfObject(gameObject);
        }

        /// <summary>Settled sigmas for the CURRENT solution type. RTK does not merely
        /// scale the standalone error — it is a different measurement, cm instead of
        /// metres, which is why the estimator's GPS gate must see the covariance and
        /// not a hardcoded number.</summary>
        public (double h, double v) SettledSigmas() => rtkSolution switch
        {
            RtkSolution.RtkFixed => (sigmaHorizontalRtkFixed, sigmaVerticalRtkFixed),
            RtkSolution.RtkFloat => (sigmaHorizontalRtkFloat, sigmaVerticalRtkFloat),
            _                    => (sigmaHorizontal, sigmaVertical),
        };

        /// <summary>Advance the RTK solution. Corrections must be flowing AND the
        /// receiver must already have a fix; losing corrections drops straight back
        /// to standalone, which is the failure mode that actually bites on the water
        /// (NTRIP over a flaky link, or out of range of the base).</summary>
        void UpdateRtkSolution(float dt)
        {
            if (!rtkEnabled || !rtkCorrectionsAvailable || !fix)
            {
                rtkSolution = RtkSolution.Standalone;
                return;
            }
            // settlingSeconds counts from the first fix of this surfacing
            rtkSolution = settlingSeconds >= rtkFixSeconds
                ? RtkSolution.RtkFixed : RtkSolution.RtkFloat;
        }

        void UpdateCovariance()
        {
            var (sh, sv) = SettledSigmas();
            double h = sh * currentInflation;
            double v = sv * currentInflation;
            positionCovariance[0] = h * h;
            positionCovariance[4] = h * h;
            positionCovariance[8] = v * v;
        }

        float ChooseTTFF(float blackout)
        {
            if (blackout <= hotStartMaxBlackout) return hotStartSeconds;
            if (blackout <= warmStartMaxBlackout) return warmStartSeconds;
            return coldStartSeconds;
        }

        void UpdateReceiverState(bool antennaDry, double deltaTime)
        {
            float dt = (float)deltaTime;

            // The debounce. A wet antenna is not yet a blackout: it has to STAY wet longer
            // than wetGraceSeconds. Once a blackout is declared it holds until the antenna is
            // dry again (state == Submerged is the latch), so this cannot chatter.
            if (antennaDry) wetSeconds = 0f;
            else wetSeconds += dt;
            bool blackedOut = !antennaDry
                && (state == GPSFixState.Submerged || wetSeconds > wetGraceSeconds);

            // A commanded restart always sends us back to acquiring, but with hot-start
            // timing: the receiver keeps its ephemeris, it just drops the bad solution.
            if (restartRequested)
            {
                restartRequested = false;
                if (!blackedOut)
                {
                    state = GPSFixState.Acquiring;
                    acquiringSeconds = 0f;
                    currentTTFF = hotStartSeconds;
                    settlingSeconds = 0f;
                    currentInflation = 1f;   // clean re-acquire, no stale-solution penalty
                    Debug.Log($"[{transform.name}] GPS restart commanded: re-acquiring (hot, {currentTTFF:F0}s)");
                }
                else
                {
                    Debug.Log($"[{transform.name}] GPS restart commanded while submerged: ignored until surfaced");
                }
            }

            if (blackedOut)
            {
                if (state != GPSFixState.Submerged)
                {
                    // First frame of a REAL blackout. The grace period was underwater too, so
                    // it counts towards the blackout — the debounce must not make a hot start
                    // out of a warm one.
                    blackoutSeconds += wetSeconds;
                    state = GPSFixState.Submerged;
                    if (wetGraceSeconds > 0f)
                        Debug.Log($"[{transform.name}] GPS antenna wet for {wetSeconds:F2}s "
                                  + $"(> {wetGraceSeconds:F2}s grace): blackout");
                }
                else blackoutSeconds += dt;
                wetSeconds = 0f;
                acquiringSeconds = 0f;
                settlingSeconds = 0f;
                currentInflation = settleSigmaMultiplier;
                return;
            }
            // Not blacked out. Either the antenna is dry, or it is wet and inside the grace —
            // in which case the receiver holds its state and every clock below keeps running,
            // which is the whole point of the grace.

            switch (state)
            {
                case GPSFixState.Submerged:
                    // just surfaced
                    state = GPSFixState.Acquiring;
                    acquiringSeconds = 0f;
                    currentTTFF = ChooseTTFF(blackoutSeconds);
                    Debug.Log($"[{transform.name}] GPS surfaced after {blackoutSeconds:F0}s blackout, "
                              + $"acquiring for {currentTTFF:F0}s");
                    blackoutSeconds = 0f;
                    break;

                case GPSFixState.Acquiring:
                    acquiringSeconds += dt;
                    if (acquiringSeconds >= currentTTFF)
                    {
                        state = GPSFixState.Settling;
                        settlingSeconds = 0f;
                        if (currentInflation < settleSigmaMultiplier && currentInflation > 1f)
                        {
                            // came from a restart: already clean
                        }
                        Debug.Log($"[{transform.name}] GPS first fix acquired, settling");
                    }
                    break;

                case GPSFixState.Settling:
                    settlingSeconds += dt;
                    currentInflation = 1f + (settleSigmaMultiplier - 1f)
                        * Mathf.Exp(-settlingSeconds / Mathf.Max(0.01f, settleTimeConstant));
                    if (currentInflation <= settledThreshold)
                    {
                        currentInflation = 1f;
                        state = GPSFixState.Tracking;
                        Debug.Log($"[{transform.name}] GPS settled after {settlingSeconds:F0}s");
                    }
                    break;

                case GPSFixState.Tracking:
                    currentInflation = 1f;
                    // FIX 2026-08-21: keep counting. `settlingSeconds` is "seconds since the
                    // first fix of this surfacing", which UpdateRtkSolution compares against
                    // rtkFixSeconds — freezing it here means a receiver that settles in under
                    // rtkFixSeconds could never be promoted to RTK FIXED afterwards.
                    settlingSeconds += dt;
                    break;
            }
        }

        public override bool UpdateSensor(double deltaTime)
        {
            if(_gpsRef == null) return false;
            if (noise == null) noise = new GaussianNoise(noiseSeed);

            bool antennaDry = _waterModel == null
                || transform.position.y > _waterModel.GetWaterLevelAt(transform.position);

            if (modelReacquisition)
            {
                UpdateReceiverState(antennaDry, deltaTime);
                // Only Settling and Tracking publish a usable fix. Acquiring is surfaced but
                // still searching -- that is the interval a mission must not trust.
                fix = state == GPSFixState.Settling || state == GPSFixState.Tracking;
            }
            else
            {
                fix = antennaDry;
                currentInflation = 1f;
                state = antennaDry ? GPSFixState.Tracking : GPSFixState.Submerged;
                // FIX 2026-08-21 — the answer to SETTLED §3s "why does the station never reach
                // RTK FIXED". With reacquisition modelling OFF, nothing ever advanced
                // `settlingSeconds`, so UpdateRtkSolution's `settlingSeconds >= rtkFixSeconds`
                // was 0 >= 12 forever and the solution was pinned at RTK FLOAT (sigma 0.35) by
                // construction. station_agent then — correctly — withheld the position forever
                // ("not accurate enough to arm on, limit 0.15"). A steady dry antenna must still
                // EARN the fix: count continuous dry seconds, reset on a wet antenna, exactly
                // the "~14 s continuously dry" arithmetic the modelled path implements.
                settlingSeconds = antennaDry ? settlingSeconds + (float)deltaTime : 0f;
            }

            UpdateRtkSolution((float)deltaTime);
            UpdateCovariance();

            if(fix)
            {
                (easting, northing, lat, lon) = GetUTMLatLon();
                alt = transform.position.y;

                if (enableNoise)
                {
                    var (sh0, sv0) = SettledSigmas();
                    double sh = sh0 * currentInflation;
                    double sv = sv0 * currentInflation;
                    double nE = noise.Sample(sh);
                    double nN = noise.Sample(sh);
                    easting += nE;
                    northing += nN;
                    // Small-offset conversion of the meter-noise to lat/lon.
                    lat += nN / 111320.0;
                    lon += nE / (111320.0 * System.Math.Cos(lat * System.Math.PI / 180.0));
                    alt += noise.Sample(sv);
                }
            }

            return fix;
        }
    }
}
