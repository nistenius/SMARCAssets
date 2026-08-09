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
        [Tooltip("Horizontal sigma per axis (east/north), m, once settled. ~1.5 m standalone; ~0.02 m if you want to pretend RTK.")]
        public double sigmaHorizontal = 1.5;
        [Tooltip("Vertical sigma, m, once settled.")]
        public double sigmaVertical = 2.0;

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

        [Header("Current receiver state")]
        public GPSFixState state = GPSFixState.Submerged;
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

        void UpdateCovariance()
        {
            double h = sigmaHorizontal * currentInflation;
            double v = sigmaVertical * currentInflation;
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

            // A commanded restart always sends us back to acquiring, but with hot-start
            // timing: the receiver keeps its ephemeris, it just drops the bad solution.
            if (restartRequested)
            {
                restartRequested = false;
                if (antennaDry)
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

            if (!antennaDry)
            {
                blackoutSeconds += dt;
                state = GPSFixState.Submerged;
                acquiringSeconds = 0f;
                settlingSeconds = 0f;
                currentInflation = settleSigmaMultiplier;
                return;
            }

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
            }

            UpdateCovariance();

            if(fix)
            {
                (easting, northing, lat, lon) = GetUTMLatLon();
                alt = transform.position.y;

                if (enableNoise)
                {
                    double sh = sigmaHorizontal * currentInflation;
                    double sv = sigmaVertical * currentInflation;
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
