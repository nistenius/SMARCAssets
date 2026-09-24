using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Smarc.Environment
{
    /// <summary>
    /// The sea at this site, in SI units — the ONE place to read and edit it (Ivan, 2026-09-22:
    /// "use SI units in the sim throughout").
    ///
    /// Written by BundleSiteBuilder from the site package's environment.json (OCEANVERSE): the
    /// MEASURED block is what the providers reported at that time (read-only record); the
    /// APPLIED block is what drives the scene, and starts equal to the measurement. Edit the
    /// applied values here, never the Ocean's WaterSurface fields: HDRP stores its wind in km/h
    /// internally, and this component is the only place that converts (KmhPerMs below).
    ///
    /// Units: metres, seconds, metres per second; directions are COMPASS degrees (0 = north,
    /// 90 = east, clockwise). Waves and wind are given FROM, the swell and the current TOWARD.
    ///
    /// The water plane is Unity y = 0 = the sea surface at the environment's time; the bridge
    /// placed terrain and hazards SeaLevelMsl_m lower than their MSL heights. The Ocean transform
    /// stays at the origin (HDRPWaterQueryModel, SETTLED 2026-08-18).
    /// </summary>
    [ExecuteAlways]
    [AddComponentMenu("Smarc/Environment/Site Environment (SI)")]
    public class SiteEnvironment : MonoBehaviour
    {
        const float KmhPerMs = 3.6f;                       // HDRP WaterSurface wind unit is km/h
        const float HdrpSwellMaxWind_ms = 250f / KmhPerMs; // 69.4 m/s
        const float HdrpRippleMaxWind_ms = 50f / KmhPerMs; // 13.9 m/s

        [Header("MEASURED — from OCEANVERSE environment.json (record, SI)")]
        public string MeasuredAtUtc = "";
        [Tooltip("Sea surface above MSL at that time, m (NOAA CO-OPS). Already applied as geometry: MSL is at Unity y = -this.")]
        public float SeaLevelMsl_m;
        [Tooltip("Significant wave height, m")] public float WaveHs_m = float.NaN;
        [Tooltip("Peak wave period, s")] public float WaveTp_s = float.NaN;
        [Tooltip("Waves come FROM, compass deg")] public float WaveFrom_deg = float.NaN;
        [Tooltip("Wind speed at ~10 m, m/s")] public float WindSpeed_ms = float.NaN;
        [Tooltip("Wind comes FROM, compass deg")] public float WindFrom_deg = float.NaN;
        [Tooltip("Surface current speed, m/s (NaN = no provider answered)")] public float CurrentSpeed_ms = float.NaN;
        [Tooltip("Current flows TOWARD, compass deg")] public float CurrentToward_deg = float.NaN;
        [TextArea(2, 6)] public string Sources = "";

        [Header("APPLIED — drives the Ocean (SI; edit here)")]
        [Tooltip("Wind that builds the HDRP swell, m/s. HDRP derives the swell height from this wind; " +
                 "a distant-storm swell (large Hs, low local wind) needs more than the local wind.")]
        [Range(0f, 69.4f)] public float SwellWind_ms = 2.25f;
        [Tooltip("The swell travels TOWARD this compass bearing, deg")]
        [Range(0f, 360f)] public float SwellToward_deg = 0f;
        [Tooltip("Local wind for the ripples, m/s (HDRP caps it at 13.9 m/s)")]
        [Range(0f, 13.9f)] public float RippleWind_ms = 2.25f;
        [Tooltip("Ripples travel TOWARD this compass bearing, deg")]
        [Range(0f, 360f)] public float RippleToward_deg = 0f;

        public enum OceanDrive { Wind, SeaState }

        [Header("SEA STATE DRIVE (2026-09-23) — the Ocean made to a stated Hs / Tp")]
        [Tooltip("Wind: SwellWind_ms above is applied as-is (HDRP decides Hs and Tp). SeaState: the swell wind " +
                 "is SOLVED so HDRP's spectral peak sits at the target Tp, and both swell-band amplitude " +
                 "multipliers are solved so the surface the PHYSICS reads has the target Hs. HDRP's own spectrum " +
                 "(Phillips, fixed-hash noise) is evaluated exactly by HdrpSeaStateModel — not a lookup table.")]
        public OceanDrive Drive = OceanDrive.Wind;
        [Tooltip("SeaState drive: take Hs/Tp from the MEASURED block (WaveHs_m / WaveTp_s — the timeline writes " +
                 "them too) when both are finite; otherwise use the targets below.")]
        public bool UseMeasuredWaves = true;
        [Tooltip("Target significant wave height, m (SeaState drive, when not taken from the measured block)")]
        public float TargetHs_m = 0.5f;
        [Tooltip("Target peak period, s (SeaState drive, when not taken from the measured block)")]
        public float TargetTp_s = 4f;
        [Tooltip("Measured / model Hs from a SeaStateVerifier wind sweep (1 = trust the model). 2026-09-13 Asko: " +
                 "0.83 at 10 km/h, 0.96 at 30 km/h. Set it from the verifier, not by eye.")]
        public float HsCorrection = 1f;

        [Header("SEA STATE — what the Ocean carries now (read-only, from HDRP's own spectrum)")]
        [Tooltip("Hs of the surface the buoyancy reads (swell bands, plus ripples only if cpuEvaluateRipples), m")]
        public float RealisedHsPhysics_m;
        [Tooltip("Hs of the rendered surface (all bands), m. If it differs from the physics Hs the hull is NOT " +
                 "floating on what you see: turn cpuEvaluateRipples on.")]
        public float RealisedHsVisual_m;
        public float RealisedTp_s, PeakWavelength_m, SwellBandMultiplier = 1f;
        [TextArea(2, 6)] public string SeaStateNote = "";

        [Header("Targets")]
        public WaterSurface Ocean;
        [Tooltip("The measured surface current lives on this CurrentField (m/s, compass TO); OFF by default.")]
        public CurrentField Current;

        /// Compass bearing (0 = north, clockwise) -> HDRP orientation (0 = +X east, counter-clockwise).
        /// ASSUMED convention — verify once: the swell must travel toward SwellToward_deg.
        public static float CompassToHdrp(float compassDeg) => Mathf.Repeat(90f - compassDeg, 360f);

        void OnEnable() { Apply(); }
        void OnValidate() { Apply(); }

        [ContextMenu("Apply to Ocean")]
        public void Apply()
        {
            if (Ocean == null) Ocean = GetComponent<WaterSurface>();
            if (Ocean == null) return;
            Ocean.largeWindSpeed = Mathf.Clamp(SwellWind_ms, 0f, HdrpSwellMaxWind_ms) * KmhPerMs;
            Ocean.largeOrientationValue = CompassToHdrp(SwellToward_deg);
            Ocean.ripplesWindSpeed = Mathf.Clamp(RippleWind_ms, 0f, HdrpRippleMaxWind_ms) * KmhPerMs;
            Ocean.ripplesOrientationValue = CompassToHdrp(RippleToward_deg);
            Ocean.largeCurrentSpeedValue = 0f;   // visual drift off; the vehicle's current is CurrentField
            ApplySeaState();
        }

        // ---- the sea state (2026-09-23) ------------------------------------------------------

        // NonSerialized: Unity's domain reload restores private STRING fields but not the (non-Serializable)
        // struct, which left a valid key pointing at an all-zero cached result (seen 2026-09-23: Hs 0, Tp 0).
        [System.NonSerialized] string _cacheKey; [System.NonSerialized] HdrpSeaStateModel.Result _cache;

        static int HdrpResolution()
        {
            var a = (QualitySettings.renderPipeline ?? GraphicsSettings.defaultRenderPipeline) as HDRenderPipelineAsset;
            return a != null ? (int)a.currentPlatformRenderPipelineSettings.waterSimulationResolution : 256;
        }

        HdrpSeaStateModel.Result EvaluateOcean(float swellWind_ms, float mult)
        {
            var i = new HdrpSeaStateModel.Inputs
            {
                Repetition_m = Ocean.repetitionSize, SwellWind_ms = swellWind_ms, Chaos = Ocean.largeChaos,
                OrientationDeg = Ocean.largeOrientationValue, Band0Multiplier = mult, Band1Multiplier = mult,
                Ripples = Ocean.ripples, RippleWind_ms = Ocean.ripplesWindSpeed / KmhPerMs, RippleChaos = Ocean.ripplesChaos,
                RippleOrientationDeg = Ocean.ripplesOrientationValue, RipplesInPhysics = Ocean.cpuEvaluateRipples,
                Resolution = HdrpResolution(),
            };
            string key = System.FormattableString.Invariant($"{i.Repetition_m}|{i.SwellWind_ms}|{i.Chaos}|{i.OrientationDeg}|{mult}|{i.Ripples}|{i.RippleWind_ms}|{i.RippleChaos}|{i.RippleOrientationDeg}|{i.RipplesInPhysics}|{i.Resolution}");
            if (key == _cacheKey) return _cache;
            _cache = HdrpSeaStateModel.Evaluate(i); _cacheKey = key;
            return _cache;
        }

        void ApplySeaState()
        {
            if (Ocean.surfaceType != WaterSurfaceType.OceanSeaLake)
            {
                SeaStateNote = $"surface type {Ocean.surfaceType}: no swell bands; the sea-state drive applies to OceanSeaLake only.";
                return;
            }
            float hs = TargetHs_m, tp = TargetTp_s; string src = "targets";
            if (UseMeasuredWaves && !float.IsNaN(WaveHs_m) && !float.IsNaN(WaveTp_s) && WaveTp_s > 0f) { hs = WaveHs_m; tp = WaveTp_s; src = "measured block"; }

            if (Drive == OceanDrive.SeaState && hs >= 0f && tp > 0f)
            {
                float v = (float)HdrpSeaStateModel.WindForPeakPeriod(tp);
                SwellWind_ms = v;
                Ocean.largeWindSpeed = v * KmhPerMs;
                var unit = EvaluateOcean(v, 1f);
                float m = (float)HdrpSeaStateModel.MultiplierForHs(hs, unit, Ocean.cpuEvaluateRipples, HsCorrection);
                Ocean.largeBand0Multiplier = m;
                Ocean.largeBand1Multiplier = m;
            }
            SwellBandMultiplier = Ocean.largeBand0Multiplier;

            var r = EvaluateOcean(Ocean.largeWindSpeed / KmhPerMs, Ocean.largeBand0Multiplier);
            float c = HsCorrection > 0f ? HsCorrection : 1f;
            RealisedHsPhysics_m = (float)r.HsPhysics_m * c;
            RealisedHsVisual_m = (float)r.HsVisual_m * c;
            RealisedTp_s = (float)r.Tp_s;
            PeakWavelength_m = (float)r.PeakWavelength_m;
            var sb = new System.Text.StringBuilder();
            sb.Append(Drive == OceanDrive.SeaState
                ? $"SeaState drive from {src}: Hs {hs:F2} m, Tp {tp:F1} s -> swell wind {SwellWind_ms:F2} m/s, band multiplier {SwellBandMultiplier:F3}."
                : "Wind drive: the Hs / Tp above are what this wind makes.");
            if (!Ocean.scriptInteractions) sb.Append("\nscriptInteractions OFF: the physics reads NO waves (a flat plane).");
            if (Ocean.ripples && !Ocean.cpuEvaluateRipples) sb.Append($"\ncpuEvaluateRipples OFF: physics Hs {RealisedHsPhysics_m:F2} m vs rendered {RealisedHsVisual_m:F2} m; the hull does not float on the ripples you see.");
            if (!r.PeakFitsPatch) sb.Append($"\nPeak wavelength {PeakWavelength_m:F0} m exceeds half the Repetition Size ({Ocean.repetitionSize:F0} m): the patch truncates the peak. Raise Repetition Size on the Water Surface.");
            if (Mathf.Abs(Ocean.largeBand1Multiplier - Ocean.largeBand0Multiplier) > 1e-4f) sb.Append("\nBand 1 multiplier differs from band 0; the reported Hs assumes both equal band 0.");
            SeaStateNote = sb.ToString();
        }
    }
}
