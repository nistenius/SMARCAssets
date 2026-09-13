using System; //Bit converter
using UnityEngine;
using System.Collections.Generic;

using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using NormalDistribution = DefaultNamespace.NormalDistribution;
using Smarc.Environment;   // SeabedAcousticMap — per-cell bottom type, see below


namespace VehicleComponents.Sensors
{
    public class SonarHit
    {
        public RaycastHit Hit;
        public float ReturnIntensity;
        public int MaterialLabel;
        Sonar sonar;

        public static readonly Dictionary<string, float> simpleMaterialReflectivity = new Dictionary<string, float>()
        {
            // GENERIC materials, for anything that is not a classified seabed: a rock wall, a
            // boulder, a quay. Deliberately NOT retuned on 2026-08-29 — the measurement that
            // collapsed the seabed spread (SETTLED §3f0h) was made on SEABED, and "inferred
            // bedrock bottom" is not the same object as "a rock surface". A scene without a
            // SeabedAcousticMap still gets these.
            {"Rock", 0.8f},
            {"Mud", 0.2f},
            // Sand and Clay added 2026-08-27 so a plain PhysicsMaterial named "Sand"/"Clay"
            // agrees with what SeabedAcousticMap returns for the same bottom type. Clay moved
            // 0.18 -> 0.45 on 2026-08-29 to stay in step with the retuned seabed classes
            // (build_splatmap.ACOUSTICS); leaving it at 0.18 would have made the fallback and
            // the map disagree by 8 dB for the same named bottom.
            {"Sand", 0.5f},
            {"Clay", 0.45f},
            // A steel hull on a soft bottom is what makes a wreck or a sunk car show up on
            // sonar at all — it must out-reflect bedrock, not merely match it.
            {"Steel", 0.95f},
            {"Buoy", 0.99f},  // Buoy, Algae, and Rope are currently just wild guesses
            {"Algae", 0.25f},
            {"Rope", 0.4f}
        };

        public static readonly Dictionary<string, int> materialLabels = new Dictionary<string, int>()
        {
            // Labels are assigned based on importance, lower values will be over-written by higher values 
            {"Rock", 1},
            {"Mud", 1},
            {"Sand", 1},   // seabed types all share 1: these labels are a priority ordering,
            {"Clay", 1},   // not an identity — see GetMaterialLabel()
            {"Steel", 5},  // a man-made target SHOULD outrank the seabed it lies on
            {"Buoy", 3},  // Buoy, Algae, and Rope are currently just wild guesses
            {"Algae", 2},
            {"Rope", 4}
        };

        public SonarHit(Sonar sonar)
        {
            ReturnIntensity = -1;
            MaterialLabel = 0;
            this.sonar = sonar;
        }

        public void Update(RaycastHit hit, float beam_intensity)
        {
            
            ReturnIntensity = GetIntensity(beam_intensity);
            MaterialLabel = GetMaterialLabel();
            this.Hit = hit;
        }

        static string CleanUpMaterialName(string name)
        {
            // name can have " (instance of)" added to it,
            // remove that...
            if(name.Contains("(")) return name.Split("(")[0].Trim();
            return name;
        }


        public float GetMaterialReflectivity()
        {
            // Return some default value for things that dont hit
            // 0 intensity = no hit
            if(!(Hit.collider)) return 0f;

            // A TerrainCollider carries ONE PhysicsMaterial for the whole terrain, so a 4 km
            // seabed would otherwise answer every ping with a single hardness. If the terrain
            // publishes a per-cell bottom-type map, ask it where this ray actually landed.
            // Additive and fail-open: no map -> the original material path below, unchanged.
            var seabed = SeabedAcousticMap.For(Hit.collider);
            if(seabed != null && seabed.TryGet(Hit.point, out float r, out _, out _, out _))
                return r;

            if(!(Hit.collider.material)) return 0.5f;

            string name = CleanUpMaterialName(Hit.collider.material.name);

            // if its a simple one, just return that
            if(simpleMaterialReflectivity.ContainsKey(name)) return simpleMaterialReflectivity[name];
            // if its a complex material that we want a function for,
            // switch for it here?
            // TODO that switch lol
            return 0.5f;
        }
        
        public int GetMaterialLabel()
        {
            // Return some default value for things that dont hit
            // 0 is default for no hit, hit w/o material, hit w/o named material no is materialLabels{}
            if(!(Hit.collider)) return 0;

            // Same per-cell seabed map as GetMaterialReflectivity. Note every seabed class
            // deliberately reports label 1: these labels are a PRIORITY ordering ("lower
            // values will be over-written by higher"), not an identity, and Rock and Mud are
            // both 1 already. Bottom type travels in the return INTENSITY.
            var seabed = SeabedAcousticMap.For(Hit.collider);
            if(seabed != null && seabed.TryGet(Hit.point, out _, out int lbl, out _, out _))
                return lbl;

            if(!(Hit.collider.material)) return 0;

            string name = CleanUpMaterialName(Hit.collider.material.name);

            // Return the label
            if(materialLabels.ContainsKey(name)) return materialLabels[name];
            // if the named material has ne specified label
            return 0;
        }

        public float GetIntensity(float beamIntensity)
        {
            // intensity of hit between 1-255
            // It is a function of
            // 1) The distance travelled by the beam.
            //
            // TIME-VARYING GAIN, 2026-08-29. The DeepVision board applies TVG to the raw
            // signal BEFORE it outputs anything, so the sensor's output -- what an operator
            // sees on a towfish display, and what lands in the .dvs -- is already
            // range-compensated. Ivan's call: the synthetic sensor should match the board's
            // OUTPUT, not the acoustic field. This supersedes §3f0e's "TVG belongs in the
            // display, the sensor stays raw", which was the right call when we had no
            // reference record to compare against and the wrong one now that we do.
            //
            // MEASURED, and it is why this is not a tuning knob:
            //     mean byte vs slant range,  6 m -> 38 m
            //     real 2024-12-10 :  39.0  35.9  26.5  25.6  25.6  25.6   (-3.6 dB, FLAT past 18 m)
            //     sim before this :  87.9  62.1  24.3  12.9   5.6   3.9   (-27 dB)
            // The old `(MaxRange - r)/MaxRange` term is not a spreading law at all -- it is a
            // linear ramp to ZERO at max range, so the outer swath went dark by construction
            // (91.3% of bins at the noise floor in the 34-40 m band, against 1.7% real).
            //
            // What replaces it: a spreading compensation r/refRange. For a flat bottom the
            // incidence term below carries h/r, so the product is constant with range --
            // which is exactly the flat profile the real record shows. Nothing here invents
            // signal; it removes a decay the real instrument also removes.
            float hitDistIntensity;
            if(sonar.IsIdeal)
            {
                // IDEAL FIDELITY: NO RANGE LAW AT ALL. Ivan's spec, 2026-08-29: "measure the
                // distance to the hit and scale a reflection value by texture and slant
                // angle" -- distance selects the BIN, it does not scale the value. What
                // remains is a pure reflectance image: beam pattern x cos(incidence) x
                // reflectivity, which is also auvlib's base model.
                //
                // The consequence is intended and must not be "fixed" by adding gain back:
                // over a flat bottom cos(incidence) = h/r, so the record falls ~11 dB from
                // 6 m to 40 m (modelled: 64 -> 19 bytes, model_ideal_fan.py). The real
                // record is FLAT past 18 m because the DeepVision board applies TVG. Putting
                // that back is realism layer 5, added and measured on its own -- not smuggled
                // into the ideal core.
                hitDistIntensity = 1f;
            }
            else if(sonar.UseTvg)
            {
                float refR = Mathf.Max(sonar.TvgReferenceRangeM, 0.1f);
                hitDistIntensity = Mathf.Pow(Mathf.Max(Hit.distance, 0.1f)/refR, sonar.TvgExponent);
            }
            else
            {
                hitDistIntensity = (sonar.MaxRange - Hit.distance) / sonar.MaxRange;
            }

            // 2) The angle of hit -> angle between the ray and normal
            // the hit originated from transform position, and hit sonarHit
            float hitAngle = Vector3.Angle(sonar.transform.position - Hit.point, Hit.normal);
            float hitAngleIntensity = Mathf.Abs(Mathf.Cos(hitAngle*Mathf.Deg2Rad));

            // 3) The properties of the point of hit -> material
            // if available, use the material of the hit object to determine the reflectivitity.
            float hitMaterialIntensity = GetMaterialReflectivity();

            // Lambert's cosine law:
            // Intensity = K * Ensonification at point * Reflectivity at p * abs(cos(incidence angle at p))
            // We just set K = 1 here.
            // Ensonification = distance traveled
            // Reflectivity = material prop.
            // Angle is obvious.
            // beamIntensity accounts for the angular dependence of the ensonification intensity, beam profile
            float intensity = beamIntensity * hitDistIntensity * hitAngleIntensity * hitMaterialIntensity;
            if(intensity > 1) intensity=1;
            if(intensity < 0) intensity=0;

            
            return intensity;
        }

        public byte[] GetBytes()
        {
            // so first, we gotta convert the unity points to ros points
            // then x,y,z need to be byte-ified
            // then a fourth "intensity" needs to be created and byte-ified
            var point = Hit.point.To<ENU>();

            var xb = BitConverter.GetBytes(point.x);
            var yb = BitConverter.GetBytes(point.y);
            var zb = BitConverter.GetBytes(point.z);

            byte[] ib = {(byte)(ReturnIntensity*255)};

            int totalBytes = xb.Length + yb.Length + zb.Length+ ib.Length;
            byte[] ret = new byte[totalBytes];
            // src, offset, dest, offset, count
            // Imma hard-code the offsets and counts, to act as a weird
            // error catching mechanism
            Buffer.BlockCopy(xb, 0, ret, 0, 4);
            Buffer.BlockCopy(yb, 0, ret, 4, 4);
            Buffer.BlockCopy(zb, 0, ret, 8, 4);
            Buffer.BlockCopy(ib, 0, ret, 12,1);

            return ret;
        }
    }


    public enum SonarType
    {
        FLS,
        SSS,
        MBES
    }


    /// <summary>
    /// How much of the real instrument's behaviour the sensor models.
    ///
    /// 2026-08-29, Ivan, after three sessions of layering realism onto the SSS: step back to
    /// a fan whose every term is understood, then re-add realism ONE measured layer at a time.
    /// The stack that grew (Lambert x texture-hash -> TVG -> footprint -> floor -> phase ->
    /// multi-look -> clamp) was individually motivated at every step and never designed as a
    /// whole, and its INTERACTIONS became the dominant unknown -- skew swinging -0.60..+0.87
    /// between uniform stretches of one run where the real record sits at +0.05 everywhere.
    ///
    ///   Ideal      one fan per transducer; per ray: reflectivity x |cos(incidence)| x beam
    ///              pattern, sorted into range bins. NO range law, NO reverberation floor, NO
    ///              additive range noise, NO along-track multi-look, NO interferometric phase
    ///              noise. Shadows are empty BY OCCLUSION -- a bin no ray reached reads zero.
    ///              This is the reference image: crisp, fully explainable, and the baseline
    ///              every realism layer must be A/B'd against.
    ///
    ///   Realistic  the full modelled stack. Preserved unchanged so the comparison is one
    ///              toggle and nothing measured so far is lost.
    ///
    /// Ideal is NOT "low quality" -- it is the model whose every byte can be predicted from
    /// geometry (see scripts/sss-tuning/model_ideal_fan.py, which predicted this mode's
    /// levels, fill and shadow contrast before a line of it was written).
    /// </summary>
    public enum SonarFidelity
    {
        Ideal,
        Realistic
    }


    [AddComponentMenu("Smarc/Sensor/Sonar")]
    public class Sonar : Sensor
    {

        [Header("Sonar")]
        public SonarType Type = SonarType.MBES;
        [Tooltip("Numer of rays cast per beam. Beam = A fan of rays.")]
        public int NumRaysPerBeam = 500;
        [Tooltip("Total opening angle of _each_ beam.")]
        public float BeamBreadthDeg = 90;
        [Tooltip("How many beams(fans) are in this arrangement of sonar. For FLS, they will be arranged left-to-right with fan opening in the forward axis. For SSS and MBES, they will be arranged left-to-right with fan opening also left-to-right.")]
        public int NumBeams = 1;
        public float MaxRange = 100;
        [Tooltip("-3dB opening angle of each beam. For beam-pattern related return intensity calculations.")]
        public float BeamBreadth3DecibelsDeg = 60;
        [Tooltip("Angle from the forward-right plane (usually horizontal-ish) of the beam. For SSS, 180-(2*(tilt+BeamBreath)) = nadir. For FLS, just the tilt downwards.")]        
        public float TiltAngleDeg = 15;
        [Tooltip("For FLS: FOV of the beams")]
        public float FLSFOVDeg = 30;



        [Header("SideScanSonar")]
        [Tooltip("There might be fewer pixels(buckets) than rays being cast.")]
        public int NumBucketsPerBeam = 1000;
        [Tooltip("Is this a normal SSS or an interferometric one?")]
        public bool isInterferometric = false;

        [Tooltip("Ideal = the bare fan (reflectivity x cos(incidence) x beam pattern, sorted " +
                 "into range), every noise and gain layer OFF, shadows empty by occlusion. " +
                 "Realistic = the full modelled stack. Defaults to Realistic here so that " +
                 "every OTHER sonar in the project is untouched; DeepVisionSSS sets it.")]
        public SonarFidelity Fidelity = SonarFidelity.Realistic;

        /// <summary>
        /// Single gate for ideal fidelity. Every realism layer asks THIS, not its own flag,
        /// so "Ideal" cannot be quietly half-true because some other component left
        /// UseReverbNoise or AlongTrackMultiLook on. A mode that can be bypassed by a
        /// neighbouring checkbox is not a mode -- it is a suggestion.
        /// </summary>
        public bool IsIdeal => Fidelity == SonarFidelity.Ideal;

        [Tooltip("Apply the receiver's time-varying gain, as the DeepVision board does before " +
                 "it outputs anything. Off = the pre-2026-08-29 linear ramp to zero at max " +
                 "range, which drove 91% of the outer swath to the noise floor.")]
        public bool UseTvg = true;

        [Tooltip("Range at which the TVG is unity, metres. Sets the absolute level of the " +
                 "record; the real 2024-12-10 record sits at ~26 bytes across the flat part " +
                 "of its swath.")]
        public float TvgReferenceRangeM = 20f;

        [Tooltip("TVG law: gain = (r/ref)^exponent. 1.0 cancels the h/r incidence falloff over " +
                 "a flat bottom and gives the flat profile the real record shows. Above 1.0 " +
                 "over-compensates and brightens the outer swath.")]
        public float TvgExponent = 1.0f;

        [Tooltip("Average each bin over the last N pings, N = beam footprint / ping spacing " +
                 "(the 0.5 deg horizontal beam re-illuminates the same ground on consecutive " +
                 "pings, which is most of why a real record looks smooth). Range-dependent: " +
                 "more looks far out, where the footprint is longer.")]
        public bool AlongTrackMultiLook = true;

        [Tooltip("Horizontal (along-track) beamwidth, degrees. DE680D datasheet: 0.5.")]
        public float HorizontalBeamwidthDeg = 0.5f;

        [Tooltip("Cap on the number of looks averaged. Bounds both cost and the along-track " +
                 "smearing of a real target.")]
        public int MaxLooks = 24;

        // -------------------------------------------------------------------------------
        // REALISM LAYER (1): THE BEAM PATTERN (2026-08-29).
        //
        // Until now the sonar had NO beam pattern at all: InitBeamProfileSimple fills the
        // profile with 1.0 for every ray, so a ray at the very edge of a 60 deg fan returned
        // as strongly as one on boresight. InitBeamProfileGaussian has existed in this file
        // the whole time and was never called.
        //
        // NOT gated by IsIdeal, deliberately. Every other realism layer adds a stochastic
        // process; this one is deterministic geometry — the transducer's own directivity —
        // so an "ideal" sonar with a real beam is a coherent thing to ask for, and keeping it
        // switchable is what lets its effect be measured alone (docs/2026-08-29-sss-realism-
        // layer-plan.md).
        //
        // WHY 0.5 AT THE EDGE, which looks like an off-by-one and is not:
        //   a stated beamwidth (DE680D: 60 deg vertical) is the ONE-WAY, -3 dB, POWER width
        //     one-way power at +-30 deg      = 0.5
        //     two-way (transmit AND receive) = 0.25
        //     amplitude                      = sqrt(0.25) = 0.5
        //   and Buckets holds an AMPLITUDE-like echo byte, not a power. InitBeamProfileGaussian
        //   computes exp(-x^2/2sigma^2) with sigma = width/(2*sqrt(2*ln2)), which is 0.500 at
        //   +-30 deg by construction. Correct as written. Do not "fix" it to 0.707.
        //
        // PREDICTED before flying, from model_ideal_fan.py: boresight sits 60 deg off nadir
        // (~12 m slant at 6 m altitude), so this BRIGHTENS mid-swath and dims both the nadir
        // edge and the far swath => profile_ratio gets WORSE (1.49 -> 1.70 modelled) and the
        // median drops toward the real record's 26. No accuracy invariant is at risk: the gain
        // varies smoothly with elevation, so a target and the seabed beside it are multiplied
        // by the same number and their CONTRAST is untouched.
        [Tooltip("Realism layer 1: apply the transducer's Gaussian directivity instead of a " +
                 "flat profile. Off = every ray in the fan returns as if it were on " +
                 "boresight, which is what shipped until 2026-08-29.")]
        public bool UseBeamPattern = false;

        // -------------------------------------------------------------------------------
        // REALISM LAYER (2): THE REVERBERATION FLOOR (2026-08-29).
        //
        // A real receiver never records silence: volume reverberation, surface-return
        // sidelobes and receiver noise put a fluctuating floor in every range bin, so an
        // acoustic shadow reads as darker-than-the-floor rather than as an exact zero.
        // Rayleigh amplitude, the standard envelope statistics for diffuse reverberation.
        //
        // WHAT THIS LAYER IS **NOT** FOR, established by modelling before it was written:
        // the plan said this layer would lift p1 from 0 to the real record's 7. That was
        // WRONG. A gamma distribution with the real record's OWN cv (0.269) and median (26)
        // already has p1 = 12.8 with no floor at all -- so the real p1 is the low tail of
        // MULTI-LOOKED BOTTOM SPECKLE, not a floor sitting in shadows. Sizing this floor to
        // hit p1 = 7 needs mean ~20, which drags the median to 35 and lifts the shadow to
        // -6.1 dB, FAILING the shadow invariant. p1 belongs to layers (3) speckle and
        // (6) multi-look; this layer's job is only to remove exact zeros, and a mean of 2
        // already removes 100% of them.
        //
        // The floor level is NOT measured: the real 2024-12-10 record contains no deep
        // shadow to reveal it. It is bounded ABOVE by the record's p1 = 7 (nothing darker
        // is ever seen) and otherwise unconstrained. 4 sits under that bound, removes every
        // exact zero, leaves the median at 26, and keeps the shadow at -9.0 dB.
        [Tooltip("Realism layer 2: Rayleigh reverberation floor in every range bin, added in " +
                 "POWER. Shadows become darker-than-floor instead of exact zeros. Level is " +
                 "bounded by the real record's p1=7, not measured — see the block comment.")]
        public bool ReverbFloorLayer = false;

        [Tooltip("Spread each ray's return across the range bins its FOOTPRINT covers " +
                 "(dr = r·dtheta·tan(incidence)), instead of dropping it in one bin. Off = the " +
                 "pre-2026-08-29 behaviour, which left 54.7% of mid-swath bins holding only " +
                 "the noise floor against 1.7% in the real record. Kept switchable so the " +
                 "difference can be measured rather than argued about.")]
        public bool FootprintSpreading = true;

        [Tooltip("Hard cap on the half-width of a ray's footprint, in bins. A finite cap is " +
                 "needed because the footprint diverges at exactly grazing incidence; it is " +
                 "NOT a shadow guard (shadows survive because the footprint comes from the " +
                 "ray's own geometry, never from the gap to its neighbour).")]
        public float MaxFootprintBins = 64f;

        // ---------------------------------------------------------------------------------
        // INTERFEROMETRY: A MEASUREMENT, NOT THE ANSWER (2026-08-28, Ivan: "mimic the real
        // sensor and its behaviour the best we can").
        //
        // What this used to emit: the elevation angle the ray was CAST at, encoded straight
        // into the message. That is ground truth wearing a sensor's name -- the same failure
        // the ForwardBottomProfiler was made to confess with `isGroundTruth`, and invariant
        // 11 one layer down. Anything developed against it (bathymetry, SSS-SLAM) met the
        // real instrument's error for the first time on the water.
        //
        // What a real interferometer does: two receive elements a baseline `d` apart see the
        // same echo with a path difference, so the phase difference is
        //     phi = 2*pi*(d/lambda)*sin(theta)
        // with theta the elevation angle off the array boresight. That phase is WRAPPED into
        // [-pi, pi] (the hardware cannot know how many cycles it lost) and it is NOISY, with
        // the noise set by the coherence gamma between the two elements:
        //     sigma_phi = sqrt((1 - gamma^2) / (2*N*gamma^2))          [Rodriguez & Martin]
        // and gamma degrades where the echo is weak.
        //
        // Defaults measured from the REAL 2024-12-10 ISSS record at Askö (4,000 pings,
        // 30 m range, 3 cm bins; `data-cube/docs/2026-08-28-sss-tuning-verification-plan.md`):
        //   * the phase is int16, +-32768 <-> +-pi
        //   * adjacent-bin coherence 0.79 in the weakest bins, 0.90 in the strongest
        //   * phase noise 0.57 rad weak, 0.46 rad strong  (which is what those gammas give)
        //   * the deterministic across-swath structure is only ~0.24 rad, i.e. THIS
        //     INSTRUMENT IS NOISE-DOMINATED, and a synthetic one that is not is not realistic
        //
        // HONESTY ABOUT PROVENANCE: those numbers come from the ISSS stick, which is NOT the
        // DeepVision unit on SAM -- it is the only interferometric record we hold. They are
        // therefore exposed as parameters with measured defaults, not baked in as constants.
        // ---------------------------------------------------------------------------------
        [Tooltip("Interferometer baseline in WAVELENGTHS (d/lambda). Sets how fast phase " +
                 "winds with elevation angle, and therefore how often it wraps. Not measured " +
                 "for any unit we own -- the real ISSS record never wraps, which only bounds " +
                 "it from above.")]
        public float InterferometricBaselineWavelengths = 1.5f;

        [Tooltip("Coherence between the two receive elements at a STRONG echo (0-1). " +
                 "Measured 0.90 on the real ISSS record.")]
        [Range(0.05f, 0.999f)] public float InterferometricCoherenceStrong = 0.90f;

        [Tooltip("Coherence at a WEAK echo. Measured 0.79 on the real ISSS record. Lower " +
                 "than the strong value is the whole point: phase must degrade in shadows.")]
        [Range(0.05f, 0.999f)] public float InterferometricCoherenceWeak = 0.79f;

        [Tooltip("Echo byte at or above which the strong-echo coherence applies; below the " +
                 "floor it is the weak value, and it interpolates between.")]
        public float InterferometricStrongEchoByte = 100f;

        
        [Header("SSS-Noise")]
        public float MultGain = 4;
        public bool UseAdditiveNoise = true;
        public float AddNoiseStd = 1;
        public float AddNoiseMean = 0;

        [Tooltip("SSS only: Rayleigh reverberation floor in every range bin (byte units). A " +
                 "real record is never black where nothing echoed — shadows are darker than " +
                 "the floor, not zero. Off by default; DeepVisionSSS turns it on.")]
        public bool UseReverbNoise = false;
        public float ReverbNoiseMean = 4f;
        readonly System.Random reverbRng = new System.Random(8271);

        NormalDistribution additiveNormal;

        //  The SideScan pixels
        [HideInInspector] public byte[] Buckets;
        // The interferometric output, named for what it carries: 16-bit signed PHASE, split
        // high/low, exactly as the real driver publishes it (port/starboard_channel_phase_15_8
        // and _7_0). It used to be a derived elevation ANGLE under different field names --
        // see the block comment at InterferometricBaselineWavelengths.
        [HideInInspector] public byte[] BucketsPhaseHigh;
        [HideInInspector] public byte[] BucketsPhaseLow;

        // Along-track multi-look state: a ring of the last MaxLooks pings' per-bin values,
        // plus where the sonar was when each was taken, so the number of looks can follow the
        // ACTUAL ping spacing rather than an assumed speed.
        float[] lookRing;          // MaxLooks * Buckets.Length
        float[] lookStepM;         // distance travelled before each stored ping
        int lookHead, lookCount;
        Vector3 lastPingPos;
        bool haveLastPingPos;
        int totalBuckets => NumBucketsPerBeam * NumBeams;


        // we use this one to keep the latest hits in memory and
        // accessible to outside easily.
        [HideInInspector] public SonarHit[] SonarHits;
        // Keeping track of lowest and highest hit heights
        // for visualization or other purposes
        [HideInInspector] public float HitsMinHeight = Mathf.Infinity;
        [HideInInspector] public float HitsMaxHeight = 0f;
        


        [Header("Load")]
        public float TimeShareInFixedUpdate;


        // Unity job structure for long-term casting of rays.
        private JobHandle handle;
        private NativeArray<RaycastHit> results;
        private NativeArray<RaycastCommand> commands;

        [HideInInspector] public int TotalRayCount => NumRaysPerBeam * NumBeams;
        [HideInInspector] public float DegreesPerRayInBeam => BeamBreadthDeg/(NumRaysPerBeam-1);
        [HideInInspector] public float DegreesPerBeamInFLS => FLSFOVDeg/(NumBeams-1);

        [HideInInspector] public List<float> BeamProfile;


        new protected void OnValidate()
        {
            base.OnValidate();
            if (Type == SonarType.SSS) NumBeams = 2;
            if (Type == SonarType.MBES)
            {
                NumBeams = 1;  
                TiltAngleDeg = -1;
            }
            if (NumRaysPerBeam <= 0) NumRaysPerBeam = 1;
        }

        new protected void Awake()
        {
            base.Awake();
            InitHits();
            RebuildBeamProfile();
            if (Type == SonarType.SSS) InitSidescanBuckets();
        }

        void InitSidescanBuckets()
        {
            // Each bucket has a 1 byte intensity value 0-255
            Buckets = new byte[totalBuckets];

            // followed by 2 bytes angle value 0-65535 [-pi,0]
            // angle is in radians, but we store it as a 16bit unsigned int
            // so we can have a resolution of pi/65535, the magic number is 20860
            BucketsPhaseHigh = new byte[totalBuckets];
            BucketsPhaseLow = new byte[totalBuckets];

            additiveNormal = new NormalDistribution(AddNoiseMean, AddNoiseStd);
        }

        void InitHits()
        {
            // Initialize all the hits as empty so we can just update them later
            // rather than spamming new ones
            SonarHits = new SonarHit[TotalRayCount];
            for(int i=0; i<TotalRayCount; i++)
            {
                SonarHits[i] = new SonarHit(this);
            }
        }

        /// <summary>
        /// Build the per-ray beam profile for the CURRENT setting. Public and re-callable so
        /// DeepVisionSSS can change the layer without a domain reload — the profile is sized
        /// by NumRaysPerBeam, which Apply() may also have just changed, so the two must be
        /// rebuilt together or the profile is indexed out of range on the next ping.
        /// </summary>
        public void RebuildBeamProfile()
        {
            if (UseBeamPattern) InitBeamProfileGaussian();
            else                InitBeamProfileSimple();
        }

        void InitBeamProfileGaussian()
        {
            // Initialize the Gaussian beam profile
            // The Gaussian is specified be its full width half max (fwhm), 
            float CalculateGaussianIntensity(float beamAngle, float beamCenter, float sigma)
            {
                var gaussianIntensity = Mathf.Exp(-(Mathf.Pow(beamAngle - beamCenter, 2) / (2 * Mathf.Pow(sigma, 2))));
                return gaussianIntensity;
            }
            var angleStepDeg = BeamBreadthDeg / (NumRaysPerBeam - 1.0f);
            var fwhmSigma = BeamBreadth3DecibelsDeg / (2 * Mathf.Sqrt(2.0f * Mathf.Log(2.0f)));
            BeamProfile = new List<float>();
            for(int i=0; i<NumRaysPerBeam; i++)
            {
                var beamAngleDeg = -BeamBreadthDeg / 2 + i * angleStepDeg;
                float intensity =
                    CalculateGaussianIntensity(beamAngle: beamAngleDeg, beamCenter: 0.0f, sigma: fwhmSigma);
                BeamProfile.Add(intensity);
            }
        }
        
        void InitBeamProfileSimple()
        {
            // Initialize simple beam profile
            BeamProfile = new List<float>();
            for(int i=0; i<NumRaysPerBeam; i++)
            {
                BeamProfile.Add(1.0f);
            }
        }

        public static (int, int) BeamNumRayNumFromRayIndex(int i, int NumRaysPerBeam)
        {
                var rayNum = i % NumRaysPerBeam;
                int beamNum = (int)i / (int)NumRaysPerBeam;
                return (beamNum, rayNum);
        }

        void UpdateSonarHits(NativeArray<RaycastHit> results)
        {
            // TODO this should be done in parallel too...?
            for(int i=0; i < TotalRayCount; i++)
            {
                var hit = results[i];
                var (beamNum, rayNum) = BeamNumRayNumFromRayIndex(i, NumRaysPerBeam);
                SonarHits[i].Update(hit, BeamProfile[rayNum]);
                if(hit.point.y > HitsMaxHeight && hit.point.y<0) HitsMaxHeight = hit.point.y;
                if(hit.point.y < HitsMinHeight) HitsMinHeight = hit.point.y;
            }
        }
            
            
        public override bool UpdateSensor(double deltaTime)
        {
            var t0 = Time.realtimeSinceStartup;
            if (results.Length > 0)
            {
                // Wait for the batch processing job to complete
                handle.Complete();

                // Update the sonarHit objects with the results of raycasts
                UpdateSonarHits(results);
                // if this is a sidescan, do some extra stuff
                if(Type == SonarType.SSS) UpdateSidescan();

                // Dispose the buffers
                results.Dispose();
                commands.Dispose();
            }


            results = new NativeArray<RaycastHit>(TotalRayCount, Allocator.Persistent);
            commands = new NativeArray<RaycastCommand>(TotalRayCount, Allocator.Persistent);

            var setupJob = new SetupSonarRaycastJob()
            {
                Commands = commands,
                NumRaysPerBeam = NumRaysPerBeam,
                SonarUp = transform.up,
                SonarForward = transform.forward,
                SonarRight = transform.right,
                SonarPosition = transform.position,
                Type = Type,
                DegreesPerRayInBeam = DegreesPerRayInBeam,
                BeamBreadthDeg = BeamBreadthDeg,
                MaxRange = MaxRange,
                TiltAngleDeg = TiltAngleDeg,
                FLSFOVDeg = FLSFOVDeg,
                DegreesPerBeamInFLS = DegreesPerBeamInFLS
            };

            JobHandle deps = setupJob.Schedule(commands.Length, 10, default(JobHandle));
            handle = RaycastCommand.ScheduleBatch(commands, results, 20, deps);

            var t1 = Time.realtimeSinceStartup;
            TimeShareInFixedUpdate = (t1-t0)/Time.fixedDeltaTime;
            if(TimeShareInFixedUpdate > 0.5f) Debug.LogWarning($"Sonar in {transform.parent.name}/{transform.name} took more than half the time in a fixedUpdate!");

            return true;
        }

        void UpdateSidescan()
        {
            if(Type != SonarType.SSS) return;
            
            // 0-out, since maybe not the same buckets will be written to.
            Array.Clear(Buckets, 0, Buckets.Length);
            Array.Clear(BucketsPhaseHigh, 0, BucketsPhaseHigh.Length);
            Array.Clear(BucketsPhaseLow, 0, BucketsPhaseLow.Length);

            int[] cnt = new int[Buckets.Length];
            float[] bucketsSum = new float[Buckets.Length];
            // 2026-08-28: ONE accumulator for the WHOLE 16-bit angle, not one per byte.
            // The previous version summed the high byte and the low byte into separate
            // accumulators and averaged each independently -- which is not the average of the
            // angle. The low byte wraps every 256 counts, so its independent mean is
            // arbitrary: two rays at 255 and 256 average to 127 instead of 255, and any
            // bucket whose rays straddle a 256 boundary is wrong by up to 128 counts
            // (~0.35 deg at the pi/65535 scaling below). Recombine first, then average.
            float[] bucketsAngleSum = new float[Buckets.Length];

            // First we gotta know what distance ranges each bucket needs to
            // have, we can ask the sonar object for its max distance;
            float minDistance = 0;
            float bucketSize = (MaxRange - minDistance) / NumBucketsPerBeam;
            var angleStepDeg = BeamBreadthDeg / (NumRaysPerBeam - 1.0f);

            for(int rayIndex = 0; rayIndex < TotalRayCount; rayIndex++)
            {
                // since buckets is a flat array...
                var (beamNum, rayNum) = Sonar.BeamNumRayNumFromRayIndex(rayIndex, NumRaysPerBeam);
                
                var sh = SonarHits[rayIndex];

                // A MISS IS NOT A HIT AT RANGE ZERO. RaycastHit.distance is 0 when the ray
                // hit nothing (too grazing to reach bottom inside MaxRange, or aimed into the
                // water column), and without this guard every such ray was binned into
                // bucket 0 with intensity 0 — diluting the nadir bins with echoes that never
                // happened. The correct record for a miss is silence in every bin.
                if(sh.Hit.collider == null) continue;

                // Ideal fidelity: the range of a hit is the range of the hit. Additive range
                // noise is realism layer material, and it is destructive here -- a 1 m sigma
                // on a 4 cm bin scatters a return 25 bins from where the geometry put it,
                // which blurs exactly the shadow edges this mode exists to show.
                double addNoise = 0;
                if(UseAdditiveNoise && !IsIdeal) addNoise = additiveNormal.Sample();

                // discritize the ray distance into a bucket
                float dis = (float)(sh.Hit.distance + addNoise);
                if(dis<0) dis=0;
                int bucketIndexInBeam = Mathf.FloorToInt((dis - minDistance)/bucketSize);
                if(bucketIndexInBeam >= NumBucketsPerBeam || bucketIndexInBeam < 0) continue;
                // bucketIndex is where in the specific bucket (usually port/strb 0/1)
                // this ray falls, but we have a flat array of buckets, so gotta place those
                // starboards further down the array
                int bucketIndex = bucketIndexInBeam + beamNum*NumBucketsPerBeam;

                // A RAY IS NOT A POINT: IT SUBTENDS A FOOTPRINT, AND THE FOOTPRINT SPANS
                // RANGE BINS (2026-08-29, measured).
                //
                // Binning each ray into the single bucket its centre lands in left **54.7% of
                // mid-swath bins carrying nothing but the reverberation floor** in a recorded
                // run, against **1.7%** in the real 2024-12-10 DeepVision record. That is not
                // "a bit sparse": it makes the record BIMODAL -- a floor population at ~3.5
                // bytes plus a signal population at ~38 -- which is the whole of the
                // synthetic record's CV 1.15 vs the real 0.28, and its skew +1.8 vs +0.05.
                // §3f0e raised rays-per-beam 512 -> one-per-bin and read the fill rise 6% ->
                // 33% as the fix; it was a third of the way. The display bridges short gaps,
                // so the picture LOOKED filled while the wire data was half empty.
                //
                // More rays cannot close it: exact coverage at grazing incidence needs ~26k
                // rays per beam (§3f0e). The physics closes it instead. A ray carries the
                // angular sector `angleStepDeg`; on a surface met at incidence `i` (from the
                // normal), that sector covers a range extent
                //         dr = r * dtheta * tan(i)
                // -- small near nadir, growing without bound towards grazing, which is
                // exactly where the gaps were. The ray's energy belongs across those bins.
                //
                // The footprint is computed from THIS RAY'S OWN geometry, never from the
                // distance to its neighbour. That distinction is what preserves acoustic
                // shadows: behind a target the next ray lands metres further out, and filling
                // "up to the neighbour" would paint the shadow bright. A ray only ever fills
                // the ground it actually illuminates.
                // The span is computed in METRES and both ends are FLOORED into bins, so
                // adjacent footprints tile without gaps. Rounding the two ends in opposite
                // directions (ceil the low, floor the high) leaves a one-bin hole wherever a
                // footprint is close to one bin wide -- measured: 85.8% mid-swath fill
                // instead of the ~98% the geometry actually gives.
                float halfM = 0.5f * bucketSize;
                if(FootprintSpreading)
                {
                    float incidenceDeg = Vector3.Angle(transform.position - sh.Hit.point, sh.Hit.normal);
                    // tan blows up at exactly 90 deg; clamp to a finite, still-large footprint
                    float tanInc = Mathf.Tan(Mathf.Min(incidenceDeg, 89.0f) * Mathf.Deg2Rad);
                    float footprintM = dis * angleStepDeg * Mathf.Deg2Rad * Mathf.Max(tanInc, 0f);
                    halfM = Mathf.Clamp(0.5f * footprintM,
                                        0.5f * bucketSize, MaxFootprintBins * bucketSize);
                }
                int loBin = Mathf.Max(0, Mathf.FloorToInt((dis - halfM - minDistance)/bucketSize));
                int hiBin = Mathf.Min(NumBucketsPerBeam - 1,
                                      Mathf.FloorToInt((dis + halfM - minDistance)/bucketSize));

                float beamAngleRad = 0f;
                if(isInterferometric)
                {
                    float beamAngleDeg;
                    if (beamNum==0) beamAngleDeg = -TiltAngleDeg - rayIndex * angleStepDeg;
                    else beamAngleDeg = -TiltAngleDeg - (TotalRayCount - rayIndex) * angleStepDeg;
                    beamAngleRad = beamAngleDeg * Mathf.Deg2Rad;
                }

                for(int bi = loBin; bi <= hiBin; bi++)
                {
                    int bIdx = bi + beamNum*NumBucketsPerBeam;
                    // intensities are stored as floats in [0,1], but we want bytes in 0-255
                    bucketsSum[bIdx] += (sh.ReturnIntensity * 255 * MultGain);

                    // Accumulate the true elevation ANGLE (radians) of the contributing rays.
                    // The conversion to a measured, wrapped, noisy PHASE happens once per
                    // bucket after the average -- a real interferometer forms one phase
                    // difference per range gate from the summed echo, not one per ray.
                    if(isInterferometric) bucketsAngleSum[bIdx] += beamAngleRad;

                    // count how many rays fell into this bucket
                    cnt[bIdx]++;
                }
            }

            // finally, we can average the rays
            for(int bucketIndex = 0; bucketIndex < Buckets.Length; bucketIndex++)
            {
                // CLAMP, do not let the cast wrap. intensity is [0,1] so the product is
                // 255*MultGain: with the shipped MultGain=1 it can never exceed 255, but any
                // gain above 1 would push the STRONGEST echoes past 255 and a float->byte
                // cast wraps them to near-black. That failure mode is invisible in a
                // waterfall (a bright target simply goes dark) and would land on whoever
                // next raised the gain to make targets brighter.
                float avg = cnt[bucketIndex] > 0 ? bucketsSum[bucketIndex]/cnt[bucketIndex] : 0f;

                // Reverberation / electronics noise floor. A real transducer never records
                // silence: volume reverberation, surface return sidelobes and receiver noise
                // put a fluctuating floor in EVERY range bin, and acoustic shadows read as
                // darker-than-the-floor, not as zero. Rayleigh-distributed amplitude — the
                // standard envelope statistics for diffuse reverberation.
                // Ideal fidelity: no floor. A bin no ray reached must read ZERO, because
                // "empty by occlusion" is the whole of step 2 -- a floor would make a real
                // shadow and an unsampled bin look identical again.
                if(ReverbFloorLayer && ReverbNoiseMean > 0f)
                {
                    // REALISM LAYER (2), added on top of the Ideal fan 2026-08-29.
                    //
                    // ADDS IN POWER, NOT AMPLITUDE. Reverberation is incoherent with the
                    // bottom echo, so the two combine as powers:  A = sqrt(As^2 + Af^2).
                    // The legacy path below adds amplitudes, which is wrong twice over:
                    // it brightens every already-bright bin by the full floor (modelled on
                    // the flown record, a mean-4 floor moved the median 26 -> 30 and pushed
                    // the whole record off the real record's median), and it makes the floor
                    // interfere with the signal rather than sit under it. Power addition
                    // measured 26 -> 26: bright bins are untouched, dark bins fill to the
                    // floor, which is what a floor IS.
                    double u = 1.0 - reverbRng.NextDouble();  // (0,1]
                    double sigma = ReverbNoiseMean / 1.2533;  // mean = sigma*sqrt(pi/2)
                    double f = sigma * System.Math.Sqrt(-2.0 * System.Math.Log(u));
                    avg = (float)System.Math.Sqrt((double)avg * avg + f * f);
                }
                else if(UseReverbNoise && ReverbNoiseMean > 0f && !IsIdeal)
                {
                    // Legacy Realistic-stack floor, kept bit-identical on purpose so the
                    // old stack remains a valid A/B reference. It is superseded by the
                    // layer above and should retire with the rest of Realistic.
                    double u = 1.0 - reverbRng.NextDouble();  // (0,1]
                    double sigma = ReverbNoiseMean / 1.2533;  // mean = sigma*sqrt(pi/2)
                    avg += (float)(sigma * System.Math.Sqrt(-2.0 * System.Math.Log(u)));
                }
                Buckets[bucketIndex] = (byte) Mathf.Clamp(avg, 0f, 255f);

                if(isInterferometric && cnt[bucketIndex] > 0)
                {
                    // TRUE ANGLE -> MEASURED PHASE. Four steps, each one a thing the real
                    // instrument does and the old code did not (see the block comment at
                    // InterferometricBaselineWavelengths).
                    float theta = bucketsAngleSum[bucketIndex]/cnt[bucketIndex];   // radians

                    // 1. the interferometer's response: phase winds with sin(elevation)
                    double phi = 2.0*System.Math.PI*InterferometricBaselineWavelengths
                                 * System.Math.Sin(theta);

                    // 2+3. coherence and its phase noise -- REALISM ONLY. In Ideal fidelity
                    // the published phase is the clean geometric value: still WRAPPED (step 4
                    // below), because wrapping is what the hardware can and cannot know, not
                    // a noise process, and a consumer that assumes an unwrapped angle must
                    // still fail here rather than at sea.
                    if(!IsIdeal)
                    {
                        // coherence falls with echo strength, so the phase degrades in
                        // shadows and at long range exactly where a real one does. Buckets[]
                        // is already written above, so this reads the echo THIS bin reports.
                        float echo = Buckets[bucketIndex];
                        float t = Mathf.Clamp01(echo / Mathf.Max(InterferometricStrongEchoByte, 1f));
                        float gamma = Mathf.Clamp(
                            Mathf.Lerp(InterferometricCoherenceWeak,
                                       InterferometricCoherenceStrong, t), 0.05f, 0.999f);

                        // phase noise from that coherence (Rodriguez & Martin, one look):
                        //    sigma = sqrt((1 - gamma^2) / (2 * gamma^2))
                        double sigma = System.Math.Sqrt((1.0 - gamma*gamma)/(2.0*gamma*gamma));
                        double u = 1.0 - reverbRng.NextDouble();      // (0,1], never log(0)
                        double v = reverbRng.NextDouble();
                        double gauss = System.Math.Sqrt(-2.0*System.Math.Log(u))
                                       * System.Math.Cos(2.0*System.Math.PI*v);
                        phi += sigma * gauss;
                    }

                    // 4. WRAP into [-pi, pi]. The hardware cannot know how many cycles it
                    //    lost, and a consumer that assumes an unwrapped angle must fail here
                    //    in the simulator too, not for the first time at sea.
                    phi = phi - 2.0*System.Math.PI*System.Math.Floor((phi + System.Math.PI)
                                                                     /(2.0*System.Math.PI));

                    // int16, +-32768 <-> +-pi, exactly the real record's encoding.
                    int q = (int) System.Math.Round(phi/System.Math.PI*32768.0);
                    q = (int) Mathf.Clamp(q, -32768f, 32767f);
                    ushort raw = (ushort)(short) q;
                    BucketsPhaseHigh[bucketIndex] = (byte)((raw >> 8) & 0xff);
                    BucketsPhaseLow[bucketIndex]  = (byte)(raw & 0xff);
                }
            }

            ApplyAlongTrackMultiLook(bucketSize);

        }

        /// <summary>
        /// The 0.5 deg HORIZONTAL beam, modelled by its effect rather than by casting it.
        ///
        /// A real DE680D's beam is 0.5 deg wide along track, so at range r it illuminates a
        /// patch r*0.00873 long -- 9 cm at 10 m, 17 cm at 20 m, 35 cm at 40 m. At 0.5 m/s and
        /// 18.75 Hz the pings are 2.7 cm apart, so the SAME ground is re-illuminated by 3
        /// pings at 10 m and 13 at 40 m, and the receiver sums them. That inherent multi-look
        /// is most of why a real record looks smooth: single-look Rayleigh has CV 0.523, the
        /// real record measures 0.280, which is ~3.5 effective looks.
        ///
        /// Our sonar casts a VERTICAL fan only -- no horizontal beamwidth at all -- so every
        /// ping saw independent ground and got none of that averaging: measured CV 0.580
        /// against the real 0.280, a factor 2.07 that is almost exactly the sqrt(looks) the
        /// beam would supply.
        ///
        /// Ivan's call (2026-08-29) between modelling this by casting azimuth samples (3-5x
        /// the raycasts) or by its equivalent effect: take the effect. For a static scene an
        /// along-track average over the pings that share the footprint is what the beam DOES,
        /// it costs one buffer, and it blurs a target along track by exactly the amount the
        /// real beam blurs it -- which is a fidelity gain, not a loss.
        ///
        /// The look count is RANGE-DEPENDENT (more looks far out, where the footprint is
        /// longer) and derived from the sonar's own measured travel between pings, never from
        /// an assumed speed -- a stationary vehicle would otherwise average forever.
        /// </summary>
        void ApplyAlongTrackMultiLook(float bucketSize)
        {
            // Ideal fidelity: one ping is one ping. Multi-look is a realism layer (it models
            // the 0.5 deg horizontal beam re-illuminating ground on consecutive pings), and
            // it smears a target along track by design -- which is right for realism and
            // wrong for a reference image.
            if(IsIdeal) return;
            if(!AlongTrackMultiLook || Buckets == null || Buckets.Length == 0) return;

            int nb = Buckets.Length;
            int cap = Mathf.Max(1, MaxLooks);
            if(lookRing == null || lookRing.Length != cap*nb)
            {
                lookRing = new float[cap*nb];
                lookStepM = new float[cap];
                lookHead = 0; lookCount = 0; haveLastPingPos = false;
            }

            Vector3 pos = transform.position;
            float step = haveLastPingPos ? Vector3.Distance(pos, lastPingPos) : 0f;
            lastPingPos = pos; haveLastPingPos = true;

            // store this ping
            lookHead = (lookHead + 1) % cap;
            lookStepM[lookHead] = step;
            for(int i = 0; i < nb; i++) lookRing[lookHead*nb + i] = Buckets[i];
            if(lookCount < cap) lookCount++;

            // A stationary sonar has no along-track extent to average over: averaging then
            // would just integrate the same patch forever and erase the speckle a real
            // instrument still shows. Below a threshold, leave the record alone.
            if(step < 1e-3f) return;

            float bwRad = HorizontalBeamwidthDeg * Mathf.Deg2Rad;
            int perBeam = NumBucketsPerBeam;
            for(int i = 0; i < nb; i++)
            {
                int binInBeam = i % perBeam;
                float r = (binInBeam + 0.5f) * bucketSize;
                int looks = Mathf.Clamp(Mathf.RoundToInt(r * bwRad / Mathf.Max(step, 1e-4f)),
                                        1, Mathf.Min(cap, lookCount));
                if(looks <= 1) continue;
                float sum = 0f;
                for(int k = 0; k < looks; k++)
                {
                    int idx = ((lookHead - k) % cap + cap) % cap;
                    sum += lookRing[idx*nb + i];
                }
                Buckets[i] = (byte)Mathf.Clamp(sum/looks, 0f, 255f);
            }
        }

        [BurstCompile]
        struct SetupSonarRaycastJob : IJobParallelFor
        {
            public NativeArray<RaycastCommand> Commands;
            public int NumRaysPerBeam;
            public Vector3 SonarUp;
            public Vector3 SonarForward;
            public Vector3 SonarRight;
            public Vector3 SonarPosition;
            public SonarType Type;
            public float DegreesPerRayInBeam;
            public float BeamBreadthDeg;
            public float MaxRange;
            public float TiltAngleDeg;
            public float FLSFOVDeg;
            public float DegreesPerBeamInFLS;

            public void Execute(int i)
            {
                var direction = -SonarUp; // default down?
                var (beamNum, rayNum) = Sonar.BeamNumRayNumFromRayIndex(i, NumRaysPerBeam);
                if(Type == SonarType.MBES)
                {
                    // MBES is just one beam looking down directly. Simplest.
                    // start a beam looking directly down
                    direction = -SonarUp;
                    // we want 0 degrees in the center and then +-Breadth/2 on the sides.
                    var rayAngle = (rayNum * DegreesPerRayInBeam) - BeamBreadthDeg/2;
                    // rotate it around the forward axis by its ray number in the beam
                    // offset half-way so the middle is directly down.
                    direction = Quaternion.AngleAxis(rayAngle, SonarForward) * direction;
                }
                if(Type == SonarType.FLS)
                {
                    // FLS is MBES, but the first ray is not in the center, its at the edge and is tilted.
                    // there are also >1 beams.

                    // we want 0 degrees at the edge and BeamBreadt degrees at the other edge of the beam.
                    var rayAngle = rayNum * DegreesPerRayInBeam;
                    // FLS beams are defined as vertical fans, sweeping side-to-side
                    // so we start a ray forward first.
                    direction = SonarForward;
                    // then we rotate _that_ to the ray angle within the beam, around the side-axis
                    // plus the tilt angle which is measured from the horizontal plane down
                    direction = Quaternion.AngleAxis(rayAngle+TiltAngleDeg, SonarRight) * direction;
                    // then we rotate it to the beam angle, around the UP axis
                    var beamAngle = (beamNum * DegreesPerBeamInFLS) - FLSFOVDeg/2;
                    direction = Quaternion.AngleAxis(beamAngle, SonarUp) * direction;
                }
                if(Type == SonarType.SSS)
                {
                    // SSS usually has 2 beams, port and starboard
                    // their position is measured from the horizontal axis towards the vertical axis
                    // called the tilt angle, so we need to further rotate rays accordingly

                    // start the ray looking down
                    direction = -SonarUp;
                    // spread the beam with 0 degeres in the middle and +/- half-breadth around it
                    var rayAngle = rayNum * DegreesPerRayInBeam - BeamBreadthDeg/2;
                    var side = (beamNum * 2)-1; // port or starboard, -1, +1
                    // tilt it
                    rayAngle += side*(90 - TiltAngleDeg - BeamBreadthDeg/2);
                    direction = Quaternion.AngleAxis(rayAngle, SonarForward) * direction;
                }
                
                // and finally, cast dem rays boi.
                // A TRIGGER IS NOT A SONAR TARGET. QueryParameters.Default leaves hitTriggers at
                // UseGlobal, and Physics.queriesHitTriggers defaults to TRUE — so every trigger
                // volume in the scene returns an echo. Found 2026-08-16 at Kristineberg: a
                // 600 x 60 x 500 m current-field trigger produced returns across the whole
                // algae farm at the 0.5 default reflectivity and material label 0, i.e. a wall
                // of "unidentified" bottom where the farm should be. Triggers are by definition
                // non-physical volumes (force fields, mission zones, water bodies), so ignoring
                // them is strictly correct for an acoustic sensor.
                Commands[i] = new RaycastCommand(
                    SonarPosition, direction,
                    new QueryParameters(layerMask: ~0,
                                        hitMultipleFaces: false,
                                        hitTriggers: QueryTriggerInteraction.Ignore,
                                        hitBackfaces: false),
                    MaxRange);
            }
        }
    }


}
