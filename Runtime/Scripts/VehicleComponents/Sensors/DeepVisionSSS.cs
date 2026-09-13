using System.Reflection;
using UnityEngine;
using ROS.Core;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// Deep Vision side scan sonar (DE340/DE680 family, as on SAM 2.2): one sonar unit
    /// with TWO transducers (port + starboard), switchable between two operating
    /// frequencies. The underlying Sonar runs in SSS mode with two beams, and SSS_Pub
    /// publishes separate port_channel / starboard_channel arrays — i.e. left/right
    /// data ready for waterfall images. With Interferometric on, per-bin angle bytes
    /// are also published, so point clouds can be built downstream.
    ///
    ///   LF 340 kHz: up to 200 m/side, 20 cm bins — wide-area search
    ///   HF 680 kHz: up to 100 m/side,  4 cm bins — high-res imaging
    ///
    /// Range is the operator's `RangeM` setting (SAM's documented default: 40 m on HF680);
    /// the bin count and the ping rate are DERIVED from it, so they cannot disagree with it.
    /// Datasheet (DE680D): horizontal beamwidth 0.5°, vertical 60°, range resolution 1 cm.
    ///
    /// Mode/interferometric take effect when entering Play (the Sonar sizes its bucket
    /// arrays in Awake); switching mid-run needs a Stop/Play, same as re-configuring
    /// the real unit's recording session.
    /// </summary>
    [DefaultExecutionOrder(-100)] // apply the preset before Sonar.Awake sizes its arrays
    [RequireComponent(typeof(Sonar))]
    [AddComponentMenu("Smarc/Sensor/DeepVisionSSS")]
    public class DeepVisionSSS : MonoBehaviour
    {
        public enum FrequencyMode { LF340, HF680 }

        [Tooltip("Operating frequency (both transducers switch together). LF340: 20 cm bins, wide-area. HF680: 4 cm bins, high-res. Applied at Play start.")]
        public FrequencyMode Mode = FrequencyMode.HF680;

        [Tooltip("Range per side, metres — the real unit's `range:=` setting. SAM's documented " +
                 "default is 40 m on HF680, which is what the 2024-12-10 Askö record was taken " +
                 "at (.dvs header: 0.03996 m x 1000 bins). Bin count and ping rate follow from " +
                 "this: bins = range / bin size, ping rate = 1500 / (2 x range).")]
        public float RangeM = 40f;

        /// <summary>Datasheet maximum for the selected mode (DE680D: 100 m/side; LF340: 200 m).</summary>
        public float MaxRangeForMode => Mode == FrequencyMode.LF340 ? 200f : 100f;

        [Tooltip("Interferometric mode: publish per-bin PHASE alongside the port/starboard echo channels. Off = plain waterfall echoes only.")]
        public bool Interferometric = true;

        // ---------------------------------------------------------------------------------
        // FIDELITY (2026-08-29). Ideal is the DEFAULT here, deliberately: after three
        // sessions the realism stack's interactions, not its individual layers, had become
        // the dominant unknown, so the burden of proof moved. Each realism layer now has to
        // re-earn its place by being switched on ALONE and measured against
        // docs/asko/sss_real_targets_2024-12-10.json, in this order:
        //   1 beam pattern -> 2 reverb floor (real p1 ~7, median 26) -> 3 speckle (CV 0.269,
        //   1-ping decorrelation) -> 4 persistent texture (CV 0.076) -> 5 board TVG ->
        //   6 multi-look -> 7 phase noise.
        // Realistic keeps the whole stack exactly as it was, so nothing measured is lost and
        // the A/B is one dropdown.
        // ---------------------------------------------------------------------------------
        [Tooltip("Ideal = the bare fan: reflectivity x cos(incidence) x beam pattern, sorted " +
                 "into range bins, shadows empty by occlusion. No floor, no speckle, no TVG, " +
                 "no multi-look, no phase noise. Realistic = the full modelled stack.")]
        public SonarFidelity Fidelity = SonarFidelity.Ideal;

        [Tooltip("Realism layer 1: the transducer's Gaussian directivity (-3 dB one-way power " +
                 "at the stated 60 deg vertical width). Off = a flat profile, i.e. no beam " +
                 "pattern at all, which is what shipped until 2026-08-29.")]
        public bool BeamPattern = false;

        [Tooltip("Realism layer 2: Rayleigh reverberation floor, added in POWER so bright bins " +
                 "are untouched and dark bins fill to the floor. Removes exact zeros; it is NOT " +
                 "what lifts p1 to the real record's 7 — that is layers 3 and 6.")]
        public bool ReverbFloor = false;

        [Tooltip("Floor level in echo bytes. Bounded above by the real record's p1 = 7 (nothing " +
                 "darker is ever seen there); not otherwise measured, because the real record " +
                 "holds no deep shadow to reveal it.")]
        public float ReverbFloorMean = 4f;

        [Tooltip("Ideal-mode multiplicative gain. MUST stay at 1: modelled, at the shipped 4 " +
                 "every bin inside ~15 m clamps to 255 and the near swath goes flat white. " +
                 "At 1 the brightest target peak is ~215 bytes, just under the ceiling.")]
        public float IdealMultGain = 1f;

        [Tooltip("Realistic-mode multiplicative gain (the value the stack was tuned at).")]
        public float RealisticMultGain = 4f;

        // 2026-08-28: what the interferometric channels carry changed, and it matters more
        // than a rename. They used to hold the elevation angle the RAY WAS CAST AT -- perfect,
        // unwrapped, noiseless, and under field names (`..._angle_high/low`) no real driver
        // uses. They now hold a modelled PHASE, wrapped and noisy, in the real driver's fields
        // (`..._phase_15_8` / `_7_0`). Defaults below are measured from the 2024-12-10 ISSS
        // record; see the block comment in Sonar.cs. THE ISSS IS NOT THIS UNIT -- it is the
        // only interferometric record we hold, so these are a starting point with a provenance,
        // not a DeepVision datasheet.
        [Tooltip("Interferometer baseline in wavelengths (d/lambda). NOT measured for the DeepVision unit; the real ISSS record never wraps, which only bounds it from above.")]
        public float BaselineWavelengths = 1.5f;

        [Tooltip("Element coherence at a strong echo. 0.90 measured on the real ISSS record.")]
        public float CoherenceStrong = 0.90f;

        [Tooltip("Element coherence at a weak echo. 0.79 measured on the real ISSS record — phase MUST degrade in shadows.")]
        public float CoherenceWeak = 0.79f;

        // THE MOUNT GEOMETRY IS OWNED HERE, because leaving it un-owned has now cost three
        // sessions (2026-08-16 twice, 2026-08-17). SAMSensorsV2.prefab is GENERATED by
        // SamV2PerceptionBuilder from SAMSensors.prefab, whose side scan is the v1 mount
        // (tilt 45 / breadth 45 -> off-nadir 0..45 deg). This component set every other
        // DeepVision parameter and left these two to be inherited, so every press of
        // "SMARC/Build SAM v2 Perception Prefabs" silently overwrote the committed
        // tilt 0 / breadth 60 with the v1 values -- which was blamed on "Unity
        // re-serializing the prefab" when it was in fact our own generator, deterministically.
        // Values below ARE the committed ones (git show HEAD:...SAMSensorsV2.prefab), so this
        // changes no number; it only makes the number survive a rebuild.
        [Tooltip("Downward tilt of the transducer mount, deg. With BeamBreadthDeg it fixes the off-nadir coverage [90-tilt-breadth, 90-tilt]; a side scan sees nothing at nadir.")]
        public float TiltAngleDeg = 0f;

        [Tooltip("Vertical opening angle of the beam, deg. 60 is the DE680D figure recorded in sss_geometry.SIM_BEAM_AS_SHIPPED -- NOT read off a DeepVision datasheet by anyone in this repo.")]
        public float BeamBreadthDeg = 60f;

        void OnValidate() { Apply(); }
        void Awake() { Apply(); }

        public void Apply()
        {
            var sonar = GetComponent<Sonar>();
            if (sonar == null) return;

            // RAYS PER BEAM IS A SAMPLING REQUIREMENT, NOT A QUALITY DIAL.
            // Rays are cast uniformly in ANGLE; UpdateSidescan bins them uniformly in RANGE.
            // Over a flat bottom at altitude h, r = h/cos(theta), so dr/dtheta = r*tan(theta):
            // the range spacing between adjacent rays grows without bound towards grazing
            // incidence. With the original 512 rays feeding 2000 bins, no more than a QUARTER
            // of the record could be sampled at all, and the sampled quarter piled up near
            // nadir — leaving the far swath as wide stripes of untouched, zero-valued bins.
            // Those stripes render as "no echo" when they actually mean "no ray", which is
            // what made the waterfall look wrong: black gaps that moved with the noise, and a
            // target that vanished whenever no ray happened to land on it.
            //
            // Exact bin coverage at grazing incidence is unreachable by raycasting (100 m at
            // 8 m altitude needs ~26000 rays for 5 cm bins), so this sets 1 ray per bin —
            // full coverage over the near and middle swath, where this vehicle actually
            // works — and the waterfall closes the residual far-field gaps by holding the
            // nearest sampled bin across SHORT runs only, so real shadows stay black.
            // 2026-08-28: RANGE IS AN OPERATOR SETTING, AND THE PING RATE FOLLOWS FROM IT.
            //
            // What was here: HF680 hardwired to 100 m / 2000 bins / 7 Hz. The real unit on SAM
            // is run at its documented default of **40 m** (`SAM_VEHICLE_SPECS.md` §5: "high
            // frequency (680 kHz) and 40 m range, switchable in the bringup (`range:=`)"), and
            // the 2024-12-10 record confirms it three ways: the .dvs header says
            // 0.03996 m x 1000 bins = 39.96 m, and the logged ping rate is 18.54 Hz.
            //
            // The 2.5x range error was not cosmetic. It made every bin 5 cm instead of 4,
            // put the swath 2.5x wider so a 3 m target occupied 1.4% of the picture instead
            // of 3.4%, and -- because the old code also FIXED the ping rate -- sampled the
            // along-track direction 2.6x more coarsely than the real instrument does.
            //
            // The ping rate is now DERIVED: a sonar cannot ping again until the previous
            // ping's outermost echo has come back, so f = c / (2R). At 40 m that is 18.75 Hz
            // against the record's measured 18.54; at 100 m it gives 7.5 Hz, which is where
            // the old hardcoded 7 came from. One number, physically, instead of two that can
            // disagree. SETTLED §3f0h.
            const float SoundSpeed = 1500f;
            float binSize;
            if (Mode == FrequencyMode.LF340)
            {
                binSize = 0.20f;                       // 20 cm bins, wide-area search
            }
            else
            {
                binSize = 0.04f;                       // 4 cm bins, as the real record carries
            }
            float range = Mathf.Clamp(RangeM, 5f, MaxRangeForMode);
            int bins = Mathf.Max(1, Mathf.RoundToInt(range / binSize));
            sonar.MaxRange = range;
            sonar.NumBucketsPerBeam = bins;
            sonar.NumRaysPerBeam = bins;               // 1 ray per bin (SETTLED §3f0e)
            float pingHz = SoundSpeed / (2f * range);
            sonar.isInterferometric = Interferometric;
            sonar.InterferometricBaselineWavelengths = BaselineWavelengths;
            sonar.InterferometricCoherenceStrong = CoherenceStrong;
            sonar.InterferometricCoherenceWeak = CoherenceWeak;
            sonar.frequency = pingHz;

            // Fidelity, and the gain that goes WITH it. Both branches are written
            // explicitly: leaving MultGain to whatever the prefab happened to serialize is
            // how a mode silently half-applies, and Ideal at gain 4 is not a dimmer Ideal,
            // it is a clamped white one.
            sonar.Fidelity = Fidelity;
            sonar.MultGain = Fidelity == SonarFidelity.Ideal ? IdealMultGain : RealisticMultGain;

            // Realism layer 1. Rebuilt HERE rather than left to Sonar.Awake because Apply()
            // has just set NumRaysPerBeam, and the profile is sized by it — build the two
            // together or the next ping indexes the profile out of range.
            sonar.UseBeamPattern = BeamPattern;
            sonar.RebuildBeamProfile();

            // Realism layer 2.
            sonar.ReverbFloorLayer = ReverbFloor;
            sonar.ReverbNoiseMean = ReverbFloorMean;

            // A real receiver records a reverberation floor in every bin, never silence;
            // shadows read as darker-than-floor. See Sonar.UseReverbNoise. Sonar.IsIdeal
            // gates this at the point of use as well -- belt and braces, because the flag
            // being true while the mode is Ideal must not produce a floor.
            sonar.UseReverbNoise = true;
            sonar.TiltAngleDeg = TiltAngleDeg;
            sonar.BeamBreadthDeg = BeamBreadthDeg;

            // Match the SSS publisher rate to the ping rate. Reflection because the
            // publisher classes are internal; ROSPublisher<T>.frequency is public.
            foreach (var mb in GetComponents<MonoBehaviour>())
            {
                if (mb == null || mb == this || !(mb is ROSBehaviour rosb)) continue;
                if (!rosb.topic.Contains("sidescan")) continue;
                var f = mb.GetType().GetField("frequency", BindingFlags.Public | BindingFlags.Instance);
                if (f != null && f.FieldType == typeof(float)) f.SetValue(mb, pingHz);
            }
        }
    }
}
