using UnityEngine;

namespace VehicleComponents.Comms
{
    /// <summary>
    /// The underwater acoustic link — SAM's **Succorfish Delphis** transceiver, fitted in the
    /// nosecone with a PingMarine diver-link connector (fleet.yaml: confidence CONFIRMED via
    /// mrlwiki.se, 2026-07-01).
    ///
    /// Confirmed as **Delphis V3**, based on Newcastle University's Nanomodem (NM3), interfaced
    /// over RS-232 via an FTDI USB-RS232 PCBA. ROS drivers: LABUST `labust_nanomodem` (low-level
    /// only — it does NOT carry arbitrary ROS messages) and `smarc_acomms`. The modem can measure
    /// its own channel (`$N` noise, `$S` spectrum, `$CM` impulse response, `$P` TWTT ping), which
    /// is how the operational range in local conditions should eventually be MEASURED rather than
    /// assumed. Full detail: `data-cube/docs/SAM_VEHICLE_SPECS.md` §2.1.
    ///
    /// DATASHEET FIGURES (Succorfish DELPHIS data sheet, succorfish.com — vendor-published):
    ///   range        up to 2 km in sea water (3.5 km fresh water)
    ///   data rate    463 bit/s maximum
    ///   carrier      24–32 kHz band
    ///   source level 168 dB re 1 µPa @ 1 m
    ///   modulation   spread spectrum (orthogonal signalling), PSK, with error correction
    ///   addressing   up to 256 uniquely addressable units
    ///   power        3–6.5 V DC; 2.5 mA listening, 5 mA receiving, 300 mA peak transmitting
    ///
    /// THE OPERATIONAL FIGURE IS NOT THE DATASHEET FIGURE, and this component keeps them apart
    /// deliberately. The 2 km is a best-case number. In Swedish archipelago conditions — shallow,
    /// hard rocky bottom, strong summer thermoclines, heavy multipath between islands — the PI's
    /// working figure is **500–1000 m, occasionally to 2 km** (Ivan, 2026-08-15: "let's keep it
    /// conservative here"). `operationalRangeM` defaults to the conservative middle of that; the
    /// datasheet maximum is recorded beside it as `specMaxRangeSeawaterM` so nobody has to
    /// rediscover it, and so raising the default is a visible decision rather than a guess.
    ///
    /// LATENCY IS THE POINT, NOT BANDWIDTH. At ~1500 m/s, a kilometre costs 0.67 s each way.
    /// That is why submerged interaction is fire-and-forget state sync and never a request the
    /// operator waits on (SYSTEMS_SPEC §3), and why only the Priority-1 report crosses this link.
    /// At 463 bit/s, a single 1 kB JSON status payload would take ~17 seconds to transmit — which
    /// is the arithmetic behind "the full mission never crosses the acoustic link" (§4).
    ///
    /// The transducer works wet. A surfaced vehicle whose transducer is still in the water keeps
    /// an acoustic link, which is correct and occasionally useful — it is the fallback when the
    /// hull is up but out of WiFi range and outside cellular coverage.
    ///
    /// WHERE THE RANGE IS MEASURED FROM IS PART OF THE MEASUREMENT (2026-08-21). `rangeM` is the
    /// distance between this component's transform and the peer's, so a wrong transducer position
    /// is a standing bias on every range this modem ever reports. That matters beyond realism:
    /// Ivan's falsifier for the fleet position estimate (2026-08-18) is *measured acoustic range
    /// versus the range implied by the two reported positions*, and a constant offset would give
    /// that cross-check a permanent, plausible-looking error in the one place built to catch
    /// position errors. The station's transducer is positioned by `DeployedTransducer`, from the
    /// water plane, for exactly this reason.
    /// </summary>
    [AddComponentMenu("Smarc/Comms/Acoustic Modem (Succorfish Delphis)")]
    public class AcousticModem : CommsModem
    {
        [Header("Succorfish Delphis — operational")]
        [Tooltip("Working range in Swedish archipelago conditions [m]. PI-stated 2026-08-15: realistically 500-1000 m, up to 2000 in good conditions. Conservative by default and deliberately NOT the datasheet number.")]
        public float operationalRangeM = 800f;

        [Header("Succorfish Delphis — datasheet (vendor-published, do not treat as operational)")]
        [Tooltip("Datasheet maximum in SEA water [m]. Reference only; operationalRangeM is what this component uses.")]
        public float specMaxRangeSeawaterM = 2000f;
        [Tooltip("Datasheet maximum in FRESH water [m]. Reference only.")]
        public float specMaxRangeFreshwaterM = 3500f;
        [Tooltip("Maximum payload rate [bit/s]. At this rate a 1 kB status payload takes ~17 s, which is why only the Priority-1 report crosses this link.")]
        public float specBitsPerSecond = 463f;
        [Tooltip("Carrier band low edge [kHz], datasheet.")]
        public float specCarrierLowKHz = 24f;
        [Tooltip("Carrier band high edge [kHz], datasheet.")]
        public float specCarrierHighKHz = 32f;
        [Tooltip("Source level [dB re 1 uPa @ 1 m], datasheet.")]
        public float specSourceLevelDb = 168f;

        [Header("Channel")]
        [Tooltip("Speed of sound [m/s]. 1500 is the usual working value; the Star-Oddi CTD is the real source of this on the hull.")]
        public float soundVelocity = 1500f;
        [Tooltip("Require the transducer to be in the water. A Delphis in air transmits nothing useful. Leave this ON for the base station too: its transducer hangs off the side on a DeployedTransducer and is genuinely wet, so the test is real rather than an obstacle to work around.")]
        public bool requireWet = true;

        public override string LinkName => "acoustic";
        // 0.5 Hz: the Priority-1 report cadence. Not a bandwidth claim — a statement about how
        // often a consumer should expect to hear anything, which is what staleness is judged on.
        public override float ExpectedHz => 0.5f;
        public override float BitsPerSecond => specBitsPerSecond;
        public override float OneWayLatencyS =>
            (rangeM > 0f && soundVelocity > 0f) ? rangeM / soundVelocity : 0f;

        public override void Evaluate()
        {
            rangeM = RangeToPeer();
            if (peer == null)
            {
                up = false;
                reason = "no acoustic peer assigned (base-station transponder)";
                return;
            }
            if (requireWet && !IsSubmerged())
            {
                up = false;
                // NOT one string for two situations. On a hull this means "surfaced"; on the base
                // station's hanging fish it means "nobody put it in the water", and those need
                // opposite actions. `DryReason` picks the wording from what the modem is actually
                // fitted to — see CommsModem.DryReason and DeployedTransducer.
                //
                // The station used to dodge this entirely: StationBuilder forced requireWet = false
                // with the comment that the case is topside. It was true of the case and false of
                // the transducer, and it bought a station whose acoustic link could never report a
                // problem it definitely had — the modelled transducer was ~0.9 m in the air
                // (2026-08-21). A link that cannot say no is not evidence when it says yes.
                reason = DryReason("transducer out of the water");
                return;
            }
            if (rangeM > operationalRangeM)
            {
                up = false;
                reason = $"out of acoustic range — {rangeM:F0} m, working limit {operationalRangeM:F0} m";
                return;
            }
            up = true;
            reason = "";
        }
    }
}
