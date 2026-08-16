using UnityEngine;

namespace VehicleComponents.Comms
{
    /// <summary>
    /// The cellular link — 4G/5G to a shore mast, which along the Swedish coast is usually
    /// available the moment the vehicle surfaces (Ivan, 2026-08-15).
    ///
    /// THIS IS THE LINK THAT CHANGES WHAT THE OPERATOR CAN KNOW, and it is worth being precise
    /// about why. WiFi only reaches ~25 m, so on any mission that leaves the immediate quayside
    /// a surfaced vehicle is beyond it. Without cellular, that vehicle has a perfect GPS fix and
    /// Mission Control has no idea — position, health and progress all sit on the hull with
    /// nobody to tell. With cellular, surfacing restores the full 1 Hz picture instantly. The
    /// difference between those two worlds is one modem, and modelling it is the point.
    ///
    /// HARDWARE (confirmed 2026-08-15 from the MRL wiki, sam:hardware:sam2-components and
    /// nosecone-v3): a **Waveshare SIM7600G-H 4G dongle** in the nosecone, connected by USB to
    /// the NanoPi Neo3 router, which brings the WAN up over QMI (`/dev/cdc-wdm0`) with APN
    /// `4g.tele2.se`. A dedicated 4G antenna sits beside the GPS antenna in the nosecone; its
    /// model is not recorded anywhere. Full detail: `data-cube/docs/SAM_VEHICLE_SPECS.md` §2.2.
    ///
    /// **This is 4G, not 5G.** The SIM7600G-H is an LTE Cat-4 class module and no 5G modem is
    /// recorded on the vehicle. `prefer5G` below is therefore aspirational, not a capability.
    ///
    /// COVERAGE, not range. A cellular link is not point-to-point with the base station — it
    /// goes to a mast and then over the internet, so "range to peer" is the wrong model. What
    /// matters is whether the vehicle is inside coverage. Two ways to say so:
    ///   * `coverageSource` set to a transform (a mast, or a stand-in for the shoreline) →
    ///     coverage is a radius around it;
    ///   * `coverageSource` empty → `assumeCoverage` decides, which is how you model "somewhere
    ///     on the Swedish coast, coverage is simply there" without placing masts in the scene.
    /// Either way, dropping `assumeCoverage` mid-mission is how you exercise a coverage hole —
    /// and a coverage hole with a surfaced vehicle is exactly the scenario above.
    ///
    /// Range figures for maritime LTE vary enormously with mast height, band and sea state;
    /// nothing here claims one. The default radius is a deliberately round, conservative
    /// placeholder to be replaced once the modem and a real coverage map are known.
    /// </summary>
    [AddComponentMenu("Smarc/Comms/Cellular Modem (4G/5G)")]
    public class CellularModem : SurfaceRadioModem
    {
        [Header("Cellular coverage")]
        [Tooltip("A mast, or any stand-in for the shore. Leave EMPTY to use assumeCoverage instead — cellular is coverage, not point-to-point range to the base station.")]
        public Transform coverageSource;
        [Tooltip("Coverage radius around coverageSource [m]. PLACEHOLDER: maritime LTE range depends on mast height, band and sea state, and no figure for this site has been measured. Replace once the modem and a coverage map are known.")]
        public float coverageRadiusM = 5000f;
        [Tooltip("Used only when coverageSource is empty: 'we are on the Swedish coast, there is coverage'. Untick mid-mission to model a coverage hole with the vehicle surfaced — the case where MC loses a perfectly healthy vehicle.")]
        public bool assumeCoverage = true;

        [Tooltip("5G where available, otherwise 4G. Cosmetic today — both behave as a full-payload surface link — but it is what the operator will ask.")]
        public bool prefer5G = true;

        public override string LinkName => "cellular";
        public override float ExpectedHz => 1.0f;
        // As with WiFi: a uniform number for payload reasoning, not a performance claim.
        public override float BitsPerSecond => 10e6f;

        void Reset()
        {
            // Cellular is not limited by range to the BASE STATION at all, so the inherited
            // point-to-point limits are set wide; coverage below is what actually decides.
            operationalRangeM = float.MaxValue;
            marginalRangeM = float.MaxValue;
            antennaHeightM = 0.15f;
        }

        public override void Evaluate()
        {
            marginal = false;
            rangeM = RangeToPeer();

            // The antenna rule is the same as WiFi's and is checked FIRST, because it is the
            // one that is true regardless of coverage: a submerged antenna is not a weak link,
            // it is no link.
            if (requireSurfaced && IsSubmerged())
            {
                up = false;
                reason = "antenna under water";
                return;
            }

            if (coverageSource != null)
            {
                float d = Vector3.Distance(transform.position, coverageSource.position);
                if (d > coverageRadiusM)
                {
                    up = false;
                    reason = $"outside cellular coverage — {d / 1000f:F1} km from {coverageSource.name}";
                    return;
                }
                up = true;
                reason = "";
                return;
            }

            if (!assumeCoverage)
            {
                up = false;
                reason = "no cellular coverage here";
                return;
            }
            up = true;
            reason = "";
        }
    }
}
