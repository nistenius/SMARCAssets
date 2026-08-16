using UnityEngine;

namespace VehicleComponents.Comms
{
    /// <summary>
    /// Shared behaviour for every RADIO link on the hull — WiFi and cellular. What they have in
    /// common is the thing that dominates both: **the antenna has to be out of the water, and it
    /// is never more than a hand's width above it.**
    ///
    /// WHY DATASHEET RANGES ARE MEANINGLESS HERE (Ivan, 2026-08-15: "as we are operating very
    /// close to water, the specs are not really representative")
    /// ----------------------------------------------------------------------------------------
    /// A radio datasheet quotes free-space or open-field range with antennas on masts. SAM's
    /// antenna sits a few centimetres above a flat, highly conductive, moving reflector. Three
    /// separate effects wreck the link budget, and all three get worse as range grows:
    ///
    ///   1. FRESNEL OBSTRUCTION. The first Fresnel zone must be clear for free-space-like loss.
    ///      Its radius at midpoint is ~sqrt(lambda*d/4) — at 2.4 GHz over 100 m that is ~1.8 m,
    ///      and neither end has anything like 1.8 m of antenna height. The sea surface cuts
    ///      straight through the zone, so the free-space model simply does not apply.
    ///   2. TWO-RAY CANCELLATION. Direct and surface-reflected rays arrive with the reflected
    ///      one phase-inverted off water. Beyond the breakpoint distance (4*pi*h1*h2/lambda)
    ///      loss climbs as 1/d^4 rather than 1/d^2. With h1 ~ 0.15 m and h2 ~ 2 m at 2.4 GHz the
    ///      breakpoint is only a few metres, so essentially the ENTIRE useful range sits in the
    ///      fourth-power region — which is why range collapses from hundreds of metres to tens.
    ///   3. SEA STATE AND ATTITUDE. The reflector moves, and so does the hull; a surfaced AUV
    ///      rolls its antenna in and out of usable orientation. This shows up as a link that
    ///      works, then does not, then does again, at constant range.
    ///
    /// So these components are driven by an OPERATIONAL range measured or stated by the people
    /// who run the vehicle, not by a computed link budget. The physics above is documented
    /// because it explains the shape of the number — and because it is the reason nobody should
    /// "fix" a suspiciously short range by substituting a datasheet figure.
    /// </summary>
    public abstract class SurfaceRadioModem : CommsModem
    {
        [Header("Antenna")]
        [Tooltip("Antenna height above the waterline when surfaced [m]. Documented because it is the term that makes near-water range collapse — see this class's summary. Not currently used to COMPUTE range; the operational range figure already embeds it.")]
        public float antennaHeightM = 0.15f;
        [Tooltip("Require the antenna to be out of the water. A radio antenna under salt water is a short circuit, not a poor link.")]
        public bool requireSurfaced = true;

        [Header("Operational range")]
        [Tooltip("Working range over water [m]. See this class's summary for why this is an operational figure and not a datasheet one.")]
        public float operationalRangeM = 25f;
        [Tooltip("Range beyond which the link is gone rather than merely poor. Between operationalRangeM and this, the link is reported UP but MARGINAL.")]
        public float marginalRangeM = 40f;

        [Header("State")]
        [Tooltip("Up, but degrading — past the working range and inside the marginal band.")]
        public bool marginal;

        /// <summary>Evaluate the shared surfaced + range logic. Derived classes call this and
        /// then add whatever else their link needs (coverage, for cellular).</summary>
        protected bool EvaluateSurfaceRadio()
        {
            marginal = false;
            rangeM = RangeToPeer();
            if (requireSurfaced && IsSubmerged())
            {
                up = false;
                reason = "antenna under water";
                return false;
            }
            if (peer == null)
            {
                up = false;
                reason = "no peer assigned";
                return false;
            }
            if (rangeM > marginalRangeM)
            {
                up = false;
                reason = $"out of range — {rangeM:F0} m, usable to about {operationalRangeM:F0} m over water";
                return false;
            }
            if (rangeM > operationalRangeM)
            {
                // Deliberately still UP. The honest behaviour near water is that the link
                // degrades before it disappears, and an operator watching a vehicle drift out
                // should see it going rather than only discover it gone.
                up = true;
                marginal = true;
                reason = $"marginal — {rangeM:F0} m, past the {operationalRangeM:F0} m working range";
                return true;
            }
            up = true;
            reason = "";
            return true;
        }
    }
}
