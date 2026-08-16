using UnityEngine;
using DefaultNamespace.Water;

namespace VehicleComponents.Comms
{
    /// <summary>
    /// One communications modem fitted to a vehicle. Base class for every link SAM can use to
    /// reach the operator: the Succorfish Delphis acoustic transceiver, the surface WiFi radio,
    /// and the cellular (4G/5G) modem.
    ///
    /// WHAT THIS MODELS, AND WHAT IT DOES NOT (Ivan, 2026-08-15)
    /// --------------------------------------------------------
    /// These modems model **what the operator can know**, never what the vehicle knows. The
    /// vehicle — Unity and the VM — is fully connected at all times: ROS topics are never
    /// throttled, dropped or degraded by anything here. SAM always knows its own depth, its own
    /// DR, its own health. The question these components answer is narrower and more
    /// interesting: *which of that, if any, reaches Mission Control right now?*
    ///
    /// That distinction is the whole point. A vehicle can be surfaced with a perfect RTK fix and
    /// still be invisible to the operator, because knowing requires a link and there may not be
    /// one. Before this existed, the sim quietly assumed a permanent 1 Hz firehose to shore,
    /// which is the one assumption a marine operations tool must never make.
    ///
    /// The manager (CommsManager) polls every modem, picks the best available, and publishes the
    /// result on `comms/link_state`. `bridge_node.py` reads that topic and gates what it sends
    /// onward to the station accordingly. The gate lives THERE, at the vehicle→station boundary,
    /// and nowhere else.
    ///
    /// SPEC PROVENANCE. Every derived class states where its numbers come from and how
    /// confident that is, using the same vocabulary as `fleet.yaml` (confirmed / user_provided /
    /// illustrative). A datasheet figure and an operational rule of thumb are both legitimate,
    /// but they must never be confused for one another in a tooltip an engineer will later quote.
    /// </summary>
    public abstract class CommsModem : MonoBehaviour
    {
        [Header("Peer")]
        [Tooltip("What this modem talks TO — the base station / topside transponder. Range is measured to this transform. If left empty the modem reports itself DOWN with a named reason rather than guessing a peer, because a link to an unknown peer is not a link.")]
        public Transform peer;

        [Header("State (read-only, updated every tick)")]
        [Tooltip("Is this link usable right now?")]
        public bool up;
        [Tooltip("Metres to the peer, straight line. -1 when there is no peer to measure to.")]
        public float rangeM = -1f;
        [Tooltip("Why the link is down, in words an operator can act on. Empty while it is up.")]
        public string reason = "not evaluated";

        /// <summary>Short name used on the wire and by the GUIs: wifi | cellular | acoustic.</summary>
        public abstract string LinkName { get; }

        /// <summary>
        /// How often a consumer should expect to hear from the vehicle over this link. Mirrors
        /// LINK_EXPECTED_HZ in bridge_node.py and Mission Control — the three must agree, or
        /// staleness means something different in each window.
        /// </summary>
        public abstract float ExpectedHz { get; }

        /// <summary>Usable payload rate, bits/second. The acoustic figure is the reason the
        /// full mission never crosses that link (SYSTEMS_SPEC §3/§4).</summary>
        public abstract float BitsPerSecond { get; }

        /// <summary>One-way propagation delay to the peer, seconds. Negligible on radio;
        /// ~0.67 s/km on acoustics, which is THE design constraint — latency before
        /// bandwidth — and the reason submerged interaction is fire-and-forget, never RPC.</summary>
        public virtual float OneWayLatencyS => 0f;

        /// <summary>Is the modem's own antenna/transducer in the water right now?</summary>
        protected bool IsSubmerged()
        {
            var water = FindWater();
            if (water == null) return false;
            return transform.position.y < water.GetWaterLevelAt(transform.position);
        }

        WaterQueryModel _water;
        protected WaterQueryModel FindWater()
        {
            if (_water != null) return _water;
            _water = FindObjectOfType<WaterQueryModel>();
            return _water;
        }

        /// <summary>Metres to the peer, or -1 if there is no peer.</summary>
        protected float RangeToPeer()
        {
            if (peer == null) return -1f;
            return Vector3.Distance(transform.position, peer.position);
        }

        /// <summary>
        /// Recompute `up` / `rangeM` / `reason`. Called by CommsManager every tick.
        /// Implementations must ALWAYS set `reason` when reporting down — a link that is off
        /// without saying why is the thing this whole feature exists to remove.
        /// </summary>
        public abstract void Evaluate();
    }
}
