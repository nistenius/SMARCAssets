using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace VehicleComponents.Comms
{
    /// <summary>
    /// Collects every CommsModem on the vehicle, decides which link the operator is actually
    /// being reached over, and publishes that as JSON for `bridge_node.py` to gate on.
    ///
    /// SELECTION. Strict preference order, best first: **wifi → cellular → acoustic → none.**
    /// Not "whichever is fastest" or a cost function — a fixed, readable order, because an
    /// operator debugging a link needs to be able to predict which one is in use without
    /// simulating a decision. WiFi first because at the quayside it is free and local; cellular
    /// next because it is the surface link that works anywhere on the coast; acoustic last
    /// because it is the one with 463 bit/s and 0.67 s/km, and nothing should choose it while a
    /// surface link exists.
    ///
    /// WHAT THIS IS FOR — and the limit of it. The vehicle is fully connected internally at all
    /// times: Unity and the VM exchange everything, always (Ivan, 2026-08-15). Nothing here
    /// throttles a ROS topic. This publishes a single fact — which link would be carrying a
    /// report to shore right now — and `bridge_node.py` uses it to decide what it may send
    /// onward to the station: the full payload on a surface link, the Priority-1 report alone on
    /// acoustics, and NOTHING at all when there is no link. That last case is the one worth
    /// building for: a surfaced vehicle with a perfect fix that Mission Control cannot see,
    /// because seeing requires a link.
    ///
    /// Publishes on `comms/link_state` (std_msgs/String, JSON). JSON rather than a custom
    /// message so it needs no new message package on either side and stays readable in
    /// `ros2 topic echo` while this is being brought up.
    /// </summary>
    [AddComponentMenu("Smarc/Comms/Comms Manager")]
    public class CommsManager : MonoBehaviour
    {
        [Header("Peer")]
        [Tooltip("The base station / topside. Assigned to every modem that has no peer of its own, so the common case needs one field set instead of three.")]
        public Transform baseStation;

        [Header("Rate")]
        [Tooltip("How often the link state is re-evaluated and published [Hz]. This is the MODEL's tick, not any link's cadence.")]
        public float frequency = 2f;

        [Header("State (read-only)")]
        [Tooltip("The link currently carrying reports to shore, or 'none'.")]
        public string best = "none";
        [Tooltip("Why there is no link, when there is none. Empty while one is up.")]
        public string reason = "not evaluated";

        // Fixed preference order -- see the class summary for why this is a list and not a
        // cost function.
        static readonly string[] Preference = { "wifi", "cellular", "acoustic" };

        readonly List<CommsModem> modems = new List<CommsModem>();
        float nextTick;

        /// <summary>The JSON last published — also what bridge_node reads. Public so a test or
        /// an inspector can read it without a ROS round trip.</summary>
        [HideInInspector] public string lastJson = "";

        void Awake()
        {
            GetComponentsInChildren(true, modems);
            if (modems.Count == 0)
                Debug.LogWarning($"[{name}] CommsManager found no CommsModem components. " +
                                 "The vehicle will report link 'none' — which the bridge reads as " +
                                 "'send nothing to shore'. Add WiFi/Cellular/Acoustic modems, or " +
                                 "remove this manager.");
            foreach (var m in modems)
                if (m.peer == null) m.peer = baseStation;
        }

        void FixedUpdate()
        {
            if (Time.time < nextTick) return;
            nextTick = Time.time + (frequency > 0f ? 1f / frequency : 0.5f);
            Evaluate();
        }

        /// <summary>Re-evaluate every modem and pick the best. Separated from FixedUpdate so it
        /// can be driven directly from a test or the editor.</summary>
        public void Evaluate()
        {
            foreach (var m in modems) m.Evaluate();

            best = "none";
            reason = "";
            foreach (var want in Preference)
            {
                foreach (var m in modems)
                {
                    if (m.LinkName == want && m.up) { best = want; break; }
                }
                if (best != "none") break;
            }

            if (best == "none")
            {
                // Say WHY there is no link, naming each modem's own refusal. "No link" with no
                // reason is the output this whole component exists to avoid: an operator seeing
                // a vehicle vanish needs to know whether it dived, drifted out of range, or lost
                // coverage, and those have completely different responses.
                var sb = new StringBuilder();
                foreach (var m in modems)
                {
                    if (sb.Length > 0) sb.Append("; ");
                    sb.Append(m.LinkName).Append(": ").Append(string.IsNullOrEmpty(m.reason) ? "down" : m.reason);
                }
                reason = sb.Length > 0 ? sb.ToString() : "no modems fitted";
            }

            lastJson = BuildJson();
        }

        string BuildJson()
        {
            var sb = new StringBuilder();
            sb.Append("{\"best\":\"").Append(best).Append("\",");
            sb.Append("\"reason\":\"").Append(Escape(reason)).Append("\",");
            sb.Append("\"links\":{");
            for (int i = 0; i < modems.Count; i++)
            {
                var m = modems[i];
                if (i > 0) sb.Append(',');
                sb.Append('"').Append(m.LinkName).Append("\":{");
                sb.Append("\"up\":").Append(m.up ? "true" : "false").Append(',');
                sb.Append("\"range_m\":").Append(m.rangeM.ToString("F1", System.Globalization.CultureInfo.InvariantCulture)).Append(',');
                sb.Append("\"expected_hz\":").Append(m.ExpectedHz.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)).Append(',');
                sb.Append("\"bps\":").Append(m.BitsPerSecond.ToString("F0", System.Globalization.CultureInfo.InvariantCulture)).Append(',');
                sb.Append("\"latency_s\":").Append(m.OneWayLatencyS.ToString("F3", System.Globalization.CultureInfo.InvariantCulture)).Append(',');
                sb.Append("\"reason\":\"").Append(Escape(m.reason)).Append('"');
                sb.Append('}');
            }
            sb.Append("}}");
            return sb.ToString();
        }

        static string Escape(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            return s.Replace("\\", "\\\\").Replace("\"", "\\\"");
        }
    }
}
