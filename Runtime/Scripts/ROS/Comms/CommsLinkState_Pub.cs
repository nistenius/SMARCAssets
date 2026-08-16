using UnityEngine;

using ROS.Core;
using RosMessageTypes.Std;

using VehicleComponents.Comms;

namespace ROS.Comms
{
    /// <summary>
    /// Publishes CommsManager's link state as a std_msgs/String of JSON on `comms/link_state`.
    ///
    /// `bridge_node.py` subscribes and uses it to decide what may be sent onward to the station:
    /// full payload on a surface link, the Priority-1 report alone on acoustics, nothing at all
    /// with no link. Unity owns this decision because availability is a geometry question —
    /// range to the base station, antenna in or out of the water — and Unity is the only process
    /// that holds the geometry.
    ///
    /// std_msgs/String + JSON rather than a custom message: no new message package is needed on
    /// either side, and the topic stays readable in `ros2 topic echo` while this is being brought
    /// up on the rig, which is worth more than type safety for a diagnostic channel.
    ///
    /// If this publisher stops, the bridge falls back to its configured mode after
    /// LINK_STATE_STALE_S rather than assuming every link is down — Unity going quiet is not
    /// evidence that the vehicle is out of contact.
    /// </summary>
    [RequireComponent(typeof(CommsManager))]
    public class CommsLinkState_Pub : ROSPublisher<StringMsg>
    {
        CommsManager manager;

        protected override void UpdateMessage()
        {
            if (manager == null) manager = GetComponent<CommsManager>();
            ROSMsg.data = manager.lastJson;
        }
    }
}
