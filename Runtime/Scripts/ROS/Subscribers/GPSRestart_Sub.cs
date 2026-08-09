using UnityEngine;
using RosMessageTypes.Std;

using SensorGPS = VehicleComponents.Sensors.GPS;

namespace ROS.Subscribers
{
    /// <summary>
    /// Lets the vehicle (or an operator) command a GPS receiver restart over ROS, the way you
    /// would power-cycle the receiver after a long dive rather than trust a stale solution.
    /// Publish anything on the topic to trigger it:
    ///     ros2 topic pub --once /&lt;robot&gt;/core/gps_restart std_msgs/msg/Empty {}
    ///
    /// Deliberately edge-triggered on message arrival, not level-triggered: a restart is an
    /// event, and a latched "true" should not keep the receiver permanently re-acquiring.
    /// </summary>
    [AddComponentMenu("Smarc/ROS/GPSRestart_Sub")]
    [RequireComponent(typeof(SensorGPS))]
    public class GPSRestart_Sub : Actuator_Sub<EmptyMsg>
    {
        SensorGPS gps;
        bool handled = true;

        void Awake()
        {
            gps = GetComponent<SensorGPS>();
        }

        protected override void UpdateVehicle(bool reset)
        {
            if (gps == null)
            {
                Debug.Log("No GPS found for GPSRestart_Sub! Disabling.");
                enabled = false;
                rosCon.Unsubscribe(topic);
                return;
            }
            // Actuator_Sub calls this every FixedUpdate with the last message; we only want
            // to act once per received message, so latch on the message counter.
            if (!ReceivedFirstMessage) return;
            if (handled) return;
            handled = true;
            gps.RestartReceiver();
        }

        // Actuator_Sub has no per-message hook, so notice new messages by watching the
        // frequency estimate it maintains: any arrival resets `handled`.
        void Update()
        {
            if (ReceivedFirstMessage && lastSeen != receivedFrequency)
            {
                lastSeen = receivedFrequency;
                handled = false;
            }
        }
        double lastSeen = -1;
    }
}
