using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core; //Clock
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using SensorMagnetometer = VehicleComponents.Sensors.Magnetometer;
using ROS.Core;


namespace ROS.Publishers
{
    /// <summary>
    /// Publishes sensor_msgs/MagneticField, matching the real vehicle's `core/compass`.
    /// See Magnetometer.cs for why the sim needs this at all.
    /// </summary>
    [AddComponentMenu("Smarc/ROS/Magnetometer_Pub")]
    [RequireComponent(typeof(SensorMagnetometer))]
    class Magnetometer_Pub: ROSSensorPublisher<MagneticFieldMsg, SensorMagnetometer>
    {
        protected override void InitPublisher()
        {
            ROSMsg.header.frame_id = $"{robot_name}/{DataSource.linkName}";
        }

        protected override void UpdateMessage()
        {
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
            ROSMsg.magnetic_field = DataSource.magneticField.To<FLU>();
            ROSMsg.magnetic_field_covariance = DataSource.magneticFieldCovariance;
        }
    }
}
