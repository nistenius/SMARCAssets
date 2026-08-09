using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core; //Clock
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using SensorIMU = VehicleComponents.Sensors.IMU;
using ROS.Core;


namespace ROS.Publishers
{
    [AddComponentMenu("Smarc/ROS/IMU_Pub")]
    [RequireComponent(typeof(SensorIMU))]
    class IMU_Pub: ROSSensorPublisher<ImuMsg, SensorIMU>
    { 
        [Tooltip("If false, orientation is in ENU in ROS.")]
        public bool useNED = false;

        protected override void InitPublisher()
        {
            ROSMsg.header.frame_id = $"{robot_name}/{DataSource.linkName}";
        }

        protected override void UpdateMessage()
        {
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
            if(useNED) ROSMsg.orientation = DataSource.orientation.To<NED>();
            else ROSMsg.orientation = DataSource.orientation.To<ENU>();
            
            ROSMsg.angular_velocity = DataSource.angularVelocity.To<FLU>();
            ROSMsg.linear_acceleration = DataSource.linearAcceleration.To<FLU>();

            ROSMsg.orientation_covariance = DataSource.orientationCovariance;
            ROSMsg.angular_velocity_covariance = DataSource.angularVelocityCovariance;
            ROSMsg.linear_acceleration_covariance = DataSource.linearAccelerationCovariance;
        }
    }
}
