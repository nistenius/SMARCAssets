using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core; //Clock

using SensorGPS = VehicleComponents.Sensors.GPS;
using ROS.Core;


namespace ROS.Publishers
{
    [AddComponentMenu("Smarc/ROS/GPS_Pub")]
    [RequireComponent(typeof(SensorGPS))]
    class GPS_Pub: ROSSensorPublisher<NavSatFixMsg, SensorGPS>
    { 
        protected override void InitPublisher()
        {
            ROSMsg.header.frame_id = $"{robot_name}/{DataSource.linkName}";
        }
        
        protected override void UpdateMessage()
        {
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
            if(DataSource.fix)
            {
                ROSMsg.status.status = NavSatStatusMsg.STATUS_FIX;
                ROSMsg.latitude = DataSource.lat;
                ROSMsg.longitude = DataSource.lon;
                ROSMsg.altitude = DataSource.alt;
                ROSMsg.position_covariance = DataSource.positionCovariance;
                ROSMsg.position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
            }
            else ROSMsg.status.status = NavSatStatusMsg.STATUS_NO_FIX;
        }

        
    }
}
