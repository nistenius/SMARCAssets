using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core; //Clock

using SensorGPS = VehicleComponents.Sensors.GPS;
using ROS.Core;


namespace ROS.Publishers
{
    [AddComponentMenu("Smarc/ROS/GPS_Pub")]
    [RequireComponent(typeof(SensorGPS))]
    // `public` since 2026-08-18: the Editor assembly (SMARC.SMARCAssets.Editor) builds the base
    // station prefab and has to AddComponent<GPS_Pub>() on it. Its siblings here are already
    // public (SonarPointCloud_Pub, DepthImage_Pub, ...); this one was internal by oversight, and
    // widening visibility cannot change any existing behaviour.
    public class GPS_Pub: ROSSensorPublisher<NavSatFixMsg, SensorGPS>
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
                // An RTK solution is GBAS-augmented, and consumers MUST be able to tell:
                // a 1.4 cm fix and a 1.5 m fix arrive in the same message type, and the
                // estimator's sigma gate / the HUD both branch on this.
                ROSMsg.status.status =
                    DataSource.rtkSolution == SensorGPS.RtkSolution.Standalone
                        ? NavSatStatusMsg.STATUS_FIX
                        : NavSatStatusMsg.STATUS_GBAS_FIX;
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
