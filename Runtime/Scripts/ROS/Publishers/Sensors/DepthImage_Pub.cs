using System;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core; // Clock

using ROS.Core;
using DepthCameraSensor = VehicleComponents.Sensors.DepthCamera;

namespace ROS.Publishers
{
    /// <summary>
    /// Publishes the ground-truth DepthCamera sensor as a 32FC1 depth image
    /// (meters, 0 = invalid), like the RealSense driver's depth stream
    /// (realsense2_camera publishes 16UC1 mm by default, 32FC1 m is its
    /// metric alternative; consumers handle both).
    /// </summary>
    [AddComponentMenu("Smarc/ROS/DepthImage_Pub")]
    public class DepthImage_Pub : ROSSensorPublisher<ImageMsg, DepthCameraSensor>
    {
        const int BYTES_PER_PIXEL = 4; // 32FC1

        protected override void InitPublisher()
        {
            var h = DataSource.textureHeight;
            var w = DataSource.textureWidth;
            ROSMsg.data = new byte[h * w * BYTES_PER_PIXEL];
            ROSMsg.encoding = "32FC1";
            ROSMsg.height = (uint)h;
            ROSMsg.width = (uint)w;
            ROSMsg.is_bigendian = 0;
            ROSMsg.step = (uint)(BYTES_PER_PIXEL * w);
            ROSMsg.header.frame_id = $"{robot_name}/{DataSource.linkName}";
        }

        protected override void UpdateMessage()
        {
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
            var depths = DataSource.Depths;
            if (depths == null) return;
            if (ROSMsg.data.Length != depths.Length * BYTES_PER_PIXEL)
                ROSMsg.data = new byte[depths.Length * BYTES_PER_PIXEL];
            Buffer.BlockCopy(depths, 0, ROSMsg.data, 0, ROSMsg.data.Length);
        }
    }
}
