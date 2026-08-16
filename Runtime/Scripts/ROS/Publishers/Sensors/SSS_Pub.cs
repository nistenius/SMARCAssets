using UnityEngine;
using System;
using Unity.Robotics.Core; //Clock
using RosMessageTypes.Smarc;

using SideScanSonar = VehicleComponents.Sensors.Sonar;
using ROS.Core;


namespace ROS.Publishers
{
    [AddComponentMenu("Smarc/ROS/SSS_Pub")]
    [RequireComponent(typeof(SideScanSonar))]
    class SSS_Pub: ROSSensorPublisher<SidescanMsg, SideScanSonar>
    {
        /// <summary>
        /// Sound speed used to turn the sonar's geometric range into the two-way travel
        /// time the Sidescan message carries. The simulator's Sonar works in metres and
        /// has no sound speed at all, so this constant exists ONLY here, at the point
        /// where metres have to be expressed as time. 1500 m/s is the standard seawater
        /// figure; a consumer that converts back with the same constant recovers the
        /// range exactly, which is the property that matters.
        /// </summary>
        public const float SoundSpeedMS = 1500f;

        protected override void InitPublisher()
        {
            ROSMsg.header.frame_id = $"{robot_name}/{DataSource.linkName}";

            // max_duration is the ONLY field in this message that tells a subscriber how
            // many metres a range bin is worth, and until 2026-08-16 it was left at 0 —
            // so every consumer had to be TOLD the range out of band, i.e. had to carry a
            // copy of a number the publisher already knew. That is the shape of an
            // invented machine identity (SETTLED §3): the fix is to send the fact, not to
            // default it at the far end. Filling it is additive and cannot break an
            // existing subscriber, since 0 was never a usable value.
            //
            // Round trip: range_per_bin = SoundSpeedMS * max_duration / 2 / n_bins.
            ROSMsg.max_duration = 2f * DataSource.MaxRange / SoundSpeedMS;
            ROSMsg.decimation = 1;
            ROSMsg.frequency_id = 0;
            ROSMsg.type = 0xE2;   // "Sonar 8 Bit", the packet type this payload actually is
            ROSMsg.port_channel = new byte[DataSource.NumBucketsPerBeam];
            ROSMsg.starboard_channel = new byte[DataSource.NumBucketsPerBeam];
            ROSMsg.port_channel_angle_high = new byte[DataSource.NumBucketsPerBeam];
            ROSMsg.port_channel_angle_low = new byte[DataSource.NumBucketsPerBeam];
            ROSMsg.starboard_channel_angle_high = new byte[DataSource.NumBucketsPerBeam];
            ROSMsg.starboard_channel_angle_low = new byte[DataSource.NumBucketsPerBeam];

        }

        protected override void UpdateMessage()
        {
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
            var mid = DataSource.NumBucketsPerBeam;
            Array.Copy(DataSource.Buckets, 0, ROSMsg.port_channel, 0, mid);
            Array.Copy(DataSource.Buckets, mid, ROSMsg.starboard_channel, 0, mid);
            Array.Copy(DataSource.BucketsAngleHigh, 0, ROSMsg.port_channel_angle_high, 0, mid);
            Array.Copy(DataSource.BucketsAngleLow, 0, ROSMsg.port_channel_angle_low, 0, mid);
            Array.Copy(DataSource.BucketsAngleHigh, mid, ROSMsg.starboard_channel_angle_high, 0, mid);
            Array.Copy(DataSource.BucketsAngleLow, mid, ROSMsg.starboard_channel_angle_low, 0, mid);
        }
    }
}
