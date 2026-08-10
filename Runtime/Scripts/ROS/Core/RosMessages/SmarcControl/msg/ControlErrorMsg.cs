//Hand-generated to match smarc_control_msgs/ControlError (2026-08-10 dashboard).
//Same wire format as Unity-ROS MessageGeneration output: eight float64 fields.
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.SmarcControl
{
    [Serializable]
    public class ControlErrorMsg : Message
    {
        public const string k_RosMessageName = "smarc_control_msgs/ControlError";
        public override string RosMessageName => k_RosMessageName;

        public double x;
        public double y;
        public double z;
        public double roll;
        public double pitch;
        public double yaw;
        public double heading;
        public double distance;

        public ControlErrorMsg()
        {
        }

        public static ControlErrorMsg Deserialize(MessageDeserializer deserializer) => new ControlErrorMsg(deserializer);

        private ControlErrorMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.x);
            deserializer.Read(out this.y);
            deserializer.Read(out this.z);
            deserializer.Read(out this.roll);
            deserializer.Read(out this.pitch);
            deserializer.Read(out this.yaw);
            deserializer.Read(out this.heading);
            deserializer.Read(out this.distance);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.x);
            serializer.Write(this.y);
            serializer.Write(this.z);
            serializer.Write(this.roll);
            serializer.Write(this.pitch);
            serializer.Write(this.yaw);
            serializer.Write(this.heading);
            serializer.Write(this.distance);
        }

        public override string ToString()
        {
            return $"ControlErrorMsg: z={z:F2} pitch={pitch:F2} yaw={yaw:F2} heading={heading:F2} distance={distance:F2}";
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
