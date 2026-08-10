//Hand-generated to match smarc_control_msgs/ControlInput (2026-08-10 dashboard).
//header + six float64: thrusterrpm1, thrusterrpm2, thrustervertical, thrusterhorizontal, vbs, lcg.
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.SmarcControl
{
    [Serializable]
    public class ControlInputMsg : Message
    {
        public const string k_RosMessageName = "smarc_control_msgs/ControlInput";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public double thrusterrpm1;
        public double thrusterrpm2;
        public double thrustervertical;
        public double thrusterhorizontal;
        public double vbs;
        public double lcg;

        public ControlInputMsg()
        {
            this.header = new Std.HeaderMsg();
        }

        public static ControlInputMsg Deserialize(MessageDeserializer deserializer) => new ControlInputMsg(deserializer);

        private ControlInputMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.thrusterrpm1);
            deserializer.Read(out this.thrusterrpm2);
            deserializer.Read(out this.thrustervertical);
            deserializer.Read(out this.thrusterhorizontal);
            deserializer.Read(out this.vbs);
            deserializer.Read(out this.lcg);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.thrusterrpm1);
            serializer.Write(this.thrusterrpm2);
            serializer.Write(this.thrustervertical);
            serializer.Write(this.thrusterhorizontal);
            serializer.Write(this.vbs);
            serializer.Write(this.lcg);
        }

        public override string ToString()
        {
            return $"ControlInputMsg: rpm={thrusterrpm1:F0} tv=({thrustervertical:F2},{thrusterhorizontal:F2}) vbs={vbs:F1} lcg={lcg:F1}";
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
