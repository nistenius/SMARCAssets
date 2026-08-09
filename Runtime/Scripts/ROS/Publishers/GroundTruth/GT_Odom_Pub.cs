using UnityEngine;
using Unity.Robotics.Core;
using ROS.Core;
using Force;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


namespace ROS.Publishers.GroundTruth
{
    [AddComponentMenu("Smarc/ROS/GT_Odom_Pub")]
    class GT_Odom_Pub : ROSPublisher<OdometryMsg>
    {
        [Tooltip("Suffix for the child frame, matching ROSTransformTreePublisher's tf_suffix. Set to '_gt' when the TF tree is published as ground truth.")]
        public string tf_suffix = "";

        MixedBody body;

        protected override void InitPublisher()
        {
            if (GetBaseLink(out var base_link))
            {
                var base_link_ab = base_link.GetComponent<ArticulationBody>();
                var base_link_rb = base_link.GetComponent<Rigidbody>();
                body = new MixedBody(base_link_ab, base_link_rb);
                if (!body.isValid)
                {
                    Debug.LogError("Base link doesnt have a valid Rigidbody or ArticulationBody.");
                    enabled = false;
                    return;
                }

                GetRobotGO(out var robotGO);
                ROSMsg.header.frame_id = "unity_origin";
                ROSMsg.child_frame_id = $"{robotGO.name}/base_link{tf_suffix}";
            }
        }

        protected override void UpdateMessage()
        {
            ROSMsg.header.stamp = new TimeStamp(Clock.time);
            ROSMsg.pose.pose.position = body.position.To<ENU>();
            ROSMsg.pose.pose.orientation = body.rotation.To<ENU>();

            ROSMsg.twist.twist.linear = body.localVelocity.To<FLU>();
            var anvel = body.transform.InverseTransformVector(body.angularVelocity);
            ROSMsg.twist.twist.angular = -anvel.To<FLU>();
        }
    }
}
