using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using ROS.Core;

namespace ROS.Publishers
{
    [AddComponentMenu("Smarc/ROS/TF_Pub")]
    public class ROSTransformTreePublisher : ROSPublisher<TFMessageMsg>
    {
        [Header("TF Tree Publisher")]
        [Tooltip("Frame between unity_origin and <robot_name>/odom. If empty, map == odom.")]
        public Transform MapFrameTransform;
        [Tooltip("Adds a prefix to all TF frames published by this publisher, except unity_origin.")]
        public string tf_prefix = "";
        [Tooltip("Adds a suffix to all TF frames published by this publisher, except unity_origin. Set to '_gt' to publish this tree as ground truth (robot/base_link_gt ...), leaving the plain frames (robot/base_link) to the state estimator + robot description. Empty = legacy behaviour where Unity truth and the estimate share frame names.")]
        public string tf_suffix = "";
        [Tooltip("Publish without any connection to the world frames. This is useful when you want to publish _just this bit_ of the TF independently of its global position.")]
        public bool OnlyRelative = false;
        TransformTreeNode BaseLinkTreeNode;
        GameObject BaseLinkGO;
        GameObject OdomLinkGO;

        List<TransformStampedMsg> staticTFs = new List<TransformStampedMsg>();

        void OnValidate()
        {
            if (frequency > 1f / Time.fixedDeltaTime)
            {
                Debug.LogWarning($"TF Publisher update frequency set to {frequency}Hz but Unity updates physics at {1f / Time.fixedDeltaTime}Hz. Setting to Unity's fixedDeltaTime!");
                frequency = 1f / Time.fixedDeltaTime;
            }

            if (topic != "/tf")
            {
                Debug.LogWarning($"TF Publisher topic set to {topic} but should be /tf. Setting to /tf!");
                topic = "/tf";
            }
        }

        protected override void InitPublisher()
        {
            // we need map(ENU) -> odom(ENU) -> base_link(ENU) -> children(FLU)
            // if this is not a robot, or doesnt have a base_link, we assume _this object_
            // will work as whatever is missing...
            if (!GetRobotGO(out OdomLinkGO))
            {
                OdomLinkGO = transform.gameObject;
            }
            if (GetBaseLink(out var baseLink))
            {
                BaseLinkGO = baseLink.gameObject;
                BaseLinkTreeNode = new TransformTreeNode(BaseLinkGO);
            }
            else
            {
                BaseLinkTreeNode = new TransformTreeNode(transform.gameObject);
            }

            // use top-level parent as the namespace for this tf tree if there is no robot name
            if (robot_name == "")
            {
                Transform topParent = transform;
                while (topParent.parent != null)
                {
                    topParent = topParent.parent;
                }
                robot_name = topParent.name;
            }

            staticTFs.Clear();
            if(!OnlyRelative) PopulateStaticFrames(staticTFs);
        }

        static void PopulateChildrenTFList(List<TransformStampedMsg> tfList, TransformTreeNode tfNode)
        {
            // TODO: Some of this could be done once and cached rather than doing from scratch every time
            // Only generate transform messages from the children, because This node will be parented to the global frame
            foreach (var childTf in tfNode.Children)
            {
                tfList.Add(TransformTreeNode.ToTransformStamped(childTf));

                if (!childTf.IsALeafNode)
                {
                    PopulateChildrenTFList(tfList, childTf);
                }
            }
        }


        void PopulateStaticFrames(List<TransformStampedMsg> tfMessageList)
        {
            // we want globally oriented transforms to be the first in the list.
            // map -> odom and odom -> base_link
            // map frame in unity is the unity origin, so we want a 0-transform for that
            // odom frame is cached in StartROS, it is the position of the robot at game start
            // we want the transform from base_link to odom

            // The tree should look like this:
            // |--------------- STATIC FRAMES ------------------------------------------------------------||------ very dynamic ---------||--- mixed ----|
            // utm -(0)-> utm_ZONE_BAND -(cached)-> unity_origin -(cached)-> VEHICLE/map -(cached)-> VEHICLE/odom -(dynamic)-> VEHICLE/base_link -> children...

            var unityToMapMsg = new TransformMsg();
            var mapToOdomMsg = new TransformMsg();

            if (MapFrameTransform == null) MapFrameTransform = OdomLinkGO.transform;

            unityToMapMsg.translation = MapFrameTransform.To<ENU>().translation;
            unityToMapMsg.rotation = MapFrameTransform.To<FLU>().rotation;

            Vector3 mapToOdomTransform = MapFrameTransform.InverseTransformPoint(OdomLinkGO.transform.position);
            var mapToOdomPos = ENU.ConvertFromRUF(mapToOdomTransform);
            mapToOdomMsg.translation = new Vector3Msg(
                mapToOdomPos.x,
                mapToOdomPos.y,
                mapToOdomPos.z);
            var mapToOdomOri = FLU.ConvertFromRUF(Quaternion.Inverse(MapFrameTransform.rotation) * OdomLinkGO.transform.rotation);
            mapToOdomMsg.rotation = new QuaternionMsg(
                mapToOdomOri.x,
                mapToOdomOri.y,
                mapToOdomOri.z,
                mapToOdomOri.w);

            var unityToMap = new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(Clock.time), "unity_origin"),
                $"{tf_prefix}{robot_name}/map{tf_suffix}",
                unityToMapMsg);

            var mapToOdom = new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(Clock.time), $"{tf_prefix}{robot_name}/map{tf_suffix}"),
                $"{tf_prefix}{robot_name}/odom{tf_suffix}",
                mapToOdomMsg);

            tfMessageList.Add(unityToMap);
            tfMessageList.Add(mapToOdom);
        }

        protected override void UpdateMessage()
        {
            // The tree should look like this:
            // |--------------- STATIC FRAMES ------------------------------------------------------------||------ very dynamic ---------||--- mixed ----|
            // utm -(0)-> utm_ZONE_BAND -(cached)-> unity_origin -(cached)-> VEHICLE/map -(cached)-> VEHICLE/odom -(dynamic)-> VEHICLE/base_link -> children...

            var tfMessageList = new List<TransformStampedMsg>();

            
            if(!OnlyRelative)
            {
                // odom -> base_link first, unless just doing relative, then no odom exists
                var odomPos = OdomLinkGO.transform.InverseTransformPoint(BaseLinkTreeNode.Transform.position);
                var odomOri = Quaternion.Inverse(OdomLinkGO.transform.rotation) * BaseLinkTreeNode.Transform.rotation;
                var rosOdomPos = ENU.ConvertFromRUF(odomPos);
                var rosOdomOri = ENU.ConvertFromRUF(odomOri);
                var odomToBaseLinkTFMSG = new TransformMsg
                {
                    translation = new Vector3Msg(
                        rosOdomPos.x,
                        rosOdomPos.y,
                        rosOdomPos.z),
                    rotation = new QuaternionMsg(
                        rosOdomOri.x,
                        rosOdomOri.y,
                        rosOdomOri.z,
                        rosOdomOri.w)
                };
                var odomToBaseLink = new TransformStampedMsg(
                    new HeaderMsg(new TimeStamp(Clock.time), "odom"),
                    BaseLinkTreeNode.name,
                    odomToBaseLinkTFMSG);

                tfMessageList.Add(odomToBaseLink);                
            }

            // base_link -> children next
            try
            {
                PopulateChildrenTFList(tfMessageList, BaseLinkTreeNode);
            }
            catch (MissingReferenceException)
            {
                // If the object tree was modified after the TF Tree was built
                // such as deleting a child object, this will throw an exception
                // So we need to re-build the TF tree and skip the publish.
                Debug.Log($"[{transform.name}] TF Tree was modified, re-building.");
                BaseLinkTreeNode = new TransformTreeNode(BaseLinkGO);
                return;
            }

            // prefix all frames with the robot name to create a namespace
            // and suffix them (e.g. '_gt') to keep simulator ground truth
            // distinguishable from the state estimator's frames.
            foreach (TransformStampedMsg msg in tfMessageList)
            {
                msg.header.frame_id = $"{tf_prefix}{robot_name}/{msg.header.frame_id}{tf_suffix}";
                msg.child_frame_id = $"{tf_prefix}{robot_name}/{msg.child_frame_id}{tf_suffix}";
            }

            // refresh the times of the static frames
            foreach (TransformStampedMsg staticTF in staticTFs)
            {
                staticTF.header.stamp = new TimeStamp(Clock.time);
            }
            // finally add the static frames last so they are not namespaced
            tfMessageList.AddRange(staticTFs);

            ROSMsg = new TFMessageMsg(tfMessageList.ToArray());
        }

        public void SetBaseLinkName(string name)
        {
            robot_name = name;
        }

    }
}
