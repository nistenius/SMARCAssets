using UnityEngine;

using RosMessageTypes.SmarcMission; // GotoWaypointMsg
using GeoRef;                        // GlobalReferencePoint
using SmarcGUI.WorldSpace;           // WaypointHoop

using ROS.Core;

namespace ROS.Subscribers
{
    /// <summary>
    /// Shows the vehicle's CURRENT waypoint (mission/last_wp, republished by the dive
    /// action server on every accepted goal) as a "hula hoop": diameter = 2 x goal
    /// tolerance, positioned at the WP's lat/lon/depth, opening facing the approach.
    ///
    /// Standalone scene object by design: drag the MissionWPHoop prefab into a scene
    /// and set RobotName to choose whose mission to mirror. The display is
    /// subscriber-driven from the VM's own plan — whatever mission the vehicle is
    /// really flying is what gets visualized (design direction: Ivan, 2026-08-09).
    /// </summary>
    [AddComponentMenu("Smarc/ROS/MissionWPHoop_Sub")]
    public class MissionWPHoop_Sub : ROSBehaviour
    {
        [Tooltip("Robot whose mission to mirror. Used to namespace the topic (/<RobotName>/<topic>) and, if the robot exists in the scene, to orient the first hoop toward it.")]
        public string RobotName = "sam_auv_v1";

        [Tooltip("Visual thickness of the hoop tube.")]
        public float TubeRadius = 0.1f;

        [Tooltip("Hoop gets a MeshCollider: visible to the 3D sonar, and a physical gate.")]
        public bool Collidable = true;

        GlobalReferencePoint globalRef;
        WaypointHoop hoop;
        Vector3 prevCenter;
        bool hasPrev = false;
        bool subscribed = false;

        void OnValidate()
        {
            // Standalone scene object: don't let ROSBehaviour try to find a robot parent.
            NotARobot = true;
        }

        void Awake()
        {
            // Must happen BEFORE ROSBehaviour.OnEnable: a relative topic without a robot
            // parent gets the component force-disabled there. Awake runs first, so we
            // namespace with the chosen robot here and OnEnable sees an absolute topic.
            NotARobot = true;
            if (!topic.StartsWith("/") && !string.IsNullOrEmpty(RobotName))
                topic = $"/{RobotName}/{topic}";
        }

        protected override void StartROS()
        {
            globalRef = FindFirstObjectByType<GlobalReferencePoint>();
            if (globalRef == null)
            {
                Debug.Log($"[{transform.name}] No GlobalReferencePoint found! Disabling.");
                enabled = false;
                return;
            }

            if (!subscribed)
            {
                rosCon.Subscribe<GotoWaypointMsg>(topic, OnWaypoint);
                subscribed = true;
            }
        }

        void EnsureHoop()
        {
            if (hoop != null) return;
            var hoopGO = new GameObject($"{RobotName}_WPHoop");
            hoopGO.transform.SetParent(transform, false); // under this scene object, world-anchored
            hoop = hoopGO.AddComponent<WaypointHoop>();
            hoop.TubeRadius = TubeRadius;
            hoop.Collidable = Collidable;
            hoopGO.SetActive(false);
        }

        Vector3 RobotPosition()
        {
            var robotGO = GameObject.Find(RobotName);
            if (robotGO != null) return robotGO.transform.position;
            return transform.position;
        }

        void OnWaypoint(GotoWaypointMsg msg)
        {
            EnsureHoop();

            var center = new Vector3(0, 0, 0);
            (center.x, center.z) = globalRef.GetUnityXZFromLatLon(msg.lat, msg.lon);
            center.y = (float)-msg.travel_depth;

            hoop.gameObject.SetActive(true);
            hoop.transform.position = center;
            hoop.SetRadius((float)msg.goal_tolerance);

            // Face the hoop along the approach: previous WP -> this WP if we have one
            // (the actual leg), otherwise robot -> WP (the transit to the first WP).
            Vector3 dir = hasPrev ? center - prevCenter : center - RobotPosition();
            dir.y = 0;
            if (dir.sqrMagnitude > 1e-6f) hoop.SetDirection(dir);

            if (!hasPrev || (center - prevCenter).sqrMagnitude > 0.25f)
            {
                prevCenter = center;
                hasPrev = true;
            }
        }

        void OnDisable()
        {
            if (hoop != null) hoop.gameObject.SetActive(false);
        }
    }
}
