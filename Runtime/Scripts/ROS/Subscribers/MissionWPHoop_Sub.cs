using UnityEngine;

using RosMessageTypes.SmarcMission; // GotoWaypointMsg
using GeoRef;                        // GlobalReferencePoint
using SmarcGUI.WorldSpace;           // WaypointHoop

using ROS.Core;

namespace ROS.Subscribers
{
    /// <summary>
    /// Subscribes to the vehicle's CURRENT waypoint (mission/last_wp, republished by the
    /// dive action server on every accepted goal) and shows it as a "hula hoop":
    /// diameter = 2 x goal tolerance, positioned at the WP's lat/lon/depth, opening
    /// facing the vehicle's approach direction.
    ///
    /// This is deliberately subscriber-driven from the VM's own mission plan — whatever
    /// mission the vehicle is really flying is what gets visualized. No mission-planner
    /// GUI involvement (design direction: Ivan, 2026-08-09).
    /// </summary>
    [AddComponentMenu("Smarc/ROS/MissionWPHoop_Sub")]
    public class MissionWPHoop_Sub : ROSBehaviour
    {
        [Tooltip("Visual thickness of the hoop tube.")]
        public float TubeRadius = 0.1f;

        GlobalReferencePoint globalRef;
        WaypointHoop hoop;
        Vector3 prevCenter;
        bool hasPrev = false;
        bool subscribed = false;

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
            // World-anchored: the hoop must NOT move with the robot, so it lives at scene root.
            var hoopGO = new GameObject($"{transform.root.name}_MissionWPHoop");
            hoop = hoopGO.AddComponent<WaypointHoop>();
            hoop.TubeRadius = TubeRadius;
            hoopGO.SetActive(false);
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
            Vector3 dir = Vector3.zero;
            if (hasPrev) dir = center - prevCenter;
            else if (GetBaseLink(out var baseLink)) dir = center - baseLink.position;
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

        void OnDestroy()
        {
            if (hoop != null) Destroy(hoop.gameObject);
        }
    }
}
