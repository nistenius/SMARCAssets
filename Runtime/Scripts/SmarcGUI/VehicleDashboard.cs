using UnityEngine;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.SmarcMission;   // GotoWaypointMsg
using RosMessageTypes.SmarcControl;   // ControlErrorMsg, ControlInputMsg (hand-generated)
using RosMessageTypes.Std;            // Int8Msg, Float32Msg, BoolMsg, StringMsg

namespace SmarcGUI
{
    /// <summary>
    /// Vehicle dashboard (2026-08-10, Ivan's spec, v2): waypoint status, an ACTION
    /// narrative line ("what is the vehicle doing right now"), health and nearest
    /// obstacle — plus small controller annotations injected into the existing
    /// banner fields (yaw err on Compass, obstacle range on Alt, depth ref+err on
    /// Depth, rpm on Speed). All SUBSCRIBER-DRIVEN from the VM's topics.
    ///
    /// The action line uses ctrl/obstacle_status ("clear"|"STOP n/N"|"ABORT")
    /// published by DiveControllerBlendPID; without it, it falls back to the
    /// detector's stop flag and stream freshness.
    /// </summary>
    public class VehicleDashboard : MonoBehaviour
    {
        [Tooltip("Robot whose topics to mirror (absolute topics /<RobotName>/...).")]
        public string RobotName = "sam_auv_v1";

        [Tooltip("Main dashboard text block (assigned by DashboardBuilder).")]
        public TMP_Text DashboardText;

        [Header("Small banner annotations (assigned by DashboardBuilder)")]
        public TMP_Text SmallCompassText;   // yaw error
        public TMP_Text SmallAltText;       // nearest obstacle
        public TMP_Text SmallDepthText;     // depth ref + error
        public TMP_Text SmallSpeedText;     // rpm command

        [Tooltip("Seconds without a controller message before the action reads idle.")]
        public float ActionStaleSec = 2.0f;

        [Tooltip("Seconds without a detector message before obstacle reads no-data.")]
        public float ObstacleStaleSec = 3.0f;

        ROSConnection ros;

        GotoWaypointMsg wp;
        ControlErrorMsg err;      float errTime = -999f;
        ControlInputMsg input;    float inputTime = -999f;
        sbyte health = -1;        float healthTime = -999f;
        float obstacleRange = -1; float obstacleTime = -999f;
        bool obstacleStop;
        string obstacleStatus = ""; float obstacleStatusTime = -999f;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            string ns = $"/{RobotName}";
            ros.Subscribe<GotoWaypointMsg>($"{ns}/mission/last_wp", m => wp = m);
            ros.Subscribe<ControlErrorMsg>($"{ns}/ctrl/conv/error", m => { err = m; errTime = Time.time; });
            ros.Subscribe<ControlInputMsg>($"{ns}/ctrl/conv/control_input", m => { input = m; inputTime = Time.time; });
            ros.Subscribe<Int8Msg>($"{ns}/smarc/vehicle_health", m => { health = m.data; healthTime = Time.time; });
            ros.Subscribe<Float32Msg>($"{ns}/perception/obstacle/nearest_range", m => { obstacleRange = m.data; obstacleTime = Time.time; });
            ros.Subscribe<BoolMsg>($"{ns}/perception/obstacle/stop", m => obstacleStop = m.data);
            ros.Subscribe<StringMsg>($"{ns}/ctrl/obstacle_status", m => { obstacleStatus = m.data; obstacleStatusTime = Time.time; });
        }

        string HealthStr()
        {
            if (Time.time - healthTime > 5f) return "<color=#888888>no data</color>";
            switch (health)
            {
                case 0: return "<color=#4CBB6C>READY</color>";
                case 1: return "<color=#D9A62E>WAITING</color>";
                case 2: return "<color=#D9534F>ERROR</color>";
                default: return $"? ({health})";
            }
        }

        string ObstacleStr()
        {
            if (Time.time - obstacleTime > ObstacleStaleSec) return "<color=#888888>no data</color>";
            string range = obstacleRange >= 0f ? $"{obstacleRange:F1} m" : "clear";
            if (obstacleStop) return $"<color=#D9534F>STOP  {range}</color>";
            if (obstacleRange >= 0f && obstacleRange < 6f) return $"<color=#D9A62E>{range}</color>";
            return $"<color=#4CBB6C>{range}</color>";
        }

        string ActionStr(bool active)
        {
            bool haveStatus = Time.time - obstacleStatusTime < 5f;
            if (haveStatus && obstacleStatus == "ABORT")
                return "<color=#D9534F>ABORTED (obstacle) — surfacing, VBS empty</color>";
            if (haveStatus && obstacleStatus.StartsWith("STOP"))
                return $"<color=#D9534F>OBSTACLE {obstacleStatus}</color> — holding depth, retry pending";
            if (!haveStatus && obstacleStop)
                return "<color=#D9534F>OBSTACLE STOP</color> — holding depth";
            if (active && err != null)
                return $"<color=#4CBB6C>driving to wp</color> — {err.distance:F1} m to go";
            return "<color=#888888>idle — waiting for a mission</color>";
        }

        void Update()
        {
            bool active = Time.time - errTime < ActionStaleSec;

            if (DashboardText != null)
            {
                string wpLine = "-";
                if (wp != null)
                {
                    string name = string.IsNullOrEmpty(wp.name) ? "wp" : wp.name;
                    wpLine = $"{name}  d{wp.travel_depth:F1}m rpm{wp.travel_rpm:F0} tol{wp.goal_tolerance:F1}m";
                }
                DashboardText.text =
                    $"<b>{RobotName}</b>   health {HealthStr()}   obst {ObstacleStr()}\n" +
                    $"wp: {wpLine}\n" +
                    $"action: {ActionStr(active)}";
            }

            // Small banner annotations: only while the controller stream is live,
            // so a parked sim shows the stock banner untouched.
            if (SmallCompassText != null)
                SmallCompassText.text = (active && err != null)
                    ? $"yaw err {err.yaw * Mathf.Rad2Deg:+0.0;-0.0}°" : "";
            if (SmallAltText != null)
                SmallAltText.text = (Time.time - obstacleTime < ObstacleStaleSec && obstacleRange >= 0f)
                    ? $"obst {obstacleRange:F1} m" : "";
            if (SmallDepthText != null)
                SmallDepthText.text = (active && err != null && wp != null)
                    ? $"ref {wp.travel_depth:F1}m  err {err.z:+0.00;-0.00}m" : "";
            if (SmallSpeedText != null)
                SmallSpeedText.text = (active && input != null)
                    ? $"rpm {input.thrusterrpm1:F0}  vbs {input.vbs:F0}%" : "";
        }
    }
}
