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
        // An abort is a MISSION-ENDING event, not a momentary condition, so it latches.
        // ctrl/obstacle_status announces ABORT and then either goes quiet or reverts to STOP
        // (the obstacle is usually still in front of us — a boat on the dock floor, 2026-08-11).
        // Reading it through a 5 s freshness window therefore dropped the banner back to
        // "OBSTACLE STOP — holding depth", which says the vehicle is still flying the mission
        // and waiting for clearance. It is not: the BT has ended and nothing is coming.
        bool aborted; float abortedTime = -999f;
        // Live setpoints from ctrl/setpoints ("depth,u,yaw_deg").
        float spDepth, spSurge, spYawDeg; float spTime = -999f;

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
            ros.Subscribe<StringMsg>($"{ns}/ctrl/obstacle_status", m =>
            {
                obstacleStatus = m.data; obstacleStatusTime = Time.time;
                if (m.data.StartsWith("ABORT")) { aborted = true; abortedTime = Time.time; }
            });
            ros.Subscribe<StringMsg>($"{ns}/ctrl/setpoints", m =>
            {
                var p = m.data.Split(',');
                if (p.Length == 3
                    && float.TryParse(p[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out spDepth)
                    && float.TryParse(p[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out spSurge)
                    && float.TryParse(p[2], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out spYawDeg))
                    spTime = Time.time;
            });
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
            // Colours are read against bright photogrammetry, not a dark scene: the old
            // #D9534F was too dark/desaturated to pick out. Brighter + bold reads at a glance.
            if (obstacleStop) return $"<b><color=#FF3B30>STOP  {range}</color></b>";
            if (obstacleRange >= 0f && obstacleRange < 6f) return $"<b><color=#FFB300>{range}</color></b>";
            return $"<color=#3DDC6B>{range}</color>";
        }

        string ActionStr(bool active)
        {
            bool haveStatus = Time.time - obstacleStatusTime < 5f;
            // Latched abort wins over everything: the mission is over regardless of what the
            // detector still reports. Which PHASE of the shutdown we are in is read from the
            // controller itself rather than a hardcoded timer — while ctrl/setpoints is still
            // live the controller is driving the surfacing; once it goes quiet, nothing is.
            if (aborted)
                return Time.time - spTime < ActionStaleSec
                    ? "<b><color=#FF3B30>ABORTED (obstacle)</color></b> — mission ended, surfacing (VBS empty)"
                    : "<b><color=#FF3B30>ABORTED (obstacle)</color></b> — mission ended, idle. "
                      + "Re-arm with reset_mission_state.sh";
            if (haveStatus && obstacleStatus.StartsWith("STOP"))
                // e.g. "STOP 1/3 abort in 12s"
                return $"<b><color=#FF3B30>OBSTACLE {obstacleStatus}</color></b> — holding depth, waiting for clearance";
            if (!haveStatus && obstacleStop)
                return "<b><color=#FF3B30>OBSTACLE STOP</color></b> — holding depth";
            if (active && err != null)
                return $"<color=#3DDC6B>driving to wp</color> — {err.distance:F1} m to go";
            return "<color=#888888>idle — waiting for a mission</color>";
        }

        // Controller yaw refs run in (-180, 180]; the banner compass reads 0-360.
        static float NormDeg(float d)
        {
            d %= 360f;
            if (d < 0f) d += 360f;
            return d;
        }

        void Update()
        {
            bool active = Time.time - errTime < ActionStaleSec;
            // Clear the latch only when a NEW mission is actually driving: control errors keep
            // arriving for a moment during the abort itself, so require them to be well after it.
            if (aborted && errTime > abortedTime + 3f) aborted = false;

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

            // Small banner annotations = the controller's LIVE SETPOINTS, shown
            // above the measured value in each field. Blank when no controller is
            // running, so a parked sim shows the stock banner untouched.
            bool haveSp = Time.time - spTime < ActionStaleSec;
            if (SmallCompassText != null)
                SmallCompassText.text = haveSp ? $"set {NormDeg(spYawDeg):F1}°" : "";
            if (SmallAltText != null)
                SmallAltText.text = (Time.time - obstacleTime < ObstacleStaleSec && obstacleRange >= 0f)
                    ? $"obst {obstacleRange:F1} m" : "";
            if (SmallDepthText != null)
                SmallDepthText.text = haveSp ? $"set {spDepth:F1} m" : "";
            if (SmallSpeedText != null)
                SmallSpeedText.text = haveSp ? $"set {spSurge:F2} m/s" : "";
        }
    }
}
