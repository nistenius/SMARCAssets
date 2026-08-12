using UnityEngine;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.SmarcMission;   // GotoWaypointMsg
using RosMessageTypes.SmarcControl;   // ControlErrorMsg, ControlInputMsg (hand-generated)
using RosMessageTypes.Std;            // Int8Msg, Float32Msg, BoolMsg, StringMsg
using RosMessageTypes.Nav;            // OdometryMsg  (attitude + pose covariance)
using RosMessageTypes.Sensor;         // NavSatFixMsg (GPS fix + accuracy)

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

        [Header("Perception visuals (created at Play — no manual scene wiring)")]
        [Tooltip("Spawn the ProximitySkirt (margin rose around the hull) and the TunnelViewer " +
                 "(planned corridor ahead) if they are not already in the scene. Turn off if " +
                 "you place and configure them by hand.")]
        public bool AutoCreatePerceptionVisuals = true;
        [Tooltip("Name of the vehicle transform the skirt follows.")]
        public string FollowLinkName = "base_link";

        [Header("Attitude in the top bar")]
        [Tooltip("Clone the Compass field twice at Play and insert Roll/Pitch right after it, " +
                 "so the top bar reads Compass | Roll | Pitch | Alt | Depth | Speed with no manual " +
                 "UI work. Turn off if you have wired the fields by hand below.")]
        public bool AutoCreateAttitudeFields = true;
        [Tooltip("Name of the existing top-bar field to clone (must contain the big value text).")]
        public string CompassFieldName = "Compass";

        [Header("Attitude fields (optional — leave empty to show attitude in the dashboard text instead)")]
        [Tooltip("Big roll value in the top bar. Duplicate the Compass field in the scene and assign here.")]
        public TMP_Text RollText;
        [Tooltip("Small setpoint line above roll. Blank while no roll setpoint is published.")]
        public TMP_Text SmallRollText;
        [Tooltip("Big pitch value in the top bar.")]
        public TMP_Text PitchText;
        [Tooltip("Small setpoint line above pitch. Blank while no pitch setpoint is published.")]
        public TMP_Text SmallPitchText;

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
        // Attitude + nav quality, from the ESTIMATOR's own output (dr/odom) and the
        // GPS driver. Deliberately the estimator's belief, not ground truth: this is
        // what the vehicle acts on, and it is the same on hardware.
        float rollDeg, pitchDeg; float attTime = -999f;
        float drSigma = -1f;              // 1-sigma horizontal position uncertainty [m]
        double gpsLat, gpsLon; sbyte gpsFix = -1; float gpsAcc = -1f; float gpsTime = -999f;
        // Roll/pitch setpoints: no controller publishes them today (ctrl/setpoints
        // carries depth,u,yaw only), so these stay blank — the hook is here so the
        // small line lights up automatically once an attitude controller does.
        float spRollDeg, spPitchDeg; float spAttTime = -999f;

        void Start()
        {
            if (AutoCreateAttitudeFields && RollText == null && PitchText == null)
                BuildAttitudeFields();
            if (AutoCreatePerceptionVisuals) BuildPerceptionVisuals();
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
            ros.Subscribe<OdometryMsg>($"{ns}/dr/odom", m =>
            {
                var q = m.pose.pose.orientation;
                // ROS quaternion -> roll/pitch (ENU/FLU, radians -> degrees)
                double sinr = 2.0 * (q.w * q.x + q.y * q.z);
                double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
                rollDeg = (float)(Mathf.Rad2Deg * System.Math.Atan2(sinr, cosr));
                double sinp = 2.0 * (q.w * q.y - q.z * q.x);
                sinp = System.Math.Max(-1.0, System.Math.Min(1.0, sinp));
                pitchDeg = (float)(Mathf.Rad2Deg * System.Math.Asin(sinp));
                attTime = Time.time;
                // pose.covariance is row-major 6x6; [0]=xx, [7]=yy
                var c = m.pose.covariance;
                if (c != null && c.Length >= 8 && c[0] > 0.0 && c[7] > 0.0)
                    drSigma = Mathf.Sqrt((float)(c[0] + c[7]));   // 1-sigma horizontal
            });
            ros.Subscribe<NavSatFixMsg>($"{ns}/core/gps", m =>
            {
                gpsLat = m.latitude; gpsLon = m.longitude;
                gpsFix = m.status.status; gpsTime = Time.time;
                var c = m.position_covariance;
                gpsAcc = (c != null && c.Length >= 5 && c[0] > 0.0)
                    ? Mathf.Sqrt((float)(c[0] + c[4])) : -1f;
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

        /// <summary>Create the two perception visuals if they are absent, so a scene
        /// needs no manual wiring to show them: the skirt is parented to nothing and
        /// follows the vehicle's base_link, the tunnel viewer stays at the scene root
        /// because it anchors the ROS odom frame (parenting it to the vehicle would
        /// make the corridor drag along with the hull — the one wiring mistake that
        /// silently produces a plausible-looking lie).</summary>
        void BuildPerceptionVisuals()
        {
            if (FindFirstObjectByType<WorldSpace.ProximitySkirt>() == null)
            {
                var go = new GameObject("ProximitySkirt");
                var skirt = go.AddComponent<WorldSpace.ProximitySkirt>();
                skirt.RobotName = RobotName;
                skirt.Follow = FindVehicleLink(FollowLinkName);
                if (skirt.Follow == null)
                    Debug.LogWarning($"[VehicleDashboard] '{FollowLinkName}' not found for " +
                                     "the proximity skirt — it will sit at the origin. " +
                                     "Assign Follow by hand or check FollowLinkName.");
            }
            if (FindFirstObjectByType<WorldSpace.TunnelViewer>() == null)
            {
                var go = new GameObject("TunnelViewer");   // scene root on purpose
                go.AddComponent<WorldSpace.TunnelViewer>().RobotName = RobotName;
            }
        }

        /// <summary>Find <RobotName>/.../<linkName>, preferring a link that actually
        /// sits under this robot — a scene can hold several vehicles and a GT twin.</summary>
        Transform FindVehicleLink(string linkName)
        {
            Transform fallback = null;
            foreach (var t in FindObjectsByType<Transform>(FindObjectsSortMode.None))
            {
                if (t.name != linkName) continue;
                fallback ??= t;
                for (var p = t.parent; p != null; p = p.parent)
                    if (p.name == RobotName) return t;      // the right robot's link
            }
            return fallback;
        }

        /// <summary>Insert Roll and Pitch into the top bar by cloning the Compass
        /// field — same background, font, size and layout, sitting right after
        /// Compass, so the bar reads Compass | Roll | Pitch | Alt | Depth | Speed.
        /// Done in code rather than by hand so the fields exist in every scene that
        /// already has a top bar, and so the theme/layout stay in one place.
        /// Falls back silently to the dashboard 'att:' row if the bar cannot be
        /// found — a missing HUD field must never take the data with it.</summary>
        void BuildAttitudeFields()
        {
            var compass = FindFieldByName(CompassFieldName);
            if (compass == null)
            {
                Debug.LogWarning($"[VehicleDashboard] top-bar field '{CompassFieldName}' not " +
                                 "found — attitude will show in the dashboard text instead.");
                return;
            }
            int idx = compass.transform.GetSiblingIndex();
            RollText  = CloneField(compass, "Roll",  "Roll:",  idx + 1, out SmallRollText);
            PitchText = CloneField(compass, "Pitch", "Pitch:", idx + 2, out SmallPitchText);
        }

        GameObject FindFieldByName(string name)
        {
            foreach (var t in FindObjectsByType<RectTransform>(FindObjectsSortMode.None))
                if (t.name == name) return t.gameObject;
            return null;
        }

        /// <summary>Clone one top-bar field. The clone's texts are identified the
        /// same way a human would: the one whose object name matches the wired
        /// SmallCompassText is the small annotation, the one whose text mentions the
        /// source field is the label, whatever remains is the value.</summary>
        TMP_Text CloneField(GameObject src, string objName, string label,
                            int siblingIndex, out TMP_Text smallText)
        {
            smallText = null;
            var go = Instantiate(src, src.transform.parent);
            go.name = objName;
            go.transform.SetSiblingIndex(siblingIndex);

            string smallName = SmallCompassText != null ? SmallCompassText.gameObject.name : null;
            TMP_Text value = null, labelText = null;
            foreach (var t in go.GetComponentsInChildren<TMP_Text>(true))
            {
                if (smallName != null && t.gameObject.name == smallName) { smallText = t; t.text = ""; continue; }
                if (t.text != null && t.text.Contains(CompassFieldName)) { labelText = t; continue; }
                if (value == null) value = t;
            }
            if (labelText != null) labelText.text = label;
            if (value != null) value.text = "--";
            return value;
        }

        /// <summary>Roll/pitch from the estimator, with setpoints when a controller
        /// publishes them. Amber past 20 deg: SAM is hydrobatic, but in a confined
        /// dock a large attitude means the sonar fan is pointing somewhere other
        /// than where the margin belief assumes.</summary>
        string AttStr()
        {
            if (Time.time - attTime > 3f) return "<color=#888888>no data</color>";
            string col = (Mathf.Abs(rollDeg) > 20f || Mathf.Abs(pitchDeg) > 20f)
                ? "#FFB300" : "#3DDC6B";
            string sp = Time.time - spAttTime < ActionStaleSec
                ? $"   <size=80%>set r{spRollDeg:F0}° p{spPitchDeg:F0}°</size>" : "";
            return $"<color={col}>roll {rollDeg:+0.0;-0.0}°   pitch {pitchDeg:+0.0;-0.0}°</color>{sp}";
        }

        /// <summary>GPS fix quality per sensor_msgs/NavSatStatus, coloured like the
        /// obstacle field: green good, amber degraded, grey/none.</summary>
        string GpsStr()
        {
            if (Time.time - gpsTime > 5f) return "<color=#888888>GPS no data</color>";
            string fix = gpsFix switch
            {
                2 => "RTK",       // STATUS_GBAS_FIX
                1 => "SBAS",      // STATUS_SBAS_FIX
                0 => "fix",       // STATUS_FIX
                _ => "NO FIX",    // -1 STATUS_NO_FIX (normal underwater)
            };
            string acc = gpsAcc >= 0f ? $" ±{gpsAcc:F1} m" : "";
            string pos = gpsFix >= 0 ? $" {gpsLat:F6},{gpsLon:F6}" : "";
            string col = gpsFix >= 1 ? "#3DDC6B" : gpsFix == 0 ? "#FFB300" : "#888888";
            return $"<color={col}>{fix}{acc}</color>{pos}";
        }

        /// <summary>The estimator's OWN uncertainty (dr/odom pose covariance), not a
        /// ground-truth comparison — so it reads the same in sim and on hardware.
        /// Amber past 1 m, red past 2.5 m: that is where Part V's margins start to
        /// be eaten by navigation error rather than by geometry.</summary>
        string DrStr()
        {
            if (Time.time - attTime > 3f) return "<color=#888888>DR no data</color>";
            if (drSigma < 0f) return "<color=#888888>DR σ n/a</color>";
            string col = drSigma > 2.5f ? "#FF3B30" : drSigma > 1.0f ? "#FFB300" : "#3DDC6B";
            return $"DR <color={col}>σ {drSigma:F2} m</color>";
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
                // Attitude: its own row while the top-bar Roll/Pitch fields are not
                // assigned in the scene. It prints "no data" rather than vanishing —
                // a field that disappears looks like a missing feature, when in fact
                // it means the estimator is down (2026-08-12: exactly that confusion).
                string attRow = (RollText == null && PitchText == null)
                    ? "att: " + AttStr() + "\n" : "";
                DashboardText.text =
                    $"<b>{RobotName}</b>   health {HealthStr()}   obst {ObstacleStr()}\n" +
                    $"wp: {wpLine}\n" +
                    attRow +
                    $"nav: {GpsStr()}   {DrStr()}\n" +
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

            // Roll / pitch in the top bar, same pattern: measured big, setpoint small.
            bool haveAtt = Time.time - attTime < 3f;
            bool haveSpAtt = Time.time - spAttTime < ActionStaleSec;
            if (RollText != null)  RollText.text  = haveAtt ? $"{rollDeg:+0.0;-0.0}°" : "--";
            if (PitchText != null) PitchText.text = haveAtt ? $"{pitchDeg:+0.0;-0.0}°" : "--";
            if (SmallRollText != null)
                SmallRollText.text = haveSpAtt ? $"set {spRollDeg:F1}°" : "";
            if (SmallPitchText != null)
                SmallPitchText.text = haveSpAtt ? $"set {spPitchDeg:F1}°" : "";
        }
    }
}
