using UnityEngine;
using UnityEngine.UI;                 // LayoutElement, VerticalLayoutGroup, ContentSizeFitter
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.SmarcMission;   // GotoWaypointMsg
using RosMessageTypes.SmarcControl;   // ControlErrorMsg, ControlInputMsg (hand-generated)
using RosMessageTypes.Std;            // Int8Msg, Float32Msg, BoolMsg, StringMsg
using RosMessageTypes.Nav;            // OdometryMsg  (attitude + pose covariance)
using RosMessageTypes.Sensor;         // NavSatFixMsg (GPS fix + accuracy)
using RosMessageTypes.Smarc;          // PercentStampedMsg (VBS fill, for the action line)

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
    ///
    /// v3 (2026-08-12, SLAM session 2): a PERCEPTION row, and the rule behind it —
    /// no field on this dashboard may render an unverified claim in green. The
    /// detector, the margin rose and the speed governor each publish their own
    /// health state; healthy, degraded and unknown are three different colours.
    /// The day this was written, "obst clear" sat green for half a day while the
    /// detector received nothing at all.
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

        [Header("Panel sizing")]
        [Tooltip("Size the dark panel to whatever the text currently is, instead of the " +
                 "hardcoded rectangle DashboardBuilder wrote. Every row added to this " +
                 "dashboard has silently overflowed the panel until someone noticed the " +
                 "text sitting on bare scene; this makes that impossible. Turn off only " +
                 "if you are laying the panel out by hand.")]
        public bool AutoFitPanel = true;
        [Tooltip("Panel never narrower than this, so a quiet dashboard keeps its shape.")]
        public float MinPanelWidth = 560f;
        [Tooltip("Longest health 'detail' string shown before it is elided. The panel now " +
                 "grows to its widest line, so an unbounded detail string would stretch it " +
                 "across the viewport.")]
        public int DetailMaxChars = 38;

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

        [Tooltip("Seconds without a perception health message before the health line " +
                 "reports the health stream itself as dead.")]
        public float HealthStaleSec = 4.0f;

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
        // Perception health (2026-08-12 session 2). The whole point of these three
        // fields is that "the detector says clear" and "the detector cannot see"
        // produced IDENTICAL cockpits all day on 2026-08-12. They no longer do:
        // ObstacleStr() refuses to paint green unless detHealth says OK.
        // Payload: STATE|rate_hz|age_s|a|b|detail (detector and rose both).
        string detHealth = ""; float detHealthTime = -999f;
        string roseHealth = ""; float roseHealthTime = -999f;
        // Governor (HT3): "cap|applied|state" from the blend controller.
        string govLine = ""; float govTime = -999f;
        // Narrative inputs (2026-08-12, Ivan: "I liked the more action what's
        // happening type of info"). Depth comes from dr/odom's own z, so the
        // action line and the nav line never disagree about where the vehicle is.
        float vbs = -1f; float vbsTime = -999f;
        float depthNow, depthPrev; float depthPrevTime = -999f, depthRate;

        void Start()
        {
            if (AutoFitPanel) FitPanelToText();
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
            ros.Subscribe<StringMsg>($"{ns}/perception/obstacle/health", m => { detHealth = m.data; detHealthTime = Time.time; });
            ros.Subscribe<StringMsg>($"{ns}/perception/rose_health", m => { roseHealth = m.data; roseHealthTime = Time.time; });
            ros.Subscribe<StringMsg>($"{ns}/ctrl/governor", m => { govLine = m.data; govTime = Time.time; });
            // VBS fill drives the narrative after a mission ends: the BT empties the
            // tank and the vehicle floats up. Reading the actuator makes "emptying
            // tank / floating to surface" an OBSERVATION rather than a guess.
            ros.Subscribe<PercentStampedMsg>($"{ns}/core/vbs_fb", m => { vbs = m.value; vbsTime = Time.time; });
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
                // Depth + its rate, for the post-mission narrative. Low-passed over
                // ~1 s: the raw difference between consecutive odom samples is noise
                // at these speeds and would make the line flicker between "rising"
                // and "sinking" several times a second.
                depthNow = -(float)m.pose.pose.position.z;
                if (Time.time - depthPrevTime > 1f)
                {
                    if (depthPrevTime > 0f)
                        depthRate = Mathf.Lerp(depthRate,
                            (depthNow - depthPrev) / (Time.time - depthPrevTime), 0.5f);
                    depthPrev = depthNow; depthPrevTime = Time.time;
                }
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

        /// <summary>State word out of a "STATE|a|b|c|d|detail" health payload.</summary>
        static string HealthState(string payload)
        {
            if (string.IsNullOrEmpty(payload)) return "";
            int bar = payload.IndexOf('|');
            return bar < 0 ? payload : payload.Substring(0, bar);
        }

        static string HealthField(string payload, int i)
        {
            if (string.IsNullOrEmpty(payload)) return "";
            var p = payload.Split('|');
            return i < p.Length ? p[i] : "";
        }

        /// <summary>Detail strings come from the nodes and are free-form; the panel now
        /// sizes to its widest line, so an unbounded one would stretch it off-screen.</summary>
        string Elide(string s) =>
            string.IsNullOrEmpty(s) || s.Length <= DetailMaxChars
                ? s : s.Substring(0, DetailMaxChars - 1) + "…";

        /// <summary>The detector's own verdict on whether its range output means
        /// anything. "" = the health topic is absent or stale, which is NOT the
        /// same as healthy and must never be rendered as such.</summary>
        string DetState() =>
            Time.time - detHealthTime > HealthStaleSec ? "" : HealthState(detHealth);

        string ObstacleStr()
        {
            if (Time.time - obstacleTime > ObstacleStaleSec) return "<b><color=#FF3B30>NO DATA</color></b>";
            string range = obstacleRange >= 0f ? $"{obstacleRange:F1} m" : "clear";
            // Colours are read against bright photogrammetry, not a dark scene: the old
            // #D9534F was too dark/desaturated to pick out. Brighter + bold reads at a glance.
            if (obstacleStop) return $"<b><color=#FF3B30>STOP  {range}</color></b>";
            if (obstacleRange >= 0f && obstacleRange < 6f) return $"<b><color=#FFB300>{range}</color></b>";
            // "clear" is a CLAIM, and on 2026-08-12 it was false for most of a day
            // while this field sat green: the detector was deaf, gating discarded
            // every point, and -1 ("nothing in the gated volume") rendered exactly
            // like an open leg. Green now requires the detector to certify it can
            // see; anything else is amber with the reason on the perception row.
            string st = DetState();
            if (st == "OK") return $"<color=#3DDC6B>{range}</color>";
            if (st == "") return $"<color=#FFB300>{range} <size=80%>(unverified)</size></color>";
            return $"<b><color=#FF3B30>{range} ({st})</color></b>";
        }

        /// <summary>The row that did not exist on 2026-08-12 and cost that whole day:
        /// is perception ALIVE, and does the belief feeding the governor know anything.
        /// Reads at a glance — green only when both streams certify themselves.</summary>
        string PerceptionStr()
        {
            string detTxt;
            if (Time.time - detHealthTime > HealthStaleSec)
                // No health topic at all: an old detector build, or a node that died.
                // Amber, never grey — grey is what everyone learned to ignore.
                detTxt = "<b><color=#FFB300>det UNREPORTED</color></b>";
            else
            {
                string st = HealthState(detHealth);
                string hz = HealthField(detHealth, 1);
                detTxt = st == "OK"
                    ? $"<color=#3DDC6B>det {hz} Hz</color>"
                    : $"<b><color=#FF3B30>det {st}</color></b> <size=80%>{Elide(HealthField(detHealth, 5))}</size>";
            }

            string roseTxt = "";
            if (Time.time - roseHealthTime <= HealthStaleSec)
            {
                string st = HealthState(roseHealth);
                if (st == "OFF") roseTxt = "   <color=#888888>rose off</color>";
                else if (st == "OK")
                {
                    // coverage is the honest headline: the fan knows a MINORITY of
                    // sectors (HT1 measured 14 %), and the governor is only as good
                    // as that number. Showing it keeps the limitation in the cockpit.
                    float cov = 0f;
                    float.TryParse(HealthField(roseHealth, 3),
                        System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture, out cov);
                    roseTxt = $"   <color=#3DDC6B>rose {cov * 100f:F0}% known</color>";
                }
                else roseTxt = $"   <b><color=#FF3B30>rose {st}</color></b>";
            }

            string govTxt = "";
            if (Time.time - govTime <= HealthStaleSec && !string.IsNullOrEmpty(govLine))
            {
                string gst = HealthField(govLine, 2);
                string applied = HealthField(govLine, 1);
                // OFF and NO_CAP are CONFIGURATION, not faults — the governor is
                // launch-gated and off by default, so red there cries wolf on the
                // normal case and devalues red everywhere else on this dashboard.
                // Grey = deliberately not running. Red is reserved for STALE, which
                // means a belief we were trusting died mid-mission.
                string col = gst switch
                {
                    "CAPPING" => "#FFB300",
                    "OK"      => "#3DDC6B",
                    "OFF"     => "#888888",
                    "NO_CAP"  => "#888888",
                    _         => "#FF3B30",
                };
                govTxt = $"   gov <color={col}>{applied} m/s {gst}</color>";
            }
            return detTxt + roseTxt + govTxt;
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
            bool driving = Time.time - spTime < ActionStaleSec;   // controller is commanding
            bool haveVbs = Time.time - vbsTime < 5f;

            // --- under way -----------------------------------------------------
            if (active && err != null)
            {
                string cap = "";
                // The governor is part of the narrative: "why am I going slowly"
                // is the first question a creeping vehicle raises.
                if (Time.time - govTime < ActionStaleSec && HealthField(govLine, 2) == "CAPPING")
                    cap = $" — <color=#FFB300>creeping at {HealthField(govLine, 1)} m/s"
                        + " (margin governor)</color>";
                return $"<color=#3DDC6B>driving to wp</color> — {err.distance:F1} m to go{cap}";
            }
            // ctrl/conv/error missing but the controller is plainly commanding.
            // Do not print "idle" — that claim was false for an entire flight on
            // 2026-08-12, at 0.48 m/s and 1.2 m depth.
            if (driving)
                return $"<color=#3DDC6B>driving</color> (depth {depthNow:F1} m, "
                     + $"set {spDepth:F1} m / {spSurge:F2} m/s) — "
                     + "<color=#FFB300>no ctrl/conv/error, progress unknown</color>";

            // --- mission over: what is it doing about it? ----------------------
            // The BT empties the VBS and the vehicle floats up. Both are readable,
            // so this is reporting rather than guessing.
            if (haveVbs && vbs < 25f && depthNow > 0.4f)
                return $"<color=#FFB300>mission ended</color> — VBS {vbs:F0} % (emptying), "
                     + $"floating up from {depthNow:F1} m at {-depthRate:F2} m/s";
            if (depthNow > 0.4f && depthRate < -0.02f)
                return $"<color=#FFB300>mission ended</color> — floating to surface, "
                     + $"{depthNow:F1} m and rising";
            if (depthNow > 0.4f)
                return $"<color=#888888>mission ended</color> — holding at {depthNow:F1} m"
                     + (haveVbs ? $", VBS {vbs:F0} %" : "");
            return "<color=#888888>surfaced, idle — waiting for a mission</color>"
                 + (haveVbs ? $"  <size=80%>VBS {vbs:F0} %</size>" : "");
        }

        /// <summary>Make the dark panel track the text instead of a fixed rectangle.
        ///
        /// DashboardBuilder writes the panel as a hardcoded 560x78 with the text
        /// stretched inside it. That is correct exactly until someone adds a row —
        /// and every row added so far (attitude, nav, now perception) has spilled
        /// out onto the bare scene, where white-on-photogrammetry is unreadable.
        /// A ContentSizeFitter over a VerticalLayoutGroup removes the class of bug
        /// rather than re-tuning the constant.
        ///
        /// Done at runtime, not only in the editor script, so existing scenes are
        /// fixed without anyone re-running a menu item — the same self-installing
        /// pattern as the attitude fields and the perception visuals.</summary>
        void FitPanelToText()
        {
            if (DashboardText == null) return;
            var panel = GetComponent<RectTransform>();
            if (panel == null || DashboardText.transform.parent != transform) return;

            // The text must not be anchor-stretched to the panel, or the panel's
            // preferred size depends on the text's size which depends on the
            // panel's — a layout cycle Unity resolves by collapsing to nothing.
            var trt = DashboardText.rectTransform;
            trt.anchorMin = trt.anchorMax = new Vector2(0f, 1f);
            trt.pivot = new Vector2(0f, 1f);

            // No wrapping setting needed: TMP reports preferredWidth as the UNWRAPPED
            // width, so the fitter always gives the panel room for the longest line —
            // which is what "the background spans the text" means. (Deliberately not
            // touching enableWordWrapping, which is [Obsolete] in newer TMP and would
            // put a warning in the Console right where we tell people to check for
            // compile errors before Play.)
            var le = DashboardText.GetComponent<LayoutElement>();
            if (le == null) le = DashboardText.gameObject.AddComponent<LayoutElement>();
            le.minWidth = MinPanelWidth;

            var vlg = GetComponent<VerticalLayoutGroup>();
            if (vlg == null) vlg = gameObject.AddComponent<VerticalLayoutGroup>();
            vlg.padding = new RectOffset(10, 10, 6, 6);
            vlg.childAlignment = TextAnchor.UpperLeft;
            vlg.childControlWidth = true;  vlg.childControlHeight = true;
            vlg.childForceExpandWidth = false; vlg.childForceExpandHeight = false;

            var fitter = GetComponent<ContentSizeFitter>();
            if (fitter == null) fitter = gameObject.AddComponent<ContentSizeFitter>();
            fitter.horizontalFit = ContentSizeFitter.FitMode.PreferredSize;
            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;
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
                    $"perc: {PerceptionStr()}\n" +
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
