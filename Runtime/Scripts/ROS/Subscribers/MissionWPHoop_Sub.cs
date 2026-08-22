using System.Collections.Generic;
using UnityEngine;

using RosMessageTypes.SmarcMission; // GotoWaypointMsg
using GeoRef;                        // GlobalReferencePoint
using SmarcGUI.WorldSpace;           // WaypointHoop

using ROS.Core;

namespace ROS.Subscribers
{
    /// <summary>
    /// Shows the vehicle's waypoints (mission/last_wp, republished by the dive action
    /// server on every accepted goal) as "hula hoops": diameter = 2 x goal tolerance,
    /// positioned at the WP's lat/lon/depth, opening facing the approach.
    ///
    /// EVERY WAYPOINT STAYS ON SCREEN (Ivan, 2026-08-18). This used to keep a single hoop
    /// and teleport it to each new goal, so exactly one waypoint existed at any moment and
    /// the flown path could never be seen as a whole — you could not even park the camera
    /// on a hoop to check whether it renders underwater, because by the time you looked the
    /// hoop had moved on. Now each waypoint gets its own hoop and they persist: the current
    /// one is bright, the ones already passed are dimmed, and the finished mission is still
    /// there to inspect afterwards.
    ///
    /// HONEST LIMITATION: this topic carries the CURRENT waypoint only, so hoops appear one
    /// at a time as the vehicle is cleared to fly each leg — this is "the whole plan so far",
    /// not "the whole plan in advance". Drawing the plan before it flies needs the plan
    /// itself, which lives in Mission Control, not on this topic. Deliberately not faked
    /// here: a display that guessed the remaining waypoints would be showing something the
    /// vehicle has not agreed to.
    ///
    /// Standalone scene object by design: drag the MissionWPHoop prefab into a scene
    /// and set RobotName to choose whose mission to mirror. The display is
    /// subscriber-driven from the VM's own plan — whatever mission the vehicle is
    /// really flying is what gets visualized (design direction: Ivan, 2026-08-09).
    ///
    /// ==================================================================================
    /// DEFECT B, 2026-08-21: THE DISPLAY NEVER HELD MORE THAN ONE HOOP.
    /// ==================================================================================
    /// Ivan's 643 s take showed exactly one hoop, and the director's overlay then sat on
    /// "'ApproachingHoop' never happened … last filmed 1" until it fell through on its ceiling.
    /// It looked like the list had EMPTIED mid-mission. It had not: it never grew.
    ///
    /// ROOT CAUSE, traced through the wire and not guessed. `WaypointId()` preferred
    /// `GotoWaypointMsg.name`, and that field is a CONSTANT for every waypoint of every plan:
    ///   * `ActionServerDiveSub.goal_callback` sets `wp_msg.name = fmt_dict.get("name", "wp")`;
    ///   * `fmt_dict` is the TST task's `params` dict — `BtActionClient` sends
    ///     `get_current_task_params()`, and Mission Control's `AUVDepthMoveToTask.toJson()`
    ///     writes `"params": {"waypoint": {...}}` and nothing else. There is no `name` key in it.
    ///   * So every waypoint arrives named **"wp"**.
    /// The consequence, line by line: waypoint 1 creates hoop 0 with id "wp"; waypoint 2 finds
    /// `hoopIds.IndexOf("wp") == 0`, the restart test `existing == 0 && hoops.Count > 1` is FALSE
    /// (there is only one hoop), so it took the `Highlight(0); return;` branch — and every
    /// subsequent waypoint did the same, forever. One hoop, parked on waypoint 1, for the whole
    /// mission.
    ///
    /// THE FIX: identity is `name @ rounded position`, so a plan whose waypoints share one name is
    /// still a plan of distinct waypoints, while a RE-SENT goal (same name, same place) still
    /// matches itself and does not stack a duplicate. The restart test additionally requires that
    /// the display had actually MOVED PAST waypoint 0, so a re-publish of the first goal while it
    /// is still the current one can never wipe the list. And every clear now carries a reason,
    /// logged and handed to `OnHoopsCleared`, because the sonar map clears with it: a display and a
    /// map that both vanish mid-take must say who did it.
    ///
    /// FOR THE RECORD, since it was suspected and is now excluded: `ClearHoops` was NEVER reached
    /// during that take (it needs `hoops.Count > 1`, which never happened), so it did not clear the
    /// sonar map either. Defect A and defect B are independent.
    /// </summary>
    [AddComponentMenu("Smarc/ROS/MissionWPHoop_Sub")]
    public class MissionWPHoop_Sub : ROSBehaviour
    {
        [Tooltip("Robot whose mission to mirror. Used to namespace the topic (/<RobotName>/<topic>) and, if the robot exists in the scene, to orient the first hoop toward it.")]
        public string RobotName = "sam_auv_v1";

        [Tooltip("Visual thickness of the hoop tube.")]
        public float TubeRadius = 0.1f;

        // See WaypointHoop.Collidable: OFF by default since 2026-08-18, because with the whole
        // plan on screen this would put a sonar target at every waypoint simultaneously.
        [Tooltip("Hoops get MeshColliders: visible to the 3D sonar, and physical gates. Leave OFF unless deliberately testing obstacle avoidance.")]
        public bool Collidable = false;

        [Tooltip("Colour of the waypoint currently being flown.")]
        public Color CurrentColor = new Color(1f, 0.6f, 0f, 1f);   // orange

        [Tooltip("Colour of waypoints already passed, so the current target stays readable.")]
        public Color PassedColor = new Color(0.25f, 0.5f, 0.7f, 1f); // muted blue

        [Tooltip("Safety cap on hoop count. A long lawnmower plan is thousands of legs; past this the oldest hoops are recycled rather than growing without bound.")]
        public int MaxHoops = 256;

        [Header("Hoop rendering — the §3o underwater defect")]
        [Tooltip("Material asset every hoop is made from. ASSIGN THIS: SMARC/Video/1 creates WaypointHoop.mat. Empty means each hoop falls back to Shader.Find, which is the path that returns null in builds and produces an HDRP material with no keywords.")]
        public Material HoopMaterial;
        [Tooltip("Light the hoops from inside. Required below the surface in the baltic water preset: at ~6 m absorption a non-emissive hoop 10 m away is absorbed to nothing whatever the shader does.")]
        public bool UseEmissive = true;
        [Tooltip("Emissive multiplier for the hoops. 3-6 reads well through murky water.")]
        public float EmissiveIntensity = 4f;
        [Tooltip("Force the hoop material's render queue (-1 = leave the material's own). Rung 3 of the fix ladder; try 3000 if the hoops are being sorted behind the water surface.")]
        public int HoopRenderQueue = -1;
        [Tooltip("Each hoop prints its material path, shader, queue, position and MeshRenderer.isVisible one frame after it is built. Turn on when a hoop does not appear; off for a take.")]
        public bool LogHoopDiagnostics = false;

        /// <summary>
        /// The hoops on screen, in the order the vehicle was cleared to fly them. Read-only, and
        /// used by the CinematicDirector to park a camera on the waypoint the vehicle is coming
        /// toward — which is also, finally, the way to look at a submerged hoop for as long as it
        /// takes to decide whether it renders.
        /// </summary>
        public System.Collections.Generic.IReadOnlyList<WaypointHoop> Hoops => hoops;

        /// <summary>
        /// Raised when the display is cleared, CARRYING THE REASON. The sonar map accumulator
        /// listens to this so a re-flown mission does not draw its cloud on top of the previous
        /// run's — which means an unexplained clear takes the map with it, on camera. Hence the
        /// string: every clear names itself.
        /// </summary>
        public event System.Action<string> OnHoopsCleared;

        /// <summary>Why the display last cleared, for the director's overlay and the Console.</summary>
        public string LastClearReason { get; private set; } = "(never cleared)";

        GlobalReferencePoint globalRef;
        readonly List<WaypointHoop> hoops = new List<WaypointHoop>();
        readonly List<string> hoopIds = new List<string>();
        Vector3 prevCenter;
        bool hasPrev = false;
        bool subscribed = false;
        bool cappedWarned = false;
        int currentIndex = -1;          // the hoop last highlighted; -1 before the first waypoint
        int waypointsReceived = 0;
        string firstSeenName = null;    // to notice, once, that every waypoint carries one name
        bool sharedNameNoted = false;

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

        WaypointHoop NewHoop(int index)
        {
            var hoopGO = new GameObject($"{RobotName}_WPHoop_{index}");
            // Created INACTIVE so the settings below land before WaypointHoop.Awake builds its
            // material. AddComponent runs Awake immediately on an active object, and a hoop that
            // built its material one line before being handed one is exactly how the serialized
            // material would have looked "assigned but ignored".
            hoopGO.SetActive(false);
            hoopGO.transform.SetParent(transform, false); // under this scene object, world-anchored
            var h = hoopGO.AddComponent<WaypointHoop>();
            h.TubeRadius = TubeRadius;
            h.Collidable = Collidable;
            h.HoopMaterial = HoopMaterial;
            h.UseEmissive = UseEmissive;
            h.EmissiveIntensity = EmissiveIntensity;
            h.RenderQueueOverride = HoopRenderQueue;
            h.LogRenderDiagnostics = LogHoopDiagnostics;
            hoopGO.SetActive(true);
            return h;
        }

        /// <summary>
        /// Drop every hoop. Called when a plan restarts, and available to the GUI.
        /// IT ALWAYS NAMES WHY: the sonar map clears with the hoops, so an anonymous clear is a
        /// visual that disappears mid-take with nothing anywhere saying what did it.
        /// </summary>
        public void ClearHoops(string reason)
        {
            int had = hoops.Count;
            foreach (var h in hoops)
                if (h != null) Destroy(h.gameObject);
            hoops.Clear();
            hoopIds.Clear();
            hasPrev = false;
            cappedWarned = false;
            currentIndex = -1;
            LastClearReason = reason;
            Debug.Log($"[{transform.name}] waypoint display CLEARED ({had} hoop(s) destroyed): {reason}");
            OnHoopsCleared?.Invoke(reason);
        }

        /// <summary>Kept for anything that called the old no-argument form.</summary>
        public void ClearHoops() => ClearHoops("cleared by hand (no reason given)");

        /// <summary>
        /// Identity of a waypoint, used to notice a re-sent goal and a plan starting over.
        ///
        /// IT IS NAME **AND** POSITION, NOT NAME ALONE (2026-08-21, defect B). Every waypoint the
        /// vehicle publishes is called "wp": `ActionServerDiveSub` writes
        /// `fmt_dict.get("name", "wp")` and the task `params` Mission Control sends contains only
        /// `waypoint`, never a `name`. Trusting that field made the whole plan one identity and the
        /// display never held more than one hoop. Position at 0.1 m keeps a RE-SENT goal identical
        /// to itself — the action server republishes the same goal while it is being flown — while
        /// keeping genuinely different waypoints apart. If the vehicle ever starts sending real
        /// per-waypoint names, this still works: the name is the leading term.
        /// </summary>
        static string WaypointId(GotoWaypointMsg msg, Vector3 center)
        {
            string n = string.IsNullOrEmpty(msg.name) ? "-" : msg.name;
            return $"{n}@{center.x:F1},{center.y:F1},{center.z:F1}";
        }

        Vector3 RobotPosition()
        {
            var robotGO = GameObject.Find(RobotName);
            if (robotGO != null) return robotGO.transform.position;
            return transform.position;
        }

        void OnWaypoint(GotoWaypointMsg msg)
        {
            var center = new Vector3(0, 0, 0);
            (center.x, center.z) = globalRef.GetUnityXZFromLatLon(msg.lat, msg.lon);
            center.y = (float)-msg.travel_depth;

            var id = WaypointId(msg, center);
            waypointsReceived++;
            NoteSharedNameOnce(msg);

            // The action server re-publishes the SAME goal while it is being flown, so an id we
            // already hold is normally just that: keep the existing hoop, do not stack duplicates.
            int existing = hoopIds.IndexOf(id);
            if (existing >= 0)
            {
                // ...unless it is the FIRST waypoint arriving again AFTER THE DISPLAY HAS MOVED
                // PAST IT, which means the plan restarted. Three conditions, all required:
                //   * it is index 0 — a lawnmower legitimately revisits interior points, and
                //     clearing on any repeat would wipe the display mid-mission;
                //   * more than one hoop exists — one hoop is not a plan to restart;
                //   * the CURRENT hoop is not still index 0 — this is the one added on 2026-08-21.
                //     Without it, any re-publish of the first goal while the vehicle is still
                //     flying leg 1 counts as a restart, and the two zig-zag legs at Beckholmen are
                //     short enough that the first goal is re-sent many times.
                if (existing == 0 && hoops.Count > 1 && currentIndex > 0)
                {
                    ClearHoops($"waypoint 1 of the plan arrived again ('{id}') after the display had " +
                               $"reached waypoint {currentIndex + 1} of {hoops.Count} — reading that as a NEW MISSION");
                }
                else
                {
                    Highlight(existing);
                    return;
                }
            }

            WaypointHoop hoop;
            if (hoops.Count >= MaxHoops)
            {
                // Recycle the oldest rather than grow without bound. Say so once: a display that
                // silently stops showing the start of a long plan is a display that lies.
                if (!cappedWarned)
                {
                    Debug.LogWarning($"[{transform.name}] {MaxHoops} waypoint hoops reached — " +
                                     "recycling the oldest. Raise MaxHoops to keep the whole plan.");
                    cappedWarned = true;
                }
                hoop = hoops[0];
                hoops.RemoveAt(0);
                hoopIds.RemoveAt(0);
                if (currentIndex >= 0) currentIndex--;   // every index shifted down by one
                hoop.transform.SetAsLastSibling();
            }
            else
            {
                hoop = NewHoop(hoops.Count);
            }

            hoop.gameObject.SetActive(true);
            hoop.transform.position = center;
            hoop.SetRadius((float)msg.goal_tolerance);

            // Face the hoop along the approach: previous WP -> this WP if we have one
            // (the actual leg), otherwise robot -> WP (the transit to the first WP).
            Vector3 dir = hasPrev ? center - prevCenter : center - RobotPosition();
            dir.y = 0;
            if (dir.sqrMagnitude > 1e-6f) hoop.SetDirection(dir);

            hoops.Add(hoop);
            hoopIds.Add(id);
            Highlight(hoops.Count - 1);

            if (!hasPrev || (center - prevCenter).sqrMagnitude > 0.25f)
            {
                prevCenter = center;
                hasPrev = true;
            }
        }

        /// <summary>Exactly one hoop is the current target; the rest are dimmed but still there.</summary>
        void Highlight(int index)
        {
            currentIndex = index;
            for (int i = 0; i < hoops.Count; i++)
                if (hoops[i] != null)
                    hoops[i].SetColor(i == index ? CurrentColor : PassedColor);
        }

        /// <summary>
        /// Say ONCE, out loud, that the vehicle is naming every waypoint the same thing. It is not
        /// an error — the wire format simply has no per-waypoint name — but it is the fact that
        /// made a name-only identity collapse the whole plan into one hoop, and the next person
        /// looking at this file should not have to re-derive it from three repositories.
        /// </summary>
        void NoteSharedNameOnce(GotoWaypointMsg msg)
        {
            if (sharedNameNoted) return;
            string n = string.IsNullOrEmpty(msg.name) ? "-" : msg.name;
            if (firstSeenName == null) { firstSeenName = n; return; }
            if (n != firstSeenName) { sharedNameNoted = true; return; }   // real per-WP names: nothing to say
            if (waypointsReceived < 3) return;
            sharedNameNoted = true;
            Debug.Log($"[{transform.name}] every waypoint so far is named '{n}' — the vehicle's " +
                      "mission/last_wp carries a constant name (ActionServerDiveSub: " +
                      "fmt_dict.get(\"name\", \"wp\"); MC's task params have no name field). Waypoint " +
                      "identity here is therefore name AND position, which is what lets the plan " +
                      "grow past one hoop. Nothing is wrong; this line exists so nobody re-derives it.");
        }
    }
}
