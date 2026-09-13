using System;
using UnityEngine;

namespace Smarc.Cinematics
{
    /// <summary>
    /// One shot in a CinematicDirector shot list: a camera behaviour, WHEN IT HANDS OVER, and what
    /// the rest of the scene should be doing while it runs (HUD, particles, sonar map).
    ///
    /// A SERIALIZABLE CLASS, NOT A ScriptableObject. The shot list is edited on the director in
    /// the Inspector, where the scene's own Transforms can be dragged into it. ScriptableObject
    /// shots would each be an asset that cannot hold a scene reference, so every anchor and
    /// look-at target would have to be re-bound by name at Play — more files and one more thing
    /// to get silently wrong before a take.
    ///
    /// THE SHOT SAYS WHAT IT IS WAITING FOR, AND IT ALWAYS HAS A CEILING (2026-08-21, Ivan flew
    /// the first take). A shot list advanced only by a key is a shot list that has to be performed
    /// live while a mission flies; a shot list advanced only by a stopwatch cuts away from the
    /// dive because the vehicle was slow that day. So each shot carries a MINIMUM duration, an
    /// optional MISSION CONDITION read from facts Unity can observe locally, and a MAXIMUM
    /// duration that it falls through on and SAYS SO. The failure mode being designed out is a
    /// shot that waits forever for something that will never come: `MaxSeconds` is that guard, and
    /// the director supplies its own fallback ceiling for any conditioned shot that was left at 0.
    ///
    /// EVERY CONDITION IS A LOCAL OBSERVATION, NOT A MISSION MODEL. The director reads the vehicle
    /// transform, the still-water plane off the WaterSurface TRANSFORM (never `GetWaterLevelAt` —
    /// SETTLED §3s), and the hoop list that `MissionWPHoop_Sub` already subscribes to. It adds no
    /// ROS subscription of its own: a video tool must not become a second consumer of mission
    /// state, disagreeing with the first one on camera. Checked 2026-08-21: nothing in Unity
    /// subscribes to a mission timer, executing tasks, or `ctrl/neutral_handoff` — the only place
    /// `executing_tasks` appears is a `//TODO` in `MQTTClientGUI.cs` — so there was no richer
    /// signal to prefer.
    /// </summary>
    [Serializable]
    public class CameraShot
    {
        public enum Mode
        {
            /// <summary>Fly along a Catmull-Rom spline through hand-placed node Transforms.</summary>
            DollySpline,
            /// <summary>Damped third-person follow of a moving target.</summary>
            FollowThirdPerson,
            /// <summary>Parked camera, fixed look-at.</summary>
            StaticLookAt,
            /// <summary>Park on the hoop the vehicle is coming toward and pan to follow it past the lens.</summary>
            WaypointCams,
            /// <summary>Rise and orbit out from a centre point to a high wide.</summary>
            OrbitZoomOut,
            /// <summary>
            /// Hold a wide of the dock while `DockDrainDirector` pumps the water out, so the
            /// accumulated map is revealed in an empty dock. Appended to the end of the enum on
            /// purpose: inserting a value would renumber every shot already serialised in a scene.
            /// The drain refuses unless the vehicle is surfaced and idle, and it says so on screen.
            /// </summary>
            DrainDock
        }

        /// <summary>
        /// What ends this shot. Nested inside CameraShot on purpose: this repo declares global
        /// types (`GizmoType`, a global `Editor` namespace) that have each cost a compile cycle,
        /// and a nested enum cannot collide with anything.
        /// </summary>
        public enum AdvanceWhen
        {
            /// <summary>Never cuts by itself. The Next key is the only way out.</summary>
            Manual,
            /// <summary>t >= Duration. For the shots that really are timed: the dolly, the final orbit.</summary>
            Duration,
            /// <summary>Vehicle speed at or above MoveSpeedThreshold. "The mission has started."</summary>
            VehicleMoving,
            /// <summary>Vehicle below the still-water plane by at least SubmergeDepth. "It has gone under."</summary>
            VehicleSubmerged,
            /// <summary>Vehicle back within SurfaceDepth of the plane AFTER having been submerged during this shot. The surface break.</summary>
            VehicleSurfaced,
            /// <summary>Vehicle climbing at or faster than AscentRate. The cue to cut to the surfacing shot BEFORE the break, not after it.</summary>
            VehicleAscending,
            /// <summary>HoopsToClear more hoops exist than when the shot started. A waypoint was cleared.</summary>
            HoopCleared,
            /// <summary>Vehicle within ApproachRange of a hoop it has not passed and that no waypoint-cam shot has filmed yet.</summary>
            ApproachingHoop,
            /// <summary>The hoop this shot latched onto is now astern of the vehicle. The pass is over.</summary>
            HoopPassed,
            /// <summary>The hoop list has not grown for IdleSeconds. The plan has stopped progressing.</summary>
            HoopsIdle,
            /// <summary>Vehicle speed below MoveSpeedThreshold for IdleSeconds. It has stopped.</summary>
            VehicleIdle,
            /// <summary>
            /// Surfaced AND stopped for IdleSeconds — the run is over. This is the same pair of
            /// facts the recorder-stop policy uses (SETTLED §3s6), and it is what the dock drain
            /// is gated on: the water may only move once the mission cannot be affected by it.
            /// Appended, never inserted — see the note on Mode.
            /// </summary>
            VehicleSurfacedAndIdle,

            /// <summary>
            /// The dock drain this shot started has finished pumping (Progress01 >= 1). Round 3,
            /// 2026-08-21: the drain no longer has a shot of its own — it runs DURING the zoom-out
            /// (CameraShot.StartDrainAtShotStart), so the shot that starts it ends when the dock is
            /// EMPTY rather than when a stopwatch says the dock ought to be empty. A refused gate
            /// leaves this false and the shot falls through on its ceiling, naming the refusal.
            /// Appended, never inserted.
            /// </summary>
            DrainComplete,

            /// <summary>
            /// A NAMED SCENE OBJECT IS ASTERN OF THE VEHICLE. The fly-past cut (2026-09-01,
            /// Askö: "we pass the mini with sonars on"). `HoopPassed` could not do this job —
            /// it can only latch onto a waypoint hoop, and the thing being passed here is a
            /// car on the seabed, not a waypoint. So this condition takes an arbitrary
            /// Transform (`AsternTarget`, or `AsternTargetName` resolved at Play) and fires
            /// once the vehicle has gone by it, held for `SteadySeconds` — which on this
            /// condition doubles as "how long the lens keeps running on after the pass",
            /// exactly as it does for HoopPassed.
            ///
            /// ASTERN IS MEASURED IN THE HORIZONTAL PLANE. The vehicle's forward tilts with
            /// its pitch, and it is pitching during a dive; a 3-D dot product would therefore
            /// call the target "astern" a little early on the way down and a little late on
            /// the way up. Flattening removes a pitch-dependent cut from a shot whose whole
            /// subject is a dive.
            ///
            /// Appended, never inserted — see the note on Mode.
            /// </summary>
            TargetAstern
        }

        [Tooltip("Shown in the director's overlay and in the log when the shot starts.")]
        public string Name = "shot";
        public Mode Kind = Mode.StaticLookAt;

        [Tooltip("Seconds the CAMERA MOVE takes: the dolly's travel time and the orbit's sweep time. It is NOT by itself what ends the shot — see Advance. For Follow/WaypointCams/Static it is unused.")]
        public float Duration = 10f;
        [Tooltip("Seconds spent easing from the previous shot's final pose into this one. 0 cuts. A cut is usually right; a blend is for the fly-in handing over to the follow.")]
        public float BlendSeconds = 0f;

        // ------------------------------------------------------------ advance

        [Header("Advance — when this shot hands over")]
        [Tooltip("The mission fact this shot waits for. Manual = only the Next key. The overlay always names this, and names it again when the shot falls through on MaxSeconds instead.")]
        public AdvanceWhen Advance = AdvanceWhen.Manual;

        [Tooltip("Earliest this shot may cut, seconds. A floor, so a condition that is ALREADY true when the shot starts cannot chain-cut through the whole storyboard in one frame. For a dolly, set this to at least Duration so the move always lands.")]
        public float MinSeconds = 0f;

        [Tooltip("Seconds after which the shot cuts ANYWAY and says it fell through. 0 means no ceiling — legal only for Manual; for anything else the director substitutes its own FallbackMaxSeconds and logs that it did. This field is the reason a rehearsal with no mission running still walks the whole storyboard instead of hanging on shot 2.")]
        public float MaxSeconds = 0f;

        [Tooltip("The condition must hold continuously for this long before it counts. Stops a single noisy frame from cutting the shot. For HoopPassed this doubles as the HOLD after the vehicle is astern — how long the lens keeps panning after it goes by.")]
        public float SteadySeconds = 0.8f;

        [Header("Advance thresholds (only the ones your condition uses matter)")]
        [Tooltip("m/s. Above this counts as moving, below as idle. The vehicle cruises at ~0.34 m/s, so 0.15 discriminates moving from parked without being noise.")]
        public float MoveSpeedThreshold = 0.15f;
        [Tooltip("Metres below the still-water plane that counts as submerged.")]
        public float SubmergeDepth = 0.60f;
        [Tooltip("Metres below the still-water plane that counts as having broken the surface.")]
        public float SurfaceDepth = 0.15f;
        [Tooltip("m/s of upward motion that counts as ascending.")]
        public float AscentRate = 0.06f;
        [Tooltip("How many NEW hoops must appear before HoopCleared fires.")]
        public int HoopsToClear = 1;
        [Tooltip("Metres. ApproachingHoop fires when the vehicle is this close to a hoop it has not passed.")]
        public float ApproachRange = 16f;
        [Tooltip("Seconds of no change for HoopsIdle / VehicleIdle.")]
        public float IdleSeconds = 12f;

        [Header("TargetAstern — the fly-past cut")]
        [Tooltip("The object the vehicle flies PAST. AdvanceWhen.TargetAstern ends the shot once " +
                 "this is behind the vehicle (horizontal test — see the enum's comment) and has " +
                 "stayed behind it for SteadySeconds. A scene Transform rather than a Vector3 so " +
                 "the seeded shot list compares equal to itself on a second press (SETTLED §3s8): " +
                 "a position derived from a scene object changes the moment that object is nudged, " +
                 "and the builder would then find a difference every time it ran.")]
        public Transform AsternTarget;
        [Tooltip("Fallback for AsternTarget: a GameObject name, resolved SCENE-WIDE at shot start " +
                 "and cached. Scene-wide on purpose — unlike LookAtName, the thing being passed is " +
                 "NOT part of the vehicle. It logs what it found and where, once per resolve.")]
        public string AsternTargetName = "";

        [Tooltip("LEGACY, kept so an older saved shot list still behaves. If Advance is Manual and this is on, the shot is treated as Advance = Duration. New shots should set Advance instead.")]
        public bool AutoAdvance = false;

        [Header("Lens")]
        public float FieldOfView = 50f;
        [Tooltip("Near clip. Small values let the lens get close to the hull without slicing it; too small costs depth precision on a wide shot.")]
        public float NearClip = 0.08f;
        public float FarClip = 3000f;

        [Header("What else is on during this shot")]
        public bool HudVisible = false;
        public bool ParticlesEnabled = true;
        public bool SonarMapVisible = true;
        public bool SonarMapLabelVisible = true;
        [Tooltip("Wipe the accumulated sonar map when this shot starts. Only for a shot that opens a take.")]
        public bool ClearMapAtStart = false;

        [Tooltip("Show the SONAR BEAMS during this shot — the live hit particles from the sonar's own RayViewer, i.e. the sensor visibly draping the dock as it measures it (Ivan, 2026-08-21). This is the sensor's instantaneous footprint; the accumulated cloud is the map. They are different visuals and both belong on the underwater shots. No collider is involved — a visualisation must not be a sonar target (§3o).")]
        public bool SonarBeamsVisible = false;

        [Tooltip("Also draw the RAY LINES, not just the hit points. Off by default: 2550 lines from a 150-beam FLS reads as a solid wall, not as beams.")]
        public bool SonarRayLinesVisible = false;

        [Tooltip("SHOW THE SIDE-SCAN WATERFALL PANEL during this shot (Ivan, 2026-09-01: the beams " +
                 "AND the existing SSSWaterfallHUD burned into frame). The director opens and closes " +
                 "the panel that is already in the scene — it does not build a second one, because a " +
                 "video copy of an instrument panel is a second instrument that can disagree with the " +
                 "first one on camera. The panel keeps its own AGC, palette and footer statistics, so " +
                 "what the video shows is what the operator sees.\n\n" +
                 "The panel's own toggle key is F6, which is ALSO the director's HUD key. While " +
                 "cinematic mode is on the director takes that key and gates the panel's own handler " +
                 "(SSSWaterfallHUD.SetDirectorControl), so one key press cannot mean two things. It " +
                 "also hides the panel's closed-state '▲ SSS waterfall (F6)' button, which would " +
                 "otherwise be burned into every frame of every shot that does not want the panel.")]
        public bool ShowSSSWaterfall = false;

        [Tooltip("NAME OF A BalticWaterPreset TO APPLY WHILE THIS SHOT RUNS — \"\" leaves the water exactly as " +
                 "the scene was saved (Ivan, 2026-08-21 round 3: \"shift water to the clear version instead and " +
                 "not take it away completely\"). This is a PLAY-MODE apply and nothing else: the director " +
                 "restores the Edit-mode preset when cinematic mode ends, and a Play-mode field write is " +
                 "discarded on Stop anyway, so the SAVED scene look is never changed by a shot. Appearance " +
                 "only — BalticWaterPreset refuses to write the transform (SETTLED §3s).")]
        public string WaterPresetName = "";

        [Tooltip("ASK DockDrainDirector TO START PUMPING WHEN THIS SHOT BEGINS, whatever the shot's Mode is. " +
                 "Round 3: the drain is no longer a shot of its own — it runs UNDER the zoom-out, so the " +
                 "camera is already moving while the water goes down instead of the take sitting through two " +
                 "separate beats. The director RETRIES the gate (surfaced AND idle, SETTLED §3s6) every half " +
                 "second while this is set, so the pump starts the moment the run is genuinely over. The gate " +
                 "itself is not weakened by this flag in any way.")]
        public bool StartDrainAtShotStart = false;

        [Tooltip("KEEP THE DOCK EMPTY THROUGH THIS SHOT. Without it the director refills the dock the moment a " +
                 "shot that did not ask for the drain begins — which is correct for every shot except the one " +
                 "that comes AFTER the drain and is supposed to fly through the empty basin. (Round 2 shipped " +
                 "with exactly that defect: the drain shot handed over to the fly-through, and the fly-through's " +
                 "first frame put the water back. Nobody saw it, because nobody had run it.) The water still " +
                 "goes back on leaving cinematic mode, on disabling the director, and on returning to any " +
                 "earlier shot — a drained dock can never outlive the take.")]
        public bool KeepDockDrained = false;

        // ------------------------------------------------------------ DrainDock

        [Header("DrainDock — the SETTLED §3s sanctioned exception")]
        [Tooltip("Where the camera stands while the dock empties. Empty uses DrainPositionWS. Read the DockDrainDirector class comment before changing anything about this shot.")]
        public Transform DrainAnchor;
        [Tooltip("Camera position for the drain shot. NEVER leave this at (0,0,0) unless you want the world origin — the same trap StaticLookAt has.")]
        public Vector3 DrainPositionWS = Vector3.zero;
        [Tooltip("What the drain shot looks at. Empty uses DrainLookAtWS.")]
        public Transform DrainLookAt;
        public Vector3 DrainLookAtWS = Vector3.zero;
        [Tooltip("Metres the camera rises over the shot, so the reveal has some movement in it rather than being a locked-off plate.")]
        public float DrainCameraRiseM = 6f;

        // ------------------------------------------------------------ DollySpline

        [Header("DollySpline")]
        [Tooltip("Ordered path nodes. Empty GameObjects are fine — the director draws the spline as a gizmo so it can be shaped in the Scene view. Fewer than 2 and the shot refuses and says so.")]
        public Transform[] Nodes = new Transform[0];
        [Tooltip("Aim along the direction of travel instead of at LookAt.")]
        public bool LookAlongPath = false;
        [Tooltip("Ease along the path. The default S-curve stops the dolly starting and stopping with a jerk.")]
        public AnimationCurve Ease = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

        [Tooltip("MOVE THE WHOLE AUTHORED PATH so it ENDS on the vehicle, resolved at the moment the shot starts. This is what stops the fly-in aiming at the wrong end of the dock: the shape you dragged in the Scene view is preserved, it is just translated to wherever the mission actually begins. Off means the path is used exactly as authored.")]
        public bool AnchorEndToTarget = true;
        [Tooltip("Also ROTATE the path about its end point so it arrives from behind the vehicle's heading. With this on the fly-in comes down the vehicle's own line whichever way the mission is laid out.")]
        public bool AlignPathToTargetHeading = true;
        [Tooltip("Where the anchored path ends, in the TARGET's yaw frame: x = right, y = up, z = forward. Negative z is behind it — the pose the follow shot then takes over from.")]
        public Vector3 EndOffsetInTargetFrame = new Vector3(1.6f, 5.0f, -11.0f);

        // ------------------------------------------------------------ shared targets

        [Header("Targets (a name is resolved at Play when the Transform is empty)")]
        public Transform LookAt;
        [Tooltip("Fallback for LookAt: a GameObject name. IT IS SEARCHED UNDER THE VEHICLE FIRST — 'base_link' is not unique at Beckholmen (the station has one too), and a scene-wide Find is what aimed the fly-in at the wrong end. A scene-wide search still happens as a last resort and logs what it found.")]
        public string LookAtName = "";
        [Tooltip("Offset applied to the look-at point, world axes. A small +Y keeps the hull off the exact centre of frame.")]
        public Vector3 LookAtOffset = Vector3.zero;

        // ------------------------------------------------------------ FollowThirdPerson

        [Header("FollowThirdPerson")]
        public Transform FollowTarget;
        public string FollowTargetName = "base_link";
        [Tooltip("Camera offset in the TARGET's yaw frame: x = right, y = up, z = forward. Negative z sits behind it.")]
        public Vector3 FollowOffset = new Vector3(1.2f, 1.1f, -4.0f);
        [Tooltip("Seconds of lag on position. Higher is smoother and later; 0.35-0.8 reads as a real camera operator.")]
        public float PositionDamping = 0.45f;
        [Tooltip("Seconds of lag on aim.")]
        public float RotationDamping = 0.25f;
        [Tooltip("Use only the target's heading, ignoring its pitch and roll, when placing the offset. On: the camera stays level while the vehicle pitches, which is what a follow helicopter does. Off: the camera rolls with the hull, which is a cockpit shot.")]
        public bool YawOnly = true;

        [Tooltip("THE MAP-BUILDING FRAMING. On, the three fields below replace FollowOffset, because on this shot the subject is the point cloud growing behind the vehicle and not the vehicle. Off, FollowOffset is used as authored.")]
        public bool UseMapFraming = false;
        [Tooltip("Metres BEHIND the vehicle for the map-building shot. This is the number that decides how much of the accumulated cloud is in frame; 15 m shows roughly a swath-and-a-half at Beckholmen's depths.")]
        public float MapFramingDistance = 15f;
        [Tooltip("Metres ABOVE the vehicle for the map-building shot. Height is what puts the cloud in frame rather than edge-on.")]
        public float MapFramingHeight = 5.5f;
        [Tooltip("Metres to the SIDE for the map-building shot, so the hull is not dead centre.")]
        public float MapFramingSide = 2.5f;

        // ------------------------------------------------------------ StaticLookAt

        [Header("StaticLookAt")]
        [Tooltip("Where the camera stands. Empty uses PositionWS.")]
        public Transform Anchor;
        public Vector3 PositionWS = Vector3.zero;

        // ------------------------------------------------------------ WaypointCams

        [Header("WaypointCams — poses generated from the waypoint hoops")]
        [Tooltip("Metres to the side of the leg. ZERO SINCE 2026-08-21 round 3 — Ivan: \"make sure the wp " +
                 "cameras taking shots of the AUV approaching are not placed outside the dock walls. you can " +
                 "even place them in the middle of the wp.\" 0 puts the lens ON the hoop centreline, so the " +
                 "vehicle comes straight down the barrel and passes through/over it, and no offset can push " +
                 "the lens through a dock wall. Whatever you set here is CLAMPED to +/- WPMaxLateralOffset.")]
        public float WPLateralOffset = 0f;
        [Tooltip("Metres above the waypoint. Negative puts the lens below the hoop centre, looking slightly up — the low angle that makes an AUV look like it is flying. Keep |value| well inside the hoop radius (2.0 m) so the lens really is in the middle of the hoop.")]
        public float WPVerticalOffset = -0.6f;
        [Tooltip("HARD CLAMP on |WPLateralOffset|, metres. The dock channel at Beckholmen is ~17.6 m wide, so " +
                 "anything past a couple of metres off the leg risks putting the lens inside — or beyond — a " +
                 "dock wall, which films concrete. The director clamps and says so once per shot rather than " +
                 "trusting the authored number.")]
        public float WPMaxLateralOffset = 2.0f;
        [Tooltip("Metres BEYOND the hoop along the leg. Positive puts the lens past the hoop so the hoop frames the oncoming vehicle.")]
        public float WPBeyondOffset = 4.0f;
        [Tooltip("LATCH ONTO ONE HOOP and stay there. This is the pass Ivan asked for: the lens holds while the vehicle comes toward it, then PANS to follow as it goes by, and the shot ends on HoopPassed. Off is the old behaviour, which re-cut to whichever hoop was nearest and never showed a whole pass.")]
        public bool WPLatchToOneHoop = true;
        [Tooltip("Seconds of lag on the pan. 0 snaps the aim to the hull, which reads as a machine; 0.10-0.20 reads as an operator swinging the head round as it goes past.")]
        public float WPPanDamping = 0.14f;
        [Tooltip("Minimum seconds between cuts when NOT latched, so the shot does not flicker between two equidistant hoops.")]
        public float WPMinCutInterval = 3.0f;
        [Tooltip("Only latch onto a hoop the vehicle is within this range of.")]
        public float WPMaxRange = 60f;

        // ------------------------------------------------------------ OrbitZoomOut

        [Header("OrbitZoomOut")]
        [Tooltip("Centre of the orbit. Empty uses the vehicle's position AT THE MOMENT THE SHOT STARTS, which is what makes it a surfacing-point orbit.")]
        public Transform OrbitCenter;
        public float StartRadius = 12f;
        public float EndRadius = 120f;
        public float StartHeight = 2f;
        public float EndHeight = 70f;
        [Tooltip("Degrees swept over the shot. 120-200 reads as a reveal; 360 reads as a turntable.")]
        public float DegreesSwept = 160f;
        [Tooltip("Starting bearing, degrees from world +Z.")]
        public float StartBearingDeg = 0f;
        [Tooltip("Ease for radius and height. Slow-out makes the reveal land instead of drifting.")]
        public AnimationCurve OrbitEase = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

        [Tooltip("THE POSE THE ORBIT MUST LAND ON. Set it and the orbit's END bearing, radius and height are " +
                 "all DERIVED from this Transform relative to the orbit centre at shot start, so the sweep " +
                 "finishes exactly here — EndRadius, EndHeight and DegreesSwept's absolute bearing are then " +
                 "unused (DegreesSwept still decides how far round it travels to get here). Round 3, Ivan: " +
                 "\"rotating the camera so we look straight along the centre of the dry dock from south\" — an " +
                 "END BEARING AUTHORED FROM THE DOCK AXIS, not a generic orbit that happens to stop somewhere. " +
                 "A scene Transform and not a Vector3 on purpose: the seeder has to compare equal to itself on " +
                 "a second press (SETTLED §3s8) and a vehicle-derived number does not.")]
        public Transform OrbitEndAnchor;
        [Tooltip("Where the lens looks at the END of the orbit; the aim eases from the orbit centre to this " +
                 "point over the shot. Put it on the dock centreline NORTH of OrbitEndAnchor and the reveal " +
                 "finishes looking straight down the dock. Empty keeps the old behaviour (look at the centre).")]
        public Transform OrbitEndLookAt;

        // ------------------------------------------------------------ derived

        /// <summary>
        /// What actually ends this shot, after the legacy `AutoAdvance` tick is folded in. Kept as
        /// a method rather than done once at load so an Inspector edit mid-Play takes effect.
        /// </summary>
        public AdvanceWhen EffectiveAdvance()
        {
            if (Advance == AdvanceWhen.Manual && AutoAdvance && Duration > 0f) return AdvanceWhen.Duration;
            return Advance;
        }

        [Tooltip("FollowThirdPerson only. > 0: the follow offset STARTS at ZoomStartMultiplier x its " +
                 "configured value and eases to 1x over this many seconds — the shot arrives from a " +
                 "distance and closes in. With the close offset's Y near the hull, the camera then " +
                 "PLUNGES THROUGH THE SURFACE after a diving AUV (Ivan, 2026-08-21) instead of " +
                 "hovering above it. 0 = fixed offset, the old behaviour.")]
        public float ZoomInSeconds = 0f;

        [Tooltip("How far out the zoom-in starts, as a multiple of the follow offset. 3 = three times " +
                 "the distance and height. IGNORED when ZoomStartOffset is non-zero.")]
        public float ZoomStartMultiplier = 3f;

        [Tooltip("EXPLICIT start pose for the zoom-in, in the target's yaw frame, eased to FollowOffset over " +
                 "ZoomInSeconds. Non-zero uses this instead of ZoomStartMultiplier, which can only scale the " +
                 "END offset and therefore cannot start BELOW the hull and finish above it. That is exactly " +
                 "what the plunge needs (Ivan, round 3): start a few metres below and behind as the camera " +
                 "goes under with the vehicle, then close in to the vehicle's OWN 3rdPersonCam pose. " +
                 "(0,0,0) means \"not set\" — a camera exactly on the hull is never an authored value.")]
        public Vector3 ZoomStartOffset = Vector3.zero;

        /// <summary>Camera offset in the target's yaw frame, after the map-building framing is applied.</summary>
        public Vector3 EffectiveFollowOffset()
        {
            return UseMapFraming
                ? new Vector3(MapFramingSide, MapFramingHeight, -Mathf.Abs(MapFramingDistance))
                : FollowOffset;
        }

        /// <summary>The follow offset at shot time t: eased from ZoomStartMultiplier x down to 1x.
        /// SmoothStep, so the arrival decelerates — a camera that brakes reads as intentional,
        /// one that stops linearly reads as a glitch.</summary>
        public Vector3 FollowOffsetAt(float t)
        {
            Vector3 off = EffectiveFollowOffset();
            if (ZoomInSeconds <= 1e-3f) return off;
            float k = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(t / ZoomInSeconds));
            // An explicit start pose wins over the multiplier: scaling the end offset cannot change
            // its SIGN, so a multiplier alone can never start below the hull and finish above it.
            if (ZoomStartOffset.sqrMagnitude > 1e-6f) return Vector3.Lerp(ZoomStartOffset, off, k);
            return off * Mathf.Lerp(Mathf.Max(1f, ZoomStartMultiplier), 1f, k);
        }

        /// <summary>The lateral offset actually used by WaypointCams, after the dock-wall clamp.</summary>
        public float EffectiveWPLateralOffset()
        {
            float max = Mathf.Abs(WPMaxLateralOffset);
            return Mathf.Clamp(WPLateralOffset, -max, max);
        }

        /// <summary>One line for the log when the shot starts. Not for the overlay — the overlay reports live state.</summary>
        public string AdvanceSummary()
        {
            var a = EffectiveAdvance();
            string min = MinSeconds > 0f ? $"min {MinSeconds:F0}s, " : "";
            string max = MaxSeconds > 0f ? $"max {MaxSeconds:F0}s" : "no ceiling of its own";
            return a == AdvanceWhen.Manual ? $"{min}MANUAL only ({max})" : $"{min}on {a} ({max})";
        }
    }
}
