using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering.HighDefinition;   // WaterSurface — read for its TRANSFORM only

using SmarcGUI.Water;              // UnderwaterParticles, DockDrainDirector
using SmarcGUI.WorldSpace;         // WaypointHoop
using ROS.Subscribers;             // MissionWPHoop_Sub
using Visualizers;                 // SonarMapAccumulator, RayViewer

namespace Smarc.Cinematics
{
    /// <summary>
    /// Drives ONE camera through an ordered shot list, and drives the rest of the scene's
    /// presentation with it: the HUD canvases, the marine-snow particles, and the sonar map's
    /// visibility and label.
    ///
    /// THE SHOT LIST RUNS ITSELF OFF THE MISSION (2026-08-21, after the first flown take).
    /// Ivan's report on the manual version was, in order: "what does F9 mean, should I press it
    /// once and let the sim run"; "the fly-in aims at the finish of the mission, not at the
    /// vehicle at the start"; "I did not see it switching between different clips". All three are
    /// the same complaint — the storyboard existed but nothing drove it. So: press record once,
    /// get the finished sequence in ONE CONTINUOUS TAKE. The keys stay, as an override; they are
    /// no longer the only way.
    ///
    /// WHAT DRIVES IT IS OBSERVED, NOT ASSUMED, AND NOT A SECOND MISSION CONSUMER. Every advance
    /// condition (CameraShot.AdvanceWhen) is evaluated from facts this scene already holds:
    ///   * the vehicle transform — position, heading, and speed/vertical rate differentiated from it;
    ///   * the still-water plane, read off the WaterSurface TRANSFORM. Never `GetWaterLevelAt`:
    ///     `HDRPWaterQueryModel` is one shared instance that seeds each search from the previous
    ///     caller's result, and a camera hundreds of metres away asking it where the water is
    ///     hands the vehicle's ForcePoints a poisoned seed (SETTLED §3s — the 67 m/s launch);
    ///   * the hoop list `MissionWPHoop_Sub` already subscribes to, which grows by one each time
    ///     the vehicle is cleared to fly a new leg.
    /// It adds NO ROS subscription of its own. Checked 2026-08-21: nothing in Unity subscribes to
    /// a mission timer, executing tasks, or `ctrl/neutral_handoff` (the one mention of
    /// `executing_tasks` in the whole project is a `//TODO` in `MQTTClientGUI.cs`), so there was
    /// no richer signal to prefer. If one is ever added, prefer it over the differentiated
    /// transform — a video tool inventing its own idea of what the mission is doing, and
    /// disagreeing with Mission Control on camera, is the thing to avoid.
    ///
    /// NO SHOT CAN HANG. Every conditioned shot carries a MaxSeconds ceiling, and any shot that
    /// was left at 0 gets the director's FallbackMaxSeconds and a warning at Play. When a shot
    /// cuts on its ceiling it says FELL THROUGH and names the condition that never arrived. This
    /// is what makes a rehearsal with no mission running, no hoops, and an idle vehicle walk the
    /// whole storyboard instead of stopping on shot 2 and looking broken.
    ///
    /// IT TAKES THE CAMERA OVER, IT DOES NOT FIGHT FOR IT. Entering cinematic mode disables every
    /// other enabled camera that renders to the screen, plus the named interaction scripts on
    /// those cameras (FlyCamera grabs the cursor and would keep flying the old view). Everything
    /// is restored on exit, so a session can drop in and out of cinematic mode without a reload.
    ///
    /// ONE PLAY PER SESSION. The estimator is expected to die across a Play cycle (SETTLED §4) and
    /// Unity Recorder records per-Play, so several takes have to fit inside one Play: the keys
    /// restart and re-arm shots without leaving Play.
    ///
    /// IT ALSO DRIVES THE DOCK DRAIN, AND THAT ONE IS NOT COSMETIC (2026-08-21). The final beat is
    /// the dock being pumped out to reveal the map in the dry. Lowering the water is the SETTLED
    /// §3s trap — the one that put SAM at 67 m/s — so this director does not do it: it asks
    /// `DockDrainDirector`, which refuses unless the vehicle is surfaced and idle, freezes every
    /// ForcePoint in the scene before anything moves, and restores everything on exit. This class
    /// guarantees only the outer half of that: leaving the drain shot, leaving cinematic mode, or
    /// disabling the director all call `Restore()`, so the water can never outlive the shot that
    /// lowered it.
    ///
    /// THE TWO DEFECTS THE 2026-08-21 TAKE FOUND, and where they actually were:
    ///   * "sonar map · 0 pts" for 643 s — `SonarMapAccumulator` bound no sonar at all, because
    ///     sam2.2 has an FLS and an SSS and no MBES and only `IncludeMBES` was ticked. Nothing here.
    ///   * "'ApproachingHoop' never happened … last filmed 1" — `MissionWPHoop_Sub` never held more
    ///     than one hoop, because every waypoint on the wire is named "wp" and identity was the
    ///     name alone. Also nothing here; but the OVERLAY made it unreadable by printing a count
    ///     and a reason that appeared to contradict each other, and that wording is now specific.
    ///
    /// ROUND 3 (2026-08-21, after take 006): SIX MINUTES OF DEAD AIR AT THE END, AND WHY.
    /// The take's own overlay recorded it: shot 7 "FELL THROUGH at 240 s — 'VehicleAscending' never
    /// happened (vertical 0.00 m/s, at 0.0 m down)", and shot 8 then waited on "the surface break —
    /// the vehicle has not been under DURING THIS SHOT". Both are correct sentences about a mission
    /// that had ALREADY ENDED: the vehicle was on the surface before shot 7 even started, so
    /// "waiting for the ascent" and "waiting for the surface break" were not late, they were MOOT —
    /// and a moot condition used to be indistinguishable from one that is merely slow, so each shot
    /// sat out its whole ceiling.
    ///
    /// THE RULE THIS ADDS: a condition that is ALREADY TRUE, or MOOT, AT THE MOMENT THE SHOT STARTS
    /// advances the shot at its MinSeconds floor, logged and shown as "condition pre-satisfied",
    /// instead of waiting out MaxSeconds. It is evaluated ONCE, in `BeginShot`, from facts already
    /// measured (so it never fires before the vehicle has been observed at all), and it never
    /// applies to `Duration` — a duration is a camera move, not a mission fact, and shortening it
    /// would cut the move off. Every late shot in the storyboard is now correct for the short
    /// mission as well as the long one: the mission may END BEFORE SHOT 7 EVER STARTS.
    ///
    /// ROUND 3 ALSO MERGES THE DRAIN INTO THE ZOOM-OUT (Ivan: "cant we just start the draining while
    /// zooming out and rotating the camera so we look straight along the centre of the dry dock from
    /// south"). `CameraShot.StartDrainAtShotStart` asks `DockDrainDirector` at the START of any shot
    /// and RETRIES the gate twice a second until it opens, so the pump runs under a camera that is
    /// already moving. The gate itself is untouched — surfaced AND idle, every ForcePoint frozen
    /// first, always restored (SETTLED §3s and its one sanctioned exception).
    ///
    /// AND IT DRIVES THE WATER LOOK PER SHOT. `CameraShot.WaterPresetName` applies a
    /// `BalticWaterPreset` preset while a shot runs and the Edit-mode preset is restored on every
    /// exit path. This is a PLAY-MODE apply on purpose: the saved scene keeps whatever look Ivan
    /// chose in Edit mode, and a Play-mode field write is discarded on Stop anyway. It exists
    /// because the honest answer to "after wp2 the water suddenly disappears" was a broken
    /// underwater VOLUME (see VideoRigSetup's volume-bounds coverage check), and the thing Ivan
    /// actually wanted from that accident — a clearer look over the map — is a preset, not a bug.
    /// </summary>
    [AddComponentMenu("Smarc/Cinematics/Cinematic Director")]
    public class CinematicDirector : MonoBehaviour
    {
        [Header("Camera")]
        [Tooltip("The camera the director drives. Left empty, one is created at Play named CinematicCam with a high depth so it is what the Game view (and Unity Recorder) sees.")]
        public Camera CinematicCamera;
        [Tooltip("Render depth given to the created camera. Higher than any other camera in the scene.")]
        public float CameraDepth = 100f;

        [Header("Shots")]
        public List<CameraShot> Shots = new List<CameraShot>();
        [Tooltip("Index of the shot that is live. Changing it in the Inspector while playing jumps to that shot.")]
        public int CurrentShot = 0;
        [Tooltip("Enter cinematic mode as soon as Play starts. Off means press the toggle key when you are ready — which is usually right, because the mission has to be started from Mission Control first.")]
        public bool StartInCinematicMode = false;

        [Header("Automatic advance")]
        [Tooltip("Master switch for the whole self-driving storyboard. Off makes every shot behave as Manual, which is the old behaviour and is what you want if you are directing by hand.")]
        public bool AutoAdvanceEnabled = true;
        [Tooltip("Ceiling applied to any shot that has a condition but was left at MaxSeconds = 0. The director says which shots it had to do this for, at Play. It exists so a shot list can never hang, not so MaxSeconds can be left blank.")]
        public float FallbackMaxSeconds = 120f;
        [Tooltip("No automatic cut may happen before this many seconds of a shot, whatever the shot's own MinSeconds says. Stops a condition that is ALREADY true at the cut from chaining through the storyboard in a single frame.")]
        public float MinShotSeconds = 1.5f;
        [Tooltip("Seconds after a MANUAL advance during which automatic advance is held off. This is how pressing Next stops fighting the pending automatic cut: the manual decision wins, cleanly, and then the new shot's own condition takes over.")]
        public float ManualGraceSeconds = 3f;
        [Tooltip("Seconds over which vehicle speed and vertical rate are smoothed before a threshold is applied. Raw frame-to-frame transform deltas are far too noisy to cut a shot on.")]
        public float SpeedSmoothingSeconds = 0.6f;

        [Header("Scene hooks (found by name at Play when left empty)")]
        [Tooltip("HUD roots toggled per shot. Canvas-Top carries the dashboard and the top bar.")]
        public List<GameObject> HudRoots = new List<GameObject>();
        public List<string> HudRootNames = new List<string> { "Canvas-Top" };
        [Tooltip("GUI chrome hidden for the whole of cinematic mode: mission lists, log, sliders. None of it belongs in a video.")]
        public List<string> AlwaysHideNames = new List<string> { "Canvas-Left", "Canvas-Right", "Canvas-Bottom", "Canvas-Under", "Canvas-Over" };
        public UnderwaterParticles Particles;
        public SonarMapAccumulator SonarMap;
        public MissionWPHoop_Sub Hoops;
        [Tooltip("Drains the dock for the final reveal. Found at Play when empty. Read its class comment: it is the SANCTIONED SETTLED §3s exception and it refuses to run unless the vehicle is surfaced and idle.")]
        public DockDrainDirector Drain;
        [Tooltip("The water's appearance presets, driven per shot by CameraShot.WaterPresetName. Found at Play when empty. The Edit-mode ActivePreset is remembered on entering cinematic mode and restored on every exit path, so a take can never leave the scene looking like something it was not saved as.")]
        public BalticWaterPreset WaterPreset;
        [Tooltip("Sonar beam visualisers, toggled per shot (CameraShot.SonarBeamsVisible). Filled at Play from every RayViewer UNDER the vehicle — never scene-wide, so a second vehicle's sonar does not end up in this take.")]
        public List<RayViewer> SonarBeams = new List<RayViewer>();

        [Tooltip("The side-scan waterfall panel already in the scene, opened and closed per shot " +
                 "(CameraShot.ShowSSSWaterfall). Found at Play when empty. The director drives the " +
                 "EXISTING panel rather than building a video copy of it: a second instrument that " +
                 "can disagree with the first one is exactly what a video must not put on screen.\n\n" +
                 "Entering cinematic mode also TAKES ITS KEY. The panel's toggle key is F6 and this " +
                 "director's HUD key is F6, so in a scene holding both, one press meant two things. " +
                 "SSSWaterfallHUD.SetDirectorControl gates the panel's own handler and hides its " +
                 "closed-state button for the duration; leaving cinematic mode gives both back, " +
                 "along with whatever open/closed state the panel had when F9 was pressed.")]
        public SSSWaterfallHUD Waterfall;
        [Tooltip("WaterSurface whose TRANSFORM Y is the still-water plane. Read for its transform only — never GetWaterLevelAt (SETTLED §3s). Left empty, the first one in the scene is used; with none, the depth conditions say they cannot be evaluated instead of guessing Y = 0.")]
        public WaterSurface Surface;
        [Tooltip("Vehicle ROOT GameObject name. Everything the shots aim at is searched UNDER this first, which is what stops 'base_link' resolving to the station's.")]
        public string VehicleName = "sam_auv_v1";
        [Tooltip("Vehicle root, used by WaypointCams, by OrbitZoomOut when no centre is given, and as the search root for every by-name target.")]
        public Transform Vehicle;
        [Tooltip("Child of the vehicle whose transform is treated as THE vehicle for speed, depth and aiming. Empty falls back to the vehicle root.")]
        public string VehicleAimChildName = "base_link";
        [Tooltip("Component type names disabled on the cameras this director takes over from. They are matched by type name, so no assembly reference is needed and a missing one is simply not found.")]
        public List<string> DisableScriptsOnOtherCameras = new List<string> { "FlyCamera", "DragZoomCamera", "SmoothFollow", "StartLookingAtRobots", "TopDownOrthoCam" };

        [Header("Keys")]
        public Key ToggleCinematicKey = Key.F9;
        public Key NextShotKey = Key.F10;
        public Key PrevShotKey = Key.F8;
        public Key RestartShotKey = Key.F7;
        public Key ToggleHudKey = Key.F6;
        public Key ToggleSonarMapKey = Key.F5;
        public Key ClearSonarMapKey = Key.F4;
        public Key ToggleOverlayKey = Key.F3;
        [Tooltip("Turns the self-driving storyboard on and off mid-take. With it off every shot is Manual.")]
        public Key ToggleAutoAdvanceKey = Key.F2;

        [Header("Overlay — TURN THIS OFF BEFORE A REAL TAKE")]
        [Tooltip("Shot name, elapsed time, WHAT THE SHOT IS WAITING FOR, and why the last cut happened. Unity Recorder captures the Game view, so anything drawn here is burned into the video. This is the rehearsal instrument: you direct by reading it, then untick it.")]
        public bool ShowOverlay = true;

        // ---------------------------------------------------------------- state

        public bool CinematicMode { get; private set; }
        float shotStartTime;
        Vector3 blendFromPos;
        Quaternion blendFromRot;
        bool hasBlendFrom;

        // advance bookkeeping
        float conditionTrueSince = -1f;
        float autoHeldUntil;
        int hoopCountAtShotStart;
        bool submergedDuringShot;
        string waitText = "";
        string lastCutReason = "";

        // vehicle facts, differentiated from the transform
        Transform vehicleAim;
        float nextHookRetry;
        Vector3 lastVehiclePos;
        bool hasLastVehiclePos;
        float smoothedSpeed;
        float smoothedVertRate;

        // hoop facts
        int lastHoopCount = -1;
        float lastHoopChangeTime;
        float surfacedIdleSince = -1f;
        [Tooltip("Highest hoop index a WaypointCams shot has already filmed. The next waypoint-pass shot picks a LATER hoop, which is what makes the two passes different shots and not the same one twice.")]
        public int LastFilmedHoopIndex = -1;

        // WaypointCams
        int wpCurrentIndex = -1;
        float wpLastCutTime = -999f;
        Vector3 wpPos;
        bool wpPassed;
        float wpPassedTime;

        // DollySpline, resolved once per shot so the path can be anchored to the vehicle
        readonly List<Vector3> dollyPts = new List<Vector3>();

        // OrbitZoomOut
        Vector3 orbitCenterWS;

        // DrainDock
        bool drainRequested;
        bool drainBegun;
        float nextDrainPoll;
        string drainStatus = "";

        // round 3: a condition that was already true / moot when the shot started
        bool preSatisfied;
        string preSatisfiedWhy = "";

        // round 3: the water look, driven per shot and always put back
        string editModePresetName = "";
        string appliedPresetName = "";

        // 2026-09-01: the waterfall panel's open/closed state as it was when F9 was pressed, so
        // leaving cinematic mode hands the operator back the panel they had — the same
        // put-it-back-as-you-found-it rule the water look and the suppressed cameras follow.
        bool waterfallWasOpen;
        bool waterfallTaken;

        // round 3: the orbit's END pose, resolved once at shot start from OrbitEndAnchor
        bool orbitEndFromAnchor;
        float orbitEndBearingDeg, orbitEndRadius, orbitEndHeight;

        readonly List<Camera> suppressedCameras = new List<Camera>();
        readonly List<MonoBehaviour> suppressedScripts = new List<MonoBehaviour>();
        readonly List<GameObject> hiddenChrome = new List<GameObject>();

        // ---------------------------------------------------------------- lifecycle

        void Start()
        {
            ResolveSceneHooks();
            ReportShotListHealth();
            if (StartInCinematicMode) EnterCinematicMode();
        }

        void ResolveSceneHooks()
        {
            if (Vehicle == null && !string.IsNullOrEmpty(VehicleName))
            {
                var go = GameObject.Find(VehicleName);
                if (go != null) Vehicle = go.transform;
                else Debug.LogWarning($"[CinematicDirector] no ACTIVE GameObject named '{VehicleName}' — " +
                                      "every advance condition that needs the vehicle will report that it cannot " +
                                      "be evaluated, and the shots will fall through on their ceilings.");
            }
            vehicleAim = ResolveVehicleAim();

            if (Particles == null) Particles = FindFirstObjectByType<UnderwaterParticles>();
            if (SonarMap == null) SonarMap = FindFirstObjectByType<SonarMapAccumulator>();
            if (Hoops == null) Hoops = FindFirstObjectByType<MissionWPHoop_Sub>();
            if (Drain == null) Drain = FindFirstObjectByType<DockDrainDirector>();
            if (Surface == null) Surface = FindFirstObjectByType<WaterSurface>();
            if (WaterPreset == null) WaterPreset = FindFirstObjectByType<BalticWaterPreset>();
            if (Waterfall == null) Waterfall = FindFirstObjectByType<SSSWaterfallHUD>();

            ResolveSonarBeams();
            if (Surface == null)
                Debug.LogWarning("[CinematicDirector] no WaterSurface in the scene — VehicleSubmerged, " +
                                 "VehicleSurfaced and VehicleAscending cannot be evaluated and will say so. " +
                                 "They are NOT silently treated as water at Y = 0.");

            if (HudRoots.Count == 0)
                foreach (var n in HudRootNames)
                {
                    var go = GameObject.Find(n);
                    if (go != null) HudRoots.Add(go);
                    else Debug.Log($"[CinematicDirector] HUD root '{n}' not found — nothing to toggle for it.");
                }
        }

        /// <summary>
        /// The transform treated as THE vehicle. Searched under the vehicle root, never scene-wide:
        /// Beckholmen contains a second `base_link` (the station's, on the dock bridge at Z 48.7)
        /// and a scene-wide `GameObject.Find("base_link")` is free to return it. That is a strong
        /// candidate for the fly-in aiming at the wrong end of the dock on 2026-08-21.
        /// </summary>
        Transform ResolveVehicleAim()
        {
            if (Vehicle == null) return null;
            if (string.IsNullOrEmpty(VehicleAimChildName)) return Vehicle;
            var child = FindDeepChild(Vehicle, VehicleAimChildName);
            if (child != null) return child;
            Debug.Log($"[CinematicDirector] no '{VehicleAimChildName}' under '{Vehicle.name}' — " +
                      "using the vehicle root itself as the aim point.");
            return Vehicle;
        }

        /// <summary>
        /// The sonar beam visualisers this director may switch on and off.
        ///
        /// UNDER THE VEHICLE ONLY, and searched with inactive included so the Console can say
        /// "there are none" rather than leaving an empty list to mean two different things. At
        /// Beckholmen these are the `RayViewer` on `Sonar3D15` and the one on `SideScanSonar
        /// DeepVision`, both inside `SAMSensorsV2`.
        /// </summary>
        void ResolveSonarBeams()
        {
            SonarBeams.Clear();
            if (Vehicle == null) return;
            Vehicle.GetComponentsInChildren(true, SonarBeams);
            if (SonarBeams.Count == 0)
                Debug.Log($"[CinematicDirector] no RayViewer under '{VehicleName}' — the per-shot " +
                          "SonarBeamsVisible toggle has nothing to drive. The beams are a RayViewer " +
                          "component sitting on the same GameObject as a Sonar.");
            else
            {
                var names = new List<string>();
                foreach (var rv in SonarBeams) if (rv != null) names.Add(rv.name);
                Debug.Log($"[CinematicDirector] {SonarBeams.Count} sonar beam visualiser(s) available: " +
                          $"{string.Join(", ", names)}. They are toggled per shot and are particles only — " +
                          "no collider, so they never become sonar targets themselves.");
            }
        }

        /// <summary>Beams on or off for the current shot. Restored to off when cinematic mode exits.</summary>
        void ApplyBeams(bool hits, bool rays)
        {
            foreach (var rv in SonarBeams)
            {
                if (rv == null) continue;
                rv.SetHitsVisible(hits);
                rv.SetRaysVisible(rays);
            }
        }

        public static Transform FindDeepChild(Transform root, string name)
        {
            if (root == null) return null;
            if (root.name == name) return root;
            for (int i = 0; i < root.childCount; i++)
            {
                var r = FindDeepChild(root.GetChild(i), name);
                if (r != null) return r;
            }
            return null;
        }

        /// <summary>
        /// Said once at Play, so the failure mode is a Console line before the take and not a shot
        /// that sits there during it.
        /// </summary>
        void ReportShotListHealth()
        {
            if (Shots.Count == 0) { Debug.LogWarning("[CinematicDirector] the shot list is EMPTY."); return; }
            var noCeiling = new List<string>();
            for (int i = 0; i < Shots.Count; i++)
            {
                var s = Shots[i];
                if (s == null) continue;
                if (s.EffectiveAdvance() != CameraShot.AdvanceWhen.Manual && s.MaxSeconds <= 0f)
                    noCeiling.Add($"{i + 1} '{s.Name}'");
            }
            if (noCeiling.Count > 0)
                Debug.LogWarning($"[CinematicDirector] {noCeiling.Count} conditioned shot(s) have no MaxSeconds of " +
                                 $"their own — {string.Join(", ", noCeiling)}. The director's FallbackMaxSeconds " +
                                 $"({FallbackMaxSeconds:F0} s) will be used so they cannot hang, but a ceiling you " +
                                 "chose is better than one it chose for you.");

            // A water preset named by a shot but absent from the component is a silent no-op at the
            // cut. Say it at Play, when it can still be fixed, and not on camera.
            var badPresets = new List<string>();
            var drainShots = new List<string>();
            for (int i = 0; i < Shots.Count; i++)
            {
                var s = Shots[i];
                if (s == null) continue;
                if (!string.IsNullOrEmpty(s.WaterPresetName) &&
                    (WaterPreset == null || WaterPreset.Find(s.WaterPresetName) == null))
                    badPresets.Add($"{i + 1} '{s.Name}' wants '{s.WaterPresetName}'");
                if (s.StartDrainAtShotStart || s.Kind == CameraShot.Mode.DrainDock)
                    drainShots.Add($"{i + 1} '{s.Name}'");
            }
            if (badPresets.Count > 0)
                Debug.LogError("[CinematicDirector] these shots name a water preset that does not exist: " +
                               $"{string.Join("; ", badPresets)}. Known: " +
                               $"{(WaterPreset != null ? string.Join(", ", WaterPreset.PresetNames()) : "NO BalticWaterPreset IN THE SCENE")}. " +
                               "Those shots will leave the water exactly as saved rather than substitute a look.");
            // A shot that asks for the waterfall in a scene that has none is a shot that will look
            // right in the Inspector and be missing an instrument in the frame. Said at Play.
            int wantWaterfall = 0;
            foreach (var s in Shots) if (s != null && s.ShowSSSWaterfall) wantWaterfall++;
            if (wantWaterfall > 0 && Waterfall == null)
                Debug.LogWarning($"[CinematicDirector] {wantWaterfall} shot(s) ask for the SSS waterfall " +
                                 "panel, but there is no SSSWaterfallHUD in this scene. Those shots play " +
                                 "without it — nothing is substituted. The panel is a component on the " +
                                 "vehicle root; add it there, or clear ShowSSSWaterfall on those shots.");
            else if (wantWaterfall > 0)
                Debug.Log($"[CinematicDirector] the SSS waterfall panel " +
                          $"('{HierarchyPath(Waterfall.transform)}') is up on {wantWaterfall} of " +
                          $"{Shots.Count} shot(s). While cinematic mode is on, {ToggleHudKey} belongs to " +
                          "this director and the panel's own key is gated — see SetDirectorControl.");

            if (drainShots.Count > 1)
                Debug.LogWarning($"[CinematicDirector] {drainShots.Count} shots ask for a dock drain " +
                                 $"({string.Join(", ", drainShots)}). Leaving one shot refills the dock before the " +
                                 "next asks again, so this works — but it is almost certainly not what you meant.");

            Debug.Log($"[CinematicDirector] {Shots.Count} shot(s) loaded; auto-advance " +
                      $"{(AutoAdvanceEnabled ? "ON" : "OFF")}. {ToggleCinematicKey} enters cinematic mode. " +
                      "A shot whose condition is already true or moot when it starts advances at its " +
                      "MinSeconds and says 'condition pre-satisfied' — that is the fix for the six minutes " +
                      "of dead air at the end of take 006, not a shortened ceiling.");
        }

        void Update()
        {
            HandleKeys();
            if (!CinematicMode) return;
            if (CinematicCamera == null) return;
            if (Shots.Count == 0) return;

            TrackFacts();
            TickDrainRequest();

            var shot = Shots[Mathf.Clamp(CurrentShot, 0, Shots.Count - 1)];
            float t = Time.time - shotStartTime;

            if (ShouldAdvance(shot, t, out var reason))
            {
                lastCutReason = $"'{shot.Name}' -> {reason}";
                Debug.Log($"[CinematicDirector] CUT: {lastCutReason}");
                BeginShot(CurrentShot + 1);
                return;
            }

            EvaluateShot(shot, t, out var pos, out var rot);

            // Ease out of the previous shot's final pose.
            if (hasBlendFrom && shot.BlendSeconds > 0f && t < shot.BlendSeconds)
            {
                float k = Mathf.SmoothStep(0f, 1f, t / shot.BlendSeconds);
                pos = Vector3.Lerp(blendFromPos, pos, k);
                rot = Quaternion.Slerp(blendFromRot, rot, k);
            }

            CinematicCamera.transform.SetPositionAndRotation(pos, rot);
            CinematicCamera.fieldOfView = shot.FieldOfView;
            CinematicCamera.nearClipPlane = shot.NearClip;
            CinematicCamera.farClipPlane = shot.FarClip;
        }

        void HandleKeys()
        {
            var kb = Keyboard.current;
            if (kb == null) return;

            if (kb[ToggleCinematicKey].wasPressedThisFrame)
            {
                if (CinematicMode) ExitCinematicMode(); else EnterCinematicMode();
            }
            if (!CinematicMode) return;

            // A manual decision cancels the pending automatic one, rather than racing it: the new
            // shot starts with its condition state cleared AND automatic advance held off for
            // ManualGraceSeconds, so the shot you asked for is the shot you get.
            if (kb[NextShotKey].wasPressedThisFrame) { HoldAuto("Next pressed"); BeginShot(CurrentShot + 1); }
            if (kb[PrevShotKey].wasPressedThisFrame) { HoldAuto("Prev pressed"); BeginShot(CurrentShot - 1); }
            if (kb[RestartShotKey].wasPressedThisFrame) { HoldAuto("Restart pressed"); BeginShot(CurrentShot); }
            if (kb[ToggleOverlayKey].wasPressedThisFrame) ShowOverlay = !ShowOverlay;
            if (kb[ToggleAutoAdvanceKey].wasPressedThisFrame)
            {
                AutoAdvanceEnabled = !AutoAdvanceEnabled;
                Debug.Log($"[CinematicDirector] automatic advance {(AutoAdvanceEnabled ? "ON" : "OFF — every shot is now Manual")}.");
            }
            if (kb[ToggleHudKey].wasPressedThisFrame) ApplyHud(!CurrentHudState());
            if (kb[ToggleSonarMapKey].wasPressedThisFrame && SonarMap != null)
                SonarMap.Visible = !SonarMap.Visible;
            if (kb[ClearSonarMapKey].wasPressedThisFrame && SonarMap != null)
                SonarMap.ClearMap();
        }

        void HoldAuto(string why)
        {
            autoHeldUntil = Time.time + Mathf.Max(0f, ManualGraceSeconds);
            lastCutReason = $"MANUAL ({why})";
        }

        // ---------------------------------------------------------------- observed facts

        /// <summary>Still-water plane. See the class comment for why this is not GetWaterLevelAt.</summary>
        public float WaterPlaneY => Surface != null ? Surface.transform.position.y : 0f;
        public bool HasWaterPlane => Surface != null;

        /// <summary>Metres below the still-water plane. Positive is under.</summary>
        public float VehicleDepth => vehicleAim != null ? WaterPlaneY - vehicleAim.position.y : 0f;
        public float VehicleSpeed => smoothedSpeed;
        /// <summary>Vertical rate, m/s, positive up.</summary>
        public float VehicleVerticalRate => smoothedVertRate;
        public int HoopCount => Hoops != null && Hoops.Hoops != null ? Hoops.Hoops.Count : 0;

        void TrackFacts()
        {
            if (vehicleAim == null) vehicleAim = ResolveVehicleAim();

            if (vehicleAim != null)
            {
                var p = vehicleAim.position;
                if (hasLastVehiclePos && Time.deltaTime > 1e-5f)
                {
                    var d = (p - lastVehiclePos) / Time.deltaTime;
                    float k = SpeedSmoothingSeconds > 1e-3f
                        ? 1f - Mathf.Exp(-Time.deltaTime / SpeedSmoothingSeconds) : 1f;
                    smoothedSpeed = Mathf.Lerp(smoothedSpeed, d.magnitude, k);
                    smoothedVertRate = Mathf.Lerp(smoothedVertRate, d.y, k);
                }
                lastVehiclePos = p;
                hasLastVehiclePos = true;

                if (HasWaterPlane && VehicleDepth > 0.05f) submergedDuringShot = true;
            }

            int hc = HoopCount;
            if (hc != lastHoopCount)
            {
                lastHoopCount = hc;
                lastHoopChangeTime = Time.time;
            }
        }

        // ---------------------------------------------------------------- the drain, run under a shot

        /// <summary>
        /// While a shot has asked for the drain, POLL the gate rather than asking once and giving up.
        ///
        /// The gate is `surfaced AND idle for RequiredIdleSeconds` (SETTLED §3s6). A shot that starts
        /// the instant the vehicle breaks the surface therefore cannot be granted a drain on its
        /// first frame — and round 2's answer, a separate drain shot several seconds later, is what
        /// put a dead beat in the take. So: `CanDrain` (which does NOT log) is polled twice a second
        /// and `Begin()` is called only once it says yes. The refusal sentence is exactly what the
        /// overlay shows in the meantime, so a drain that never starts always says which half of the
        /// gate was missing, in metres and m/s.
        /// </summary>
        void TickDrainRequest()
        {
            if (!drainRequested || Drain == null) return;
            if (drainBegun) { drainStatus = Drain.StatusLine; return; }
            if (Time.time < nextDrainPoll) return;
            nextDrainPoll = Time.time + 0.5f;

            if (Drain.CanDrain(out string reason))
            {
                drainBegun = Drain.Begin();
                drainStatus = Drain.StatusLine;
            }
            else drainStatus = reason;
        }

        /// <summary>Stop asking for a drain and put the water back. Called whenever a shot that did not ask for one begins, and on every exit path.</summary>
        void ReleaseDrain()
        {
            if (drainRequested && Drain != null) Drain.Restore();
            drainRequested = false;
            drainBegun = false;
            drainStatus = "";
        }

        // ---------------------------------------------------------------- the water look, per shot

        /// <summary>
        /// Apply a named appearance preset for the duration of a shot, in PLAY MODE ONLY.
        ///
        /// This never writes the transform (BalticWaterPreset refuses to) and it never marks the
        /// scene dirty — authoring the saved look is `SMARC/Video/3`'s job, in Edit mode, where it
        /// survives. Here the Play-mode-edit-is-lost-on-Stop behaviour that cost this project a
        /// measured station position (SETTLED §3s2) is the DESIRED property: whatever a take does to
        /// the water is gone when Play ends, and `RestoreWaterPreset` puts it back even sooner.
        /// </summary>
        void ApplyWaterPreset(string name)
        {
            if (!Application.isPlaying || string.IsNullOrEmpty(name)) return;
            if (WaterPreset == null)
            {
                Debug.LogWarning($"[CinematicDirector] a shot asks for water preset '{name}' but there is no " +
                                 "BalticWaterPreset in the scene — the water is left exactly as saved. " +
                                 "Run SMARC/Video/2 to add one.");
                return;
            }
            if (appliedPresetName == name) return;
            if (WaterPreset.Find(name) == null)
            {
                Debug.LogError($"[CinematicDirector] shot asks for water preset '{name}', which does not exist. " +
                               $"Known presets: {string.Join(", ", WaterPreset.PresetNames())}. The water is left " +
                               "as saved — a look that cannot be found is not silently substituted.");
                return;
            }
            if (WaterPreset.ApplyPreset(name)) appliedPresetName = name;
        }

        /// <summary>Put the water look back to whatever the SCENE was saved with. Idempotent.</summary>
        void RestoreWaterPreset()
        {
            if (WaterPreset == null) return;
            if (string.IsNullOrEmpty(appliedPresetName)) return;
            if (!string.IsNullOrEmpty(editModePresetName) && editModePresetName != appliedPresetName)
            {
                WaterPreset.ApplyPreset(editModePresetName);
                Debug.Log($"[CinematicDirector] water look restored to the Edit-mode preset '{editModePresetName}' " +
                          $"(the take had it on '{appliedPresetName}'). Nothing was saved either way.");
            }
            appliedPresetName = "";
        }

        /// <summary>
        /// The hoop the vehicle is coming toward: the nearest one it has not yet passed, later than
        /// any hoop a waypoint-pass shot has already used, and within range. Returns -1 when there
        /// is no such hoop, which is a normal answer and not an error — the hoop list only ever
        /// holds the plan SO FAR (the topic carries the current waypoint only, SETTLED §3o).
        /// </summary>
        // Why the last ApproachHoop() call came up empty. Kept so the overlay can be specific
        // instead of saying "none" and leaving the reader to guess between four causes.
        int lastApproachSkippedFilmed, lastApproachOutOfRange, lastApproachAstern;
        float lastApproachNearest = float.MaxValue;

        int ApproachHoop(float maxRange, bool skipAlreadyFilmed, out float distance)
        {
            distance = float.MaxValue;
            lastApproachSkippedFilmed = lastApproachOutOfRange = lastApproachAstern = 0;
            lastApproachNearest = float.MaxValue;
            if (Hoops == null || vehicleAim == null) return -1;
            var hoops = Hoops.Hoops;
            if (hoops == null || hoops.Count == 0) return -1;

            int best = -1;
            for (int i = 0; i < hoops.Count; i++)
            {
                var h = hoops[i];
                if (h == null) continue;
                float d = Vector3.Distance(h.transform.position, vehicleAim.position);
                if (d < lastApproachNearest) lastApproachNearest = d;
                if (skipAlreadyFilmed && i <= LastFilmedHoopIndex) { lastApproachSkippedFilmed++; continue; }
                if (d > maxRange) { lastApproachOutOfRange++; continue; }
                // The hoop's +Z is the direction of travel through it, so a vehicle that has not
                // reached it yet is on the -Z side.
                if (Vector3.Dot(h.transform.position - vehicleAim.position, h.transform.forward) <= 0f)
                { lastApproachAstern++; continue; }
                if (d < distance) { distance = d; best = i; }
            }
            return best;
        }

        /// <summary>
        /// The object an `AdvanceWhen.TargetAstern` shot is flying past, resolved once and cached
        /// onto the shot. SCENE-WIDE by design and not under the vehicle: the thing being passed is
        /// a fixture on the seabed, not part of the hull. It says what it found and where, once,
        /// because "the cut never came" and "the cut aimed at the wrong object" look identical from
        /// the frame — which is the 2026-08-21 wrong-end-of-the-dock lesson in a new place.
        /// </summary>
        Transform ResolveAsternTarget(CameraShot shot)
        {
            if (shot.AsternTarget != null) return shot.AsternTarget;
            if (string.IsNullOrEmpty(shot.AsternTargetName)) return null;
            var go = GameObject.Find(shot.AsternTargetName);
            if (go == null) return null;
            shot.AsternTarget = go.transform;
            Debug.Log($"[CinematicDirector] '{shot.Name}' fly-past target '{shot.AsternTargetName}' " +
                      $"resolved scene-wide to '{HierarchyPath(go.transform)}' at {go.transform.position}. " +
                      "If the cut lands on the wrong thing, this line names what it latched onto.");
            return shot.AsternTarget;
        }

        /// <summary>
        /// Is `target` behind the vehicle? HORIZONTAL test — the vehicle's forward tilts with its
        /// pitch and it is pitching through this whole shot, so a 3-D dot product would make the
        /// cut depend on the dive angle. `along` is the signed along-track distance in metres,
        /// positive once the target is astern, which is what the overlay counts down.
        /// </summary>
        bool TargetIsAstern(Transform target, out float along, out float lateral)
        {
            along = 0f; lateral = 0f;
            if (target == null || vehicleAim == null) return false;
            Vector3 fwd = vehicleAim.forward; fwd.y = 0f;
            if (fwd.sqrMagnitude < 1e-6f) return false;
            fwd.Normalize();
            Vector3 d = vehicleAim.position - target.position; d.y = 0f;
            along = Vector3.Dot(d, fwd);
            lateral = Vector3.Cross(Vector3.up, fwd).sqrMagnitude > 0f
                    ? Vector3.Dot(d, Vector3.Cross(Vector3.up, fwd).normalized) : 0f;
            return along > 0f;
        }

        bool HoopIsAstern(int index)
        {
            if (Hoops == null || vehicleAim == null) return false;
            var hoops = Hoops.Hoops;
            if (hoops == null || index < 0 || index >= hoops.Count) return false;
            var h = hoops[index];
            if (h == null) return false;
            return Vector3.Dot(vehicleAim.position - h.transform.position, h.transform.forward) > 0f;
        }

        // ---------------------------------------------------------------- advance

        float EffectiveMaxSeconds(CameraShot shot)
        {
            if (shot.MaxSeconds > 0f) return shot.MaxSeconds;
            if (shot.EffectiveAdvance() == CameraShot.AdvanceWhen.Manual) return 0f;
            return Mathf.Max(0f, FallbackMaxSeconds);
        }

        /// <summary>
        /// Decides whether this shot ends now, and says why. `waitText` is left holding what the
        /// shot is waiting for, in words, for the overlay — Ivan directs by reading that during a
        /// rehearsal, so "waiting" has to be a sentence and not a boolean.
        /// </summary>
        bool ShouldAdvance(CameraShot shot, float t, out string reason)
        {
            reason = null;
            var mode = shot.EffectiveAdvance();
            float maxS = EffectiveMaxSeconds(shot);
            bool isLast = CurrentShot >= Shots.Count - 1;

            string tail = maxS > 0f ? $" (falls through at {maxS:F0} s)" : "";

            if (!AutoAdvanceEnabled) { waitText = $"AUTO OFF — {NextShotKey} to cut"; return false; }
            if (isLast) { waitText = $"last shot — holds until {NextShotKey} or {ToggleCinematicKey}"; return false; }

            float floor = Mathf.Max(MinShotSeconds, shot.MinSeconds);
            if (t < floor)
            {
                waitText = mode == CameraShot.AdvanceWhen.Manual
                    ? $"{NextShotKey} to cut"
                    : preSatisfied
                        ? $"condition PRE-SATISFIED ({preSatisfiedWhy}) — cutting at the {floor:F1} s minimum, {floor - t:F1} s to go"
                        : $"{floor - t:F1} s minimum, then: {Describe(mode, shot)}";
                return false;
            }
            if (Time.time < autoHeldUntil)
            {
                waitText = $"manual hold {autoHeldUntil - Time.time:F1} s, then: {Describe(mode, shot)}";
                return false;
            }

            // ROUND 3 — THE MOOT-CONDITION RULE. If the fact this shot waits for was already true, or
            // can no longer happen, at the moment the shot STARTED, the shot hands over as soon as its
            // minimum is served instead of sitting out the ceiling. Take 006 lost about six minutes to
            // exactly this: the vehicle had surfaced before shot 7 began, so "waiting for the ascent"
            // and then "waiting for the surface break" were both true sentences about an event that
            // was never going to arrive. Duration is excluded on purpose — it times a CAMERA MOVE, and
            // cutting a move short is a different kind of wrong.
            if (preSatisfied && mode != CameraShot.AdvanceWhen.Manual && mode != CameraShot.AdvanceWhen.Duration)
            {
                reason = $"condition pre-satisfied at shot start — {preSatisfiedWhy}; advanced at the " +
                         $"{floor:F1} s minimum instead of waiting out the " +
                         (maxS > 0f ? $"{maxS:F0} s ceiling" : "ceiling");
                return true;
            }

            if (mode != CameraShot.AdvanceWhen.Manual)
            {
                bool raw = RawCondition(shot, mode, t, out var detail);
                if (raw)
                {
                    if (conditionTrueSince < 0f) conditionTrueSince = Time.time;
                    float held = Time.time - conditionTrueSince;
                    if (held >= Mathf.Max(0f, shot.SteadySeconds))
                    {
                        reason = $"{mode} met at {t:F1} s ({detail})";
                        return true;
                    }
                    waitText = $"{detail} — holding {held:F1}/{shot.SteadySeconds:F1} s{tail}";
                    return false;
                }
                conditionTrueSince = -1f;
                waitText = $"{detail}{tail}";
            }
            else
            {
                waitText = $"{NextShotKey} to cut{tail}";
            }

            if (maxS > 0f && t >= maxS)
            {
                reason = mode == CameraShot.AdvanceWhen.Duration
                    ? $"duration {maxS:F1} s elapsed"
                    : $"FELL THROUGH at {maxS:F1} s — '{mode}' never happened ({waitText})";
                return true;
            }
            return false;
        }

        static string Describe(CameraShot.AdvanceWhen mode, CameraShot s)
        {
            switch (mode)
            {
                case CameraShot.AdvanceWhen.Duration: return $"duration {s.Duration:F1} s";
                case CameraShot.AdvanceWhen.VehicleMoving: return $"vehicle moving >= {s.MoveSpeedThreshold:F2} m/s";
                case CameraShot.AdvanceWhen.VehicleSubmerged: return $"vehicle {s.SubmergeDepth:F2} m under";
                case CameraShot.AdvanceWhen.VehicleSurfaced: return "vehicle breaking the surface";
                case CameraShot.AdvanceWhen.VehicleAscending: return $"vehicle climbing >= {s.AscentRate:F2} m/s";
                case CameraShot.AdvanceWhen.HoopCleared: return $"{s.HoopsToClear} more waypoint(s) cleared";
                case CameraShot.AdvanceWhen.ApproachingHoop: return $"a new hoop within {s.ApproachRange:F0} m";
                case CameraShot.AdvanceWhen.HoopPassed: return "the latched hoop to go astern";
                case CameraShot.AdvanceWhen.HoopsIdle: return $"no new waypoint for {s.IdleSeconds:F0} s";
                case CameraShot.AdvanceWhen.VehicleIdle: return $"vehicle stopped for {s.IdleSeconds:F0} s";
                case CameraShot.AdvanceWhen.VehicleSurfacedAndIdle:
                    return $"vehicle surfaced AND stopped for {s.IdleSeconds:F0} s";
                case CameraShot.AdvanceWhen.DrainComplete: return "the dock to finish draining";
                case CameraShot.AdvanceWhen.TargetAstern:
                    return $"'{(s.AsternTarget != null ? s.AsternTarget.name : s.AsternTargetName)}' to go astern";
                default: return "the Next key";
            }
        }

        /// <summary>
        /// Was this shot's condition ALREADY TRUE, or already MOOT, at the moment the shot started?
        ///
        /// Evaluated ONCE, in BeginShot, and only from facts that have actually been measured —
        /// `hasLastVehiclePos` guards the case where the director has never seen the vehicle move, in
        /// which case "the vehicle is idle" is an artefact of a zeroed smoother and not an
        /// observation. Two conditions are MOOT rather than true, and they are the two that cost take
        /// 006 its ending: `VehicleAscending` and `VehicleSurfaced` cannot occur for a vehicle that is
        /// already on the surface and has not been under during this shot.
        ///
        /// It deliberately does NOT pre-satisfy the plan-progression conditions (ApproachingHoop,
        /// HoopPassed, HoopCleared, HoopsIdle): those describe something the mission is about to do,
        /// their "already true" case is a legitimate immediate cut, and the MinSeconds floor already
        /// handles it. Nor `Duration`, which is a camera move.
        /// </summary>
        bool ConditionPreSatisfied(CameraShot shot, CameraShot.AdvanceWhen mode, out string why)
        {
            why = "";
            if (vehicleAim == null || !hasLastVehiclePos) return false;

            switch (mode)
            {
                case CameraShot.AdvanceWhen.VehicleMoving:
                    if (smoothedSpeed < shot.MoveSpeedThreshold) return false;
                    why = $"the vehicle was ALREADY moving ({smoothedSpeed:F2} >= {shot.MoveSpeedThreshold:F2} m/s)";
                    return true;

                case CameraShot.AdvanceWhen.VehicleIdle:
                    if (smoothedSpeed >= shot.MoveSpeedThreshold) return false;
                    why = $"the vehicle was ALREADY stopped ({smoothedSpeed:F2} < {shot.MoveSpeedThreshold:F2} m/s)";
                    return true;

                case CameraShot.AdvanceWhen.VehicleSubmerged:
                    if (!HasWaterPlane || VehicleDepth < shot.SubmergeDepth) return false;
                    why = $"the vehicle was ALREADY {VehicleDepth:F2} m under (needs {shot.SubmergeDepth:F2})";
                    return true;

                case CameraShot.AdvanceWhen.VehicleAscending:
                    // MOOT, not true: a vehicle on the surface has nothing left to climb.
                    if (!HasWaterPlane || VehicleDepth > shot.SurfaceDepth) return false;
                    why = $"the ascent is MOOT — the vehicle was already at the surface ({VehicleDepth:F2} m down) " +
                          "when this shot started, so it can never be seen to climb during it";
                    return true;

                case CameraShot.AdvanceWhen.VehicleSurfaced:
                    // MOOT for the same reason, and specifically because this condition requires the
                    // vehicle to have been UNDER during this shot — which a surfaced vehicle will not be.
                    if (!HasWaterPlane || submergedDuringShot || VehicleDepth > shot.SurfaceDepth) return false;
                    why = $"the surface break is MOOT — the vehicle was already up ({VehicleDepth:F2} m down) " +
                          "and never went under during this shot";
                    return true;

                case CameraShot.AdvanceWhen.VehicleSurfacedAndIdle:
                    if (!HasWaterPlane) return false;
                    if (VehicleDepth > shot.SurfaceDepth || smoothedSpeed >= shot.MoveSpeedThreshold) return false;
                    // Both halves true at shot start; the IdleSeconds hold is what MinSeconds now serves.
                    why = $"the run was ALREADY over at shot start — {VehicleDepth:F2} m down, {smoothedSpeed:F2} m/s";
                    return true;

                case CameraShot.AdvanceWhen.TargetAstern:
                    {
                        // ALREADY TRUE, not moot: the vehicle is past the thing this shot was going
                        // to film it passing. Waiting out a 180 s ceiling for a pass that has already
                        // happened is precisely the take-006 dead air, in a new condition — which is
                        // why this one participates in the rule rather than being an exception to it.
                        var tgt = ResolveAsternTarget(shot);
                        if (tgt == null) return false;
                        if (!TargetIsAstern(tgt, out float along, out _)) return false;
                        why = $"'{tgt.name}' was ALREADY {along:F1} m astern when this shot started — " +
                              "the pass is behind us, not ahead";
                        return true;
                    }
            }
            return false;
        }

        /// <summary>
        /// The condition itself, plus the words the overlay shows. When a fact is simply not
        /// available — no vehicle, no water plane, no hoops yet — this says so rather than
        /// returning false and letting the shot look like it is waiting for something that is
        /// almost here. The ceiling is what rescues those cases, on purpose.
        /// </summary>
        bool RawCondition(CameraShot shot, CameraShot.AdvanceWhen mode, float t, out string detail)
        {
            bool needsVehicle = mode != CameraShot.AdvanceWhen.Duration;
            if (needsVehicle && vehicleAim == null)
            {
                detail = $"waiting: {Describe(mode, shot)} — NO VEHICLE '{VehicleName}' in the scene, cannot tell";
                return false;
            }

            switch (mode)
            {
                case CameraShot.AdvanceWhen.Duration:
                    detail = $"duration {t:F1}/{shot.Duration:F1} s";
                    return t >= shot.Duration;

                case CameraShot.AdvanceWhen.VehicleMoving:
                    detail = $"waiting: vehicle moving — {smoothedSpeed:F2} of {shot.MoveSpeedThreshold:F2} m/s";
                    return smoothedSpeed >= shot.MoveSpeedThreshold;

                case CameraShot.AdvanceWhen.VehicleIdle:
                    detail = $"waiting: vehicle stopped — {smoothedSpeed:F2} m/s";
                    return smoothedSpeed < shot.MoveSpeedThreshold;

                case CameraShot.AdvanceWhen.VehicleSurfacedAndIdle:
                    {
                        // THE RUN-IS-OVER FACT. Both halves, or neither: a vehicle sitting still at
                        // 12 m is not finished, and one crossing the surface at 0.4 m/s is not
                        // finished either. Same pair as the recorder-stop policy (§3s6), and it is
                        // what the dock drain is gated on.
                        if (!HasWaterPlane)
                        { detail = "waiting: surfaced AND idle — NO WaterSurface, cannot tell"; return false; }
                        bool up = VehicleDepth <= shot.SurfaceDepth;
                        bool still = smoothedSpeed < shot.MoveSpeedThreshold;
                        if (up && still) surfacedIdleSince = surfacedIdleSince < 0f ? Time.time : surfacedIdleSince;
                        else surfacedIdleSince = -1f;
                        float held = surfacedIdleSince < 0f ? 0f : Time.time - surfacedIdleSince;
                        detail = $"waiting: surfaced AND idle — {VehicleDepth:F2} m down " +
                                 $"(needs <= {shot.SurfaceDepth:F2}), {smoothedSpeed:F2} m/s " +
                                 $"(needs < {shot.MoveSpeedThreshold:F2}), held {held:F1}/{shot.IdleSeconds:F0} s";
                        return up && still && held >= shot.IdleSeconds;
                    }

                case CameraShot.AdvanceWhen.VehicleSubmerged:
                    if (!HasWaterPlane) { detail = "waiting: vehicle submerged — NO WaterSurface, cannot tell"; return false; }
                    detail = $"waiting: vehicle submerged — {VehicleDepth:F2} of {shot.SubmergeDepth:F2} m under";
                    return VehicleDepth >= shot.SubmergeDepth;

                case CameraShot.AdvanceWhen.VehicleAscending:
                    if (!HasWaterPlane) { detail = "waiting: vehicle ascending — NO WaterSurface, cannot tell"; return false; }
                    detail = $"waiting: the ascent — vertical {smoothedVertRate:F2} m/s " +
                             $"(needs +{shot.AscentRate:F2}), at {VehicleDepth:F1} m down";
                    return smoothedVertRate >= shot.AscentRate;

                case CameraShot.AdvanceWhen.VehicleSurfaced:
                    if (!HasWaterPlane) { detail = "waiting: the surface break — NO WaterSurface, cannot tell"; return false; }
                    if (!submergedDuringShot)
                    {
                        detail = $"waiting: the surface break — the vehicle has not been under during this shot ({VehicleDepth:F2} m)";
                        return false;
                    }
                    detail = $"waiting: the surface break — {VehicleDepth:F2} m under, needs {shot.SurfaceDepth:F2}";
                    return VehicleDepth <= shot.SurfaceDepth;

                case CameraShot.AdvanceWhen.HoopCleared:
                    {
                        int gained = HoopCount - hoopCountAtShotStart;
                        detail = Hoops == null
                            ? "waiting: a waypoint cleared — NO MissionWPHoop_Sub in the scene, cannot tell"
                            : $"waiting: {shot.HoopsToClear} waypoint(s) cleared — {gained} so far ({HoopCount} hoops)";
                        return Hoops != null && gained >= Mathf.Max(1, shot.HoopsToClear);
                    }

                case CameraShot.AdvanceWhen.ApproachingHoop:
                    {
                        if (Hoops == null) { detail = "waiting: a hoop to approach — NO MissionWPHoop_Sub, cannot tell"; return false; }
                        int idx = ApproachHoop(shot.ApproachRange, true, out float d);
                        if (idx < 0)
                        {
                            // REWRITTEN 2026-08-21. The old wording could read "none (0 hoops, last
                            // filmed 1)" — a sentence whose two halves contradict each other and
                            // that cost an evening of "the list emptied mid-mission". It now says
                            // how many hoops exist AND why each was rejected, so the next reader
                            // gets the diagnosis off the frame instead of out of the source.
                            detail = HoopCount == 0
                                ? "waiting: approaching a hoop — THE HOOP LIST IS EMPTY (no waypoint has been " +
                                  $"cleared to fly yet; last clear: {(Hoops != null ? Hoops.LastClearReason : "n/a")})"
                                : $"waiting: approaching a NEW hoop within {shot.ApproachRange:F0} m — " +
                                  $"{HoopCount} hoop(s) exist; {lastApproachSkippedFilmed} already filmed " +
                                  $"(index <= {LastFilmedHoopIndex}), {lastApproachOutOfRange} out of range " +
                                  $"(nearest {(lastApproachNearest < float.MaxValue ? lastApproachNearest.ToString("F1") + " m" : "n/a")}), " +
                                  $"{lastApproachAstern} already astern";
                            return false;
                        }
                        detail = $"waiting: approaching hoop {idx + 1} of {HoopCount} — {d:F1} of {shot.ApproachRange:F0} m";
                        return true;
                    }

                case CameraShot.AdvanceWhen.HoopPassed:
                    {
                        int idx = wpCurrentIndex;
                        if (idx < 0) { detail = "waiting: a hoop to latch onto — none in range yet"; return false; }
                        if (!wpPassed)
                        {
                            float d = Hoops != null && Hoops.Hoops != null && idx < Hoops.Hoops.Count && Hoops.Hoops[idx] != null
                                ? Vector3.Distance(Hoops.Hoops[idx].transform.position, vehicleAim.position) : -1f;
                            detail = $"waiting: hoop {idx + 1} to go astern — {d:F1} m out, still ahead";
                            return false;
                        }
                        detail = $"hoop {idx + 1} is astern — panning for {Time.time - wpPassedTime:F1} s";
                        return true;
                    }

                case CameraShot.AdvanceWhen.DrainComplete:
                    {
                        if (Drain == null)
                        {
                            detail = "waiting: the dock to drain — NO DockDrainDirector in the scene, cannot tell. " +
                                     "Run SMARC/Video/2.";
                            return false;
                        }
                        if (!drainRequested)
                        {
                            detail = "waiting: the dock to drain — but THIS SHOT NEVER ASKED FOR ONE. Tick " +
                                     "'Start Drain At Shot Start' on it, or give it a different Advance.";
                            return false;
                        }
                        if (!drainBegun)
                        {
                            // The gate's own sentence, in metres and m/s — not a boolean.
                            detail = "waiting: the dock to drain — the pump has NOT started: " + drainStatus;
                            return false;
                        }
                        detail = $"waiting: the dock to finish draining — {Drain.Progress01 * 100f:F0}% pumped out";
                        return Drain.Progress01 >= 1f;
                    }

                case CameraShot.AdvanceWhen.TargetAstern:
                    {
                        var tgt = ResolveAsternTarget(shot);
                        if (tgt == null)
                        {
                            // Named and not found is a different failure from "not named at all",
                            // and only one of them is a configuration mistake.
                            detail = string.IsNullOrEmpty(shot.AsternTargetName) && shot.AsternTarget == null
                                ? "waiting: something to go astern — THIS SHOT NAMES NOTHING TO PASS. Set " +
                                  "Astern Target (or Astern Target Name) on it, or give it a different Advance."
                                : $"waiting: '{shot.AsternTargetName}' to go astern — NO OBJECT OF THAT NAME " +
                                  "in the scene, so this shot can only fall through on its ceiling. " +
                                  "Run SMARC/Video/A1.";
                            return false;
                        }
                        bool astern = TargetIsAstern(tgt, out float along, out float lateral);
                        detail = astern
                            ? $"'{tgt.name}' is astern — {along:F1} m behind, {Mathf.Abs(lateral):F1} m " +
                              $"to {(lateral > 0f ? "port" : "starboard")}, running on for " +
                              $"{shot.SteadySeconds:F1} s"
                            : $"waiting: '{tgt.name}' to go astern — {-along:F1} m ahead, " +
                              $"{Mathf.Abs(lateral):F1} m to {(lateral > 0f ? "port" : "starboard")}";
                        return astern;
                    }

                case CameraShot.AdvanceWhen.HoopsIdle:
                    {
                        if (Hoops == null) { detail = "waiting: the plan to stop growing — NO MissionWPHoop_Sub, cannot tell"; return false; }
                        float quiet = Time.time - lastHoopChangeTime;
                        detail = $"waiting: no new waypoint for {shot.IdleSeconds:F0} s — {quiet:F1} s quiet";
                        return quiet >= shot.IdleSeconds;
                    }
            }
            detail = "waiting: the Next key";
            return false;
        }

        // ---------------------------------------------------------------- mode

        public void EnterCinematicMode()
        {
            ResolveSceneHooks();
            EnsureCamera();
            SuppressOtherCameras();
            HideChrome();
            CinematicMode = true;
            hasLastVehiclePos = false;
            smoothedSpeed = 0f;
            smoothedVertRate = 0f;
            LastFilmedHoopIndex = -1;
            lastHoopCount = HoopCount;
            lastHoopChangeTime = Time.time;
            lastCutReason = "";
            surfacedIdleSince = -1f;
            drainRequested = false;
            drainBegun = false;
            drainStatus = "";
            if (Drain != null) Drain.Restore();
            // Remember the look the SCENE was saved with, before any shot changes it. This is read
            // once per entry into cinematic mode, so F9-off always puts back what F9-on found.
            editModePresetName = WaterPreset != null ? WaterPreset.ActivePreset : "";
            appliedPresetName = "";
            // Take the waterfall panel — and with it, F6 — for the duration. See the field's tooltip
            // and SSSWaterfallHUD.SetDirectorControl for why the key is borrowed rather than moved.
            if (Waterfall != null)
            {
                waterfallWasOpen = Waterfall.open;
                Waterfall.SetDirectorControl(true, name);
                waterfallTaken = true;
            }
            if (Particles != null) Particles.SetCamera(CinematicCamera);
            BeginShot(CurrentShot);
            Debug.Log($"[CinematicDirector] cinematic mode ON — {Shots.Count} shot(s), auto-advance " +
                      $"{(AutoAdvanceEnabled ? "ON: press record ONCE and let it run" : "OFF: every shot is manual")}. " +
                      $"{NextShotKey}=next {PrevShotKey}=prev {RestartShotKey}=restart {ToggleHudKey}=HUD " +
                      $"{ToggleSonarMapKey}=map {ClearSonarMapKey}=clear map {ToggleOverlayKey}=overlay " +
                      $"{ToggleAutoAdvanceKey}=auto on/off {ToggleCinematicKey}=exit.");
        }

        public void ExitCinematicMode()
        {
            CinematicMode = false;
            if (CinematicCamera != null) CinematicCamera.enabled = false;
            RestoreOtherCameras();
            RestoreChrome();
            ApplyHud(true);
            ApplyBeams(false, false);
            // Whatever else happens on the way out, the water goes back to where §3s requires it —
            // both its LEVEL (the drain) and its LOOK (the per-shot preset).
            ReleaseDrain();
            RestoreWaterPreset();
            ReleaseWaterfall();
            if (Particles != null) Particles.SetCamera(null);
            Debug.Log("[CinematicDirector] cinematic mode OFF — cameras, GUI, HUD, sonar beams, " +
                      "the waterfall panel and its key, the water level and the water look all restored.");
        }

        /// <summary>Give the waterfall panel, its key and its open/closed state back. Idempotent.</summary>
        void ReleaseWaterfall()
        {
            if (!waterfallTaken) return;
            waterfallTaken = false;
            if (Waterfall == null) return;
            Waterfall.SetDirectorControl(false, name);
            Waterfall.open = waterfallWasOpen;
        }

        // Belt and braces: leaving Play, deleting the director, or disabling it must not leave the
        // dock drained, the water wearing a preset the scene was not saved with, or the waterfall
        // panel deaf to its own key with no director left to press anything. DockDrainDirector
        // restores itself too; every one of these paths is cheap and none is allowed to be the only one.
        void OnDisable() { ReleaseDrain(); RestoreWaterPreset(); ReleaseWaterfall(); }

        void EnsureCamera()
        {
            if (CinematicCamera == null)
            {
                var go = new GameObject("CinematicCam");
                go.transform.SetParent(transform, false);
                CinematicCamera = go.AddComponent<Camera>();
                // Deliberately NO AudioListener: a second listener in the scene makes Unity warn
                // every frame and the existing GUI camera already has one.
            }
            CinematicCamera.enabled = true;
            CinematicCamera.depth = CameraDepth;
            CinematicCamera.targetTexture = null;
        }

        void SuppressOtherCameras()
        {
            suppressedCameras.Clear();
            suppressedScripts.Clear();
            foreach (var c in Camera.allCameras)
            {
                if (c == CinematicCamera) continue;
                if (!c.enabled) continue;
                if (c.targetTexture != null) continue;   // renders to a texture, not to the screen
                c.enabled = false;
                suppressedCameras.Add(c);

                foreach (var mb in c.GetComponents<MonoBehaviour>())
                {
                    if (mb == null || !mb.enabled) continue;
                    if (!DisableScriptsOnOtherCameras.Contains(mb.GetType().Name)) continue;
                    mb.enabled = false;
                    suppressedScripts.Add(mb);
                }
            }
            if (suppressedCameras.Count > 0)
                Debug.Log($"[CinematicDirector] took over from {suppressedCameras.Count} camera(s); " +
                          $"disabled {suppressedScripts.Count} interaction script(s). Restored on exit.");
        }

        void RestoreOtherCameras()
        {
            foreach (var c in suppressedCameras) if (c != null) c.enabled = true;
            foreach (var m in suppressedScripts) if (m != null) m.enabled = true;
            suppressedCameras.Clear();
            suppressedScripts.Clear();
        }

        void HideChrome()
        {
            hiddenChrome.Clear();
            foreach (var n in AlwaysHideNames)
            {
                var go = GameObject.Find(n);
                if (go == null || !go.activeSelf) continue;
                go.SetActive(false);
                hiddenChrome.Add(go);
            }
        }

        void RestoreChrome()
        {
            foreach (var go in hiddenChrome) if (go != null) go.SetActive(true);
            hiddenChrome.Clear();
        }

        bool CurrentHudState()
        {
            foreach (var go in HudRoots) if (go != null) return go.activeSelf;
            return false;
        }

        void ApplyHud(bool on)
        {
            foreach (var go in HudRoots) if (go != null) go.SetActive(on);
        }

        // ---------------------------------------------------------------- shots

        public void NextShot() { HoldAuto("NextShot() called"); BeginShot(CurrentShot + 1); }
        public void PrevShot() { HoldAuto("PrevShot() called"); BeginShot(CurrentShot - 1); }

        public void BeginShot(int index)
        {
            if (Shots.Count == 0) return;

            // Remember where the previous shot left the lens, so BlendSeconds has something to
            // ease out of.
            if (CinematicCamera != null)
            {
                blendFromPos = CinematicCamera.transform.position;
                blendFromRot = CinematicCamera.transform.rotation;
                hasBlendFrom = true;
            }

            CurrentShot = Mathf.Clamp(index, 0, Shots.Count - 1);
            var shot = Shots[CurrentShot];
            shotStartTime = Time.time;

            // Every piece of per-shot advance state is cleared here. That is what makes a manual
            // advance CANCEL a pending automatic one rather than race it.
            conditionTrueSince = -1f;
            hoopCountAtShotStart = HoopCount;
            submergedDuringShot = HasWaterPlane && vehicleAim != null && VehicleDepth > 0.05f;
            waitText = "";

            ApplyHud(shot.HudVisible);
            if (Particles != null) Particles.Enabled = shot.ParticlesEnabled;
            if (SonarMap != null)
            {
                if (shot.ClearMapAtStart) SonarMap.ClearMap();
                SonarMap.Visible = shot.SonarMapVisible;
                SonarMap.ShowLabel = shot.SonarMapLabelVisible;
            }
            ApplyBeams(shot.SonarBeamsVisible, shot.SonarRayLinesVisible);
            // The waterfall panel is a per-shot instrument like the HUD and the map label. Only
            // while the director actually holds it: jumping shots with F10 outside cinematic mode
            // cannot happen, but a director that failed to take the panel must not silently drive it.
            if (waterfallTaken && Waterfall != null) Waterfall.open = shot.ShowSSSWaterfall;

            // Per-shot setup that has to happen once, not every frame.
            wpCurrentIndex = -1;
            wpLastCutTime = -999f;
            wpPassed = false;
            surfacedIdleSince = -1f;

            // Leaving a draining shot puts the water back. The drain must never outlive the shot that
            // asked for it — a scene left with the water 7 m down is the §3s trap, armed. Round 3:
            // "the shot that asked for it" is now either a DrainDock shot OR any shot with
            // StartDrainAtShotStart, because the drain runs under the zoom-out.
            // …and "the shot that asked for it" now includes the shot that flies through the RESULT.
            // Round 2's rule refilled the dock on the first frame of the fly-through, which is the
            // one shot in the storyboard whose entire subject is the dock being empty.
            bool wantsDrain = shot.Kind == CameraShot.Mode.DrainDock || shot.StartDrainAtShotStart;
            if (!wantsDrain && !shot.KeepDockDrained) ReleaseDrain();
            // Only a shot that ASKED starts one. KeepDockDrained means "do not refill", never
            // "start draining" — otherwise jumping straight to the fly-through with F10 would begin
            // a drain that shot never requested.
            if (wantsDrain && !drainRequested)
            {
                drainRequested = true;
                drainBegun = false;
                nextDrainPoll = 0f;                 // poll on the very next frame
                if (Drain == null)
                {
                    drainStatus = "NO DockDrainDirector in the scene — nothing to drain; this shot plays " +
                                  "over a full dock. Run SMARC/Video/2 to add one.";
                    Debug.LogWarning("[CinematicDirector] " + drainStatus);
                }
                else
                {
                    drainStatus = "drain requested — polling the surfaced-and-idle gate";
                    Debug.Log($"[CinematicDirector] '{shot.Name}' asked for the dock drain at shot start; the " +
                              "gate (surfaced AND idle) is polled twice a second until it opens. " +
                              "The camera does not wait for it.");
                }
            }

            // The water LOOK for this shot. "" means leave it exactly as the scene was saved.
            ApplyWaterPreset(shot.WaterPresetName);

            if (shot.Kind == CameraShot.Mode.OrbitZoomOut)
            {
                orbitCenterWS = shot.OrbitCenter != null ? shot.OrbitCenter.position
                              : vehicleAim != null ? vehicleAim.position
                              : Vehicle != null ? Vehicle.position
                              : transform.position;
                ResolveOrbitEnd(shot);
            }
            if (shot.Kind == CameraShot.Mode.DollySpline) BuildDollyPath(shot);

            // ROUND 3: is the fact this shot waits for already true, or already moot? Decided here,
            // once, from measured state — see ConditionPreSatisfied. Never for the first shot after
            // entering cinematic mode, because nothing has been measured yet (hasLastVehiclePos).
            var mode0 = shot.EffectiveAdvance();
            preSatisfied = AutoAdvanceEnabled && ConditionPreSatisfied(shot, mode0, out preSatisfiedWhy);
            if (!preSatisfied) preSatisfiedWhy = "";

            Debug.Log($"[CinematicDirector] shot {CurrentShot + 1}/{Shots.Count}: '{shot.Name}' ({shot.Kind}), " +
                      $"{shot.AdvanceSummary()}, HUD {(shot.HudVisible ? "on" : "off")}, " +
                      $"map {(shot.SonarMapVisible ? "on" : "off")}, " +
                      $"waterfall {(shot.ShowSSSWaterfall ? "UP" : "down")}" +
                      (string.IsNullOrEmpty(shot.WaterPresetName) ? "" : $", water '{shot.WaterPresetName}'") +
                      (preSatisfied ? $".\n  CONDITION PRE-SATISFIED: {preSatisfiedWhy} — this shot will hand " +
                                      $"over at its {Mathf.Max(MinShotSeconds, shot.MinSeconds):F1} s minimum " +
                                      "rather than wait out its ceiling."
                                    : "."));
        }

        /// <summary>
        /// Resolve the orbit's END pose from `OrbitEndAnchor`, once, at shot start.
        ///
        /// The bearing, radius and height are DERIVED from where the anchor stands relative to this
        /// shot's orbit centre, so the sweep finishes exactly on the anchor rather than wherever
        /// StartBearingDeg + DegreesSwept happened to land. That is what makes "look straight along
        /// the centre of the dry dock from south" an authored END BEARING (Ivan, round 3) instead of
        /// a generic turntable: the anchor is placed on the dock axis by SMARC/Video/2c, so the
        /// bearing comes from the dock, not from a constant.
        /// </summary>
        void ResolveOrbitEnd(CameraShot shot)
        {
            orbitEndFromAnchor = false;
            if (shot.OrbitEndAnchor == null) return;

            Vector3 d = shot.OrbitEndAnchor.position - orbitCenterWS;
            Vector3 flat = new Vector3(d.x, 0f, d.z);
            if (flat.sqrMagnitude < 1e-4f)
            {
                Debug.LogWarning($"[CinematicDirector] '{shot.Name}' has an OrbitEndAnchor sitting directly over " +
                                 "the orbit centre, so it defines no bearing — falling back to " +
                                 "EndRadius/EndHeight/DegreesSwept as authored.");
                return;
            }
            orbitEndRadius = flat.magnitude;
            orbitEndHeight = d.y;
            orbitEndBearingDeg = Mathf.Atan2(flat.x, flat.z) * Mathf.Rad2Deg;
            orbitEndFromAnchor = true;

            Debug.Log($"[CinematicDirector] '{shot.Name}' orbit END taken from '{HierarchyPath(shot.OrbitEndAnchor)}': " +
                      $"bearing {orbitEndBearingDeg:F0}° from world +Z, radius {orbitEndRadius:F0} m, " +
                      $"height {orbitEndHeight:F0} m above the centre at {orbitCenterWS}. It sweeps " +
                      $"{shot.DegreesSwept:F0}° to get there, so it starts at " +
                      $"{orbitEndBearingDeg - shot.DegreesSwept:F0}°" +
                      (shot.OrbitEndLookAt != null
                          ? $", and the aim eases from the centre to '{HierarchyPath(shot.OrbitEndLookAt)}'."
                          : ". No OrbitEndLookAt, so it keeps looking at the orbit centre."));
        }

        static int CountNodes(CameraShot s)
        {
            if (s.Nodes == null) return 0;
            int n = 0;
            foreach (var t in s.Nodes) if (t != null) n++;
            return n;
        }

        /// <summary>
        /// Resolve the dolly's authored nodes into world points ONCE, at the moment the shot
        /// starts, and — this is the fix for the fly-in aiming at the wrong end of the dock —
        /// move the whole path so it ENDS on the vehicle wherever the mission happens to begin.
        ///
        /// It TRANSLATES (and optionally rotates about the end point). It does not re-derive the
        /// shape: the shape is what Ivan drags in the Scene view and it has to survive. The seeded
        /// path used to be laid out from the water quad's extents, and at Beckholmen that quad is
        /// centred near the world origin while the vehicle starts at roughly (61, 112) — about
        /// 130 m away — so the dolly was flying over open harbour and aiming, via a scene-wide
        /// `GameObject.Find("base_link")`, at whatever base_link it found first.
        /// </summary>
        void BuildDollyPath(CameraShot shot)
        {
            dollyPts.Clear();
            if (shot.Nodes != null)
                foreach (var n in shot.Nodes) if (n != null) dollyPts.Add(n.position);

            if (dollyPts.Count < 2)
            {
                Debug.LogWarning($"[CinematicDirector] shot '{shot.Name}' is a DollySpline with fewer than two " +
                                 "nodes — it will hold at the camera's current pose. Drag path nodes into Nodes.");
                return;
            }
            if (!shot.AnchorEndToTarget) return;

            // `==` and not `??`: Unity overloads the equality operator so a DESTROYED object
            // compares equal to null, and `??` bypasses that overload entirely.
            var target = ResolveLookAt(shot);
            if (target == null) target = vehicleAim;
            if (target == null)
            {
                Debug.LogWarning($"[CinematicDirector] shot '{shot.Name}' wants to anchor its path to the vehicle " +
                                 "but no target could be resolved — the authored path is used as-is, which is " +
                                 "exactly the case that aimed the fly-in at the wrong end of the dock.");
                return;
            }

            int last = dollyPts.Count - 1;
            Vector3 have = dollyPts[last];

            float rotatedDeg = 0f;
            if (shot.AlignPathToTargetHeading)
            {
                Vector3 authored = dollyPts[last] - dollyPts[last - 1];
                authored.y = 0f;
                Vector3 wantedDir = target.forward;
                wantedDir.y = 0f;
                if (authored.sqrMagnitude > 1e-6f && wantedDir.sqrMagnitude > 1e-6f)
                {
                    rotatedDeg = Vector3.SignedAngle(authored.normalized, wantedDir.normalized, Vector3.up);
                    var q = Quaternion.AngleAxis(rotatedDeg, Vector3.up);
                    for (int i = 0; i < dollyPts.Count; i++)
                        dollyPts[i] = have + q * (dollyPts[i] - have);
                }
            }

            Quaternion frame = Quaternion.Euler(0f, target.eulerAngles.y, 0f);
            Vector3 want = target.position + frame * shot.EndOffsetInTargetFrame;
            Vector3 delta = want - dollyPts[last];
            for (int i = 0; i < dollyPts.Count; i++) dollyPts[i] += delta;

            Debug.Log($"[CinematicDirector] '{shot.Name}' anchored to '{HierarchyPath(target)}' at " +
                      $"{target.position} — path rotated {rotatedDeg:F0}° about its end and moved " +
                      $"{delta.magnitude:F1} m; it now lands at {dollyPts[dollyPts.Count - 1]}. " +
                      "The fly-in therefore ends on the vehicle wherever the mission starts.");
        }

        void EvaluateShot(CameraShot shot, float t, out Vector3 pos, out Quaternion rot)
        {
            pos = CinematicCamera.transform.position;
            rot = CinematicCamera.transform.rotation;

            switch (shot.Kind)
            {
                case CameraShot.Mode.DollySpline: EvalDolly(shot, t, ref pos, ref rot); break;
                case CameraShot.Mode.FollowThirdPerson: EvalFollow(shot, t, ref pos, ref rot); break;
                case CameraShot.Mode.StaticLookAt: EvalStatic(shot, ref pos, ref rot); break;
                case CameraShot.Mode.WaypointCams: EvalWaypointCams(shot, ref pos, ref rot); break;
                case CameraShot.Mode.OrbitZoomOut: EvalOrbit(shot, t, ref pos, ref rot); break;
                case CameraShot.Mode.DrainDock: EvalDrain(shot, t, ref pos, ref rot); break;
            }
        }

        /// <summary>
        /// The drain reveal. The CAMERA here is nearly static — a slow rise on a wide of the dock —
        /// because the movement in the shot is the water going down, not the lens going anywhere.
        ///
        /// The camera pose is the easy half. The important half is that this shot does NOT itself
        /// move the water: it asks `DockDrainDirector.Begin()`, which refuses unless the vehicle is
        /// surfaced and idle and which freezes every ForcePoint in the scene before anything moves
        /// (SETTLED §3s, and the sanctioned exception to it — read that class comment). A refusal
        /// is not a failure of this shot: the shot still plays, the dock stays full, and the
        /// overlay says which fact was missing.
        /// </summary>
        void EvalDrain(CameraShot shot, float t, ref Vector3 pos, ref Quaternion rot)
        {
            // The REQUEST is made in BeginShot and polled in TickDrainRequest — for this Mode and
            // for any shot with StartDrainAtShotStart, through exactly one code path. Round 3 kept
            // this Mode for manual use; the seeded storyboard no longer contains one, because the
            // drain now runs under the zoom-out.
            Vector3 basePos = shot.DrainAnchor != null ? shot.DrainAnchor.position : shot.DrainPositionWS;
            float u = shot.Duration > 1e-3f ? Mathf.Clamp01(t / shot.Duration) : 0f;
            pos = basePos + Vector3.up * (shot.DrainCameraRiseM * Mathf.SmoothStep(0f, 1f, u));

            Vector3 aim = shot.DrainLookAt != null ? shot.DrainLookAt.position : shot.DrainLookAtWS;
            rot = AimAt(pos, aim + shot.LookAtOffset);
        }

        // ---------------------------------------------------------------- shot maths

        /// <summary>
        /// Resolve a by-name target UNDER THE VEHICLE FIRST. `GameObject.Find` is scene-wide and
        /// `base_link` is not unique at Beckholmen — the station prefab has one too, on the dock
        /// bridge — so a scene-wide search can silently aim the camera at the wrong object at the
        /// wrong end of the dock. The scene-wide search is still the last resort, and it says out
        /// loud what it found and where it is.
        /// </summary>
        Transform ResolveByName(string name, string what)
        {
            if (string.IsNullOrEmpty(name)) return null;
            // Retried, not re-run every frame: this is on the per-frame path and ResolveSceneHooks
            // does several FindObjectsByType sweeps and logs when it comes up empty.
            if (Vehicle == null && Time.time >= nextHookRetry)
            {
                nextHookRetry = Time.time + 2f;
                ResolveSceneHooks();
            }

            if (Vehicle != null)
            {
                var under = FindDeepChild(Vehicle, name);
                if (under != null) return under;
            }
            var go = GameObject.Find(name);
            if (go == null) return null;

            Debug.LogWarning($"[CinematicDirector] {what} '{name}' is not under the vehicle " +
                             $"'{VehicleName}', so a scene-wide search was used and it returned " +
                             $"'{HierarchyPath(go.transform)}' at {go.transform.position}. " +
                             "If a shot aims at the wrong thing, this line is why: more than one " +
                             "object in this scene answers to that name.");
            return go.transform;
        }

        Transform ResolveLookAt(CameraShot shot)
        {
            if (shot.LookAt != null) return shot.LookAt;
            var t = ResolveByName(shot.LookAtName, "look-at target");
            if (t != null) { shot.LookAt = t; return t; }
            return vehicleAim;
        }

        Transform ResolveFollow(CameraShot shot)
        {
            if (shot.FollowTarget != null) return shot.FollowTarget;
            var t = ResolveByName(shot.FollowTargetName, "follow target");
            if (t != null) { shot.FollowTarget = t; return t; }
            return vehicleAim != null ? vehicleAim : Vehicle;
        }

        void EvalDolly(CameraShot shot, float t, ref Vector3 pos, ref Quaternion rot)
        {
            if (dollyPts.Count < 2) return;

            float u = shot.Duration > 0f ? Mathf.Clamp01(t / shot.Duration) : 1f;
            if (shot.Ease != null && shot.Ease.length > 0) u = shot.Ease.Evaluate(u);

            pos = CatmullRom(dollyPts, u);

            if (shot.LookAlongPath)
            {
                var ahead = CatmullRom(dollyPts, Mathf.Clamp01(u + 0.01f));
                var dir = ahead - pos;
                if (dir.sqrMagnitude > 1e-6f) rot = Quaternion.LookRotation(dir.normalized, Vector3.up);
            }
            else
            {
                var target = ResolveLookAt(shot);
                if (target != null) rot = AimAt(pos, target.position + shot.LookAtOffset);
            }
        }

        void EvalFollow(CameraShot shot, float t, ref Vector3 pos, ref Quaternion rot)
        {
            var target = ResolveFollow(shot);
            if (target == null) return;

            Quaternion frame = shot.YawOnly
                ? Quaternion.Euler(0f, target.eulerAngles.y, 0f)
                : target.rotation;

            // FollowOffsetAt(t): eased zoom-in from ZoomStartMultiplier x the offset. The camera
            // arrives from a distance, settles close, and — because the close offset's Y sits
            // near the hull — follows the vehicle THROUGH the surface when it dives.
            Vector3 wanted = target.position + frame * shot.FollowOffsetAt(t);
            Vector3 aimPoint = target.position + shot.LookAtOffset;

            // Exponential smoothing with a time constant, so the feel does not change with frame
            // rate — the follow has to look identical in a 4K recording and in the editor.
            float kp = shot.PositionDamping > 1e-3f ? 1f - Mathf.Exp(-Time.deltaTime / shot.PositionDamping) : 1f;
            float kr = shot.RotationDamping > 1e-3f ? 1f - Mathf.Exp(-Time.deltaTime / shot.RotationDamping) : 1f;

            pos = Vector3.Lerp(pos, wanted, kp);
            rot = Quaternion.Slerp(rot, AimAt(pos, aimPoint), kr);
        }

        void EvalStatic(CameraShot shot, ref Vector3 pos, ref Quaternion rot)
        {
            pos = shot.Anchor != null ? shot.Anchor.position : shot.PositionWS;
            var target = shot.LookAt != null || !string.IsNullOrEmpty(shot.LookAtName) ? ResolveLookAt(shot) : null;
            if (target != null) rot = AimAt(pos, target.position + shot.LookAtOffset);
            else if (shot.Anchor != null) rot = shot.Anchor.rotation;
        }

        /// <summary>
        /// The waypoint camera. Rather than pre-placing a camera per waypoint by hand — which
        /// would have to be redone every time the plan changes — the pose is derived from the hoop
        /// the vehicle is currently approaching: the lens sits just past the hoop, off to one side
        /// and low, so the vehicle comes toward it and passes beside it THROUGH the hoop.
        ///
        /// IT LATCHES ONTO ONE HOOP AND PANS TO FOLLOW (Ivan, 2026-08-21: "perhaps turning to
        /// follow while passing"). The old version re-cut to whichever hoop was nearest, so the
        /// approach was always interrupted before the pass. Now: latch, hold while it comes on,
        /// pan with it as it goes by (WPPanDamping is the swing of the head), and hand over on
        /// AdvanceWhen.HoopPassed. `LastFilmedHoopIndex` is raised on the latch, so the second
        /// waypoint-pass shot in the storyboard picks a LATER hoop and is a different shot.
        ///
        /// It can only use hoops that exist, and the hoop display only ever holds "the plan so
        /// far" (SETTLED §3o: the topic carries the current waypoint only). That is fine for
        /// filming and it is why the shot latches as the mission progresses rather than being cued
        /// in advance.
        /// </summary>
        void EvalWaypointCams(CameraShot shot, ref Vector3 pos, ref Quaternion rot)
        {
            var target = ResolveFollow(shot);
            if (target == null) target = vehicleAim;
            if (target == null || Hoops == null) return;

            var hoops = Hoops.Hoops;
            if (hoops == null || hoops.Count == 0) return;

            bool mayCut = wpCurrentIndex < 0
                          || (!shot.WPLatchToOneHoop && Time.time - wpLastCutTime >= shot.WPMinCutInterval);

            if (mayCut)
            {
                int best = ApproachHoop(shot.WPMaxRange, shot.WPLatchToOneHoop, out _);
                if (best >= 0 && best != wpCurrentIndex)
                {
                    wpCurrentIndex = best;
                    wpLastCutTime = Time.time;
                    wpPassed = false;
                    if (shot.WPLatchToOneHoop && best > LastFilmedHoopIndex) LastFilmedHoopIndex = best;

                    var h = hoops[best];
                    Vector3 fwd = h.transform.forward;              // direction of travel
                    Vector3 right = Vector3.Cross(Vector3.up, fwd).normalized;
                    if (right.sqrMagnitude < 1e-4f) right = h.transform.right;
                    // Alternate sides so consecutive passes are not the same shot twice. With the
                    // round-3 default of ZERO lateral offset this is a no-op and the lens sits ON the
                    // hoop centreline — Ivan: "you can even place them in the middle of the wp". The
                    // clamp is what stops an authored offset from putting the lens through a dock
                    // wall, which films concrete and used to look like the shot simply not firing.
                    float lateral = shot.EffectiveWPLateralOffset();
                    if (!Mathf.Approximately(lateral, shot.WPLateralOffset))
                        Debug.LogWarning($"[CinematicDirector] '{shot.Name}' WPLateralOffset " +
                                         $"{shot.WPLateralOffset:F2} m CLAMPED to {lateral:F2} m " +
                                         $"(|max| {Mathf.Abs(shot.WPMaxLateralOffset):F2} m). The Beckholmen dock " +
                                         "channel is about 17.6 m wide; a lens further off the leg than this is " +
                                         "inside or beyond a wall.");
                    float side = (best % 2 == 0) ? 1f : -1f;
                    wpPos = h.transform.position
                          + fwd * shot.WPBeyondOffset
                          + right * (side * lateral)
                          + Vector3.up * shot.WPVerticalOffset;
                    Debug.Log($"[CinematicDirector] '{shot.Name}' latched onto hoop {best + 1}/{hoops.Count}; " +
                              $"lens parked at {wpPos} — {shot.WPBeyondOffset:F1} m beyond the hoop, " +
                              $"{lateral:F2} m to the side, {shot.WPVerticalOffset:F2} m below its centre. " +
                              "The vehicle comes straight at it and passes through.");
                }
            }

            if (wpCurrentIndex < 0) return;

            if (!wpPassed && HoopIsAstern(wpCurrentIndex))
            {
                wpPassed = true;
                wpPassedTime = Time.time;
            }

            pos = wpPos;
            // The pan: a damped swing of the head, not a rigid lock, so the pass reads as an
            // operator following it through rather than as a tracking gimbal.
            var aim = AimAt(pos, target.position + shot.LookAtOffset);
            float kr = shot.WPPanDamping > 1e-3f ? 1f - Mathf.Exp(-Time.deltaTime / shot.WPPanDamping) : 1f;
            rot = Quaternion.Slerp(rot, aim, kr);
        }

        void EvalOrbit(CameraShot shot, float t, ref Vector3 pos, ref Quaternion rot)
        {
            float u = shot.Duration > 0f ? Mathf.Clamp01(t / shot.Duration) : 0f;
            float e = shot.OrbitEase != null && shot.OrbitEase.length > 0 ? shot.OrbitEase.Evaluate(u) : u;

            // With an OrbitEndAnchor the END of the sweep is a pose, not three loose numbers: the
            // bearing, radius and height were derived from it in ResolveOrbitEnd, so the orbit lands
            // exactly there — elevated at the south end, looking north down the dock (round 3).
            float endRadius = orbitEndFromAnchor ? orbitEndRadius : shot.EndRadius;
            float endHeight = orbitEndFromAnchor ? orbitEndHeight : shot.EndHeight;
            float startBearingDeg = orbitEndFromAnchor
                ? orbitEndBearingDeg - shot.DegreesSwept
                : shot.StartBearingDeg;

            float radius = Mathf.Lerp(shot.StartRadius, endRadius, e);
            float height = Mathf.Lerp(shot.StartHeight, endHeight, e);
            float bearing = (startBearingDeg + shot.DegreesSwept * e) * Mathf.Deg2Rad;

            var centre = shot.OrbitCenter != null ? shot.OrbitCenter.position : orbitCenterWS;
            pos = centre + new Vector3(Mathf.Sin(bearing) * radius, height, Mathf.Cos(bearing) * radius);

            // The final shot is about the DOCK AND THE MAP, so it looks at the orbit centre — the
            // point the vehicle surfaced at — and not at the vehicle, which would keep the lens
            // pinned to a 1.5 m hull while 100 m of point cloud slid out of frame.
            Vector3 aim = shot.LookAt != null ? shot.LookAt.position + shot.LookAtOffset
                                              : centre + shot.LookAtOffset;
            // …and if an end look-at is authored, the aim EASES onto it over the sweep, so the shot
            // arrives looking straight down the dock centreline instead of snapping there.
            if (shot.OrbitEndLookAt != null)
                aim = Vector3.Lerp(aim, shot.OrbitEndLookAt.position + shot.LookAtOffset, e);
            rot = AimAt(pos, aim);
        }

        static Quaternion AimAt(Vector3 from, Vector3 to)
        {
            var d = to - from;
            if (d.sqrMagnitude < 1e-8f) return Quaternion.identity;
            return Quaternion.LookRotation(d.normalized, Vector3.up);
        }

        /// <summary>Catmull-Rom through the node list, with the ends clamped so the path starts and finishes exactly on the first and last node.</summary>
        public static Vector3 CatmullRom(Transform[] nodes, float u)
        {
            var pts = new List<Vector3>();
            if (nodes != null) foreach (var t in nodes) if (t != null) pts.Add(t.position);
            return CatmullRom(pts, u);
        }

        public static Vector3 CatmullRom(List<Vector3> pts, float u)
        {
            int n = pts != null ? pts.Count : 0;
            if (n == 0) return Vector3.zero;
            if (n == 1) return pts[0];
            if (n == 2) return Vector3.Lerp(pts[0], pts[1], u);

            float scaled = Mathf.Clamp01(u) * (n - 1);
            int i = Mathf.Min((int)scaled, n - 2);
            float f = scaled - i;

            Vector3 p0 = pts[Mathf.Max(i - 1, 0)];
            Vector3 p1 = pts[i];
            Vector3 p2 = pts[i + 1];
            Vector3 p3 = pts[Mathf.Min(i + 2, n - 1)];

            float f2 = f * f, f3 = f2 * f;
            return 0.5f * ((2f * p1) +
                           (-p0 + p2) * f +
                           (2f * p0 - 5f * p1 + 4f * p2 - p3) * f2 +
                           (-p0 + 3f * p1 - 3f * p2 + p3) * f3);
        }

        static string HierarchyPath(Transform t)
        {
            if (t == null) return "(none)";
            var s = t.name;
            while (t.parent != null) { t = t.parent; s = t.name + "/" + s; }
            return s;
        }

        // ---------------------------------------------------------------- overlay + gizmos

        GUIStyle overlayStyle, overlaySubStyle;

        void OnGUI()
        {
            if (!ShowOverlay || !CinematicMode || Shots.Count == 0) return;
            if (overlayStyle == null)
            {
                overlayStyle = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, richText = false };
                overlaySubStyle = new GUIStyle(GUI.skin.label) { richText = false };
            }
            int fs = Mathf.Max(11, Mathf.RoundToInt(Screen.height * 0.016f));
            overlayStyle.fontSize = fs;
            overlaySubStyle.fontSize = Mathf.Max(10, Mathf.RoundToInt(fs * 0.85f));
            overlayStyle.normal.textColor = new Color(1f, 0.85f, 0.35f, 0.95f);
            overlaySubStyle.normal.textColor = new Color(1f, 1f, 1f, 0.85f);

            var shot = Shots[Mathf.Clamp(CurrentShot, 0, Shots.Count - 1)];
            float t = Time.time - shotStartTime;
            float maxS = EffectiveMaxSeconds(shot);

            string head = $"[{CurrentShot + 1}/{Shots.Count}] {shot.Name}  ({shot.Kind})  {t:F1} s" +
                          (maxS > 0f ? $" / max {maxS:F0} s" : "") +
                          (AutoAdvanceEnabled ? "" : "   AUTO OFF");
            string wait = string.IsNullOrEmpty(waitText) ? "waiting: —" : waitText;
            string last = string.IsNullOrEmpty(lastCutReason) ? "" : $"last cut: {lastCutReason}";

            // A frame in which the water is not where SETTLED §3s says it should be always carries
            // the reason it is not — whichever shot asked for the drain, not only a DrainDock one.
            if (drainRequested && !string.IsNullOrEmpty(drainStatus))
                last = (last.Length > 0 ? last + "   ·   " : "") + "DRAIN: " + drainStatus;

            float w = Screen.width * 0.86f;
            float x = Screen.width * 0.07f;
            GUI.Label(new Rect(x, 10f, w, fs * 1.6f), head, overlayStyle);
            GUI.Label(new Rect(x, 10f + fs * 1.5f, w, fs * 1.6f), wait, overlaySubStyle);
            if (last.Length > 0)
                GUI.Label(new Rect(x, 10f + fs * 2.8f, w, fs * 1.6f), last, overlaySubStyle);
            GUI.Label(new Rect(x, 10f + fs * 4.1f, w, fs * 1.6f),
                      $"{ToggleOverlayKey} hides this  ·  {NextShotKey}/{PrevShotKey} cut  ·  " +
                      $"{ToggleAutoAdvanceKey} auto  ·  {ToggleCinematicKey} exit", overlaySubStyle);
        }

        void OnDrawGizmos()
        {
            if (Shots == null) return;

            // In Play, draw the path the dolly is ACTUALLY flying — after anchoring — so the
            // difference between the authored shape and the anchored one is visible rather than
            // inferred.
            if (Application.isPlaying && dollyPts.Count >= 2)
            {
                Gizmos.color = new Color(0.3f, 1f, 0.5f, 0.9f);
                Vector3 p = CatmullRom(dollyPts, 0f);
                for (int i = 1; i <= 64; i++)
                {
                    var c = CatmullRom(dollyPts, i / 64f);
                    Gizmos.DrawLine(p, c);
                    p = c;
                }
            }

            foreach (var shot in Shots)
            {
                if (shot == null || shot.Kind != CameraShot.Mode.DollySpline) continue;
                if (CountNodes(shot) < 2) continue;
                Gizmos.color = new Color(1f, 0.75f, 0.2f, 0.9f);
                Vector3 prev = CatmullRom(shot.Nodes, 0f);
                const int segs = 64;
                for (int i = 1; i <= segs; i++)
                {
                    var cur = CatmullRom(shot.Nodes, i / (float)segs);
                    Gizmos.DrawLine(prev, cur);
                    prev = cur;
                }
                foreach (var t in shot.Nodes)
                    if (t != null) Gizmos.DrawWireSphere(t.position, 0.6f);
            }
        }
    }
}
