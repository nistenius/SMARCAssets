using System.Collections.Generic;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

using SmarcGUI.Water;             // BalticWaterPreset
using Smarc.Cinematics;           // CinematicDirector, CameraShot
using Visualizers;                // SonarMapAccumulator, SSSWaterfallHUD, RayViewer
using ROS.Subscribers;            // MissionWPHoop_Sub
using VehicleComponents.Sensors;  // Sonar, DeepVisionSSS

/// <summary>
/// The Askö mini-cooper FLY-PAST video rig — `SMARC/Video/A1` and `A5`.
///
/// WHY THIS IS A SEPARATE FILE AND NOT MORE OF `VideoRigSetup`. `SMARC/Video/2`, `2b`, `2c` and
/// `2d` are Beckholmen: they measure a dock floor by raycast, lay a fly-in down a dock centreline,
/// resize an underwater `volumeBounds` box, and drive a dock drain. None of those exist here, and
/// three of the four would do the wrong thing in this scene. What IS shared — the storyboard
/// comparator, the marker helper, the move-if-needed writer — is CALLED from `VideoRigSetup`
/// rather than copied (SETTLED §3s8: two copies of "correct" is how a builder and its own checker
/// start disagreeing).
///
/// WHAT THE SCENE ALREADY HAS, and what this therefore does NOT create (measured 2026-09-01 out
/// of `Assets/Scenes/AskoCurated.unity`):
///   * ONE active `sam_auv_v1` (the sam2.2 prefab) at (-151.14, -0.10, -210.67), yaw 262°;
///   * `MMTMiniCooper` at (-171.83, -8.22, -155.39), on a seabed the 0.125 m DV patch puts at
///     -8.05 m — the car has settled 0.17 m into the grid;
///   * an `SSSWaterfallHUD` ON THE VEHICLE ROOT (F6), which this rig DRIVES rather than replaces;
///   * a standalone `SonarMapAccumulator` (FLS on, SSS off, blue ramp) — left exactly as it is,
///     because the accumulator is not what this video is about (Ivan, 2026-09-01), and the shots
///     simply keep it out of frame;
///   * `AskoCuratedWorld ▸ Ocean` at (0,0,0), and it is where the BalticWaterPreset goes.
///
/// THE OCEAN IS AN INFINITE SURFACE, AND THAT IS WHY THE BECKHOLMEN WATER TRAP DOES NOT BITE.
/// `surfaceType: 0` (OceanSeaLake) + `geometryType: 3` (Infinite) ⇒ HDRP's `IsInfinite()` is TRUE,
/// so the underwater view is bounded by `volumeDepth` (60 m down) and `volumeHeight` (4 m up) and
/// `volumeBounds` is never consulted (HDRP 17.3,
/// `HDRenderPipeline.WaterSystem.Underwater.cs` ~line 60). Beckholmen's Water is a Pool and takes
/// the other branch, which is the whole of SETTLED §3s9. Nothing here resizes a collider.
///
/// AND `scriptInteractions` IS ON HERE, ON PURPOSE. On a displaced ocean the water level under a
/// ForcePoint is not the transform's Y, and the CPU simulation is what `HDRPWaterQueryModel`
/// searches. Turning it off to match Beckholmen does not make the vehicle safer; it makes buoyancy
/// wrong. `A5` says so rather than flagging it.
///
/// WHAT IT WILL NOT DO, by construction — the same list `VideoRigSetup` carries:
///   * move, disable or re-parent the Ocean (SETTLED §3s, §3o);
///   * touch `scriptInteractions`;
///   * add a collider to anything (a visualisation must never become a sonar target, §3o/§3g);
///   * run any prefab builder (SETTLED §5).
///
/// IT ALSO OWNS TWO FIELDS THE OPERATOR EDITS, AND IT REPRODUCES HIS ANSWERS RATHER THAN FIGHTING
/// HIM. `HudVisible` is seeded TRUE on every shot and `WaterPresetName` EMPTY on every shot, because
/// that is what Ivan set by hand on 2026-09-01 (dashboard in frame; the water look chosen in the
/// Editor, so a shot must not override it at a cut). A seeder that writes a field the operator also
/// writes has exactly two honest options — leave it alone, or agree with him — and this one agrees.
/// `ShowOverlay` is the third: it is set OFF once, when the director is CREATED, and never written
/// again, so a rebuild cannot switch off the overlay a rehearsal is being directed by.
///
/// AND IT IS IDEMPOTENT. A second press must print exactly
/// `NOTHING TO CHANGE, and nothing was written` and leave the scene clean. Every position it
/// writes is a pure function of two scene objects (the vehicle and the car), so pressing it again
/// recomputes the same numbers; every assignment is compared before it is made, and the scene is
/// marked dirty only if something actually changed. Press it twice and `shasum` the scene file —
/// that is the standing builder test, and this project has already shipped one UNTESTED "a second
/// press does nothing" that was false six presses running (SETTLED §3s8).
/// </summary>
public static class AskoVideoRigSetup
{
    // ================================================================ the geometry, ONE COPY
    //
    // Every number the storyboard and the markers are built from lives here, and the readiness
    // report reads the same block. Derived in `docs/2026-09-01-asko-video-runbook.md` §2 BEFORE
    // any of it was written (the 2026-08-29 model-before-you-implement rule); the derivation is
    // there, the consequences are here, and neither is a re-typed copy of the other.
    internal static class PassSpec
    {
        /// <summary>Metres the TRACK passes to PORT of the car — i.e. the car is this far to
        /// STARBOARD of the vehicle at closest approach. THE number the whole shot depends on.
        ///
        /// The side scan shipped on sam2.2 is tilt 0° / breadth 60°, which is off-nadir
        /// **30°..90°** — a 60° NADIR GAP, not the 0..45° fan an older note claims (that is the
        /// REGRESSED_BEAM_2026_08_16 signature, `sss_geometry.py`). So the constraint runs the
        /// OTHER WAY: lateral offset must be at LEAST Δdepth·tan30° = 0.577·Δdepth, or the car
        /// falls in the hole under the vehicle and is never ensonified at all.
        ///
        /// At the pass Δdepth is 3.72 m (car base -8.22, sonar -4.50), so the gap edge is at
        /// 2.15 m and 4.50 m puts the car 50.4° off nadir — 20° clear of the gap, 10° off the
        /// beam axis, at slant 5.84 m = range bin 146 of 1000. Wider is better for the sonar and
        /// worse for the lens; 4.50 m is where those two stop fighting. Runbook §2 has the table.
        /// </summary>
        public const float TrackLateralM = 4.50f;

        /// <summary>Vehicle depth on the pass, metres. Minimum altitude over the measured seabed
        /// is 3.42 m at wp2 — clear of STOP_M (2.5 m) and of the 1.0 m default min_altitude.</summary>
        public const float PassDepthM = 4.50f;

        /// <summary>Where `VideoTarget_Mini` sits above the car's own origin. The car's transform
        /// origin is at its base; 0.70 m puts the marker in the body, which is what a lens should
        /// be aimed at and what "astern" should be measured from.</summary>
        public const float MiniMarkerRiseM = 0.70f;

        // ---- the chase pose. Ivan, verbatim: 2 m behind, 0.5 m above the hull.
        public static readonly Vector3 ChaseOffset = new Vector3(0f, 0.5f, -2.0f);

        /// <summary>Vertical FOV on the chase, degrees. 75 and not 55: at 16:9 that is a 53.7°
        /// horizontal half-frame, and the car passes 66° off the vehicle's forward. A 55° lens
        /// loses it 3 m before the hull is abeam.</summary>
        public const float ChaseFov = 75f;

        /// <summary>Metres the chase lens AIMS to starboard of `base_link`, so the hull sits left
        /// of centre and the car's side of the world gets the frame. With the lens 2 m astern this
        /// swings the axis atan(0.8/2.0) = 21.8°, which is exactly what carries the car through
        /// closest approach instead of out of the right-hand edge.</summary>
        public const float ChaseAimStarboardM = 0.8f;
        public const float ChaseAimDownM = 0.5f;

        // ---- A2b, THE MINI-CAM (Ivan, 2026-09-01: "a bottom-up looking camera at the front of the
        //      mini looking towards the oncoming AUV when it passes the mini. Just a few seconds
        //      shot during the fly by"). Runbook §1.7 is the derivation; these are its outputs.
        //
        //      WHICH END OF THE CAR THE LENS STANDS AT WAS DECIDED BY ARITHMETIC, NOT BY TASTE, and
        //      it came out the opposite way to the first sketch. A lens parked UP-track of the car —
        //      on the side the vehicle comes from — has the vehicle almost abeam of it by the time a
        //      "few seconds" shot can start (at 0.34 m/s, six to ten seconds is two to three metres
        //      of travel), so there is no approach left to film. Parked DOWN-track it looks back over
        //      the car at a vehicle that is still 6.6 m up-track, and the range closes 7.9 -> 5.3 m
        //      across the shot: the AUV grows by half and comes at the lens nearly head-on (17° off
        //      its own axis at the cut). Down-track is also the end the car's nose points at — the
        //      prefab is "+Z = nose" (mmt_mini.json) and its yaw is -1.3° against a track heading of
        //      339.5°, so the nose leads by 19° — which is why this reads as "the front of the mini".
        /// <summary>Metres DOWN-track of the car (i.e. past it, on the side its nose points) that the
        /// mini-cam stands. 3.8 m leaves 2.26 m of clear water between the lens and the car's nose.</summary>
        public const float MiniCamAlongM = 3.8f;

        /// <summary>Metres the mini-cam is set over from the car TOWARD THE TRACK. The car is 4.50 m
        /// off the track, so 2.50 m puts the lens 2.00 m from the track — the whole reason being
        /// light: the closest lens-to-AUV range falls from 5.7 m to 5.1 m and the transmittance at
        /// the pass rises from 25 % to 29 % (§1.5's extinction line). Any further over and the car
        /// swings out of frame at the cut as well as at the pass.</summary>
        public const float MiniCamTowardTrackM = 2.50f;

        /// <summary>Metres above the CAR'S OWN ORIGIN. The origin is the car's base and it has settled
        /// 0.17 m into the grid, so 0.95 m here is 0.78 m over the measured seabed under the car
        /// (-8.05) and about 0.85 m over the seabed where the lens actually stands. It is also 0.32 m
        /// BELOW the car's roof (base -8.22 + 1.278 m of car = -6.94), which is what makes the shot
        /// read as bottom-up rather than as a hovering wide.</summary>
        public const float MiniCamRiseM = 0.95f;

        /// <summary>Vertical FOV on the mini-cam. 60 and not the chase's 75: at 16:9 that is a 45.7°
        /// horizontal half-frame, and at the cut the car's body centre is 44° off the aim — inside
        /// the frame by less than two degrees. 75 would hold the car longer and shrink the AUV to a
        /// tenth of the frame width; 50 would lose the car at the very first frame.</summary>
        public const float MiniCamFov = 60f;

        /// <summary>Metres the mini-cam AIMS to starboard of the vehicle — toward the car. Same trick
        /// as the chase's 0.8 m: it swings the axis about 12°, which is what carries the car into the
        /// right-hand edge of the frame while the AUV sits comfortably left of centre.</summary>
        public const float MiniCamAimStarboardM = 1.5f;
        /// <summary>…and a little below the hull, so the AUV rides above the centre line and the
        /// seabed and the car's sill get the bottom of the frame. This is the bottom-up read.</summary>
        public const float MiniCamAimDownM = 0.3f;

        /// <summary>Metres BEFORE the closest-approach point that `VideoTrigger_PreMini` sits, on the
        /// track. THIS IS THE FIELD THAT SETS THE SHOT'S LENGTH, and it was chosen from the clock and
        /// not from the frame: the shot runs from (this distance, less A2's steady hold) to the pass,
        /// plus MiniCamRunOnS. At 0.34 m/s that is 10.3 s and at 0.50 m/s it is 7.5 s — the "few
        /// seconds" Ivan asked for. The first sketch said 10–14 m; at 0.34 m/s that is a 31–43 s
        /// shot, which is a different film. Move this marker, not the anchor, to change the length.</summary>
        public const float PreMiniTriggerM = 3.0f;

        /// <summary>Seconds A2 holds after the trigger goes astern. A debounce and nothing more — the
        /// run-on past the CAR now belongs to A2b, so this is small on purpose. It comes straight off
        /// the front of the mini-cam shot, which is why it is in the arithmetic above.</summary>
        public const float PreMiniSteadyS = 0.5f;
        /// <summary>Seconds the mini-cam keeps rolling after the car is astern of the vehicle. Two
        /// seconds is 0.7 m of travel at the cruise: the AUV is still closing on the lens when we
        /// cut, which is the right frame to leave on.</summary>
        public const float MiniCamRunOnS = 2.0f;
        public const float MiniCamMinS = 4f;
        /// <summary>Ceiling. About 4x the predicted 10 s: enough that a slow run still gets its pass,
        /// short enough that a mission that stops dead does not leave the take on a parked lens.</summary>
        public const float MiniCamMaxS = 45f;

        /// <summary>Cruise speed used ONLY to print predicted seconds. The mission is seeded without
        /// a speed override, so this is `tst.py`'s default and the same number the runbook's 272 s
        /// comes from. It is a prediction, never a control input.</summary>
        public const float CruiseSpeedMS = 0.34f;
        /// <summary>The other end of the plausible speed range, for the same prediction.</summary>
        public const float CruiseSpeedFastMS = 0.50f;

        /// <summary>Seabed under the car, metres, sampled off the 0.125 m DV patch the scene renders.
        /// REPORT ONLY — nothing is placed with it. Every marker is placed off the car's own
        /// transform, so this constant cannot silently move a camera; it only lets `A5` say how far
        /// over the bottom the lens ended up.</summary>
        public const float MeasuredSeabedUnderCarY = -8.05f;
        /// <summary>Car height, metres, from `mmt_mini.json` (the generator's own dimensions block:
        /// 3.078 long x 1.416 wide x 1.278 high, "+Z = nose, origin centred and resting on y = 0").
        /// REPORT ONLY, and it is why `A5` can say where the roofline is relative to the lens.</summary>
        public const float CarHeightM = 1.278f;
        public const float CarLengthM = 3.078f;
        public const float CarWidthM = 1.416f;

        // ---- shot 1, the surface trail. The lens stays over the waterline: at the 0.6 m transit
        //      depth it is at +1.4 m, and it is still at +0.5 m when the shot cuts at 1.5 m under.
        public static readonly Vector3 SurfaceTrailOffset = new Vector3(0.4f, 2.0f, -7.0f);
        public const float SurfaceTrailFov = 55f;
        public const float CutToPlungeDepthM = 1.5f;

        // ---- the optional parting look (runbook §5.4). Placed whether or not shot 3 uses it, so
        //      switching to it during a rehearsal is two Inspector fields and no measuring.
        public const float PartingCamBackM = 3.0f;      // down-track from the car
        public const float PartingCamTowardTrackM = 2.0f;
        public const float PartingCamRiseM = 1.8f;      // above the car's base

        // ---- water
        public const string PresetMeasured = "asko_measured";
        public const string PresetClear = "asko_clear";
        public const string PresetClear6 = "asko_clear_6";
        public const float ClearAbsorptionM = 12.0f;
        public const float Clear6AbsorptionM = 6.0f;

        // ---- ceilings. Modelled from the seeded mission at the 0.34 m/s default: the plunge cut
        //      lands ~58 s after the mission starts and the car goes astern ~174 s after it, so
        //      these are roughly 3x and 2x the predicted time. A ceiling exists so no shot can
        //      hang, not so the shot list can be run off a stopwatch.
        public const float Shot1MaxS = 180f;
        public const float Shot2MaxS = 240f;
    }

    const string RigName = "AskoVideoRig";
    const string MiniName = "MMTMiniCooper";
    const string VehicleName = "sam_auv_v1";
    const string MiniMarker = "VideoTarget_Mini";
    const string PartingMarker = "VideoParting_Cam";
    const string MiniCamMarker = "VideoCam_MiniFront";
    const string PreMiniTrigger = "VideoTrigger_PreMini";

    // ================================================================ A1

    [MenuItem("SMARC/Video/A1 - Add Asko video rig (mini-cooper fly-past)", false, 180)]
    public static void AddAskoRig()
    {
        var changes = new List<string>();

        // ---------------------------------------------------------------- the scene's own facts
        var surface = Object.FindFirstObjectByType<WaterSurface>();
        if (surface == null)
        {
            Debug.LogError("[AskoVideoRig] no WaterSurface in the open scene. Is AskoCurated.unity the " +
                           "open scene? The Ocean comes in with the AskoCuratedWorld prefab. Nothing written.");
            return;
        }
        if (Mathf.Abs(surface.transform.position.y) > 1e-4f)
            Debug.LogError($"[AskoVideoRig] the Ocean transform is at Y = {surface.transform.position.y:F3}, " +
                           "not 0 (SETTLED §3s: a non-zero water plane hands ForcePoints divergent water " +
                           "levels and the vehicle leaves at 67 m/s). Move the TERRAIN, never the water. " +
                           "The rig is still added — nothing here writes that transform.");

        var vehicle = GameObject.Find(VehicleName);
        var mini = GameObject.Find(MiniName);
        if (vehicle == null || mini == null)
        {
            Debug.LogError($"[AskoVideoRig] need both '{VehicleName}' and '{MiniName}' as ACTIVE objects in " +
                           $"the open scene; found vehicle {(vehicle != null ? "yes" : "NO")}, car " +
                           $"{(mini != null ? "yes" : "NO")}. Every position this menu writes is derived from " +
                           "those two, so it refuses rather than seeding a rig at the world origin. " +
                           "Nothing written.");
            return;
        }

        if (!TryTrackAxis(vehicle.transform, mini.transform, out Vector3 fwd, out Vector3 right))
        {
            Debug.LogError("[AskoVideoRig] the vehicle and the car are at the same place in plan, so there is " +
                           "no track axis to derive. Nothing written.");
            return;
        }

        // ---------------------------------------------------------------- the water look
        var preset = surface.GetComponent<BalticWaterPreset>();
        if (preset == null)
        {
            preset = Undo.AddComponent<BalticWaterPreset>(surface.gameObject);
            preset.Surface = surface;
            // TODAY'S LOOK FIRST, BEFORE ANYTHING TOUCHES IT (Ivan, 2026-09-01). The scene ships at
            // absorption 2.1 m and that is a measured decision somebody made about this site; the
            // video is not allowed to be the reason it is lost.
            preset.CaptureIntoPreset(PassSpec.PresetMeasured);
            preset.ActivePreset = PassSpec.PresetMeasured;
            PrefabUtility.RecordPrefabInstancePropertyModifications(preset);
            changes.Add($"BalticWaterPreset added to '{surface.name}'; the scene's own look captured as " +
                        $"'{PassSpec.PresetMeasured}' (absorption {surface.absorptionDistance:F2} m) BEFORE " +
                        "anything else, and left as the active preset.");
        }
        else if (preset.Find(PassSpec.PresetMeasured) == null)
        {
            // The component exists but the capture never happened — capture now, from whatever the
            // scene currently says, and SAY that this may no longer be the original 2.1 m look.
            preset.CaptureIntoPreset(PassSpec.PresetMeasured);
            PrefabUtility.RecordPrefabInstancePropertyModifications(preset);
            changes.Add($"'{PassSpec.PresetMeasured}' was missing and has been captured from the surface AS IT " +
                        $"IS NOW (absorption {surface.absorptionDistance:F2} m). If a preset has been applied " +
                        "since the rig was first built, this is that look and not the shipped 2.1 m one — " +
                        "check it against git before trusting it as 'measured'.");
        }
        EnsureAskoPreset(preset, PassSpec.PresetClear, PassSpec.ClearAbsorptionM, changes);
        EnsureAskoPreset(preset, PassSpec.PresetClear6, PassSpec.Clear6AbsorptionM, changes);

        // ---------------------------------------------------------------- the rig
        var rigGO = GameObject.Find(RigName);
        if (rigGO == null)
        {
            rigGO = new GameObject(RigName);
            Undo.RegisterCreatedObjectUndo(rigGO, "Create AskoVideoRig");
            changes.Add($"created the '{RigName}' root.");
        }

        var director = rigGO.GetComponentInChildren<CinematicDirector>();
        if (director == null)
        {
            var go = new GameObject("CinematicDirector");
            Undo.RegisterCreatedObjectUndo(go, "Create CinematicDirector");
            go.transform.SetParent(rigGO.transform, false);
            director = go.AddComponent<CinematicDirector>();
            // TAKE-READY THE MOMENT IT EXISTS. `ShowOverlay` defaults to ON in the component, which
            // is right for a rehearsal and wrong for a take — Recorder captures the Game view and
            // burns the overlay in. It is set here, in the CREATION branch only, and never written
            // again: a rebuild must not reach in and switch off the overlay Ivan is directing by
            // (F3 toggles it live, and `A5` warns when it is on). Same reasoning as the shot
            // defaults below, which do reproduce his settings because the seeder owns those fields.
            director.ShowOverlay = false;
            changes.Add("created the CinematicDirector (Show Overlay OFF — take-ready; F3 or the " +
                        "Inspector turns it on for a rehearsal and nothing here writes it again).");
        }

        var map = Object.FindFirstObjectByType<SonarMapAccumulator>();
        var waterfall = Object.FindFirstObjectByType<SSSWaterfallHUD>();
        var hoops = Object.FindFirstObjectByType<MissionWPHoop_Sub>(FindObjectsInactive.Include);

        // THE UNDO ENTRY IS TAKEN LAZILY, ON THE FIRST REAL CHANGE. `Undo.RecordObject` marks the
        // scene dirty by itself, so calling it up front would fail the press-twice test even when
        // every field below already holds the right value — a builder that reports "nothing to
        // change" while dirtying the scene has not been shown to be idempotent, it has been shown
        // to be quiet (SETTLED §3s8).
        bool recorded = false;
        System.Action record = () =>
        {
            if (recorded) return;
            recorded = true;
            Undo.RecordObject(director, "Wire the Asko video rig");
        };
        SetField(ref director.VehicleName, VehicleName, "director.VehicleName", changes, record);
        SetField(ref director.VehicleAimChildName, "base_link", "director.VehicleAimChildName", changes, record);
        SetObj(ref director.Vehicle, vehicle.transform, "director.Vehicle", changes, record);
        SetObj(ref director.Surface, surface, "director.Surface", changes, record);
        SetObj(ref director.WaterPreset, preset, "director.WaterPreset", changes, record);
        SetObj(ref director.SonarMap, map, "director.SonarMap", changes, record);
        SetObj(ref director.Waterfall, waterfall, "director.Waterfall", changes, record);
        SetObj(ref director.Hoops, hoops, "director.Hoops", changes, record);
        SetField(ref director.FallbackMaxSeconds, 180f, "director.FallbackMaxSeconds", changes, record);
        SetField(ref director.AutoAdvanceEnabled, true, "director.AutoAdvanceEnabled", changes, record);
        SetField(ref director.StartInCinematicMode, false, "director.StartInCinematicMode", changes, record);

        // ---------------------------------------------------------------- the markers
        // Derived positions, snapped EXACTLY: they are a pure function of the car's transform and
        // the track axis, so a second press recomputes the same numbers and writes nothing — and
        // if the car or the vehicle is ever moved, the next press MOVES them and says so. A marker
        // that silently keeps pointing at where the car used to be is worse than one that moves.
        float waterY = surface.transform.position.y;
        var miniMarker = VideoRigSetup.EnsureMarker(director, MiniMarker,
            () => mini.transform.position + Vector3.up * PassSpec.MiniMarkerRiseM, out bool mmNew);
        var partingCam = VideoRigSetup.EnsureMarker(director, PartingMarker,
            () => PartingCamPos(mini.transform, fwd, right), out bool pcNew);
        // A2b's two markers. The ANCHOR is where the lens stands, the TRIGGER is where the shot
        // starts — and the trigger is a point on the TRACK, not on the car, because what starts the
        // shot is the vehicle reaching a place, not the vehicle being near a thing.
        var miniCam = VideoRigSetup.EnsureMarker(director, MiniCamMarker,
            () => MiniCamPos(mini.transform, fwd, right), out bool mcNew);
        var preTrigger = VideoRigSetup.EnsureMarker(director, PreMiniTrigger,
            () => PreMiniTriggerPos(mini.transform, fwd, right, waterY), out bool ptNew);
        if (mmNew) changes.Add($"created '{MiniMarker}' at {miniMarker.position}.");
        if (pcNew) changes.Add($"created '{PartingMarker}' at {partingCam.position}.");
        if (mcNew) changes.Add($"created '{MiniCamMarker}' at {miniCam.position} — the mini-cam's lens, " +
                               $"{PassSpec.MiniCamAlongM:F1} m down-track of the car, " +
                               $"{PassSpec.MiniCamTowardTrackM:F1} m toward the track, " +
                               $"{PassSpec.MiniCamRiseM:F2} m above the car's base.");
        if (ptNew) changes.Add($"created '{PreMiniTrigger}' at {preTrigger.position} — on the track " +
                               $"{PassSpec.PreMiniTriggerM:F1} m before the closest-approach point. THIS " +
                               "MARKER SETS THE LENGTH of the mini-cam shot; move it, not the lens.");

        var moved = new StringBuilder();
        VideoRigSetup.MoveIfNeeded(miniMarker, mini.transform.position + Vector3.up * PassSpec.MiniMarkerRiseM,
                                   MiniMarker, moved);
        VideoRigSetup.MoveIfNeeded(partingCam, PartingCamPos(mini.transform, fwd, right), PartingMarker, moved);
        VideoRigSetup.MoveIfNeeded(miniCam, MiniCamPos(mini.transform, fwd, right), MiniCamMarker, moved);
        VideoRigSetup.MoveIfNeeded(preTrigger, PreMiniTriggerPos(mini.transform, fwd, right, waterY),
                                   PreMiniTrigger, moved);
        if (moved.Length > 0)
            changes.Add("markers re-snapped to the car and the track axis (the car or the vehicle has moved " +
                        $"since they were placed):\n{moved}");

        // ---------------------------------------------------------------- the storyboard
        var wanted = BuildFlyPastStoryboard(miniMarker, miniCam, preTrigger, right);
        if (!VideoRigSetup.StoryboardMatches(director.Shots, wanted))
        {
            if (director.Shots.Count > 0)
            {
                var names = new List<string>();
                for (int i = 0; i < director.Shots.Count; i++)
                {
                    var s = director.Shots[i];
                    names.Add($"  {i + 1}. {(s == null ? "(null)" : s.Name)}" +
                              (s == null ? "" : $"  [{s.Kind}, {s.AdvanceSummary()}]"));
                }
                bool go = EditorUtility.DisplayDialog(
                    "Replace the Askö storyboard?",
                    $"The CinematicDirector holds {director.Shots.Count} shot(s):\n\n" +
                    string.Join("\n", names) +
                    $"\n\nThese will be REPLACED by the seeded {wanted.Count}-shot fly-past.\n" +
                    "The markers are reused, not recreated, so any nudging you did to them survives.\n\n" +
                    "Nothing else in the scene is touched.",
                    "Replace them", "Cancel");
                if (!go)
                {
                    Debug.Log("[AskoVideoRig] storyboard re-seed REFUSED at the dialog. Everything else above " +
                              "was still applied — read the change list at the end of this run.");
                    wanted = null;
                }
            }
            if (wanted != null)
            {
                Undo.RecordObject(director, "Seed the Asko fly-past storyboard");
                director.Shots = wanted;
                director.CurrentShot = 0;
                changes.Add($"seeded the {wanted.Count}-shot fly-past storyboard.");
            }
        }

        // ---------------------------------------------------------------- report
        var sb = new StringBuilder();
        if (changes.Count == 0)
        {
            Debug.Log("[AskoVideoRig] NOTHING TO CHANGE, and nothing was written. " +
                      "(That is the pass condition for a second press: this exact line, and a scene that " +
                      "does not go dirty. `shasum Assets/Scenes/AskoCurated.unity` before and after.)");
            return;
        }

        EditorUtility.SetDirty(director);
        EditorSceneManager.MarkSceneDirty(rigGO.scene);
        Selection.activeGameObject = rigGO;

        sb.AppendLine("[AskoVideoRig] the Askö mini-cooper fly-past rig is in place. What changed:");
        foreach (var c in changes) sb.AppendLine("  * " + c);
        sb.AppendLine();
        sb.AppendLine(DescribeGeometry(vehicle.transform, mini.transform, surface, fwd, right));
        sb.AppendLine();
        sb.AppendLine("THE MINI-CAM (shot A2b) IS A SECOND CAMERA, NOT A SECOND MECHANISM. It is a " +
                      "StaticLookAt parked on 'VideoCam_MiniFront' and tracking base_link, cut in by the " +
                      "SAME AdvanceWhen.TargetAstern the chase already used — pointed at a marker on the " +
                      "track instead of at the car. No new C# was needed for it, and the field that sets " +
                      "its LENGTH is the position of 'VideoTrigger_PreMini', not a number in a shot.");
        sb.AppendLine();
        sb.AppendLine("F6 BELONGS TO THE DIRECTOR DURING A TAKE. The SSSWaterfallHUD's toggle key is F6 and " +
                      "so is CinematicDirector.ToggleHudKey; entering cinematic mode calls " +
                      "SSSWaterfallHUD.SetDirectorControl(true), which GATES the panel's own handler and " +
                      "hides its closed-state '▲ SSS waterfall' button so it cannot be recorded into the " +
                      "video. Leaving cinematic mode gives the key, the button and the panel's " +
                      "open/closed state back. The panel is NOT rebound: outside a take F6 still opens it.");
        sb.AppendLine();
        sb.AppendLine("SAVE THE SCENE (Cmd-S), then PRESS A1 AGAIN: it must print " +
                      "'NOTHING TO CHANGE, and nothing was written' and the scene must not go dirty. " +
                      "Then SMARC/Video/5 for the readiness lines.");
        Debug.Log(sb.ToString());
    }

    // ================================================================ helpers

    /// <summary>
    /// The track axis: FORWARD is from the vehicle's start toward the car, flattened. Stated as an
    /// assumption because it is one — the run is a straight line from where the vehicle sits to
    /// past the car, and if the vehicle is re-posed the whole geometry moves with it. `right` is
    /// the vehicle's starboard, and the car is deliberately on that side.
    /// </summary>
    static bool TryTrackAxis(Transform vehicle, Transform mini, out Vector3 fwd, out Vector3 right)
    {
        fwd = mini.position - vehicle.position;
        fwd.y = 0f;
        right = Vector3.right;
        if (fwd.sqrMagnitude < 1e-4f) return false;
        fwd.Normalize();
        right = Vector3.Cross(Vector3.up, fwd).normalized;
        return true;
    }

    static Vector3 PartingCamPos(Transform mini, Vector3 fwd, Vector3 right)
    {
        return mini.position
             - fwd * PassSpec.PartingCamBackM
             - right * PassSpec.PartingCamTowardTrackM
             + Vector3.up * PassSpec.PartingCamRiseM;
    }

    /// <summary>
    /// Where the A2b lens stands: DOWN-track of the car (`+fwd`, the end its nose points at), set
    /// over toward the track, and low. All three offsets are off the car's own transform, so the
    /// lens follows the car if the car is ever moved, and a second press writes nothing.
    /// </summary>
    static Vector3 MiniCamPos(Transform mini, Vector3 fwd, Vector3 right)
    {
        return mini.position
             + fwd * PassSpec.MiniCamAlongM
             - right * PassSpec.MiniCamTowardTrackM
             + Vector3.up * PassSpec.MiniCamRiseM;
    }

    /// <summary>
    /// Where A2 hands over to the mini-cam: a point ON THE TRACK, `PreMiniTriggerM` before the
    /// closest-approach point. Its height is the pass depth below the still-water plane — which
    /// `TargetAstern` does not read (that test is horizontal) but which puts the gizmo on the line
    /// the vehicle actually flies, so the Scene view shows the cue where the cue happens.
    /// </summary>
    static Vector3 PreMiniTriggerPos(Transform mini, Vector3 fwd, Vector3 right, float waterY)
    {
        Vector3 closest = mini.position - right * PassSpec.TrackLateralM;
        Vector3 p = closest - fwd * PassSpec.PreMiniTriggerM;
        p.y = waterY - PassSpec.PassDepthM;
        return p;
    }

    /// <summary>
    /// Add a shooting preset if it is missing, and NEVER overwrite one that is already there.
    ///
    /// The colours are the Beckholmen `baltic` family shifted a little greener, and the absorption
    /// is the number the runbook's §2 model produced — not a taste. HDRP's own extinction (measured
    /// out of `WaterSurface.cs:239`) is
    ///     extinction = (-ln 0.02 / absorptionDistance) * max(1 - refractionColor, 0.01)
    /// so the transmittance over a path R is 0.02^(R*(1-refractionColor)/absorptionDistance). The
    /// lens passes 5.31 m from the car at its closest; at 12 m that is 20-28 % of the light and the
    /// car READS, and at 6 m it is 4-8 % and the car is a silhouette. Both presets are seeded so
    /// the A/B is one Inspector click.
    ///
    /// ROUND 2, 2026-09-01: NO SHOT NAMES ANY OF THESE ANY MORE. Ivan set the look in the Editor —
    /// the Ocean is saved at absorption 30.2 m, which is also what `asko_measured` captured, because
    /// A1 captures what it FINDS and what it found had already been changed — and every shot's
    /// `WaterPresetName` is empty so nothing overrides that at a cut. These presets are the A/B and
    /// nothing else. `A5` prints the transmittance off the LIVE surface, which is the number that
    /// describes the take, and off `asko_clear`, which is the number that describes the option.
    /// </summary>
    static void EnsureAskoPreset(BalticWaterPreset preset, string name, float absorptionM, List<string> changes)
    {
        if (preset.Find(name) != null) return;
        Undo.RecordObject(preset, "Add Asko water preset");
        preset.Presets.Add(new BalticWaterPreset.WaterLook
        {
            Name = name,
            AbsorptionDistance = absorptionM,
            ScatteringColor = new Color(0.075f, 0.190f, 0.165f, 1f),
            RefractionColor = new Color(0.060f, 0.260f, 0.240f, 1f),
            MaxRefractionDistance = 0.9f,
            AmbientScattering = 0.42f,
            HeightScattering = 0.18f,
            DisplacementScattering = 0.12f,
            DirectLightTipScattering = 0.45f,
            DirectLightBodyScattering = 0.40f,
            Caustics = true,
            CausticsIntensity = 0.30f,
            UnderWater = true,
            AbsorptionDistanceMultiplier = 1f,
            UnderWaterAmbientProbeContribution = 0.95f,
            UnderWaterScatteringColor = new Color(0.060f, 0.170f, 0.150f, 1f),
            UnderWaterRefraction = false,
            // Surface fields taken from the Askö Ocean as shipped, so a preset change is a change
            // of LOOK UNDER THE WATER and not a different sea state on the surface shots.
            StartSmoothness = 0.95f,
            EndSmoothness = 0.85f,
            RipplesWindSpeed = 8f
        });
        PrefabUtility.RecordPrefabInstancePropertyModifications(preset);
        changes.Add($"added water preset '{name}' (absorption {absorptionM:F1} m).");
    }

    static void SetField<T>(ref T field, T value, string label, List<string> changes, System.Action record)
    {
        // EqualityComparer rather than `==`: a type parameter cannot use the `==` operator, and
        // `.Equals` would need a null guard for every reference type that comes through here.
        if (EqualityComparer<T>.Default.Equals(field, value)) return;
        record();
        changes.Add($"{label}: {field} -> {value}");
        field = value;
    }

    // Unity overloads `==` on Object so a DESTROYED object compares equal to null; `ReferenceEquals`
    // would bypass that overload and call a dead object "unchanged". Hence the explicit `==`.
    static void SetObj<T>(ref T field, T value, string label, List<string> changes, System.Action record)
        where T : Object
    {
        if (field == value) return;
        record();
        changes.Add($"{label}: {(field == null ? "(none)" : field.name)} -> {(value == null ? "(none)" : value.name)}");
        field = value;
    }

    // ================================================================ the four shots

    /// <summary>
    /// Ivan's brief, 2026-09-01, in his order: trail it along the surface toward the car; go in
    /// with it as it dives, on to a chase 2 m behind and 0.5 m above the hull; pass the car with
    /// the sonars on and the waterfall up — and (second round, same day) cut to a camera on the
    /// seabed at the front of the mini for a few seconds of the AUV coming straight at the lens.
    ///
    /// The cuts are read off facts, not off a stopwatch — the same rule as the Beckholmen list.
    /// A1 hands over when the vehicle has visibly gone under; A2 hands over when the vehicle
    /// reaches a marked point on the track a few metres short of the car; A2b hands over when the
    /// CAR is astern (`AdvanceWhen.TargetAstern`, new for this video, because `HoopPassed` can only
    /// latch onto a waypoint and the thing being passed here is a car on the seabed). Both of the
    /// last two use the SAME condition on DIFFERENT objects, which is the whole reason the mini-cam
    /// needed no new C#.
    ///
    /// TWO FIELDS ARE SET THE WAY IVAN LEFT THEM IN THE SCENE, and that is deliberate. `HudVisible`
    /// is TRUE on every shot (he wants the dashboard in frame) and `WaterPresetName` is EMPTY on
    /// every shot (he chose the look in the Editor, and a shot that names a preset would override
    /// it at the cut). The seeder owns those fields, so if it did not reproduce his choices a
    /// rebuild would quietly undo them — the §3s8 shape, in the one place where the operator and
    /// the builder both write. The presets are still seeded on the Ocean; they are one Inspector
    /// click, not a shot field.
    /// </summary>
    static List<CameraShot> BuildFlyPastStoryboard(Transform miniMarker, Transform miniCam,
                                                   Transform preTrigger, Vector3 right)
    {
        // The chase aims a little to STARBOARD of the hull, and this is the field that decides
        // whether the car is in frame at all. WORLD axes, because that is what LookAtOffset is —
        // legitimate here only because the run is a straight line at a fixed heading. It is derived
        // from the track's own `right` vector, so it is a bearing off the geometry and not a
        // constant somebody would have to re-derive after the vehicle moved.
        Vector3 chaseAim = right * PassSpec.ChaseAimStarboardM + Vector3.down * PassSpec.ChaseAimDownM;
        // …and the mini-cam aims toward the car for the same reason, off the same vector.
        Vector3 miniCamAim = right * PassSpec.MiniCamAimStarboardM + Vector3.down * PassSpec.MiniCamAimDownM;

        return new List<CameraShot>
        {
            // 1 — the surface trail. The lens is 2 m ABOVE the hull and the hull is 0.6 m under, so
            //     the lens is 1.4 m over the waterline and stays over it: at the 1.5 m cut depth it
            //     is still at +0.5 m. Ivan: "camera trails SAM above the surface".
            new CameraShot
            {
                Name = "A1 surface trail toward the mini",
                Kind = CameraShot.Mode.FollowThirdPerson,
                FieldOfView = PassSpec.SurfaceTrailFov, NearClip = 0.08f,
                Advance = CameraShot.AdvanceWhen.VehicleSubmerged,
                MinSeconds = 6f, MaxSeconds = PassSpec.Shot1MaxS,
                SteadySeconds = 0.6f, SubmergeDepth = PassSpec.CutToPlungeDepthM,
                FollowTargetName = "base_link",
                FollowOffset = PassSpec.SurfaceTrailOffset,
                PositionDamping = 0.55f, RotationDamping = 0.35f, YawOnly = true,
                LookAtOffset = new Vector3(0f, 0.1f, 0f),
                HudVisible = true, ParticlesEnabled = false,
                SonarMapVisible = false, SonarMapLabelVisible = false,
                SonarBeamsVisible = true, ShowSSSWaterfall = false,
                WaterPresetName = ""
            },

            // 2 — THE PLUNGE, THEN THE CHASE. The 1.2 s BLEND is the plunge: it eases the lens from
            //     shot 1's above-water pose to a pose 0.5 m over a hull that is already 1.5 m under,
            //     i.e. through the surface. Short on purpose — a slow crossing reads as a mistake, a
            //     quick one reads as a dive. This is the Beckholmen 2a→2b pattern with one shot
            //     fewer, because here the dive and the chase are one continuous move.
            //
            //     IT NOW ENDS SHORT OF THE CAR, on `VideoTrigger_PreMini` — a point on the track
            //     PreMiniTriggerM before closest approach — because the pass itself belongs to A2b.
            //     Same condition as before (`TargetAstern`), a different object: that is why the
            //     mini-cam needed no new C#. The steady hold is small (a debounce, not a run-on):
            //     every tenth of a second here comes off the front of the mini-cam shot.
            new CameraShot
            {
                Name = "A2 plunge in, chase down toward the mini",
                Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 1.2f,
                FieldOfView = PassSpec.ChaseFov, NearClip = 0.04f,
                Advance = CameraShot.AdvanceWhen.TargetAstern,
                MinSeconds = 10f, MaxSeconds = PassSpec.Shot2MaxS,
                SteadySeconds = PassSpec.PreMiniSteadyS,
                AsternTarget = preTrigger, AsternTargetName = PreMiniTrigger,
                FollowTargetName = "base_link",
                FollowOffset = PassSpec.ChaseOffset,
                PositionDamping = 0.35f, RotationDamping = 0.28f, YawOnly = true,
                LookAtOffset = chaseAim,
                HudVisible = true, ParticlesEnabled = false,
                SonarMapVisible = false, SonarMapLabelVisible = false,
                SonarBeamsVisible = true, ShowSSSWaterfall = true,
                WaterPresetName = ""
            },

            // 2b — THE MINI-CAM. Ivan, 2026-09-01: "a bottom-up looking camera at the front of the
            //      mini looking towards the oncoming AUV when it passes the mini. Just a few seconds
            //      shot during the fly by (similar to the Beckholmen shot)."
            //
            //      It is the Beckholmen waypoint-cam BEAT, not its mechanism. `WaypointCams` parks
            //      the lens on a hoop and pans as the vehicle comes down the barrel — but there is
            //      no hoop at the car, the hoop list only ever holds the plan so far (§3o), and the
            //      thing worth standing next to here is a car on the seabed. `StaticLookAt` with an
            //      anchor and a look-at gives the same beat from a place chosen by geometry: parked
            //      lens, vehicle comes on, lens swings with it, cut.
            //
            //      A HARD CUT IN (BlendSeconds 0). A blend would fly the lens from 2 m behind the
            //      hull down to the seabed in a second, which is a move nobody made; the cut is what
            //      makes it read as a second camera.
            //
            //      IT TRACKS THE VEHICLE, so the framing survives the vehicle not flying the modelled
            //      track: the aim is `base_link` (resolved UNDER the vehicle first, so it cannot
            //      latch onto some other base_link), biased MiniCamAimStarboardM toward the car and
            //      MiniCamAimDownM below the hull. What geometry buys is the ANGLE and the RANGE, not
            //      the centring. Runbook §1.7: the AUV enters frame 7.46 m out, 17° off head-on and
            //      22° above the lens's horizon, and closes to 5.11 m at 33° as it draws level with
            //      the car. On the look the scene is currently SAVED with (absorption 30.2 m) that is
            //      47 % of the green channel rising to 59 %; on `asko_clear` (12 m) it would be 17 %
            //      rising to 29 %. Either reads; the first reads easily. The car's body centre sits
            //      44° off the aim at the cut — inside the 45.7° horizontal half-frame of a 60° lens
            //      by a degree and a half — and swings out at 53° by the time the AUV is level. That
            //      is the intended composition and not an accident: the wreck is the foreground the
            //      shot opens on, the AUV is what it ends on.
            new CameraShot
            {
                Name = "A2b mini-cam — the AUV comes at you",
                Kind = CameraShot.Mode.StaticLookAt,
                BlendSeconds = 0f,
                FieldOfView = PassSpec.MiniCamFov, NearClip = 0.04f,
                Advance = CameraShot.AdvanceWhen.TargetAstern,
                MinSeconds = PassSpec.MiniCamMinS, MaxSeconds = PassSpec.MiniCamMaxS,
                SteadySeconds = PassSpec.MiniCamRunOnS,
                AsternTarget = miniMarker, AsternTargetName = MiniMarker,
                Anchor = miniCam,
                LookAtName = "base_link",
                LookAtOffset = miniCamAim,
                HudVisible = true, ParticlesEnabled = false,
                SonarMapVisible = false, SonarMapLabelVisible = false,
                // The beams stay on, and this is the one shot that sees them FROM OUTSIDE: the fan
                // sweeping the seabed past the lens is the sensor made visible, which is the whole
                // argument for having a camera down here at all.
                SonarBeamsVisible = true, ShowSSSWaterfall = true,
                WaterPresetName = ""
            },

            // 3 — the run-out. THE CHASE IS HELD, deliberately, and it is the last shot, so it holds
            //     until you stop recording.
            //
            //     The plan doc offered a parting look from a lens parked by the car instead. The
            //     geometry says pick one or the other and not both: the car and the receding vehicle
            //     are 4.5 m apart in plan, so a lens close enough for the car to READ (3-4 m, ~38 %
            //     of the light at absorption 12) sees them 55-80° apart and cannot hold both, while
            //     a lens far enough back to hold both (10 m) puts the car at 8 % and it goes dark.
            //     Keeping the chase avoids inventing a shot the water cannot carry — and the
            //     WATERFALL is where the car's return lives anyway, which is the point of the video.
            //     `VideoParting_Cam` is placed regardless; runbook §5.4 is the two-field switch.
            //
            //     THE BLEND IS 1.0 s AND NOT A CUT, and that is a change from the 3-shot list. It
            //     used to hand over chase-to-chase, where the blend was invisible because both poses
            //     were the same. It now hands over from a lens parked on the seabed, and `EvalFollow`
            //     damps out of WHEREVER THE CAMERA IS STANDING — so BlendSeconds 0 does not give a
            //     cut here, it gives an exponential lurch off the anchor. The choice is between an
            //     eased move and a lurch, and 1.0 s eases it: the camera lifts off the bottom and
            //     settles back behind the hull as one deliberate move.
            new CameraShot
            {
                Name = "A3 run-out, chase held",
                Kind = CameraShot.Mode.FollowThirdPerson,
                BlendSeconds = 1.0f,
                FieldOfView = PassSpec.ChaseFov, NearClip = 0.04f,
                Advance = CameraShot.AdvanceWhen.Manual,
                MinSeconds = 0f, MaxSeconds = 0f,
                FollowTargetName = "base_link",
                FollowOffset = PassSpec.ChaseOffset,
                PositionDamping = 0.35f, RotationDamping = 0.28f, YawOnly = true,
                LookAtOffset = chaseAim,
                HudVisible = true, ParticlesEnabled = false,
                SonarMapVisible = false, SonarMapLabelVisible = false,
                SonarBeamsVisible = true, ShowSSSWaterfall = true,
                WaterPresetName = ""
            }
        };
    }

    // ================================================================ A5 / the readiness lines

    [MenuItem("SMARC/Video/A5 - Report Asko scene readiness", false, 181)]
    public static void ReportAskoReadiness()
    {
        var sb = new StringBuilder();
        sb.AppendLine("[AskoVideoRig] readiness for the Askö mini-cooper fly-past:");
        if (!AppendReadiness(sb))
            sb.AppendLine("  FAIL  no AskoVideoRig in the open scene — run SMARC/Video/A1.");
        Debug.Log(sb.ToString());
    }

    /// <summary>Called from `SMARC/Video/5` so that one menu item still answers "is this scene
    /// ready". Appends nothing at all when this scene carries no Askö rig.</summary>
    public static void AppendReadinessIfPresent(StringBuilder sb)
    {
        if (GameObject.Find(RigName) == null) return;
        sb.AppendLine();
        sb.AppendLine("[AskoVideoRig] this scene also carries the Askö fly-past rig:");
        AppendReadiness(sb);
    }

    static bool AppendReadiness(StringBuilder sb)
    {
        var rigGO = GameObject.Find(RigName);
        if (rigGO == null) return false;
        var director = rigGO.GetComponentInChildren<CinematicDirector>();
        var surface = Object.FindFirstObjectByType<WaterSurface>();
        var vehicle = GameObject.Find(VehicleName);
        var mini = GameObject.Find(MiniName);

        // ---- the water --------------------------------------------------------------------
        if (surface == null) sb.AppendLine("  FAIL  no WaterSurface.");
        else
        {
            float y = surface.transform.position.y;
            sb.AppendLine(Mathf.Abs(y) < 1e-4f
                ? "  ok    Ocean transform Y = 0 (SETTLED §3s trap not armed)."
                : $"  FAIL  Ocean transform Y = {y:F3}. Move the TERRAIN, not the water.");

            bool infinite = BalticWaterPreset.IsInfiniteSurface(surface);
            sb.AppendLine(infinite
                ? $"  ok    the Ocean is an INFINITE surface, so the underwater view is bounded by " +
                  $"volumeDepth {surface.volumeDepth:F0} m below the plane and volumeHeight " +
                  $"{surface.volumeHeight:F0} m above it — `volumeBounds` is never consulted and the " +
                  "Beckholmen box-collider trap (SETTLED §3s9) does NOT apply here. Every camera in " +
                  "this storyboard sits between +2 m and -7.3 m (the mini-cam is the deep one)."
                : $"  WARN  surfaceType {surface.surfaceType} / geometryType {surface.geometryType} is NOT " +
                  "infinite, so HDRP tests the CAMERA against `volumeBounds` and renders no underwater " +
                  "view outside it. That is the Beckholmen defect; this scene was not supposed to have it.");

            sb.AppendLine(surface.scriptInteractions
                ? "  ok    CPU 'Script Interactions' ON — correct and REQUIRED on a displaced ocean " +
                  "(HDRPWaterQueryModel has nothing to search without it). Do NOT turn it off to match " +
                  "Beckholmen: that does not make the vehicle safer, it makes buoyancy wrong."
                : "  WARN  CPU 'Script Interactions' is OFF on an ocean surface. Buoyancy reads the CPU " +
                  "simulation; with displacement and no simulation the water level under a ForcePoint " +
                  "is not the transform Y.");

            var preset = surface.GetComponent<BalticWaterPreset>();
            if (preset == null) sb.AppendLine("  FAIL  no BalticWaterPreset on the Ocean — run SMARC/Video/A1.");
            else
            {
                var m = preset.Find(PassSpec.PresetMeasured);
                var c = preset.Find(PassSpec.PresetClear);
                sb.AppendLine(m != null
                    ? $"  ok    '{PassSpec.PresetMeasured}' captured, absorption {m.AbsorptionDistance:F2} m " +
                      "(the scene's own look, kept before anything changed it)."
                    : $"  FAIL  '{PassSpec.PresetMeasured}' is missing — today's look is not saved anywhere. " +
                      "Run SMARC/Video/A1 BEFORE applying any other preset.");
                if (c == null)
                    sb.AppendLine($"  WARN  '{PassSpec.PresetClear}' is missing — no shot names it any more, so " +
                                  "nothing breaks, but the one-click A/B against the shipped look is gone. " +
                                  "Run SMARC/Video/A1.");
                else
                {
                    // The model, evaluated against what the scene ACTUALLY holds rather than against
                    // the number the runbook was written with. Green channel: it is the one that
                    // survives furthest in this water and therefore the one that decides visibility.
                    float lens = Mathf.Sqrt(PassSpec.TrackLateralM * PassSpec.TrackLateralM + 2.82f * 2.82f);
                    float tG = Transmittance(lens, c.AbsorptionDistance, c.RefractionColor.g);
                    sb.AppendLine($"  info  '{PassSpec.PresetClear}' absorption {c.AbsorptionDistance:F1} m " +
                                  $"would put about {tG * 100f:F0} % of the green channel at the chase's " +
                                  $"closest lens-to-car range ({lens:F2} m) " +
                                  "(HDRP: 0.02^(R·(1-refractionColor)/absorption), WaterSurface.cs:239). " +
                                  "One Inspector click if the take needs it.");
                }
                // THE NUMBER THAT ACTUALLY DESCRIBES THE TAKE. Every shot now carries an EMPTY
                // WaterPresetName (Ivan chose the look in the Editor, 2026-09-01), so the water
                // during a take is whatever the surface holds right now — not `asko_clear`. This
                // reads the live surface, so it cannot quietly go on quoting a preset nobody applies.
                {
                    float rChase = Mathf.Sqrt(PassSpec.TrackLateralM * PassSpec.TrackLateralM + 2.82f * 2.82f);
                    float tNow = Transmittance(rChase, surface.absorptionDistance, surface.refractionColor.g);
                    sb.AppendLine($"  {(tNow >= 0.15f ? "ok  " : "WARN")}  THE TAKE RUNS ON THE SAVED LOOK " +
                                  $"(absorption {surface.absorptionDistance:F2} m, active preset " +
                                  $"'{preset.ActivePreset}'): every shot's Water Preset Name is empty on " +
                                  $"purpose, so nothing is applied at a cut. At the chase's {rChase:F2} m that " +
                                  $"is about {tNow * 100f:F0} % of the green channel" +
                                  (tNow >= 0.15f
                                      ? " — the car will read."
                                      : " — the car will be a silhouette. Apply a clearer preset on the Ocean " +
                                        "IN EDIT MODE and save, or accept it; do not re-add a preset name to a " +
                                        "shot, because that overrides whatever you chose here."));
                }
                sb.AppendLine($"  info  active preset '{preset.ActivePreset}'; known: " +
                              $"{string.Join(", ", preset.PresetNames())}.");
            }
        }

        // ---- the marker on the car ---------------------------------------------------------
        if (director == null) { sb.AppendLine("  FAIL  no CinematicDirector under AskoVideoRig."); return true; }
        var miniMarker = director.transform.Find(MiniMarker);
        if (mini == null) sb.AppendLine($"  FAIL  no '{MiniName}' in the scene.");
        else if (miniMarker == null) sb.AppendLine($"  FAIL  no '{MiniMarker}' marker — run SMARC/Video/A1.");
        else
        {
            float d = Vector3.Distance(miniMarker.position,
                                       mini.transform.position + Vector3.up * PassSpec.MiniMarkerRiseM);
            sb.AppendLine(d < 0.1f
                ? $"  ok    '{MiniMarker}' is on the car at {miniMarker.position} " +
                  $"({PassSpec.MiniMarkerRiseM:F2} m above its origin). This is what TargetAstern watches."
                : $"  FAIL  '{MiniMarker}' is {d:F2} m off the car — the fly-past cut would fire in the wrong " +
                  "place. Press SMARC/Video/A1: it re-snaps the marker and says that it did.");
        }

        // ---- the seeded track, so it can be compared against the mission by eye -------------
        if (vehicle != null && mini != null &&
            TryTrackAxis(vehicle.transform, mini.transform, out Vector3 fwd, out Vector3 right))
        {
            sb.AppendLine("  info  " + DescribeGeometry(vehicle.transform, mini.transform, surface, fwd, right)
                                        .Replace("\n", "\n        "));
            sb.AppendLine($"  info  seed_asko_minipass_mission.py was generated for vehicle " +
                          $"{Fmt(vehicle.transform.position)} and car {Fmt(mini.transform.position)}. " +
                          "Those two lines must match the SCENE POSITIONS printed in the script's header; " +
                          "if they do not, the plan and the camera are describing different runs.");

            // ---- A2b, the mini-cam ----------------------------------------------------------
            AppendMiniCamLines(sb, director, mini.transform, surface, fwd, right);
        }

        // ---- the side scan the whole geometry depends on ------------------------------------
        AppendSonarLines(sb, vehicle);

        // ---- the waterfall and the F-key -----------------------------------------------------
        if (director.Waterfall == null)
            sb.AppendLine("  FAIL  the director has no SSSWaterfallHUD — the panel is the second half of " +
                          "this video. Run SMARC/Video/A1.");
        else
        {
            var wf = director.Waterfall;
            sb.AppendLine($"  ok    waterfall panel wired ('{HierarchyPath(wf.transform)}'), " +
                          $"{wf.panelWidth}x{wf.panelHeight}, {wf.gain} gain, ±{wf.dynamicRangeDb / 2f:F0} dB, " +
                          $"brightness {wf.brightness:F2}.");
            bool collides = wf.inputToggleKey == director.ToggleHudKey;
            sb.AppendLine(!collides
                ? $"  ok    no F-key collision: the panel is on {wf.inputToggleKey} and the director's HUD is " +
                  $"on {director.ToggleHudKey}."
                : wf.suppressOwnKeyWhileDirected
                    ? $"  ok    F-key collision RESOLVED BY GATING: both are {wf.inputToggleKey}, and entering " +
                      "cinematic mode calls SetDirectorControl(true), which suppresses the panel's own " +
                      "handler and hides its closed-state button for the duration. Outside a take " +
                      $"{wf.inputToggleKey} still opens the panel, which is what it has always done."
                    : $"  FAIL  BOTH the panel and the director's HUD are on {wf.inputToggleKey}, and " +
                      "`suppressOwnKeyWhileDirected` is UNTICKED, so one press means two things during a " +
                      "take. Tick it, or move the panel to a key the director does not use (F2–F10).");
        }

        // ---- the shots -----------------------------------------------------------------------
        int wantWaterfall = 0, beams = 0;
        for (int i = 0; i < director.Shots.Count; i++)
        {
            var s = director.Shots[i];
            if (s == null) continue;
            if (s.ShowSSSWaterfall) wantWaterfall++;
            if (s.SonarBeamsVisible) beams++;
            if (s.Advance == CameraShot.AdvanceWhen.TargetAstern && s.AsternTarget == null &&
                string.IsNullOrEmpty(s.AsternTargetName))
                sb.AppendLine($"  FAIL  shot {i + 1} '{s.Name}' advances on TargetAstern but names nothing to " +
                              "pass — it can only fall through on its ceiling.");
        }
        sb.AppendLine($"  info  {director.Shots.Count} shot(s); waterfall up on {wantWaterfall}, " +
                      $"sonar beams on {beams}.");
        for (int i = 0; i < director.Shots.Count; i++)
        {
            var s = director.Shots[i];
            if (s != null)
                sb.AppendLine($"          {i + 1}. {s.Name}  [{s.Kind}]  {s.AdvanceSummary()}" +
                              (string.IsNullOrEmpty(s.WaterPresetName) ? "" : $"  water '{s.WaterPresetName}'") +
                              (s.ShowSSSWaterfall ? "  +WATERFALL" : ""));
        }
        sb.AppendLine(director.ShowOverlay
            ? "  WARN  Show Overlay is ON. Correct for the dress rehearsal — it is how you direct — and " +
              "WRONG for a take: Unity Recorder captures the Game view and burns it in."
            : "  ok    Show Overlay is off (take-ready).");
        return true;
    }

    /// <summary>
    /// A2b's readiness: the two markers, the wiring, and the SHOT ARITHMETIC — recomputed here from
    /// the live scene rather than quoted from the runbook, so a marker somebody nudged shows up as a
    /// different number instead of as a surprise on the take.
    ///
    /// The wiring is checked BY REFERENCE and not by shot name. A storyboard can be renamed, reordered
    /// or half-re-seeded; what makes this shot work is that ONE shot stands on `VideoCam_MiniFront`,
    /// that the shot BEFORE it hands over on `VideoTrigger_PreMini`, and that the mini-cam itself
    /// hands over on `VideoTarget_Mini`. Those three facts are what this looks for.
    /// </summary>
    static void AppendMiniCamLines(StringBuilder sb, CinematicDirector director, Transform mini,
                                   WaterSurface surface, Vector3 fwd, Vector3 right)
    {
        var miniMarker = director.transform.Find(MiniMarker);
        var miniCam = director.transform.Find(MiniCamMarker);
        var trigger = director.transform.Find(PreMiniTrigger);
        float waterY = surface != null ? surface.transform.position.y : 0f;

        if (miniCam == null || trigger == null)
        {
            sb.AppendLine($"  FAIL  the mini-cam shot needs '{MiniCamMarker}' " +
                          $"({(miniCam == null ? "MISSING" : "ok")}) and '{PreMiniTrigger}' " +
                          $"({(trigger == null ? "MISSING" : "ok")}) under the CinematicDirector. " +
                          "Run SMARC/Video/A1 — it places both and says where.");
            return;
        }

        // ---- are they where the geometry says? ----
        float dCam = Vector3.Distance(miniCam.position, MiniCamPos(mini, fwd, right));
        sb.AppendLine(dCam < 0.1f
            ? $"  ok    '{MiniCamMarker}' at {Fmt(miniCam.position)} — {PassSpec.MiniCamAlongM:F1} m " +
              $"down-track of the car (the end its nose points at), {PassSpec.MiniCamTowardTrackM:F2} m " +
              $"toward the track, {miniCam.position.y - PassSpec.MeasuredSeabedUnderCarY:F2} m over the " +
              $"measured seabed and {(mini.position.y + PassSpec.CarHeightM) - miniCam.position.y:F2} m " +
              "BELOW the car's roofline — which is what makes it a bottom-up shot."
            : $"  WARN  '{MiniCamMarker}' is {dCam:F2} m off where the geometry puts it. That is fine if you " +
              "moved it on purpose (the numbers below are computed from where it actually is); press " +
              "SMARC/Video/A1 to snap it back.");

        Vector3 closest = mini.position - right * PassSpec.TrackLateralM;
        closest.y = waterY - PassSpec.PassDepthM;
        float dTrig = Vector3.Distance(trigger.position, PreMiniTriggerPos(mini, fwd, right, waterY));
        float trigAlong = Vector3.Dot(closest - trigger.position, fwd);
        float trigLateral = Vector3.Dot(trigger.position - closest, right);
        sb.AppendLine((dTrig < 0.1f ? "  ok    " : "  WARN  ") +
                      $"'{PreMiniTrigger}' at {Fmt(trigger.position)} — {trigAlong:F2} m before the " +
                      $"closest-approach point, {Mathf.Abs(trigLateral):F2} m off the track line. " +
                      (dTrig < 0.1f ? "This marker, not the lens, is what sets the shot's length."
                                    : "It has been moved off the seeded point; the seconds below are " +
                                      "computed from where it actually is."));

        // ---- the shot arithmetic, at both ends of the plausible cruise ----
        float slow = trigAlong / Mathf.Max(PassSpec.CruiseSpeedMS, 1e-3f)
                   - PassSpec.PreMiniSteadyS + PassSpec.MiniCamRunOnS;
        float fast = trigAlong / Mathf.Max(PassSpec.CruiseSpeedFastMS, 1e-3f)
                   - PassSpec.PreMiniSteadyS + PassSpec.MiniCamRunOnS;
        float floorS = Mathf.Max(director.MinShotSeconds, PassSpec.MiniCamMinS);
        sb.AppendLine((slow <= PassSpec.MiniCamMaxS && fast >= floorS ? "  ok    " : "  WARN  ") +
                      $"predicted ON SCREEN: {fast:F1} s at {PassSpec.CruiseSpeedFastMS:F2} m/s, " +
                      $"{slow:F1} s at the {PassSpec.CruiseSpeedMS:F2} m/s default " +
                      $"(= {trigAlong:F2} m / v - {PassSpec.PreMiniSteadyS:F1} s of A2 hold + " +
                      $"{PassSpec.MiniCamRunOnS:F1} s of run-on). Floor {floorS:F1} s, ceiling " +
                      $"{PassSpec.MiniCamMaxS:F0} s. Want it longer? Move '{PreMiniTrigger}' further " +
                      "up-track — a metre is about three seconds at the default cruise.");

        // ---- what the lens will see ----
        Vector3 lens = miniCam.position;
        Vector3 cut = closest - fwd * Mathf.Max(0f, trigAlong - PassSpec.CruiseSpeedMS * PassSpec.PreMiniSteadyS);
        Vector3 end = closest + fwd * (PassSpec.CruiseSpeedMS * PassSpec.MiniCamRunOnS);
        float rCut = Vector3.Distance(cut, lens);
        float rCpa = Vector3.Distance(closest, lens);
        float rEnd = Vector3.Distance(end, lens);
        float aCut = surface != null ? Transmittance(rCut, surface.absorptionDistance, surface.refractionColor.g) : 0f;
        float aCpa = surface != null ? Transmittance(rCpa, surface.absorptionDistance, surface.refractionColor.g) : 0f;
        float elevCpa = Mathf.Asin(Mathf.Clamp((closest.y - lens.y) / Mathf.Max(rCpa, 1e-3f), -1f, 1f)) * Mathf.Rad2Deg;
        sb.AppendLine((aCpa >= 0.15f ? "  ok    " : "  WARN  ") +
                      $"the AUV enters frame {rCut:F2} m out ({aCut * 100f:F0} % green) and is {rCpa:F2} m " +
                      $"out ({aCpa * 100f:F0} %) as it draws level with the car, {elevCpa:F0}° above the " +
                      $"lens's horizon; at the cut back to the chase it is {rEnd:F2} m out and still closing. " +
                      (aCpa >= 0.15f ? "It will read." : "It will be a silhouette — see the water line above."));

        // ---- will the car be in frame at the cut? 16:9 assumed, and said so. ----
        var shot = FindMiniCamShot(director, miniCam);
        float fov = shot != null ? shot.FieldOfView : PassSpec.MiniCamFov;
        float halfH = Mathf.Atan(Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad) * 16f / 9f) * Mathf.Rad2Deg;
        Vector3 aimAtCut = cut + (shot != null ? shot.LookAtOffset
                                               : right * PassSpec.MiniCamAimStarboardM
                                                 + Vector3.down * PassSpec.MiniCamAimDownM);
        Vector3 carCentre = mini.position + Vector3.up * (PassSpec.CarHeightM * 0.5f);
        float offAxis = Vector3.Angle(aimAtCut - lens, carCentre - lens);
        sb.AppendLine((offAxis <= halfH ? "  ok    " : "  info  ") +
                      $"at the cut the car's body centre is {offAxis:F0}° off the aim and the lens's " +
                      $"horizontal half-frame is {halfH:F0}° (FOV {fov:F0} vertical, AT 16:9 — set the Game " +
                      $"view aspect explicitly). " +
                      (offAxis <= halfH
                          ? "The wreck opens the shot in the right-hand edge of frame and clears out as the " +
                            "AUV closes."
                          : "The car is OUT of frame at the cut; the shot is the AUV alone. More car: move " +
                            $"'{MiniCamMarker}' further down-track, or widen the shot's Field Of View."));

        // ---- the wiring, by reference ----
        if (shot == null)
            sb.AppendLine($"  FAIL  no shot stands on '{MiniCamMarker}' — the marker is placed but no " +
                          "StaticLookAt shot uses it as its Anchor. Run SMARC/Video/A1.");
        else
        {
            int idx = director.Shots.IndexOf(shot);
            bool exitOk = shot.Advance == CameraShot.AdvanceWhen.TargetAstern && shot.AsternTarget == miniMarker;
            var entry = idx > 0 ? director.Shots[idx - 1] : null;
            bool entryOk = entry != null && entry.Advance == CameraShot.AdvanceWhen.TargetAstern &&
                           entry.AsternTarget == trigger;
            sb.AppendLine($"  {(exitOk && entryOk ? "ok  " : "FAIL")}  mini-cam wiring: shot {idx + 1} " +
                          $"'{shot.Name}' [{shot.Kind}] " +
                          $"anchored on '{MiniCamMarker}', aims '{(string.IsNullOrEmpty(shot.LookAtName) ? (shot.LookAt != null ? shot.LookAt.name : "NOTHING") : shot.LookAtName)}'; " +
                          $"ENTRY = {(entryOk ? $"shot {idx} '{entry.Name}' on '{PreMiniTrigger}'" : "NOT the pre-mini trigger — the shot before it does not hand over on that marker")}; " +
                          $"EXIT = {(exitOk ? $"'{MiniMarker}' astern + {shot.SteadySeconds:F1} s" : "NOT the car — this shot can only fall through on its ceiling")}.");
            if (shot.LookAt == null && string.IsNullOrEmpty(shot.LookAtName))
                sb.AppendLine("  FAIL  the mini-cam shot has no Look At and no Look At Name, so EvalStatic " +
                              "will hold the anchor's own rotation and film whatever it happens to face. " +
                              "Set Look At Name to 'base_link'.");
        }
    }

    /// <summary>The StaticLookAt shot standing on the mini-cam anchor, or null.</summary>
    static CameraShot FindMiniCamShot(CinematicDirector director, Transform miniCam)
    {
        if (director.Shots == null) return null;
        foreach (var s in director.Shots)
            if (s != null && s.Kind == CameraShot.Mode.StaticLookAt && s.Anchor == miniCam) return s;
        return null;
    }

    /// <summary>
    /// The side-scan geometry, read off the LIVE components rather than off a constant, because the
    /// mount has been silently re-serialized in this project before (SETTLED §3k: a prefab rebuild
    /// wrote the v1 tilt 45 / breadth 45 back over the committed tilt 0 / breadth 60, and that one
    /// bad read propagated into a constant, its own guard test, and a farm prior). If this ever
    /// prints 0..45°, the mount has regressed and the fly-past lateral offset is wrong by design.
    /// </summary>
    static void AppendSonarLines(StringBuilder sb, GameObject vehicle)
    {
        if (vehicle == null) { sb.AppendLine("  FAIL  no vehicle to read the sonar geometry from."); return; }

        Sonar sss = null;
        foreach (var s in vehicle.GetComponentsInChildren<Sonar>(true))
            if (s.Type == SonarType.SSS) { sss = s; break; }
        if (sss == null) { sb.AppendLine("  FAIL  no SSS-type Sonar under the vehicle."); return; }

        var dv = sss.GetComponent<DeepVisionSSS>();
        // Sonar.MaxRange / NumBucketsPerBeam are written by DeepVisionSSS.Apply() at Awake, so in
        // EDIT mode the authoritative numbers are the DeepVision settings, not the Sonar's.
        float tilt = dv != null ? dv.TiltAngleDeg : sss.TiltAngleDeg;
        float breadth = dv != null ? dv.BeamBreadthDeg : sss.BeamBreadthDeg;
        float range = dv != null ? Mathf.Clamp(dv.RangeM, 5f, dv.MaxRangeForMode) : sss.MaxRange;
        float binM = dv != null ? (dv.Mode == DeepVisionSSS.FrequencyMode.LF340 ? 0.20f : 0.04f)
                                : range / Mathf.Max(1, sss.NumBucketsPerBeam);

        float thetaMin = Mathf.Max(0f, 90f - tilt - breadth);
        float thetaMax = Mathf.Min(90f, 90f - tilt);
        sb.AppendLine($"  info  SSS mount tilt {tilt:F0}° / breadth {breadth:F0}° ⇒ off-nadir " +
                      $"{thetaMin:F0}°..{thetaMax:F0}° with a {2f * thetaMin:F0}° NADIR GAP; " +
                      $"{(dv != null ? dv.Mode.ToString() : "?")} at {range:F0} m/side, " +
                      $"{binM * 100f:F0} cm bins, ping {(1500f / (2f * range)):F1} Hz.");
        if (Mathf.Abs(tilt - 45f) < 1f && Mathf.Abs(breadth - 45f) < 1f)
            sb.AppendLine("  FAIL  that is the REGRESSED_BEAM_2026_08_16 signature (tilt 45 / breadth 45, " +
                          "off-nadir 0..45°). SAMSensorsV2.prefab has been re-serialized back to the v1 " +
                          "mount. `git diff -- Runtime/Prefabs/Components/SAMSensorsV2.prefab` before " +
                          "believing any sonar geometry in this scene (SETTLED §3k).");

        // The model, against what the scene actually holds.
        float dz = 3.72f;                                    // car base -8.22 under a sonar at -4.50
        float gapEdge = dz * Mathf.Tan(thetaMin * Mathf.Deg2Rad);
        float maxLat = range > dz ? Mathf.Sqrt(range * range - dz * dz) : 0f;
        float L = PassSpec.TrackLateralM;
        float theta = Mathf.Atan2(L, dz) * Mathf.Rad2Deg;
        float slant = Mathf.Sqrt(L * L + dz * dz);
        bool sees = L >= gapEdge && L <= maxLat;
        sb.AppendLine((sees ? "  ok    " : "  FAIL  ") +
                      $"at the pass the car is {L:F2} m to starboard and {dz:F2} m below the sonar ⇒ " +
                      $"{theta:F1}° off nadir, slant {slant:F2} m = range bin " +
                      $"{Mathf.RoundToInt(slant / Mathf.Max(binM, 1e-3f))} of " +
                      $"{Mathf.RoundToInt(range / Mathf.Max(binM, 1e-3f))}. The visible lateral window at " +
                      $"that depth difference is {gapEdge:F2}..{maxLat:F1} m" +
                      (sees ? "." : " — THE CAR IS OUTSIDE IT and the waterfall will show nothing where " +
                                    "the video says it should."));

        int viewers = vehicle.GetComponentsInChildren<RayViewer>(true).Length;
        sb.AppendLine(viewers > 0
            ? $"  ok    {viewers} RayViewer(s) under the vehicle — the per-shot beam toggle has something to " +
              "drive, and none of them carries a collider (§3o)."
            : "  WARN  no RayViewer under the vehicle: SonarBeamsVisible has nothing to switch on.");

        if (dv != null)
            sb.AppendLine($"  info  fidelity {dv.Fidelity}, beam pattern {(dv.BeamPattern ? "ON" : "off")}, " +
                          $"reverb floor {(dv.ReverbFloor ? "ON" : "off")}" +
                          (dv.Fidelity == SonarFidelity.Ideal && dv.ReverbFloor
                              ? " — NOTE: Sonar gates the floor on IsIdeal, so under Ideal fidelity that tick " +
                                "is inert. The take runs on the bare fan plus the accepted beam pattern."
                              : "."));
    }

    static float Transmittance(float rangeM, float absorptionM, float refractionChannel)
    {
        // HDRP 17.3, WaterSurface.cs:239 — MEASURED, not modelled:
        //   extinction = (-ln 0.02 / absorptionDistance) * max(1 - refractionColor, 0.01)
        float k = Mathf.Max(1f - refractionChannel, 0.01f);
        return Mathf.Exp(Mathf.Log(0.02f) / Mathf.Max(absorptionM, 1e-3f) * k * rangeM);
    }

    static string DescribeGeometry(Transform vehicle, Transform mini, WaterSurface surface,
                                   Vector3 fwd, Vector3 right)
    {
        float headingDeg = Mathf.Atan2(fwd.x, fwd.z) * Mathf.Rad2Deg;
        if (headingDeg < 0f) headingDeg += 360f;
        Vector3 closest = mini.position - right * PassSpec.TrackLateralM;
        float runIn = Vector3.Dot(closest - vehicle.position, fwd);
        float waterY = surface != null ? surface.transform.position.y : 0f;
        return
            $"THE TRACK, derived from the two objects and not typed in:\n" +
            $"  vehicle {Fmt(vehicle.position)} -> car {Fmt(mini.position)}\n" +
            $"  heading {headingDeg:F2}° from world +Z; the car is {PassSpec.TrackLateralM:F2} m to " +
            $"STARBOARD of the track, and the closest-approach point is " +
            $"{Fmt(new Vector3(closest.x, waterY, closest.z))} in plan, {runIn:F1} m along the run.\n" +
            $"  pass depth {PassSpec.PassDepthM:F2} m; chase lens {PassSpec.ChaseOffset} in base_link, " +
            $"FOV {PassSpec.ChaseFov:F0}°, aiming {PassSpec.ChaseAimStarboardM:F1} m to starboard and " +
            $"{PassSpec.ChaseAimDownM:F1} m down of the hull — that aim bias is what keeps the car in " +
            "frame through closest approach.\n" +
            $"  MINI-CAM: lens {PassSpec.MiniCamAlongM:F1} m DOWN-track of the car and " +
            $"{PassSpec.MiniCamTowardTrackM:F2} m toward the track, {PassSpec.MiniCamRiseM:F2} m over the " +
            $"car's base, FOV {PassSpec.MiniCamFov:F0}°, tracking base_link. It cuts in when the vehicle " +
            $"passes a marker {PassSpec.PreMiniTriggerM:F1} m short of the closest-approach point and cuts " +
            $"out {PassSpec.MiniCamRunOnS:F1} s after the car is astern — about " +
            $"{PassSpec.PreMiniTriggerM / PassSpec.CruiseSpeedMS - PassSpec.PreMiniSteadyS + PassSpec.MiniCamRunOnS:F0} s " +
            $"on screen at the {PassSpec.CruiseSpeedMS:F2} m/s default.";
    }

    static string Fmt(Vector3 v) => $"({v.x:F2}, {v.y:F2}, {v.z:F2})";

    static string HierarchyPath(Transform t)
    {
        if (t == null) return "(none)";
        var s = t.name;
        while (t.parent != null) { t = t.parent; s = t.name + "/" + s; }
        return s;
    }
}
