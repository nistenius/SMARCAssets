using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

using VehicleComponents.Sensors;   // Sonar, SonarHit, SonarType
using ROS.Subscribers;             // MissionWPHoop_Sub (plan-restart signal)
using SmarcGUI.Water;              // DockDrainDirector.DrainInProgress — the one time the plane moves

namespace Visualizers
{
    /// <summary>
    /// Accumulates the vehicle's sonar returns into a growing 3D point cloud, in world space,
    /// live, while the mission flies. After a zig-zag down the Beckholmen dry dock the cloud is
    /// the dock: floor, walls, and whatever was on them.
    ///
    /// THIS IS NOT SLAM, AND THE LABEL ON SCREEN SAYS SO.
    /// Every point is placed at the raycast's own world-space hit, i.e. at the position Unity
    /// knows the sonar was in — SIMULATION GROUND TRUTH, not an estimate. No pose graph, no
    /// registration, no drift, no loop closure. A map built from a perfect pose is a picture of
    /// the sensor's coverage, not a navigation result, and this project has already paid for one
    /// component handing ground truth to a consumer under an estimate's label (invariant 11: a
    /// SIGSEGV'd estimator read green for a session because a probe was reading Unity's own
    /// `smarc/odom`). So the on-screen label is `sonar map (ground-truth posed)` and it is on by
    /// default. Turning it off is a per-shot decision the CinematicDirector makes; changing what
    /// it says is not.
    ///
    /// IT TAPS THE SENSOR IN-PROCESS. `Sonar.SonarHits` is already a public array of world-space
    /// hits with intensities, refreshed every sensor tick. Going out over ROS and back would add
    /// a serialization round trip and a second copy of the same numbers to render one visual.
    ///
    /// IT ADDS NO GEOMETRY TO THE PHYSICS SCENE. No collider is ever created, on purpose: the
    /// sonar raycasts the physics scene, so a map object with a collider would be a map of
    /// itself (SETTLED §3o, §3g).
    ///
    /// PERFORMANCE SHAPE. Points are voxel-filtered on the way in (VoxelSize), so revisiting the
    /// same wall costs nothing after the first pass, and they are written into fixed-size mesh
    /// chunks; only the chunk currently filling is re-uploaded. A finished chunk is never touched
    /// again.
    ///
    /// ==================================================================================
    /// DEFECT A, 2026-08-21: THIS ACCUMULATED 0 POINTS FOR A WHOLE 643 s MISSION.
    /// ==================================================================================
    /// ROOT CAUSE, measured out of the asset files and not guessed:
    ///   * `Beckholmen.unity` holds TWO prefab instances both renamed `sam_auv_v1` — an INACTIVE
    ///     `sam_auv_v1.prefab` (guid 78d04dcb…, `m_IsActive: 0`) and the ACTIVE `sam2.2.prefab`
    ///     (guid c1e7da78…, `m_IsActive: 1`). The active one — the one that flies — carries
    ///     `SAMSensorsV2.prefab`, whose sonars are `SideScanSonar DeepVision` (**Type = SSS**) and
    ///     the nested `Sonar3D15` (**Type = FLS**, 5 Hz, 17 rays x 150 beams, 15 m range).
    ///   * There is NO MBES anywhere on that vehicle. `sam_auv_v1.prefab` uses the older
    ///     `SAMSensors.prefab`, which does have a `MultiBeamSonar` — and it is the INACTIVE twin.
    ///   * The defaults here were `IncludeMBES = true, IncludeSSS = false, IncludeFLS = false`.
    ///     `FindSonars()` therefore filtered out EVERY sonar the vehicle has, `Sonars.Count` was 0,
    ///     and the map could not grow. The 5 Hz in the frame evidence is the Sonar3D15's own rate:
    ///     the sensor was firing the whole time and nothing was reading it.
    /// WHAT CHANGED:
    ///   1. The type flags now default to what this vehicle actually carries (FLS + MBES on).
    ///   2. `TapEverySonarIfFilterMatchesNone`: when the filter matches nothing but the robot HAS
    ///      sonars, it taps them all and says so as an ERROR naming each one. A visualiser that
    ///      shows nothing is worse than one that shows the wrong sonar and admits it.
    ///   3. `MinIntensity` defaults to 0. The old 0.02 is not harmless on a SHORT-RANGE sonar:
    ///      `SonarHit.GetIntensity` scales by `(MaxRange - distance) / MaxRange`, so on the 15 m
    ///      Sonar3D15 a return at 14 m starts at 0.067 before the incidence-angle and material
    ///      terms, and a grazing hit on 0.5-reflectivity concrete lands under 0.02.
    ///   4. THE LABEL AND THE CONSOLE NOW TELL THE TRUTH WHILE IT IS EMPTY. Every
    ///      `DiagnosticIntervalSec` with zero points, it prints the full hierarchy path of every
    ///      sonar it bound, whether that sonar is firing at all (how many of its rays hit
    ///      anything), and the per-reason discard counts — no-collider / below-intensity /
    ///      own-hull / already-in-voxel. The on-screen label says NO SONAR BOUND or
    ///      "N returns, all discarded" instead of a serene "0 pts".
    /// That last point is the general lesson, not the sonar one: a readout that can show nothing
    /// for ten minutes without complaining is the §3s stale-`AppliedBuoyancyForce` defect wearing
    /// a new coat.
    ///
    /// ==================================================================================
    /// ROUND 3, 2026-08-21: RETURNS FROM ABOVE THE WATER LINE ARE NOT SONAR RETURNS.
    /// ==================================================================================
    /// Take 006 drew yellow and red points along the tops of the quay walls. They are real
    /// raycasts — the FLS is a geometric raycast against the physics scene and nothing in it stops
    /// at the water surface, so a vehicle near the surface "images" the concrete standing in the
    /// AIR in front of it. A real sonar cannot do that: sound does not cross the water line and
    /// come back. So this is a FIDELITY defect in the picture, not a cosmetic preference, and the
    /// filter is `DiscardHitsAboveWater`: any hit whose world Y is above (still-water plane −
    /// `AboveWaterMarginM`) is discarded and COUNTED, like every other discard reason.
    ///
    /// The plane is read off the `WaterSurface` TRANSFORM and NEVER from `GetWaterLevelAt`
    /// (SETTLED §3s: one shared `HDRPWaterQueryModel`, one shared search seed, and a caller far
    /// from the hull poisons the ForcePoints' next query). With no WaterSurface in the scene the
    /// filter turns ITSELF OFF and says so, rather than assuming Y = 0 — the same rule the station
    /// transducer follows (§3s8). The one moment the plane legitimately moves is a sanctioned
    /// `DockDrainDirector` drain, and the cached plane is deliberately NOT refreshed while that
    /// runs: the dock emptying must not retroactively change what counts as "above water".
    ///
    /// AND THE MAP IS BLUE BY DEPTH (Ivan: "color the mapped part in blue from dark (at depth) to
    /// lighter (at shallow) grading"). Depth is `water plane Y − hit Y`, over a FIXED range
    /// (`DepthRampShallowM`..`DepthRampDeepM`, 0–8 m). Fixed and not auto-rescaled on purpose: a
    /// point's colour is baked into its vertex UV when it is added, finished chunks are never
    /// touched again, and an auto-rescaling ramp would therefore leave old points carrying an old
    /// scale's meaning while the legend said something else. A ramp that means two things at once
    /// is the same failure as a label that reads "0 pts" for two different reasons.
    /// </summary>
    [AddComponentMenu("Smarc/Visualizers/Sonar Map Accumulator")]
    public class SonarMapAccumulator : MonoBehaviour
    {
        public enum ColorSource { Depth, Intensity, MaterialLabel }

        [Header("Which sonars")]
        [Tooltip("Robot whose sonars to tap. Used to find them and to ignore returns off the vehicle's own hull.")]
        public string RobotName = "sam_auv_v1";
        [Tooltip("Find every Sonar under the named robot at Play. Turn off to drive the list by hand.")]
        public bool AutoFindSonars = true;
        [Tooltip("Sonars to accumulate. Filled at Play when AutoFindSonars is on.")]
        public List<Sonar> Sonars = new List<Sonar>();
        [Tooltip("Include the downward multibeam. sam2.2 does NOT have one — only the older sam_auv_v1 prefab does (SAMSensors/MultiBeamSonar). Left on because it costs nothing and the older vehicle is still flown elsewhere.")]
        public bool IncludeMBES = true;
        [Tooltip("Include the side scans. OFF, and Ivan asked for it explicitly on 2026-08-21 round 3: " +
                 "\"skip the side scan in the 3d point cloud map building here\". They draw the dock WALLS at " +
                 "the cost of many more points, and §3l applies — a side scan sees nothing at or above its own " +
                 "depth, so a 2.5 m run images the floor and the lower walls and nothing else. " +
                 "SMARC/Video/2 turns this back OFF if it has been ticked for a test.")]
        public bool IncludeSSS = false;
        [Tooltip("Include forward-looking sonars. ON since 2026-08-21: Sonar3D15 (Type = FLS, 5 Hz, 150 beams x 17 rays, 15 m) is the ONLY forward sensor sam2.2 carries, and it is what drapes the dock. With this off the map is empty — that was defect A.")]
        public bool IncludeFLS = true;
        [Tooltip("If the Include* flags match NO sonar but the robot has some, tap them all anyway and log an ERROR naming each. A map that stays empty for ten minutes is worse than a map that admits it is showing a sonar you did not tick.")]
        public bool TapEverySonarIfFilterMatchesNone = true;

        [Header("Sampling")]
        [Tooltip("How often the hit buffer is read, Hz. The sonar itself runs at its own frequency; sampling faster than that just re-reads the same ping.")]
        public float SampleHz = 5f;
        [Tooltip("Take every Nth ray. 500 rays per beam is far more than a picture needs; 3 keeps the swath dense and the point count sane.")]
        public int DecimationStride = 3;
        [Tooltip("Drop returns weaker than this (0-1). 0 keeps everything including grazing hits at the swath edge. KEEP IT AT 0 on a short-range sonar: SonarHit.GetIntensity scales by (MaxRange - distance)/MaxRange, so on the 15 m Sonar3D15 a far grazing return is already below 0.02 before anything else is applied.")]
        public float MinIntensity = 0f;
        [Tooltip("Merge points closer together than this, in metres. 0 disables the filter and the cloud will smear where the vehicle loiters. This is a FILTER, not a resolution claim: the raycast is exact, the display is decimated.")]
        public float VoxelSize = 0.2f;
        [Tooltip("Discard returns off the vehicle's own hull and anything parented to it.")]
        public bool IgnoreHitsOnRobot = true;

        [Header("Above the water line — a fidelity filter, not a cosmetic one")]
        [Tooltip("DISCARD EVERY RETURN FROM ABOVE THE WATER LINE (Ivan, 2026-08-21 round 3: \"cut away all " +
                 "sonar echos from above the surface\"). The sonar is a geometric raycast and nothing in it " +
                 "stops at the surface, so a vehicle near the top images the quay standing in the AIR — take " +
                 "006 drew points along the dock edges. A real sonar cannot return from above the water line. " +
                 "With no WaterSurface in the scene this turns itself OFF and says so; it never assumes Y = 0.")]
        public bool DiscardHitsAboveWater = true;
        [Tooltip("Metres BELOW the still-water plane where the cut-off actually sits. A small positive value " +
                 "also removes the ragged band of returns right at the water line, which is where a surfaced " +
                 "vehicle's beams graze the surface and read as noise.")]
        public float AboveWaterMarginM = 0.10f;
        [Tooltip("The WaterSurface whose TRANSFORM Y is the still-water plane. Found at Play when empty. Read " +
                 "for its transform ONLY — never GetWaterLevelAt (SETTLED §3s).")]
        public WaterSurface WaterPlaneSource;

        [Header("Budget")]
        [Tooltip("Hard cap on accumulated points. Past this the map stops growing and says so once, rather than growing until the frame rate tells you.")]
        public int MaxPoints = 300000;
        [Tooltip("Points per mesh chunk. Each point is a 3-vertex flake, so this times 3 is the vertex count; the mesh uses 32-bit indices so it is not the old 65k limit that binds, it is upload cost.")]
        public int PointsPerChunk = 40000;

        [Header("Look")]
        [Tooltip("Size of one point's flake, metres across.")]
        public float PointSize = 0.14f;
        public ColorSource ColorBy = ColorSource.Depth;
        [Tooltip("DEPTH IS MEASURED FROM THE WATER PLANE, not from world zero. On (the default), the ramp " +
                 "coordinate is (waterPlaneY - hitY) mapped over DepthRampShallowM..DepthRampDeepM. Off falls " +
                 "back to the old world-Y fields below, which are kept so an older saved scene still behaves.")]
        public bool DepthRampRelativeToWaterPlane = true;
        [Tooltip("Depth in METRES BELOW THE WATER PLANE that maps to the LIGHT end of the ramp (shallow).")]
        public float DepthRampShallowM = 0f;
        [Tooltip("Depth in METRES BELOW THE WATER PLANE that maps to the DARK end of the ramp (deep). " +
                 "FIXED, and deliberately not auto-fitted to the deepest point measured so far: a point's " +
                 "colour is baked into its vertex UV when it is added and finished chunks are never rewritten, " +
                 "so an auto-rescaling ramp would leave old points coloured on an old scale while the legend " +
                 "claimed a new one. Beckholmen's dock floor is about 7 m down, hence 8.")]
        public float DepthRampDeepM = 8f;
        [Tooltip("LEGACY (DepthRampRelativeToWaterPlane off): world Y that maps to the START of the gradient.")]
        public float DepthRampTop = 0f;
        [Tooltip("LEGACY (DepthRampRelativeToWaterPlane off): world Y that maps to the END of the gradient.")]
        public float DepthRampBottom = -14f;
        [Tooltip("Colour ramp. Baked into a 256x1 texture at Play and read through the material's colour map, so no custom shader is needed.")]
        public Gradient Ramp = new Gradient();
        [Tooltip("Material for the cloud. ASSIGN THIS: SMARC/Video/1 creates SonarMap.mat (HDRP/Unlit, emissive). Left empty the component builds one from Shader.Find and says so.")]
        public Material MapMaterial;
        [Tooltip("Emissive multiplier for the cloud, so it reads through murky water like the hoops do.")]
        public float EmissiveIntensity = 2.5f;

        [Header("Visibility")]
        [Tooltip("Show the accumulated cloud. The CinematicDirector drives this per shot.")]
        public bool Visible = true;
        [Tooltip("Keep accumulating even while hidden, so a shot that turns the map on mid-mission shows everything measured so far and not only what arrived after the cut.")]
        public bool AccumulateWhileHidden = true;

        [Header("The label — read the class comment before changing it")]
        public bool ShowLabel = true;
        public string MapLabel = "sonar map (ground-truth posed)";
        [Tooltip("Second line. The point of it is that nobody watching the video can mistake this for SLAM.")]
        public string MapSubLabel = "sonar returns placed from simulation ground-truth pose — not SLAM";
        [Tooltip("0 top-left, 1 top-right, 2 bottom-left, 3 bottom-right.")]
        [Range(0, 3)] public int LabelCorner = 2;
        [Tooltip("Label height as a fraction of screen height, so it reads the same at 1080p and 4K.")]
        public float LabelScale = 0.018f;
        [Tooltip("Append the live point count to the label.")]
        public bool LabelShowsPointCount = true;

        [Header("Clearing")]
        [Tooltip("Clear the map when the waypoint hoops clear, i.e. when the first waypoint of a plan arrives again. A re-flown mission should not draw on top of the previous run.")]
        public bool ClearOnPlanRestart = true;

        [Header("Diagnostics — the answer to 'why is it 0 pts?'")]
        [Tooltip("Seconds between the zero-points report. While the map has no points, the Console gets the full hierarchy path of every bound sonar, whether it is firing, and the per-reason discard counts. Set 0 to silence it (do not).")]
        public float DiagnosticIntervalSec = 10f;

        // ---------------------------------------------------------------- state

        readonly List<Chunk> chunks = new List<Chunk>();
        readonly HashSet<long> occupied = new HashSet<long>();
        Transform robotRoot;
        Transform cloudRoot;
        Material runtimeMat;
        Texture2D rampTex;
        float nextSample;
        bool cappedWarned;
        GUIStyle labelStyle, subLabelStyle;

        // Why points did not arrive. Reset every Sample(), so the report is about the LAST ping and
        // not about all of history — "it discarded 4 million returns" is not a diagnosis, "the last
        // ping had 2550 rays, 2550 of them hit nothing" is.
        int lastRaysRead, lastNoCollider, lastBelowIntensity, lastOwnHull, lastVoxelDup, lastAccepted;
        int lastAboveWater;
        long totalRaysRead, totalHits, totalAccepted, totalAboveWater;

        // The still-water plane, LATCHED. See the class comment: read off the transform, never
        // GetWaterLevelAt, and never refreshed while a sanctioned drain has it deliberately off zero.
        float waterPlaneY;
        bool hasWaterPlane;
        float nextDiagnostic;
        bool filterFallbackEngaged;
        string bindingSummary = "(not resolved yet)";

        public int PointCount { get; private set; }

        /// <summary>Sonars bound at Play. Public so the readiness report and the director can say what is being tapped.</summary>
        public int BoundSonarCount => Sonars != null ? Sonars.Count : 0;
        /// <summary>One line describing what this is tapping, for anything that wants to report it.</summary>
        public string BindingSummary => bindingSummary;
        /// <summary>Returns seen since Play that carried a collider. 0 with sonars bound means nothing is in range.</summary>
        public long ReturnsSeen => totalHits;

        class Chunk
        {
            public GameObject go;
            public Mesh mesh;
            public List<Vector3> verts = new List<Vector3>();
            public List<Vector2> uvs = new List<Vector2>();
            public List<int> tris = new List<int>();
            public int points;
            public bool dirty;
        }

        // ---------------------------------------------------------------- setup

        void Reset()
        {
            SeedRamp();
        }

        /// <summary>The shallow (t = 0) end of the shipped blue ramp. Public so the readiness report can identify it.</summary>
        public static readonly Color RampShallow = new Color(0.62f, 0.94f, 1.00f);
        /// <summary>The deep (t = 1) end of the shipped blue ramp.</summary>
        public static readonly Color RampDeep = new Color(0.02f, 0.06f, 0.26f);
        /// <summary>The shallow end of the ramp shipped BEFORE 2026-08-21 round 3 (sand → violet). Used to spot a scene that still carries it.</summary>
        public static readonly Color LegacyRampShallow = new Color(1.00f, 0.85f, 0.35f);

        /// <summary>
        /// The shipped depth ramp: DARK NAVY AT DEPTH, LIGHT CYAN NEAR THE SURFACE (Ivan,
        /// 2026-08-21 round 3). t = 0 is shallow and t = 1 is deep, which is the direction
        /// `RampCoord` produces.
        ///
        /// Public because `AddComponent` from an editor script does NOT call `Reset()` — a
        /// component created by the video-rig menu would otherwise ship Unity's default
        /// black-to-white gradient and the cloud would read as soot.
        /// </summary>
        public void SeedRamp()
        {
            Ramp = new Gradient();
            Ramp.SetKeys(
                new[]
                {
                    new GradientColorKey(RampShallow,                        0.00f),  // shallow: light cyan
                    new GradientColorKey(new Color(0.28f, 0.68f, 0.95f),     0.35f),  // sky blue
                    new GradientColorKey(new Color(0.10f, 0.32f, 0.72f),     0.70f),  // deep blue
                    new GradientColorKey(RampDeep,                           1.00f)   // deepest: dark navy
                },
                new[] { new GradientAlphaKey(1f, 0f), new GradientAlphaKey(1f, 1f) });
        }

        /// <summary>
        /// True when the serialized ramp is one this component did not author in its current form:
        /// either Unity's empty/default gradient, or the pre-round-3 sand → violet ramp. The video
        /// rig menu uses this to REPLACE it, because a default changed in code is not a change to
        /// the serialized asset and the only proof is reading it (SETTLED §3o, the Collidable trap).
        /// </summary>
        public bool RampNeedsReseeding(out string why)
        {
            if (Ramp == null || Ramp.colorKeys == null || Ramp.colorKeys.Length == 0)
            { why = "the gradient is empty (Unity's black-to-white default)"; return true; }

            Color first = Ramp.colorKeys[0].color;
            if (Approximately(first, LegacyRampShallow))
            { why = "it is the pre-round-3 sand-to-violet ramp"; return true; }
            if (Approximately(first, RampShallow)) { why = "it is already the blue depth ramp"; return false; }

            why = $"it starts at {first} — hand-edited, so it is left alone";
            return false;
        }

        static bool Approximately(Color a, Color b)
        {
            return Mathf.Abs(a.r - b.r) < 0.01f && Mathf.Abs(a.g - b.g) < 0.01f && Mathf.Abs(a.b - b.b) < 0.01f;
        }

        void Start()
        {
            if (Ramp == null || Ramp.colorKeys == null || Ramp.colorKeys.Length == 0) SeedRamp();

            robotRoot = ResolveRobotRoot();
            ResolveWaterPlane();

            if (AutoFindSonars) FindSonars();

            ReportBinding();

            cloudRoot = new GameObject("SonarMapCloud").transform;
            cloudRoot.SetParent(transform, false);

            rampTex = BuildRampTexture(256);
            runtimeMat = BuildMaterial();

            if (ClearOnPlanRestart)
            {
                var hoopSub = FindFirstObjectByType<MissionWPHoop_Sub>();
                if (hoopSub != null) hoopSub.OnHoopsCleared += OnPlanRestarted;
                else Debug.Log("[SonarMapAccumulator] ClearOnPlanRestart is on but there is no MissionWPHoop_Sub " +
                               "in the scene to signal a restart — clear the map by hand or from the director.");
            }
        }

        void OnDisable()
        {
            var hoopSub = FindFirstObjectByType<MissionWPHoop_Sub>();
            if (hoopSub != null) hoopSub.OnHoopsCleared -= OnPlanRestarted;
        }

        /// <summary>
        /// The hoop display cleared. It NAMES why, and so does this: the map disappearing mid-take
        /// must never be an unattributed event.
        /// </summary>
        void OnPlanRestarted(string reason)
        {
            Debug.Log($"[SonarMapAccumulator] clearing the map because the hoop display cleared: {reason}");
            ClearMap();
        }

        /// <summary>
        /// The vehicle root, chosen deliberately rather than by whatever `GameObject.Find` hands back.
        ///
        /// BECKHOLMEN HAS TWO OBJECTS NAMED `sam_auv_v1`: an inactive `sam_auv_v1.prefab` instance
        /// and the active `sam2.2.prefab` instance. `GameObject.Find` skips inactive objects, so it
        /// happens to return the right one — but "happens to" is not a property, so this enumerates
        /// every match, including inactive ones, and says out loud when there is more than one.
        /// </summary>
        Transform ResolveRobotRoot()
        {
            if (string.IsNullOrEmpty(RobotName)) return null;

            var matches = new List<Transform>();
            foreach (var t in FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (t != null && t.parent == null && t.name == RobotName) matches.Add(t);

            Transform active = null;
            foreach (var t in matches) if (t.gameObject.activeInHierarchy) { active = t; break; }

            if (matches.Count == 0)
            {
                Debug.LogWarning($"[SonarMapAccumulator] no GameObject named '{RobotName}' in the scene — " +
                                 "cannot auto-find sonars and cannot filter returns off the vehicle's own hull.");
                return null;
            }
            if (matches.Count > 1)
            {
                var lines = new List<string>();
                foreach (var t in matches)
                    lines.Add($"'{HierarchyPath(t)}' active={t.gameObject.activeInHierarchy} at {t.position}");
                Debug.LogWarning($"[SonarMapAccumulator] {matches.Count} objects are named '{RobotName}': " +
                                 $"{string.Join(" | ", lines)}. Binding to the ACTIVE one. If the sonars you " +
                                 "expected are on the other, that is the object to activate — the inactive twin " +
                                 "never pings and nothing downstream would say so.");
            }
            if (active == null)
                Debug.LogWarning($"[SonarMapAccumulator] every GameObject named '{RobotName}' is INACTIVE. " +
                                 "Its sonars will never fire and the map cannot grow.");
            return active != null ? active : matches[0];
        }

        void FindSonars()
        {
            Sonars.Clear();
            filterFallbackEngaged = false;

            // Include INACTIVE in the sweep so the report can tell "there are no sonars" apart from
            // "the sonars are on a disabled object" — two very different problems that both look
            // like an empty map.
            var all = FindObjectsByType<Sonar>(FindObjectsInactive.Include, FindObjectsSortMode.None);

            var candidates = new List<Sonar>();
            foreach (var s in all)
            {
                if (s == null) continue;
                if (robotRoot != null && !s.transform.IsChildOf(robotRoot)) continue;
                candidates.Add(s);
            }

            foreach (var s in candidates)
            {
                if (!s.gameObject.activeInHierarchy) continue;
                if (s.Type == SonarType.MBES && !IncludeMBES) continue;
                if (s.Type == SonarType.SSS && !IncludeSSS) continue;
                if (s.Type == SonarType.FLS && !IncludeFLS) continue;
                Sonars.Add(s);
            }

            if (Sonars.Count > 0 || candidates.Count == 0) return;

            // THE DEFECT-A CASE. Every sonar the vehicle has was filtered out by the type flags.
            var why = new List<string>();
            foreach (var s in candidates)
                why.Add($"'{HierarchyPath(s.transform)}' Type={s.Type} " +
                        $"(needs Include{s.Type} = true) active={s.gameObject.activeInHierarchy}");

            if (TapEverySonarIfFilterMatchesNone)
            {
                foreach (var s in candidates) if (s.gameObject.activeInHierarchy) Sonars.Add(s);
                filterFallbackEngaged = true;
                Debug.LogError($"[SonarMapAccumulator] the Include* flags matched NONE of the " +
                               $"{candidates.Count} sonar(s) on '{RobotName}' — tapping all of them anyway " +
                               $"and saying so, because an empty map is not a measurement. " +
                               $"{string.Join("; ", why)}. Tick the right Include flag and this line goes away. " +
                               "(This is exactly what produced '0 pts' for the whole 643 s take on 2026-08-21: " +
                               "sam2.2 has an FLS and an SSS and no MBES, and only IncludeMBES was on.)");
            }
            else
            {
                Debug.LogError($"[SonarMapAccumulator] the Include* flags matched NONE of the " +
                               $"{candidates.Count} sonar(s) on '{RobotName}', and " +
                               "TapEverySonarIfFilterMatchesNone is off — THE MAP WILL STAY EMPTY. " +
                               string.Join("; ", why));
            }
        }

        /// <summary>Said once at Play, in the same words the on-screen label uses while it is empty.</summary>
        void ReportBinding()
        {
            if (Sonars.Count == 0)
            {
                bindingSummary = "NO SONAR BOUND";
                Debug.LogError("[SonarMapAccumulator] no sonars to tap. The map will stay empty, which is a " +
                               "CONFIGURATION FACT and not a measurement — check RobotName and the Include* " +
                               "flags. The label on screen will say so for the whole take rather than " +
                               "reading a serene '0 pts'.");
                return;
            }

            var lines = new List<string>();
            foreach (var s in Sonars)
            {
                if (s == null) continue;
                lines.Add($"'{HierarchyPath(s.transform)}' Type={s.Type} {s.frequency:F0} Hz " +
                          $"{s.TotalRayCount} rays ({s.NumBeams} beam(s) x {s.NumRaysPerBeam}) range {s.MaxRange:F0} m " +
                          $"enabled={s.isActiveAndEnabled}");
            }
            bindingSummary = $"{Sonars.Count} sonar(s)" + (filterFallbackEngaged ? " (FILTER OVERRIDDEN)" : "");
            Debug.Log($"[SonarMapAccumulator] tapping {Sonars.Count} sonar(s):\n  " + string.Join("\n  ", lines) +
                      $"\n  sampling at {SampleHz:F0} Hz, stride {DecimationStride}, MinIntensity {MinIntensity:F3}, " +
                      $"voxel {VoxelSize:F2} m, ignore-own-hull {IgnoreHitsOnRobot}.");
        }

        static string HierarchyPath(Transform t)
        {
            if (t == null) return "(none)";
            var s = t.name;
            while (t.parent != null) { t = t.parent; s = t.name + "/" + s; }
            return s;
        }

        /// <summary>
        /// Latch the still-water plane off the WaterSurface TRANSFORM. Never `GetWaterLevelAt`
        /// (SETTLED §3s). With no WaterSurface the above-water filter turns ITSELF off and says so:
        /// a missing measurement is not the same as a measurement of zero, and this project has
        /// already paid for a component that assumed the difference away.
        /// </summary>
        void ResolveWaterPlane()
        {
            if (WaterPlaneSource == null) WaterPlaneSource = FindFirstObjectByType<WaterSurface>();
            hasWaterPlane = WaterPlaneSource != null;
            if (hasWaterPlane)
            {
                waterPlaneY = WaterPlaneSource.transform.position.y;
                if (DiscardHitsAboveWater)
                    Debug.Log($"[SonarMapAccumulator] above-water filter ON: discarding every return above " +
                              $"Y = {waterPlaneY - AboveWaterMarginM:F2} m (still-water plane " +
                              $"{waterPlaneY:F2} m off '{HierarchyPath(WaterPlaneSource.transform)}' minus a " +
                              $"{AboveWaterMarginM:F2} m margin). A raycast sonar images the quay standing in " +
                              "the air; a real one cannot.");
            }
            else if (DiscardHitsAboveWater)
            {
                DiscardHitsAboveWater = false;
                Debug.LogWarning("[SonarMapAccumulator] no WaterSurface in the scene, so the above-water filter " +
                                 "is TURNED OFF rather than run against an assumed Y = 0. Returns from above the " +
                                 "water line — the quay tops — will therefore be accumulated. Assign " +
                                 "Water Plane Source if the surface lives in another loaded scene.");
            }
        }

        /// <summary>
        /// The cut-off height, refreshed each sample EXCEPT while a sanctioned drain has the plane
        /// deliberately off zero — the dock emptying must not retroactively redefine "above water"
        /// for points already in the map.
        /// </summary>
        void RefreshWaterPlane()
        {
            if (!hasWaterPlane || WaterPlaneSource == null) return;
            if (DockDrainDirector.DrainInProgress) return;
            waterPlaneY = WaterPlaneSource.transform.position.y;
        }

        /// <summary>Still-water plane Y, latched off the transform. Public so the readiness report can print what the filter is using.</summary>
        public float WaterPlaneY => waterPlaneY;
        /// <summary>Whether a still-water plane was found at all.</summary>
        public bool HasWaterPlane => hasWaterPlane;
        /// <summary>Returns discarded for being above the water line, since Play.</summary>
        public long AboveWaterDiscards => totalAboveWater;

        Material BuildMaterial()
        {
            Material m;
            if (MapMaterial != null)
            {
                m = new Material(MapMaterial);
            }
            else
            {
                var shader = Shader.Find("HDRP/Unlit");
                if (shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
                if (shader == null) shader = Shader.Find("Unlit/Texture");
                if (shader == null)
                {
                    Debug.LogError("[SonarMapAccumulator] no unlit shader found and no MapMaterial assigned — " +
                                   "run SMARC/Video/1 - Create video materials and assign SonarMap.mat.");
                    return null;
                }
                Debug.LogWarning($"[SonarMapAccumulator] no MapMaterial assigned; falling back to " +
                                 $"Shader.Find(\"{shader.name}\"), which is null in player builds. Assign SonarMap.mat.");
                m = new Material(shader);
            }

            // The colour ramp travels in UV0, so a stock HDRP/Unlit draws a per-point coloured
            // cloud with no custom shader: HDRP's Lit and Unlit do not read mesh vertex colours.
            if (m.HasProperty("_UnlitColor")) m.SetColor("_UnlitColor", Color.white);
            if (m.HasProperty("_UnlitColorMap")) m.SetTexture("_UnlitColorMap", rampTex);
            else if (m.HasProperty("_BaseColorMap")) m.SetTexture("_BaseColorMap", rampTex);
            else if (m.HasProperty("_MainTex")) m.SetTexture("_MainTex", rampTex);

            if (EmissiveIntensity > 0f)
            {
                if (m.HasProperty("_EmissiveColorMap")) m.SetTexture("_EmissiveColorMap", rampTex);
                if (m.HasProperty("_EmissiveColor")) m.SetColor("_EmissiveColor", Color.white * EmissiveIntensity);
                if (m.HasProperty("_EmissiveIntensity")) m.SetFloat("_EmissiveIntensity", EmissiveIntensity);
                if (m.HasProperty("_UseEmissiveIntensity")) m.SetFloat("_UseEmissiveIntensity", 1f);
                m.EnableKeyword("_EMISSIVE_COLOR_MAP");
                m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
            }
            HDMaterial.ValidateMaterial(m);
            return m;
        }

        Texture2D BuildRampTexture(int width)
        {
            var t = new Texture2D(width, 1, TextureFormat.RGBA32, false, false)
            { wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear, name = "SonarMapRamp" };
            var px = new Color[width];
            for (int i = 0; i < width; i++) px[i] = Ramp.Evaluate(i / (float)(width - 1));
            t.SetPixels(px);
            t.Apply(false, false);
            return t;
        }

        // ---------------------------------------------------------------- accumulation

        void Update()
        {
            if (!Visible && !AccumulateWhileHidden) { SetCloudActive(false); return; }
            SetCloudActive(Visible);

            if (Time.time < nextSample) return;
            nextSample = Time.time + 1f / Mathf.Max(0.1f, SampleHz);

            Sample();
            UploadDirtyChunk();
            MaybeReportZero();
        }

        /// <summary>
        /// While the map is empty, say why — every DiagnosticIntervalSec, naming the bound sonars by
        /// full hierarchy path and splitting the discarded returns by reason. The failure this
        /// exists for is the one that happened: ten minutes of 5 Hz sonar and a label reading
        /// "0 pts" with nothing anywhere saying that no sonar was ever bound.
        /// </summary>
        void MaybeReportZero()
        {
            if (DiagnosticIntervalSec <= 0f) return;
            if (PointCount > 0) return;
            if (Time.time < nextDiagnostic) return;
            nextDiagnostic = Time.time + DiagnosticIntervalSec;

            if (Sonars.Count == 0)
            {
                Debug.LogError($"[SonarMapAccumulator] 0 points after {Time.time:F0} s and NO SONAR IS BOUND. " +
                               $"RobotName '{RobotName}', robot root " +
                               $"{(robotRoot != null ? "'" + HierarchyPath(robotRoot) + "'" : "NOT FOUND")}, " +
                               $"IncludeMBES={IncludeMBES} IncludeSSS={IncludeSSS} IncludeFLS={IncludeFLS}. " +
                               "Nothing is being read; this is not a measurement of an empty dock.");
                return;
            }

            var lines = new List<string>();
            foreach (var s in Sonars)
            {
                if (s == null) { lines.Add("(a bound sonar was destroyed)"); continue; }
                int hitsNow = 0, rays = 0;
                var buf = s.SonarHits;
                if (buf != null)
                {
                    rays = buf.Length;
                    for (int i = 0; i < buf.Length; i++)
                        if (buf[i] != null && buf[i].Hit.collider != null) hitsNow++;
                }
                lines.Add($"'{HierarchyPath(s.transform)}' Type={s.Type} {s.frequency:F0} Hz " +
                          $"enabled={s.isActiveAndEnabled} buffer={(buf == null ? "NULL (Awake never ran)" : rays + " rays")} " +
                          $"firing={(hitsNow > 0 ? hitsNow + " rays currently hit something" : "NO RAY HITS ANYTHING RIGHT NOW")} " +
                          $"range {s.MaxRange:F0} m at {s.transform.position}");
            }

            Debug.LogWarning(
                $"[SonarMapAccumulator] 0 points after {Time.time:F0} s while {Sonars.Count} sonar(s) are bound.\n" +
                "  bound: " + string.Join("\n  bound: ", lines) + "\n" +
                $"  last sample read {lastRaysRead} ray(s) (stride {Mathf.Max(1, DecimationStride)}): " +
                $"{lastNoCollider} hit nothing, {lastBelowIntensity} below MinIntensity {MinIntensity:F3}, " +
                $"{lastOwnHull} on the vehicle's own hull, " +
                $"{lastAboveWater} ABOVE THE WATER LINE (filter {(DiscardHitsAboveWater ? $"on, cut-off Y = {waterPlaneY - AboveWaterMarginM:F2}" : "off")}), " +
                $"{lastVoxelDup} already in an occupied voxel, {lastAccepted} accepted.\n" +
                $"  totals since Play: {totalRaysRead} rays read, {totalHits} with a collider, " +
                $"{totalAboveWater} discarded above water, {totalAccepted} accepted.\n" +
                "  If 'hit nothing' is everything, the sonar is pointing at open water or out of range " +
                $"(this one reaches {(Sonars.Count > 0 && Sonars[0] != null ? Sonars[0].MaxRange.ToString("F0") : "?")} m). " +
                "If 'below MinIntensity' is everything, set MinIntensity to 0. If 'own hull' is everything, " +
                "the returns are off the vehicle and IgnoreHitsOnRobot is doing its job.");
        }

        void SetCloudActive(bool on)
        {
            if (cloudRoot != null && cloudRoot.gameObject.activeSelf != on)
                cloudRoot.gameObject.SetActive(on);
        }

        void Sample()
        {
            if (PointCount >= MaxPoints)
            {
                if (!cappedWarned)
                {
                    Debug.LogWarning($"[SonarMapAccumulator] point cap {MaxPoints} reached — the map has STOPPED " +
                                     "growing. Raise MaxPoints or raise VoxelSize; it will not silently thin itself.");
                    cappedWarned = true;
                }
                return;
            }

            int stride = Mathf.Max(1, DecimationStride);
            RefreshWaterPlane();
            // The cut-off, computed once per sample and not per ray.
            bool cutAbove = DiscardHitsAboveWater && hasWaterPlane;
            float cutoffY = waterPlaneY - AboveWaterMarginM;

            // Per-reason counters, reset every sample: the report is about the LAST ping, because
            // "it discarded four million returns" is not a diagnosis and "the last ping read 850
            // rays and 850 of them hit nothing" is.
            lastRaysRead = lastNoCollider = lastBelowIntensity = lastOwnHull = lastVoxelDup = lastAccepted = 0;
            lastAboveWater = 0;

            foreach (var sonar in Sonars)
            {
                if (sonar == null || !sonar.isActiveAndEnabled) continue;
                var hits = sonar.SonarHits;
                if (hits == null) continue;

                for (int i = 0; i < hits.Length; i += stride)
                {
                    lastRaysRead++; totalRaysRead++;
                    var sh = hits[i];
                    if (sh == null) { lastNoCollider++; continue; }
                    var hit = sh.Hit;
                    // A ray that hit nothing leaves a default RaycastHit: no collider.
                    if (hit.collider == null) { lastNoCollider++; continue; }
                    totalHits++;
                    if (sh.ReturnIntensity < MinIntensity) { lastBelowIntensity++; continue; }
                    if (IgnoreHitsOnRobot && robotRoot != null && hit.collider.transform.IsChildOf(robotRoot))
                    { lastOwnHull++; continue; }

                    // ABOVE THE WATER LINE IS NOT A SONAR RETURN. Take 006 draped the quay tops.
                    if (cutAbove && hit.point.y > cutoffY)
                    { lastAboveWater++; totalAboveWater++; continue; }

                    if (!Claim(hit.point)) { lastVoxelDup++; continue; }

                    AddPoint(hit.point, hit.normal, RampCoord(hit.point, sh));
                    lastAccepted++; totalAccepted++;
                    if (PointCount >= MaxPoints) return;
                }
            }
        }

        /// <summary>Voxel filter. Returns false if this cell already holds a point.</summary>
        bool Claim(Vector3 p)
        {
            if (VoxelSize <= 0f) return true;
            long x = (long)Mathf.Floor(p.x / VoxelSize);
            long y = (long)Mathf.Floor(p.y / VoxelSize);
            long z = (long)Mathf.Floor(p.z / VoxelSize);
            // 21 bits per axis: +/- 1 million cells, i.e. +/- 200 km at 0.2 m. Enough.
            long key = ((x & 0x1FFFFF) << 42) | ((y & 0x1FFFFF) << 21) | (z & 0x1FFFFF);
            return occupied.Add(key);
        }

        float RampCoord(Vector3 p, SonarHit sh)
        {
            switch (ColorBy)
            {
                case ColorSource.Intensity:
                    return Mathf.Clamp01(sh.ReturnIntensity);
                case ColorSource.MaterialLabel:
                    // 0 unlabelled, 1 rock/mud, 2 algae, 3 buoy, 4 rope (SonarHit.materialLabels)
                    return Mathf.Clamp01(sh.MaterialLabel / 4f);
                default:
                    // DEPTH BELOW THE WATER PLANE, over a FIXED range. 0 = shallow = the light end
                    // of the blue ramp; 1 = DepthRampDeepM or deeper = dark navy. Fixed rather than
                    // auto-fitted because this number is baked into the point's vertex UV the moment
                    // it is added and finished chunks are never revisited — a ramp that rescales
                    // itself as the mission deepens silently changes what already-drawn points mean.
                    if (DepthRampRelativeToWaterPlane && hasWaterPlane)
                    {
                        float relSpan = DepthRampDeepM - DepthRampShallowM;
                        if (Mathf.Abs(relSpan) < 1e-4f) return 0f;
                        return Mathf.Clamp01(((waterPlaneY - p.y) - DepthRampShallowM) / relSpan);
                    }
                    float span = DepthRampTop - DepthRampBottom;
                    if (Mathf.Abs(span) < 1e-4f) return 0f;
                    return Mathf.Clamp01((DepthRampTop - p.y) / span);
            }
        }

        void AddPoint(Vector3 p, Vector3 normal, float t)
        {
            var chunk = ActiveChunk();

            // A flake: a small triangle lying in the surveyed surface, so the cloud reads as a
            // surface from a distance and as discrete measurements up close. Three vertices per
            // point, and no billboarding work per frame.
            if (normal.sqrMagnitude < 1e-6f) normal = Vector3.up;
            normal.Normalize();
            Vector3 a = Vector3.Cross(normal, Mathf.Abs(normal.y) > 0.9f ? Vector3.right : Vector3.up).normalized;
            Vector3 b = Vector3.Cross(normal, a);

            float r = PointSize * 0.5f;
            int v0 = chunk.verts.Count;
            chunk.verts.Add(p + a * r);
            chunk.verts.Add(p + (-a * 0.5f + b * 0.866f) * r);
            chunk.verts.Add(p + (-a * 0.5f - b * 0.866f) * r);

            var uv = new Vector2(Mathf.Clamp01(t), 0.5f);
            chunk.uvs.Add(uv); chunk.uvs.Add(uv); chunk.uvs.Add(uv);

            chunk.tris.Add(v0); chunk.tris.Add(v0 + 1); chunk.tris.Add(v0 + 2);

            chunk.points++;
            chunk.dirty = true;
            PointCount++;
        }

        Chunk ActiveChunk()
        {
            if (chunks.Count > 0 && chunks[chunks.Count - 1].points < Mathf.Max(1000, PointsPerChunk))
                return chunks[chunks.Count - 1];

            var go = new GameObject($"SonarMapChunk_{chunks.Count:D3}");
            go.transform.SetParent(cloudRoot, false);
            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            // NO COLLIDER, EVER. The sonar raycasts the physics scene; a map with a collider is a
            // map of itself.
            mr.sharedMaterial = runtimeMat;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mr.receiveShadows = false;

            var mesh = new Mesh { name = go.name, indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            mf.sharedMesh = mesh;

            var c = new Chunk { go = go, mesh = mesh };
            chunks.Add(c);
            return c;
        }

        /// <summary>Re-upload only the chunk currently filling. Finished chunks are never touched again.</summary>
        void UploadDirtyChunk()
        {
            if (chunks.Count == 0) return;
            var c = chunks[chunks.Count - 1];
            if (!c.dirty) return;
            c.mesh.Clear();
            c.mesh.SetVertices(c.verts);
            c.mesh.SetUVs(0, c.uvs);
            c.mesh.SetTriangles(c.tris, 0, true);
            c.mesh.RecalculateBounds();
            c.dirty = false;
        }

        // ---------------------------------------------------------------- controls

        /// <summary>Drop the whole map. Wired to the plan-restart signal and available to the director.</summary>
        public void ClearMap()
        {
            foreach (var c in chunks)
            {
                if (c.mesh != null) Destroy(c.mesh);
                if (c.go != null) Destroy(c.go);
            }
            chunks.Clear();
            occupied.Clear();
            PointCount = 0;
            cappedWarned = false;
            // So the zero-report comes back promptly after a clear rather than a full interval later.
            nextDiagnostic = Time.time + Mathf.Max(2f, DiagnosticIntervalSec);
            Debug.Log($"[SonarMapAccumulator] map cleared ({bindingSummary} still bound).");
        }

        public void SetVisible(bool on) => Visible = on;

        // ---------------------------------------------------------------- label

        void OnGUI()
        {
            if (!ShowLabel || !Visible) return;

            if (labelStyle == null)
            {
                labelStyle = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, richText = false };
                subLabelStyle = new GUIStyle(GUI.skin.label) { richText = false };
            }
            int fs = Mathf.Max(11, Mathf.RoundToInt(Screen.height * LabelScale));
            labelStyle.fontSize = fs;
            subLabelStyle.fontSize = Mathf.Max(9, Mathf.RoundToInt(fs * 0.72f));
            labelStyle.normal.textColor = new Color(1f, 1f, 1f, 0.95f);
            subLabelStyle.normal.textColor = new Color(1f, 1f, 1f, 0.70f);

            // THE LABEL MUST NOT BE SERENE WHILE IT IS BROKEN. "0 pts" read exactly the same on
            // 2026-08-21 whether the map was empty because the dock was empty or because no sonar
            // was ever bound. Those are not the same statement and the frame now says which.
            string main = MapLabel;
            if (LabelShowsPointCount)
            {
                if (Sonars.Count == 0) main += "  ·  NO SONAR BOUND — not a measurement";
                // A new filter is a new way for the map to be empty, so it gets its own sentence:
                // "all discarded" must never be able to hide which filter did the discarding.
                else if (PointCount == 0 && totalHits > 0 && totalAboveWater >= totalHits)
                    main += $"  ·  0 pts — {totalHits} return(s), ALL ABOVE THE WATER LINE " +
                            $"(cut-off Y = {waterPlaneY - AboveWaterMarginM:F2})";
                else if (PointCount == 0) main += $"  ·  0 pts — {Sonars.Count} sonar(s) bound, {totalHits} return(s) so far, all discarded";
                else main += $"  ·  {PointCount:N0} pts";
            }

            float pad = fs * 1.2f;
            float w = Mathf.Max(labelStyle.CalcSize(new GUIContent(main)).x,
                                subLabelStyle.CalcSize(new GUIContent(MapSubLabel)).x) + pad;
            float h = fs * 2.6f + pad * 0.5f;

            float x = (LabelCorner == 1 || LabelCorner == 3) ? Screen.width - w - pad : pad;
            float y = (LabelCorner == 2 || LabelCorner == 3) ? Screen.height - h - pad : pad;

            var box = new Rect(x, y, w, h);
            var prev = GUI.color;
            GUI.color = new Color(0f, 0f, 0f, 0.45f);
            GUI.DrawTexture(box, Texture2D.whiteTexture);
            GUI.color = prev;

            GUI.Label(new Rect(x + pad * 0.4f, y + pad * 0.2f, w, fs * 1.4f), main, labelStyle);
            GUI.Label(new Rect(x + pad * 0.4f, y + pad * 0.2f + fs * 1.3f, w, fs * 1.2f), MapSubLabel, subLabelStyle);
        }

        void OnDestroy()
        {
            if (runtimeMat != null) Destroy(runtimeMat);
            if (rampTex != null) Destroy(rampTex);
        }
    }
}
