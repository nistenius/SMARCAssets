using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

using Force;   // ForcePoint — the components that make a moving water plane dangerous

namespace SmarcGUI.Water
{
    /// <summary>
    /// Animates the dry dock being PUMPED OUT at the end of a mission, so the accumulated sonar map
    /// can be seen standing in an empty dock (Ivan, 2026-08-21).
    ///
    /// ===================================================================================
    /// THIS IS THE FIRST SANCTIONED EXCEPTION TO SETTLED §3s — READ THIS BEFORE TOUCHING IT
    /// ===================================================================================
    /// §3s: **the Ocean/Water transform stays at (0,0,0)**. `HDRPWaterQueryModel.GetWaterLevelAt`
    /// discards the convergence bool from `ProjectPointOnWaterSurface` AND seeds every search from
    /// the previous caller's result — one shared instance, one shared seed. With the water plane
    /// away from world zero, two `ForcePoint`s 1.1 m apart measured −46.77 m and −47.13 m of the
    /// same plane: 0.36 m of disagreement, differential buoyancy, torque, and SAM at 67 m/s.
    ///
    /// The rule is not "never move the water"; the rule is **never let a ForcePoint query a water
    /// plane that is not at world zero**. This component moves the water only while that is
    /// guaranteed, and it guarantees it by construction:
    ///
    ///   1. **PLAY MODE ONLY.** In Edit mode it refuses and moves nothing, so the scene can never
    ///      be SAVED with the water at a non-zero Y. (`BalticWaterPreset.WarnIfWaterMoved` and
    ///      `SMARC/Video/5 - Report scene readiness` both check that at author time; this is what
    ///      keeps their check meaningful.)
    ///   2. **THE MISSION MUST BE OVER.** `CanDrain()` requires the same facts the recorder-stop
    ///      policy uses (§3s6): the vehicle is at the surface AND it has been idle for
    ///      `RequiredIdleSeconds`. A drain during a mission is refused, by name, in the log.
    ///   3. **EVERY ForcePoint IN THE SCENE IS DISABLED BEFORE THE WATER MOVES**, and every body
    ///      they drive is frozen (`Rigidbody.isKinematic` / root `ArticulationBody.immovable`).
    ///      Scene-wide, not vehicle-only: §3s is a property of the SHARED query model, so a buoy
    ///      or a second vehicle elsewhere is exposed to the identical mechanism. With no ForcePoint
    ///      enabled, `GetWaterLevelAt` is not called at all and the defect has no path to run.
    ///      **Freezing happens FIRST and unfreezing happens LAST**, always in that order.
    ///   4. **IT ALWAYS RESTORES.** `Restore()` puts the water back at its recorded Y and re-enables
    ///      exactly what it disabled — from `Abort()`, from `OnDisable`, from `OnDestroy`, and from
    ///      `OnApplicationQuit`. Stopping Play mid-drain leaves nothing behind, because none of it
    ///      was ever serialised.
    ///   5. **IT NAMES ITSELF ON SCREEN.** `StatusLine` is what the CinematicDirector's overlay
    ///      shows, so a frame in which the water is not where §3s says it should be always carries
    ///      the reason it is not.
    ///
    /// What it does NOT do: disable the Water object (that removes `WaterQueryModel` and every
    /// vehicle loses buoyancy — §3o), touch `scriptInteractions`, call `GetWaterLevelAt` itself, or
    /// add a collider to anything.
    ///
    /// The underwater volume: HDRP renders the underwater view inside `WaterSurface.volumeBounds`.
    /// If that collider is a CHILD of the water transform it follows for free; if it is not, it is
    /// translated by the same delta and put back, so the "underwater" region drops with the water
    /// instead of leaving a slab of green fog hanging over a dry dock.
    /// </summary>
    [AddComponentMenu("Smarc/Water/Dock Drain Director")]
    public class DockDrainDirector : MonoBehaviour
    {
        [Header("What to drain")]
        [Tooltip("The WaterSurface to lower. Left empty, the first one in the scene is used. Its transform Y is what moves — nothing else about it is touched.")]
        public WaterSurface Surface;

        [Tooltip("Metres to lower the water by. Beckholmen's dock floor is about 7 m down, so 7-8 empties it.")]
        public float DrainDepthM = 7.5f;

        [Tooltip("Seconds the pump-out takes. Long enough to read as pumping, short enough to keep in a video.")]
        public float DrainSeconds = 25f;

        [Tooltip("Shape of the pump-out. A real dock empties fast at first and slows as the head drops; the default ease does roughly that.")]
        public AnimationCurve DrainEase = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

        [Header("The gate — when a drain is allowed at all")]
        [Tooltip("Vehicle root GameObject name. The surfaced/idle test is read off this object's transform, exactly like the CinematicDirector's own conditions.")]
        public string VehicleName = "sam_auv_v1";

        [Tooltip("Child treated as THE vehicle. Empty falls back to the root. Searched UNDER the vehicle only — 'base_link' is not unique at Beckholmen.")]
        public string VehicleAimChildName = "base_link";

        [Tooltip("Metres below the still-water plane that still counts as surfaced.")]
        public float SurfacedDepthM = 0.35f;

        [Tooltip("m/s below which the vehicle counts as stopped.")]
        public float IdleSpeedThreshold = 0.12f;

        [Tooltip("Seconds the vehicle must have been surfaced AND stopped before a drain is allowed. The same shape as the recorder-stop policy (SETTLED §3s6): the run is over when the vehicle says so, not when a timer says so.")]
        public float RequiredIdleSeconds = 5f;

        [Tooltip("Let the drain run even if the vehicle is still moving or under. OFF. Turn it on only to rehearse the animation with no mission, and expect the log to say loudly that the gate was bypassed.")]
        public bool BypassGateForRehearsal = false;

        [Header("Readouts — NOT serialised, they describe a live state")]
        [System.NonSerialized] public bool Draining;
        [System.NonSerialized] public float Progress01;
        [System.NonSerialized] public float CurrentWaterY;
        [System.NonSerialized] public string StatusLine = "dock full — no drain requested";

        // ---------------------------------------------------------------- restore state

        bool armed;                       // true between Begin() and Restore()
        float originalWaterY;
        float startTime;
        Transform waterTf;
        Transform volumeBoundsTf;
        bool volumeBoundsIsChild;
        Vector3 volumeBoundsOriginalPos;

        readonly List<ForcePoint> disabledPoints = new List<ForcePoint>();
        readonly List<Rigidbody> frozenRigidbodies = new List<Rigidbody>();
        readonly List<bool> rigidbodyWasKinematic = new List<bool>();
        readonly List<ArticulationBody> frozenArticulations = new List<ArticulationBody>();
        readonly List<bool> articulationWasImmovable = new List<bool>();

        Transform vehicleAim;
        Vector3 lastVehiclePos;
        bool hasLastVehiclePos;
        float smoothedSpeed;
        float surfacedIdleSince = -1f;

        /// <summary>
        /// Set while a sanctioned drain is running, so anything that checks the §3s invariant can
        /// tell "somebody moved the water and did not say why" from "the drain is running and has
        /// frozen everything that could be hurt by it".
        /// </summary>
        public static bool DrainInProgress { get; private set; }

        // ---------------------------------------------------------------- lifecycle

        void Awake()
        {
            ResolveSurface();
        }

        void ResolveSurface()
        {
            if (Surface == null) Surface = FindFirstObjectByType<WaterSurface>();
            waterTf = Surface != null ? Surface.transform : null;
        }

        Transform ResolveVehicleAim()
        {
            if (vehicleAim != null) return vehicleAim;
            var go = GameObject.Find(VehicleName);
            if (go == null) return null;
            if (string.IsNullOrEmpty(VehicleAimChildName)) { vehicleAim = go.transform; return vehicleAim; }
            var child = FindDeepChild(go.transform, VehicleAimChildName);
            vehicleAim = child != null ? child : go.transform;
            return vehicleAim;
        }

        static Transform FindDeepChild(Transform root, string name)
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

        void Update()
        {
            TrackVehicle();
            if (armed) StepDrain();
        }

        void TrackVehicle()
        {
            var t = ResolveVehicleAim();
            if (t == null) { surfacedIdleSince = -1f; return; }

            if (hasLastVehiclePos && Time.deltaTime > 1e-5f)
            {
                float raw = (t.position - lastVehiclePos).magnitude / Time.deltaTime;
                float k = 1f - Mathf.Exp(-Time.deltaTime / 0.6f);
                smoothedSpeed = Mathf.Lerp(smoothedSpeed, raw, k);
            }
            lastVehiclePos = t.position;
            hasLastVehiclePos = true;

            // While the drain is running the water is deliberately not at the still-water plane, so
            // the gate is judged against the plane the drain STARTED from, never against the moving
            // one — otherwise lowering the water would make the vehicle look 7 m "surfaced".
            float planeY = armed ? originalWaterY : StillWaterY;
            float depth = planeY - t.position.y;

            bool ok = depth <= SurfacedDepthM && smoothedSpeed < IdleSpeedThreshold;
            if (!ok) surfacedIdleSince = -1f;
            else if (surfacedIdleSince < 0f) surfacedIdleSince = Time.time;
        }

        /// <summary>The still-water plane, read off the TRANSFORM. Never `GetWaterLevelAt` (§3s).</summary>
        public float StillWaterY => waterTf != null ? waterTf.position.y : 0f;

        /// <summary>Seconds the vehicle has been surfaced AND stopped, or -1.</summary>
        public float SurfacedIdleFor => surfacedIdleSince < 0f ? -1f : Time.time - surfacedIdleSince;

        /// <summary>
        /// Whether a drain may start, and why not when it may not. The reason is a sentence because
        /// it goes on screen and into the log — "false" is not an answer anybody can act on.
        /// </summary>
        public bool CanDrain(out string reason)
        {
            if (!Application.isPlaying)
            {
                reason = "REFUSED: not in Play mode. The drain moves the Water transform, and a scene " +
                         "must never be SAVED with the water off zero (SETTLED §3s).";
                return false;
            }
            ResolveSurface();
            if (Surface == null || waterTf == null)
            {
                reason = "REFUSED: no WaterSurface in the scene — nothing to drain, and no still-water plane to restore to.";
                return false;
            }
            if (!armed && Mathf.Abs(waterTf.position.y) > 1e-3f)
            {
                reason = $"REFUSED: the Water transform is already at Y = {waterTf.position.y:F3}, not 0. " +
                         "Something else has moved it — fix that first (SETTLED §3s); the drain will not " +
                         "stack on top of an unexplained offset.";
                return false;
            }
            if (BypassGateForRehearsal)
            {
                reason = "gate BYPASSED for rehearsal — the vehicle is NOT known to be surfaced and idle. " +
                         "This is a rehearsal setting; untick it before a take.";
                return true;
            }
            var t = ResolveVehicleAim();
            if (t == null)
            {
                reason = $"REFUSED: no ACTIVE GameObject named '{VehicleName}' — the surfaced-and-idle test " +
                         "cannot be evaluated, and it is not assumed true.";
                return false;
            }
            float depth = StillWaterY - t.position.y;
            if (surfacedIdleSince < 0f)
            {
                reason = $"REFUSED: the vehicle is not surfaced-and-idle ({depth:F2} m down, {smoothedSpeed:F2} m/s; " +
                         $"needs <= {SurfacedDepthM:F2} m and < {IdleSpeedThreshold:F2} m/s).";
                return false;
            }
            float held = Time.time - surfacedIdleSince;
            if (held < RequiredIdleSeconds)
            {
                reason = $"waiting: surfaced and idle for {held:F1} of {RequiredIdleSeconds:F0} s " +
                         $"({depth:F2} m down, {smoothedSpeed:F2} m/s).";
                return false;
            }
            reason = $"allowed: surfaced ({depth:F2} m) and idle ({smoothedSpeed:F2} m/s) for {held:F0} s.";
            return true;
        }

        // ---------------------------------------------------------------- the drain itself

        /// <summary>
        /// Start pumping. Returns false and says why if the gate refuses; nothing is touched in that
        /// case — in particular nothing is frozen, so a refused drain has no side effects at all.
        /// </summary>
        public bool Begin()
        {
            if (armed) return true;
            if (!CanDrain(out string reason))
            {
                StatusLine = "drain " + reason;
                Debug.LogWarning($"[DockDrainDirector] drain {reason}");
                return false;
            }

            originalWaterY = waterTf.position.y;

            // ---- FREEZE FIRST. Nothing below this line may run before this returns. ----
            FreezeEverythingThatQueriesTheWater();

            // ---- the underwater volume, if it is not already carried by the water transform ----
            volumeBoundsTf = Surface.volumeBounds != null ? Surface.volumeBounds.transform : null;
            volumeBoundsIsChild = volumeBoundsTf != null && volumeBoundsTf.IsChildOf(waterTf);
            if (volumeBoundsTf != null && !volumeBoundsIsChild)
                volumeBoundsOriginalPos = volumeBoundsTf.position;

            armed = true;
            Draining = true;
            DrainInProgress = true;
            startTime = Time.time;
            Progress01 = 0f;
            CurrentWaterY = originalWaterY;

            Debug.LogWarning(
                "[DockDrainDirector] DRAINING — this is the SANCTIONED SETTLED §3s exception, and here is " +
                $"the guarantee that makes it safe: {disabledPoints.Count} ForcePoint(s) disabled, " +
                $"{frozenRigidbodies.Count} Rigidbody(ies) and {frozenArticulations.Count} " +
                "ArticulationBody(ies) frozen BEFORE the water moved, scene-wide. Nothing will call " +
                $"GetWaterLevelAt while the plane is off zero. Water {originalWaterY:F3} -> " +
                $"{originalWaterY - DrainDepthM:F3} over {DrainSeconds:F0} s; restored on exit. " +
                $"Gate: {(BypassGateForRehearsal ? "BYPASSED (rehearsal)" : "vehicle surfaced and idle")}.");
            return true;
        }

        void StepDrain()
        {
            float u = DrainSeconds > 1e-3f ? Mathf.Clamp01((Time.time - startTime) / DrainSeconds) : 1f;
            float e = DrainEase != null && DrainEase.length > 0 ? DrainEase.Evaluate(u) : u;
            Progress01 = u;

            float y = originalWaterY - DrainDepthM * e;
            SetWaterY(y);

            StatusLine = u >= 1f
                ? $"dock DRAINED — water {DrainDepthM:F1} m down, physics frozen (sanctioned §3s exception)"
                : $"PUMPING OUT — {DrainDepthM * e:F1} of {DrainDepthM:F1} m, {u * 100f:F0}%, physics frozen";
        }

        void SetWaterY(float y)
        {
            if (waterTf == null) return;
            float delta = y - waterTf.position.y;
            var p = waterTf.position; p.y = y; waterTf.position = p;
            CurrentWaterY = y;

            if (volumeBoundsTf != null && !volumeBoundsIsChild)
            {
                var q = volumeBoundsTf.position; q.y += delta; volumeBoundsTf.position = q;
            }
        }

        /// <summary>Put the water back and unfreeze. Safe to call when nothing was ever armed.</summary>
        public void Restore()
        {
            if (!armed) return;

            SetWaterY(originalWaterY);
            if (volumeBoundsTf != null && !volumeBoundsIsChild)
                volumeBoundsTf.position = volumeBoundsOriginalPos;

            // ---- UNFREEZE LAST, and only after the water is back at its recorded plane. ----
            UnfreezeEverything();

            armed = false;
            Draining = false;
            DrainInProgress = false;
            Progress01 = 0f;
            StatusLine = "dock refilled — water restored to Y = " + originalWaterY.ToString("F3") + ", physics live";
            Debug.Log($"[DockDrainDirector] restored: water back to Y = {originalWaterY:F3}, " +
                      "every ForcePoint and body re-enabled exactly as found.");
        }

        /// <summary>Same as Restore, named for the case where the drain is cut short.</summary>
        public void Abort(string why)
        {
            if (!armed) return;
            Debug.LogWarning($"[DockDrainDirector] drain aborted: {why}");
            Restore();
        }

        void OnDisable() { Restore(); }
        void OnDestroy() { Restore(); }
        void OnApplicationQuit() { Restore(); }

        // ---------------------------------------------------------------- freeze / unfreeze

        /// <summary>
        /// Disable every ForcePoint in the scene and freeze every body they drive.
        ///
        /// SCENE-WIDE ON PURPOSE. §3s is a property of the ONE shared `HDRPWaterQueryModel`: a
        /// ForcePoint on a buoy a hundred metres away poisons the seed for the next caller just as
        /// well as one on the hull. Freezing only the vehicle would leave the mechanism armed and
        /// the evidence somewhere nobody is looking.
        /// </summary>
        void FreezeEverythingThatQueriesTheWater()
        {
            disabledPoints.Clear();
            frozenRigidbodies.Clear(); rigidbodyWasKinematic.Clear();
            frozenArticulations.Clear(); articulationWasImmovable.Clear();

            var bodies = new HashSet<Rigidbody>();
            var arts = new HashSet<ArticulationBody>();

            foreach (var fp in FindObjectsByType<ForcePoint>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
            {
                if (fp == null || !fp.enabled) continue;
                if (fp.ConnectedRigidbody != null) bodies.Add(fp.ConnectedRigidbody);
                if (fp.ConnectedArticulationBody != null) arts.Add(fp.ConnectedArticulationBody);
                fp.enabled = false;
                disabledPoints.Add(fp);
            }

            foreach (var rb in bodies)
            {
                if (rb == null) continue;
                frozenRigidbodies.Add(rb);
                rigidbodyWasKinematic.Add(rb.isKinematic);
                if (!rb.isKinematic)
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                    rb.isKinematic = true;
                }
            }

            // Only the ROOT of an articulation chain can be made immovable; the links follow it.
            var roots = new HashSet<ArticulationBody>();
            foreach (var ab in arts)
            {
                if (ab == null) continue;
                var root = ab;
                while (root != null && !root.isRoot)
                {
                    var parent = root.transform.parent != null
                        ? root.transform.parent.GetComponentInParent<ArticulationBody>() : null;
                    if (parent == null) break;
                    root = parent;
                }
                if (root != null) roots.Add(root);
            }
            foreach (var root in roots)
            {
                if (root == null) continue;
                frozenArticulations.Add(root);
                articulationWasImmovable.Add(root.immovable);
                if (!root.immovable)
                {
                    // Unity 6 names: ArticulationBody.linearVelocity, not .velocity (Teleporter_Sub
                    // already uses this spelling in this repo).
                    root.linearVelocity = Vector3.zero;
                    root.angularVelocity = Vector3.zero;
                    root.immovable = true;
                }
            }
        }

        void UnfreezeEverything()
        {
            for (int i = 0; i < frozenArticulations.Count; i++)
            {
                var ab = frozenArticulations[i];
                if (ab != null) ab.immovable = articulationWasImmovable[i];
            }
            for (int i = 0; i < frozenRigidbodies.Count; i++)
            {
                var rb = frozenRigidbodies[i];
                if (rb != null) rb.isKinematic = rigidbodyWasKinematic[i];
            }
            foreach (var fp in disabledPoints) if (fp != null) fp.enabled = true;

            disabledPoints.Clear();
            frozenRigidbodies.Clear(); rigidbodyWasKinematic.Clear();
            frozenArticulations.Clear(); articulationWasImmovable.Clear();
        }
    }
}
