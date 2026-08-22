using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace VehicleComponents.Comms
{
    /// <summary>
    /// A transducer that hangs over the side on a cable, and therefore sits at a depth referenced
    /// to THE WATER PLANE — not to whatever the case happens to be bolted to.
    ///
    /// WHAT THIS REPLACES, AND WHY IT WAS WRONG (2026-08-21, Ivan).
    /// `StationBuilder` used to put `acoustic_link` at a FIXED OFFSET BELOW `base_link`, i.e. below
    /// the CASE. Ivan's description of the real deployment is: "the base station prefab is now
    /// placed at an approximate pos where it could be for real also. It's a bridge of the dock so
    /// the transducer is dropped away from the walls. The case sits some meter above the water line
    /// but we always drop the transducer at least 50cm down below the water level." With the case a
    /// metre up on a dock bridge, that offset put the modelled transducer roughly a metre IN THE AIR.
    ///
    /// The offset is wrong in PRINCIPLE and not merely by 1.4 m. A number measured from the case
    /// changes meaning every time the case moves — quay edge, dock bridge, Milou's fore deck — and
    /// nothing in the scene reports that it changed. The cable does not care how high the case is;
    /// the operator drops the fish until it is half a metre under, at every site, every time. So
    /// the model that matches the operation is: keep the case's X/Z (it hangs straight down) and
    /// take Y from `waterPlaneY - DipDepthM`. Re-place the station anywhere and the transducer is
    /// still at the configured depth, which is the property the real deployment guarantees and the
    /// case height does not.
    ///
    /// WHY THIS IS NOT COSMETIC. Ivan's own falsifier for the fleet position estimate (2026-08-18)
    /// is: compare the MEASURED acoustic range against the range IMPLIED by the two agents'
    /// reported positions, and treat a disagreement as evidence that somebody's position is wrong.
    /// A wrong constant in the transducer offset injects a permanent bias into exactly that
    /// comparison — in the one component whose job is to catch position errors. A 1.4 m standing
    /// offset is small against 800 m of range and fatal to a metre-level cross-check.
    ///
    /// IT MUST NEVER CALL GetWaterLevelAt (SETTLED §3s). `HDRPWaterQueryModel` is a single shared
    /// instance that seeds each search from the PREVIOUS caller's result. A station asking it
    /// where the water is — tens or hundreds of metres from the hull — hands the vehicle's
    /// ForcePoints a poisoned seed on their next query, and that is the measured mechanism behind
    /// SAM tumbling to 67 m/s. So this reads the still-water plane straight off the WaterSurface
    /// transform's Y, exactly as `UnderwaterParticles.WaterPlaneY` already does. A hanging
    /// transducer is not riding the swell anyway: the still-water plane IS the right model for it.
    ///
    /// IF THERE IS NO WaterSurface, IT REFUSES BY NAME and leaves the transducer where it is.
    /// Assuming Y = 0 would be inventing a measurement, which is the failure mode this project
    /// keeps paying for (the station's "MEASURED ±0.014 m" that was a hardcoded constant 49.4 m
    /// away, SETTLED §3s2).
    ///
    /// ---------------------------------------------------------------------------------------
    /// IT DEPLOYS ONLY AGAINST WATER IN ITS OWN SCENE — AND THAT IS A MEASURED FIX, NOT A TIDY-UP
    /// (2026-08-21, second defect, found because Ivan pressed the menu item six times).
    ///
    /// The first version searched ALL loaded scenes for a WaterSurface. `[ExecuteAlways]` means
    /// this component also runs inside the PREVIEW SCENE that `PrefabUtility.LoadPrefabContents`
    /// opens — so when `SMARC/Station/Update transducer deployment (surgical)` opened the prefab,
    /// this component woke up in there, found the OPEN BECKHOLMEN SCENE's water, and wrote
    /// `waterPlaneY - DipDepthM` into the prefab contents. The updater then read that as an
    /// authored defect, "corrected" it back to zero and saved — every press, forever. Two writers
    /// on one value, with a prefab rewritten on each press: SETTLED §5's shape exactly.
    /// The evidence it left behind is in the prefab file: `Deployed: 1` and `ActualDepthM: 0.5`
    /// were serialised into an ASSET, and only `Measure()` writes those, and only when it has found
    /// a water plane — which no prefab contains.
    ///
    /// The rule below is the principled one and the churn fix falls out of it for free: a station
    /// placed in scene A must never read scene B's water plane. Two scenes open at once (a site
    /// scene plus an additively-loaded rig) is the normal case here, and silently borrowing the
    /// other one's sea level would be an invented measurement wearing a plausible number. So:
    ///   * water must live in THE SAME SCENE as this transducer, and a `Surface` assigned across
    ///     scenes is refused BY NAME rather than used;
    ///   * inside a prefab asset or a prefab preview scene it refuses QUIETLY (there is no water in
    ///     a prefab, and this is a normal situation, not an operator error) and moves nothing.
    ///
    /// AND IT NEVER WRITES A FIELD IT DOES NOT OWN. The resolved surface is cached in a
    /// NON-SERIALISED field, so resolving one can never author a cross-scene reference into an
    /// asset. The three read-only outputs are `[System.NonSerialized]` for the same reason: a live
    /// readout must not be able to be BAKED into a prefab or a scene, where it would outlive what
    /// it describes (§3s's stale `AppliedBuoyancyForce`, which is how that trap presented). They
    /// are drawn by `DeployedTransducerEditor` instead of by the default drawer.
    /// </summary>
    [ExecuteAlways]
    [AddComponentMenu("Smarc/Comms/Deployed Transducer (over the side)")]
    public class DeployedTransducer : MonoBehaviour
    {
        [Header("Deployment")]
        [Tooltip("How far BELOW THE STILL-WATER PLANE the transducer hangs [m]. Ivan, 2026-08-21: \"we always drop the transducer at least 50cm down below the water level\". This is measured from the water, never from the case — that is the whole point of this component.")]
        public float DipDepthM = 0.5f;

        [Tooltip("Refuse to deploy shallower than this [m]. A configured 0 would put the transducer exactly ON the surface, which in practice means dry, ventilated and deaf — and it would do it silently. The floor is physical, not a policy: below this depth the model is not describing a deployed transducer at all.")]
        public float MinDipDepthM = 0.1f;

        [Tooltip("Operating practice [m]. Deploying shallower than this is allowed but WARNS, naming the practice, so an unusual depth is a visible decision instead of a typo nobody notices.")]
        public float PracticeDipDepthM = 0.5f;

        [Header("Water plane (never GetWaterLevelAt — SETTLED 3s)")]
        [Tooltip("WaterSurface whose transform Y is the still-water plane. It must be in THIS transducer's own scene — a station in one scene reading another scene's sea level is an invented measurement, and it is also what made the prefab updater fight this component. Left empty, the first WaterSurface IN THIS SCENE is used. If there is none, this component REFUSES and says so rather than assuming Y = 0. This component never writes this field; it caches what it resolved somewhere that cannot be serialised.")]
        public WaterSurface Surface;

        [Header("Gizmo")]
        [Tooltip("What the cable hangs from, for the Scene-view drop line. Left empty, the parent transform (the case / base_link) is used.")]
        public Transform CaseAnchor;

        [Tooltip("Keep the transducer snapped to the water plane while editing the scene, so moving the case never silently changes the deployed depth. Off means it is only positioned at Play.")]
        public bool AutoDeployInEditor = true;

        // ---- State ---------------------------------------------------------------------------
        // NOT SERIALISED, deliberately. These describe THIS INSTANT in THIS scene; serialising them
        // let `Deployed: 1` / `ActualDepthM: 0.5` be written into the station PREFAB ASSET on
        // 2026-08-21 — an asset claiming to be deployed 0.5 m under, with nothing to be under.
        // A readout that can be baked is a readout that can outlive what it describes.

        /// <summary>Is the transducer actually below the water plane right now?</summary>
        [System.NonSerialized] public bool Deployed;

        /// <summary>Metres below the still-water plane the transducer ACTUALLY is — measured from
        /// the transform, not copied from DipDepthM. The two disagreeing is the whole reason this
        /// readout exists. NaN when there is no water plane to measure against.</summary>
        [System.NonSerialized] public float ActualDepthM = float.NaN;

        /// <summary>Why the transducer is not deployed, in words an operator can act on. Empty
        /// while it is deployed.</summary>
        [System.NonSerialized] public string RefusalReason = "not evaluated";

        // Log each distinct refusal once. A component that runs in edit mode and shouts every
        // frame trains people to ignore the console, which is worse than saying nothing.
        [System.NonSerialized] string lastLogged;

        void OnEnable() { Deploy(); }
        void Start() { Deploy(); }

        void Update()
        {
            if (!Application.isPlaying && !AutoDeployInEditor) { Measure(); return; }
            Deploy();
        }

        // ---- Where this component is allowed to work -------------------------------------------

        /// <summary>
        /// Can this transducer deploy AT ALL where it currently lives? False inside a prefab asset
        /// or a prefab preview scene — there is no water in a prefab, and reading some other open
        /// scene's water plane from in there is how this component ended up fighting
        /// `StationBuilder` over the same value (see the class comment).
        ///
        /// `quiet` distinguishes "normal situation, nothing to fix" from "a misconfiguration an
        /// operator must act on": a prefab that is merely open is not an error and must not turn
        /// the console red, but it must still SAY what it is, in `RefusalReason`.
        /// </summary>
        public bool CanDeployHere(out string why, out bool quiet)
        {
            why = "";
            quiet = false;

#if UNITY_EDITOR
            if (UnityEditor.EditorUtility.IsPersistent(this))
            {
                quiet = true;
                why = "this transducer is part of a PREFAB ASSET, not a scene. A prefab has no water " +
                      "in it, and borrowing some other open scene's water plane would bake that " +
                      "scene's sea level into an asset every scene shares. Nothing moved; the " +
                      "transducer deploys when the station is placed in a scene.";
                return false;
            }

            if (UnityEditor.SceneManagement.EditorSceneManager.IsPreviewSceneObject(this))
            {
                quiet = true;
                why = "this transducer is in a PREFAB PREVIEW SCENE (Prefab Mode, or a prefab opened " +
                      "by a builder), which has no water of its own. Nothing moved. Deploying here " +
                      "against the open site scene's water is exactly the two-writers defect of " +
                      "2026-08-21: the prefab updater would then 'correct' the value back on every " +
                      "press, forever.";
                return false;
            }
#endif

            if (!gameObject.scene.IsValid())
            {
                quiet = true;
                why = "this transducer is not in a loaded scene, so it has no water of its own. " +
                      "Nothing moved.";
                return false;
            }

            return true;
        }

        [System.NonSerialized] WaterSurface resolved;
        [System.NonSerialized] float nextSurfaceSearch;
        [System.NonSerialized] string lastResolveWhy;

        /// <summary>The WaterSurface driving the still-water plane, resolved lazily and ONLY from
        /// this transducer's own scene. Throttled while unresolved: this is called from the gizmo
        /// path too, and a scene-wide search every frame in a scene that simply has no Ocean is a
        /// cost with no possible payoff.
        ///
        /// It caches into a non-serialised field and NEVER writes `Surface`. The old version
        /// assigned it, which in a prefab preview scene meant authoring a reference to another
        /// scene's object into a prefab asset — the reason the saved prefab carries
        /// `Surface: {fileID: 0}`, a reference Unity had to drop on the way out.</summary>
        public WaterSurface ResolveSurface() { TryResolveSurface(out var s, out _); return s; }

        /// <summary>Resolve, and NAME the refusal. Three outcomes worth telling apart: an assigned
        /// surface that lives in another scene (a real misconfiguration), no surface in this scene
        /// while others exist elsewhere (the additive-scene trap, worth saying out loud), and no
        /// surface anywhere.</summary>
        public bool TryResolveSurface(out WaterSurface surface, out string why)
        {
            why = "";

            if (Surface != null)
            {
                if (Surface.gameObject.scene == gameObject.scene)
                {
                    surface = Surface;
                    lastResolveWhy = null;
                    return true;
                }
                surface = null;
                why = $"the assigned WaterSurface '{Surface.name}' lives in scene " +
                      $"'{SceneName(Surface.gameObject.scene)}' but this transducer is in " +
                      $"'{SceneName(gameObject.scene)}'. REFUSED: a station in one scene must never " +
                      "take its sea level from another — that is an invented measurement with a " +
                      "plausible number on it. Assign a WaterSurface from this scene, or clear the " +
                      "field and let this scene's Ocean be found.";
                lastResolveWhy = why;
                return false;
            }

            if (resolved != null && resolved.gameObject.scene == gameObject.scene)
            {
                surface = resolved;
                lastResolveWhy = null;
                return true;
            }
            resolved = null;

            float now = Time.realtimeSinceStartup;
            if (now < nextSurfaceSearch)
            {
                surface = null;
                // The STORED reason, not a new "throttled" one. A second sentence here would
                // alternate with the real refusal every half second and `Fail`'s log-once guard —
                // which compares against the last message — would print both, forever. A refusal
                // that changes its wording while nothing changes is noise wearing the clothes of
                // news.
                why = lastResolveWhy ?? "no WaterSurface resolved in this scene yet.";
                return false;
            }
            nextSurfaceSearch = now + 0.5f;

            var all = FindObjectsByType<WaterSurface>(FindObjectsSortMode.None);
            int elsewhere = 0;
            foreach (var s in all)
            {
                if (s == null) continue;
                if (s.gameObject.scene == gameObject.scene)
                {
                    resolved = s;
                    surface = s;
                    lastResolveWhy = null;
                    return true;
                }
                elsewhere++;
            }

            surface = null;
            why = elsewhere > 0
                ? $"no WaterSurface in this transducer's own scene '{SceneName(gameObject.scene)}' " +
                  $"({elsewhere} found in OTHER loaded scenes, deliberately not used — a station in " +
                  "one scene must not take its sea level from another). REFUSED rather than " +
                  "assuming Y = 0. Assign 'Surface' from this scene, or move the station into the " +
                  "scene that has the Ocean."
                : $"no WaterSurface in this scene ('{SceneName(gameObject.scene)}'), so the " +
                  "still-water plane is unknown. REFUSED rather than assuming Y = 0 — an assumed " +
                  "water level is an invented measurement. Assign 'Surface', or open a scene that " +
                  "has an Ocean.";
            lastResolveWhy = why;
            return false;
        }

        static string SceneName(UnityEngine.SceneManagement.Scene s)
            => string.IsNullOrEmpty(s.name) ? "<no scene>" : s.name;

        /// <summary>
        /// Still-water plane Y. False when there is no usable WaterSurface — and then the caller
        /// must refuse, not substitute a zero. See the class comment.
        /// </summary>
        public bool TryGetWaterPlaneY(out float y) => TryGetWaterPlaneY(out y, out _);

        /// <summary>As above, and NAMES why not.</summary>
        public bool TryGetWaterPlaneY(out float y, out string why)
        {
            if (!TryResolveSurface(out var s, out why)) { y = float.NaN; return false; }
            y = s.transform.position.y;
            return true;
        }

        /// <summary>Where this transducer SHOULD be: the case's X/Z, the water plane's Y minus the
        /// dip depth. False (and NaN) when the water plane is unknown.</summary>
        public bool TryGetTargetPosition(out Vector3 target)
        {
            target = transform.position;
            if (!TryGetWaterPlaneY(out float planeY)) return false;
            target = new Vector3(transform.position.x, planeY - EffectiveDipDepthM(), transform.position.z);
            return true;
        }

        /// <summary>The configured dip depth, unmodified. Deliberately NOT clamped to
        /// `MinDipDepthM`: too-shallow values are REFUSED in <see cref="Deploy"/>, because a clamp
        /// makes a typo behave like a decision and leaves nothing for anyone to notice.</summary>
        public float EffectiveDipDepthM() => DipDepthM;

        /// <summary>
        /// Is a point below the still-water plane? This is the honest wetness test for anything on
        /// a hanging transducer, and it is what `CommsModem.IsSubmerged()` prefers when a
        /// DeployedTransducer governs the modem — so a station never touches the shared water
        /// query and never poisons the vehicle's buoyancy.
        /// </summary>
        public bool IsWet(Vector3 worldPos)
        {
            if (!TryGetWaterPlaneY(out float planeY)) return false;
            return worldPos.y < planeY;
        }

        /// <summary>
        /// Put the transducer at `waterPlaneY - DipDepthM`, keeping the case's X/Z. Safe to call
        /// every frame: it only writes the transform when it is actually off by more than a
        /// millimetre, and in the editor it hands the scene's dirty flag back exactly as it found
        /// it, so a derived number can never be the reason ⌘S never settles.
        /// </summary>
        public void Deploy()
        {
            if (!CanDeployHere(out string whereWhy, out bool quiet))
            {
                if (quiet) RefuseQuietly(whereWhy); else Fail(whereWhy);
                Measure(true);
                return;
            }

            if (DipDepthM < MinDipDepthM)
            {
                Fail($"dip depth {DipDepthM:F2} m is below the {MinDipDepthM:F2} m minimum — " +
                     "a transducer at or above the surface is dry, ventilated and deaf. " +
                     "REFUSED; the transducer has not been moved.");
                Measure(true);
                return;
            }

            if (!TryGetWaterPlaneY(out float planeY, out string waterWhy))
            {
                Fail(waterWhy);
                Measure(true);
                return;
            }

            if (DipDepthM < PracticeDipDepthM)
                WarnOnce($"dip depth {DipDepthM:F2} m is shallower than the {PracticeDipDepthM:F2} m " +
                         "we actually deploy at (Ivan, 2026-08-21: \"we always drop the transducer at " +
                         "least 50cm down below the water level\"). Deploying anyway — but this is now " +
                         "a decision on the record rather than a default.");

            var want = new Vector3(transform.position.x, planeY - DipDepthM, transform.position.z);
            if ((transform.position - want).sqrMagnitude > 1e-6f)   // 1 mm
                WriteTransform(want);

            Measure();
        }

        /// <summary>
        /// The only place this component moves anything, so the "did I dirty the scene?" question
        /// has exactly one answer to check.
        ///
        /// WHY THE DIRTY FLAG IS RESTORED. This Y is DERIVED — from the water plane and the
        /// configured dip — and is recomputed on every load, so it is not information that needs
        /// saving. If a self-recomputed number could dirty the scene, an `[ExecuteAlways]`
        /// component would leave Beckholmen permanently modified and ⌘S would never settle: the
        /// editor would be asking Ivan to save a value it is about to derive again anyway. The
        /// flag is only cleared when the scene was CLEAN immediately before this write — meaning
        /// this write is the only change there is — so a real edit of Ivan's is never discarded.
        /// </summary>
        void WriteTransform(Vector3 want)
        {
            // CORRECTED 2026-08-21, on the first compile: this used to restore the scene's dirty
            // flag via `EditorSceneManager.ClearSceneDirtiness(scene)`, which **does not exist in
            // Unity 6.3** (CS0117). The restoration is not needed, and reaching for it was solving
            // a problem the caller had already solved: the write is gated on the position being
            // off by more than PositionEpsilon, so it happens ONCE and then stops. A component
            // that writes every tick would indeed leave Beckholmen permanently modified; this one
            // converges, and after it converges there is nothing to dirty. Marking the scene dirty
            // for the one write that genuinely moved something is correct — the transform really
            // did change, and ⌘S should say so.
            transform.position = want;
        }

        /// <summary>Recompute the read-only fields from where the transducer ACTUALLY is. Kept
        /// separate from Deploy() on purpose: the readout must describe the transform, never echo
        /// the setting, or it stops being able to disagree with it.</summary>
        void Measure(bool refusedByGuard = false)
        {
            if (!TryGetWaterPlaneY(out float planeY))
            {
                Deployed = false;
                ActualDepthM = float.NaN;
                // "not evaluated" counts as empty. It is the field's initial value, and leaving it
                // standing would report a component that HAS run as one that never did — a stale
                // readout outliving what it describes, which is §3s's own lesson about
                // AppliedBuoyancyForce.
                if (string.IsNullOrEmpty(RefusalReason) || RefusalReason == "not evaluated")
                    RefusalReason = "no usable WaterSurface — cannot tell whether the transducer is wet";
                return;
            }

            ActualDepthM = planeY - transform.position.y;

            // A guard refusal outranks the geometry. If DipDepthM was refused and the transducer
            // happens to still be wet from an earlier deployment, saying "deployed" would let a
            // rejected configuration look accepted — the exact silence this component exists to
            // remove. Fail() has already written the reason; leave it standing.
            if (refusedByGuard) { Deployed = false; return; }

            Deployed = ActualDepthM > 0f;
            if (Deployed)
            {
                RefusalReason = "";
                lastLogged = null;
            }
            else
            {
                RefusalReason = "transducer above the water line — not deployed " +
                                $"({-ActualDepthM:F2} m above the surface)";
            }
        }

        void Fail(string why)
        {
            RefusalReason = "transducer above the water line — not deployed: " + why;
            Deployed = false;
            if (lastLogged == why) return;
            lastLogged = why;
            Debug.LogError($"[DeployedTransducer] {name}: {why}", this);
        }

        /// <summary>A refusal that is NORMAL rather than wrong — a prefab being open is not an
        /// operator error. It still has to be visible where someone would look for it (the
        /// Inspector, and `CommsModem.DryReason`), but it must not turn the console red on every
        /// press of a menu item, because a console people stop reading is the failure mode this
        /// project has already paid for.</summary>
        void RefuseQuietly(string why)
        {
            RefusalReason = "not deployed: " + why;
            Deployed = false;
        }

        void WarnOnce(string why)
        {
            if (lastLogged == why) return;
            lastLogged = why;
            Debug.LogWarning($"[DeployedTransducer] {name}: {why}", this);
        }

        /// <summary>
        /// The cheap discriminator between "configured 0.5 m" and "actually 0.5 m", visible in the
        /// Scene view without pressing Play: the drop line from the case, the water line it passes
        /// through, and the transducer at the bottom of it. The numeric label is drawn by
        /// `DeployedTransducerEditor` (Handles only exist in the editor assembly).
        ///
        /// Colour carries the verdict: cyan deployed, red dry. A gizmo that looks the same whether
        /// or not it is working is a gizmo that discriminates nothing.
        /// </summary>
        void OnDrawGizmos()
        {
            var anchor = CaseAnchor != null ? CaseAnchor : transform.parent;
            Vector3 top = anchor != null ? anchor.position : transform.position + Vector3.up;
            Vector3 fish = transform.position;

            bool haveWater = TryGetWaterPlaneY(out float planeY);
            bool wet = haveWater && fish.y < planeY;

            Gizmos.color = wet ? new Color(0.25f, 0.9f, 1f, 0.95f) : new Color(1f, 0.25f, 0.2f, 0.95f);
            Gizmos.DrawLine(top, fish);
            Gizmos.DrawSphere(fish, 0.06f);
            Gizmos.DrawWireCube(top, new Vector3(0.06f, 0.06f, 0.06f));

            if (!haveWater) return;

            // The water line, drawn where the cable crosses it, so "how deep" is a distance you
            // can see rather than a number you have to trust.
            Vector3 cross = new Vector3(fish.x, planeY, fish.z);
            Gizmos.color = new Color(0.6f, 0.85f, 1f, 0.8f);
            Gizmos.DrawLine(cross + new Vector3(-0.6f, 0f, 0f), cross + new Vector3(0.6f, 0f, 0f));
            Gizmos.DrawLine(cross + new Vector3(0f, 0f, -0.6f), cross + new Vector3(0f, 0f, 0.6f));
        }
    }
}
