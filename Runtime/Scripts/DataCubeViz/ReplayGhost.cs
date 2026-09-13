using System.Linq;
using UnityEngine;

namespace DataCubeViz
{
    /// <summary>
    /// Drives a transform along one replayed track — the ghost vehicle.
    ///
    /// THE ONE RULE THIS COMPONENT ENFORCES AT RUNTIME: A GHOST PUBLISHES NOTHING
    /// -------------------------------------------------------------------------
    /// Invariant 11 in SYSTEMS_SPEC was paid for: a health probe read `smarc/odom`, Unity was
    /// also publishing it from ground truth, and a SIGSEGV'd state estimator showed green for an
    /// entire session. A ghost is a *second* body in the scene with a full set of positions — if
    /// it carries any ROS publisher, it becomes a second writer on the live vehicle's topics and
    /// reproduces that failure with a recording from last month as the source.
    ///
    /// So Awake() searches this object's hierarchy for anything deriving from ROSBehaviour or
    /// named like a publisher, DESTROYS it, and logs what it removed. It does not warn and
    /// continue: a warning in a console with 80 nodes' worth of traffic is not a guard.
    ///
    /// It also disables colliders. The ghost is not in the water; it must not push the live
    /// vehicle, trip a protective stop, or return an echo to a sonar that casts with mask ~0
    /// (the reason StationBuilder's "no colliders" rule exists at all).
    /// </summary>
    [AddComponentMenu("DataCube/Replay Ghost")]
    public class ReplayGhost : MonoBehaviour
    {
        [Header("Source")]
        public ReplayPlayer Player;

        [Tooltip("Which track drives this ghost. Leave empty to use the first mappable track. " +
                 "The panel lists the keys; they are topic names like /sam/dr/odom.")]
        public string TrackKey = "";

        [Header("Body")]
        [Tooltip("The visual to move. Leave empty to move this GameObject itself.")]
        public Transform Body;

        [Tooltip("Material to REPLACE every renderer's material with, so the ghost reads as a " +
                 "recording rather than a second live vehicle. Assign a TRANSPARENT material " +
                 "(HDRP/Unlit with Surface Type = Transparent). Left empty, the component only " +
                 "lowers the alpha of the existing materials — which does nothing visible on an " +
                 "OPAQUE HDRP material, and it will say so instead of pretending it worked.")]
        public Material GhostMaterial;

        [Range(0.05f, 1f)] public float GhostAlpha = 0.45f;

        public bool ApplyGhostMaterial = true;

        [Header("Orientation")]
        [Tooltip("Rotate the body to the replayed heading (or course over ground when the bag " +
                 "carried no heading — the panel says which).")]
        public bool ApplyHeading = true;

        [Tooltip("Added to the replayed heading. Use when the model's nose is not +Z.")]
        public float HeadingOffsetDeg = 0f;

        /// <summary>False before the track's first sample: the source had not reported yet.
        /// The ghost hides rather than sitting at the origin.</summary>
        public bool HasPosition { get; private set; }
        public bool HeadingIsCourseOverGround { get; private set; } = true;
        public ReplayTrack Track { get; private set; }

        Renderer[] _renderers;

        void Awake()
        {
            if (Body == null) Body = transform;
            StripAnythingThatCouldPublish();
            DisableColliders();
            _renderers = GetComponentsInChildren<Renderer>(true);
        }

        /// <summary>
        /// Remove every component that could put this ghost's pose onto the ROS graph.
        ///
        /// Matched by base type where the type exists in this assembly's references, and by
        /// type-name otherwise — SMARCAssets has publishers under several namespaces and a
        /// name check catches the ones a type reference would miss without coupling this
        /// component to every one of them. Over-matching here is safe: a ghost has no business
        /// carrying anything with "Publisher" in its name.
        /// </summary>
        void StripAnythingThatCouldPublish()
        {
            var removed = 0;
            foreach (var c in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (c == null || c == this) continue;
                var n = c.GetType().Name;
                var full = c.GetType().FullName ?? "";
                bool publishes =
                    n.Contains("Publisher") || n.Contains("ROSPublish") ||
                    n.EndsWith("Publish") || full.Contains("ROSBehaviour") ||
                    IsSubclassByName(c.GetType(), "ROSBehaviour") ||
                    IsSubclassByName(c.GetType(), "ROSPublisher") ||
                    IsSubclassByName(c.GetType(), "ROSSensorPublisher");
                if (!publishes) continue;
                Debug.LogWarning($"[DataCubeViz] ghost '{name}': removing {n} on " +
                                 $"'{c.gameObject.name}'. A replay ghost must never publish — " +
                                 $"it would become a second writer on the live vehicle's topics " +
                                 $"(SYSTEMS_SPEC invariants 11 and 12).", this);
                Destroy(c);
                removed++;
            }
            if (removed > 0)
                Debug.Log($"[DataCubeViz] ghost '{name}': {removed} publishing component(s) removed.", this);
        }

        static bool IsSubclassByName(System.Type t, string baseName)
        {
            for (var b = t.BaseType; b != null; b = b.BaseType)
                if (b.Name == baseName) return true;
            return false;
        }

        void DisableColliders()
        {
            foreach (var col in GetComponentsInChildren<Collider>(true)) col.enabled = false;
            foreach (var rb in GetComponentsInChildren<Rigidbody>(true))
            {
                rb.isKinematic = true;
                rb.detectCollisions = false;
            }
            foreach (var ab in GetComponentsInChildren<ArticulationBody>(true))
                ab.enabled = false;
        }

        void Start()
        {
            if (Player == null) Player = FindFirstObjectByType<ReplayPlayer>();
            if (Player == null)
            {
                Debug.LogError($"[DataCubeViz] ghost '{name}': no ReplayPlayer assigned or found " +
                               $"in the scene. Nothing to follow.", this);
                enabled = false;
                return;
            }
            // Named handler, not a lambda: a lambda cannot be unsubscribed, and a ghost that
            // outlives its subscription keeps a destroyed object alive through the player's
            // event list. Cheap to get right, tedious to find later.
            Player.Loaded += OnPlayerLoaded;
            if (Player.Data != null) Bind();
            if (ApplyGhostMaterial) ApplyAlpha();
        }

        void OnDestroy()
        {
            if (Player != null) Player.Loaded -= OnPlayerLoaded;
        }

        void OnPlayerLoaded(ReplayFile _) => Bind();

        void Bind()
        {
            // Unload() fires Loaded(null): no data is not a missing track, so no warning —
            // just let LateUpdate hide the ghost until something is loaded again.
            if (Player.Data == null) { Track = null; return; }
            Track = string.IsNullOrWhiteSpace(TrackKey)
                ? Player.MappableTracks.FirstOrDefault()
                : Player.TrackByKey(TrackKey);

            if (Track == null)
            {
                Debug.LogWarning($"[DataCubeViz] ghost '{name}': track '{TrackKey}' is not in " +
                                 $"this replay. Available: " +
                                 $"{string.Join(", ", Player.MappableTracks.Select(t => t.Key))}", this);
                return;
            }
            Debug.Log($"[DataCubeViz] ghost '{name}' follows {Track.Key}\n{Track.Provenance()}", this);
        }

        void LateUpdate()
        {
            if (Player == null || Track == null) { SetVisible(false); return; }

            HasPosition = Player.PoseAt(Track, Player.Clock.Time, out var pos, out var headingDeg,
                                        out var isCourse);
            HeadingIsCourseOverGround = isCourse;
            SetVisible(HasPosition);
            if (!HasPosition) return;

            Body.position = pos;
            if (ApplyHeading)
                Body.rotation = Quaternion.Euler(0f, headingDeg + HeadingOffsetDeg, 0f);
        }

        void SetVisible(bool v)
        {
            if (_renderers == null) return;
            foreach (var r in _renderers) if (r != null) r.enabled = v;
        }

        void ApplyAlpha()
        {
            if (GhostMaterial != null)
            {
                foreach (var r in GetComponentsInChildren<Renderer>(true))
                {
                    var mats = new Material[r.sharedMaterials.Length];
                    for (int i = 0; i < mats.Length; i++) mats[i] = GhostMaterial;
                    r.sharedMaterials = mats;
                }
                return;
            }

            // No ghost material: lower the alpha of what is already there. On an OPAQUE HDRP
            // material this changes a number nothing reads — the ghost will look exactly like
            // a second live vehicle, which is the one thing it must not look like. Say so once,
            // rather than leaving the operator to wonder why the setting did nothing.
            bool anyOpaque = false;
            foreach (var r in GetComponentsInChildren<Renderer>(true))
            {
                foreach (var m in r.materials)
                {
                    if (m == null) continue;
                    // HDRP marks transparency with _SurfaceType (0 = opaque, 1 = transparent).
                    if (m.HasProperty("_SurfaceType") && m.GetFloat("_SurfaceType") < 0.5f)
                        anyOpaque = true;
                    if (m.HasProperty("_UnlitColor"))
                    {
                        var c = m.GetColor("_UnlitColor"); c.a = GhostAlpha; m.SetColor("_UnlitColor", c);
                    }
                    if (m.HasProperty("_BaseColor"))
                    {
                        var c = m.GetColor("_BaseColor"); c.a = GhostAlpha; m.SetColor("_BaseColor", c);
                    }
                    else if (m.HasProperty("_Color"))
                    {
                        var c = m.color; c.a = GhostAlpha; m.color = c;
                    }
                }
            }
            if (anyOpaque)
                Debug.LogWarning($"[DataCubeViz] ghost '{name}': GhostAlpha was applied to " +
                                 $"OPAQUE material(s), where alpha does nothing — this ghost will " +
                                 $"look like a live vehicle. Assign a transparent GhostMaterial " +
                                 $"(HDRP/Unlit, Surface Type = Transparent) to actually fade it.", this);
        }

        /// <summary>One line for the panel: what this ghost is and how honest its heading is.</summary>
        public string StatusLine()
        {
            if (Track == null) return "no track bound";
            if (!HasPosition) return $"{Track.Key}: not reporting yet at this time";
            var h = HeadingIsCourseOverGround ? "course over ground (bag had no heading)" : "reported heading";
            var d = Track.HasDepth ? Track.DepthSource : "no depth in bag — drawn at surface";
            return $"{Track.Key}\n  position: {Track.PositionSource}\n  depth: {d}\n  facing: {h}";
        }
    }
}
