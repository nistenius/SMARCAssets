using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DataCubeViz
{
    /// <summary>
    /// Draws every mappable track in the loaded replay as a LineRenderer, one child per track.
    ///
    /// WHY EVERY TRACK, NOT THE BEST ONE
    /// ---------------------------------
    /// Ivan's rule from Debrief: all available tracks, toggleable, played together — the point
    /// being that DR-vs-fused divergence becomes something you SEE rather than a number in a
    /// post-run script. The station already refuses to "correct" tracks onto each other (it
    /// anchors a frame by consensus and reports each track's own deviation), so the disagreement
    /// survives all the way here. Drawing only the best track would throw away the finding.
    ///
    /// A track's line also carries a progress split: the part already played is drawn solid, the
    /// rest dimmed. That is the same "what has happened vs what is planned" distinction MC draws,
    /// and it is what makes a scrubbed replay legible at a glance.
    /// </summary>
    [AddComponentMenu("DataCube/Replay Track Renderer")]
    [RequireComponent(typeof(ReplayPlayer))]
    public class ReplayTrackRenderer : MonoBehaviour
    {
        [Header("Appearance")]
        [Tooltip("Unlit material every track line is made from. ASSIGN THIS if you build a " +
                 "player: left empty the component falls back to Shader.Find, which returns " +
                 "null in builds and after a pipeline change — it will say so rather than " +
                 "drawing nothing quietly. In the editor the fallback is fine.")]
        public Material LineMaterial;

        public float LineWidth = 0.35f;

        [Tooltip("Colours are assigned in track order and reused if there are more tracks.")]
        public Color[] Palette =
        {
            new(0.20f, 0.85f, 1.00f),   // cyan   — usually the fused/geographic reference
            new(1.00f, 0.65f, 0.10f),   // amber  — usually dr/odom, the estimate
            new(0.55f, 1.00f, 0.45f),   // green
            new(1.00f, 0.40f, 0.75f),   // pink
            new(0.75f, 0.70f, 1.00f),   // violet
            new(1.00f, 0.95f, 0.45f),   // yellow
        };

        [Tooltip("How much the not-yet-played remainder of a track is faded.")]
        [Range(0f, 1f)] public float FutureAlpha = 0.22f;

        [Tooltip("Draw the whole track solid instead of splitting at the playhead.")]
        public bool ShowWholeTrackSolid = false;

        ReplayPlayer _player;
        readonly Dictionary<string, TrackLines> _lines = new();
        readonly HashSet<string> _hidden = new();
        Material _material;

        class TrackLines
        {
            public ReplayTrack Track;
            public LineRenderer Past;
            public LineRenderer Future;
            public Vector3[] Points;
            public Color Colour;
        }

        void Awake()
        {
            _player = GetComponent<ReplayPlayer>();
            _material = BuildBaseMaterial();
        }

        /// <summary>
        /// Unlit, so a track stays readable underwater — the scene's lighting is exactly what
        /// makes everything else hard to see down there. Follows the same assign-or-fall-back
        /// pattern (and the same warning) as SonarMapAccumulator and UnderwaterParticles,
        /// because Shader.Find is a known trap in this project: it returns null in player
        /// builds and after a pipeline change.
        /// </summary>
        Material BuildBaseMaterial()
        {
            if (LineMaterial != null) return new Material(LineMaterial);

            var shader = Shader.Find("HDRP/Unlit");
            if (shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            if (shader == null) shader = Shader.Find("Sprites/Default");
            if (shader == null)
            {
                Debug.LogError("[DataCubeViz] no unlit shader found and no LineMaterial " +
                               "assigned — track lines cannot be drawn. Assign LineMaterial.", this);
                return null;
            }
            Debug.LogWarning($"[DataCubeViz] no LineMaterial assigned; falling back to " +
                             $"Shader.Find(\"{shader.name}\"), which is null in player builds.", this);
            return new Material(shader);
        }

        /// <summary>HDRP's Unlit uses _UnlitColor; the built-in and URP shaders use _Color /
        /// _BaseColor. Set whichever exists rather than assuming a pipeline.</summary>
        static void Tint(Material m, Color c)
        {
            if (m == null) return;
            if (m.HasProperty("_UnlitColor")) m.SetColor("_UnlitColor", c);
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            if (m.HasProperty("_Color")) m.SetColor("_Color", c);
        }

        void OnEnable()
        {
            if (_player == null) _player = GetComponent<ReplayPlayer>();
            _player.Loaded += Rebuild;
            if (_player.Data != null) Rebuild(_player.Data);
        }

        void OnDisable()
        {
            if (_player != null) _player.Loaded -= Rebuild;
        }

        void OnDestroy()
        {
            if (_material != null) Destroy(_material);
        }

        public void Rebuild(ReplayFile file)
        {
            foreach (var tl in _lines.Values)
            {
                if (tl.Past != null) Destroy(tl.Past.gameObject);
                if (tl.Future != null) Destroy(tl.Future.gameObject);
            }
            _lines.Clear();
            if (file == null) return;

            int i = 0;
            foreach (var track in _player.MappableTracks)
            {
                var pts = _player.WorldPoints(track);
                if (pts.Length < 2) { i++; continue; }
                var colour = Palette.Length > 0 ? Palette[i % Palette.Length] : Color.white;
                _lines[track.Key] = new TrackLines
                {
                    Track = track,
                    Points = pts,
                    Colour = colour,
                    Past = MakeLine($"{track.Key} (played)", colour, 1f),
                    Future = MakeLine($"{track.Key} (remaining)", colour, FutureAlpha),
                };
                i++;
            }
            Redraw(_player.Clock.Time);
        }

        LineRenderer MakeLine(string name, Color colour, float alpha)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.widthMultiplier = LineWidth;
            lr.numCapVertices = 2;
            var tinted = new Color(colour.r, colour.g, colour.b, alpha);
            if (_material != null)
            {
                var m = new Material(_material);
                Tint(m, tinted);
                lr.material = m;
            }
            lr.startColor = lr.endColor = tinted;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.positionCount = 0;
            return lr;
        }

        void Update()
        {
            if (_player.Data == null) return;
            Redraw(_player.Clock.Time);
        }

        void Redraw(double t)
        {
            foreach (var tl in _lines.Values)
            {
                bool visible = !_hidden.Contains(tl.Track.Key);
                if (tl.Past != null) tl.Past.enabled = visible;
                if (tl.Future != null) tl.Future.enabled = visible && !ShowWholeTrackSolid;
                if (!visible) continue;

                if (ShowWholeTrackSolid)
                {
                    SetPoints(tl.Past, tl.Points, 0, tl.Points.Length);
                    continue;
                }

                int split = ReplayMath.FloorIndex(tl.Track.T, t);
                // Before the track's first sample nothing is "past" — an empty past line is the
                // honest rendering of "this source had not reported yet", and is why the split
                // is clamped at -1 rather than at 0.
                if (split < 0)
                {
                    SetPoints(tl.Past, tl.Points, 0, 0);
                    SetPoints(tl.Future, tl.Points, 0, tl.Points.Length);
                    continue;
                }
                split = Mathf.Min(split, tl.Points.Length - 1);
                SetPoints(tl.Past, tl.Points, 0, split + 1);
                SetPoints(tl.Future, tl.Points, split, tl.Points.Length - split);
            }
        }

        static void SetPoints(LineRenderer lr, Vector3[] all, int start, int count)
        {
            if (lr == null) return;
            if (count <= 1) { lr.positionCount = 0; return; }
            lr.positionCount = count;
            for (int i = 0; i < count; i++) lr.SetPosition(i, all[start + i]);
        }

        // -- visibility, driven by the panel ---------------------------------------------------

        public bool IsVisible(string key) => !_hidden.Contains(key);

        public void SetVisible(string key, bool visible)
        {
            if (visible) _hidden.Remove(key); else _hidden.Add(key);
        }

        public Color ColourOf(string key) =>
            _lines.TryGetValue(key, out var tl) ? tl.Colour : Color.white;

        public IEnumerable<string> Keys => _lines.Keys.ToList();
    }
}
