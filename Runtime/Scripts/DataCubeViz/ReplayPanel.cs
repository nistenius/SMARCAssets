using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace DataCubeViz
{
    /// <summary>
    /// The in-scene transport: play / pause / scrub / speed, the track list, and the provenance
    /// of whatever is on screen.
    ///
    /// IMGUI ON PURPOSE
    /// ----------------
    /// This is a first version meant to be pressed Play on and judged. IMGUI needs no prefab, no
    /// Canvas, no scene wiring and no serialized references — so it cannot be lost to a Play-mode
    /// edit, which is the trap that cost this project a measured station position. When the shape
    /// is settled it can become a proper uGUI panel in SmarcGUI's style.
    ///
    /// WHAT THE PANEL IS FOR, BEYOND BUTTONS
    /// -------------------------------------
    /// It is where a rendered position names its source. Every track row shows its topic, and
    /// the ghost block states whether it is facing a reported heading or course over ground and
    /// whether the bag carried depth at all. The bag's own warnings — truncated recording,
    /// message definitions resolved from today's source tree, two publishers on one topic — are
    /// listed, not just logged, because the person looking at the ghost is the person who needs
    /// them.
    /// </summary>
    [AddComponentMenu("DataCube/Replay Panel")]
    [RequireComponent(typeof(ReplayPlayer))]
    public class ReplayPanel : MonoBehaviour
    {
        [Header("Placement")]
        public bool Visible = true;

        [Tooltip("Show/hide the panel. Uses the New Input System — this project is set to " +
                 "activeInputHandler: 1 (new only), where UnityEngine.Input throws at runtime.")]
        public Key ToggleKey = Key.F9;

        public Vector2 Position = new(12, 12);
        public float Width = 430f;

        /* HUD ORGANISATION (Ivan, 2026-08-27: "organize the Unity HUD overlays so we don't have
           confusing overlaps"). This panel used to be a fixed BeginArea at top-left, always fully
           open — which painted 600 px of replay UI straight over SmarcGUI's mission plans and
           vehicle-health cards the moment a scene loaded, replay or no replay. Three changes:

           1. It is a DRAGGABLE GUI.Window now — grab the title bar and move it off whatever it
              is covering. The position survives domain reloads via PlayerPrefs, so a layout you
              fixed once stays fixed.
           2. It COLLAPSES to its title bar ([–] button, or automatically while NO replay is
              loaded) — a live sim shows a one-line "DATA CUBE — replay (F9)" chip instead of a
              full panel of buttons about data that is not there.
           3. The [×] button hides it entirely; F9 (ToggleKey) brings it back. */
        [Tooltip("Start collapsed to the title bar. The panel auto-collapses anyway while no " +
                 "replay is loaded; this also collapses it when one IS loaded.")]
        public bool StartCollapsed = false;

        bool _collapsed;
        bool _userExpandedWithoutData;   // an operator who clicks [+] with no data means it
        Rect _win;                        // live window rect — the draggable truth
        const string PrefKeyX = "DataCubeViz.ReplayPanel.x";
        const string PrefKeyY = "DataCubeViz.ReplayPanel.y";

        ReplayPlayer _player;
        ReplayTrackRenderer _tracks;
        ReplayGhost[] _ghosts;

        Vector2 _scroll;
        bool _showWarnings;
        bool _showTracks = true;
        bool _showFiles;
        string[] _files = System.Array.Empty<string>();
        GUIStyle _header, _mono, _small;

        void Awake()
        {
            _player = GetComponent<ReplayPlayer>();
            _tracks = GetComponent<ReplayTrackRenderer>();
        }

        void Start()
        {
            Refresh();
            _collapsed = StartCollapsed;
            // PlayerPrefs, wrapped: a headless/batch run has no prefs backend worth failing on.
            try
            {
                _win = new Rect(PlayerPrefs.GetFloat(PrefKeyX, Position.x),
                                PlayerPrefs.GetFloat(PrefKeyY, Position.y), Width, 0);
            }
            catch { _win = new Rect(Position.x, Position.y, Width, 0); }
        }

        void Refresh()
        {
            _ghosts = FindObjectsByType<ReplayGhost>(FindObjectsSortMode.None);
            _files = ReplayLoader.FindReplays();
        }

        void Update()
        {
            // Keyboard.current is null when no keyboard is present (a headless/batch run, or a
            // build on a device without one). Guarding is not defensive noise: the alternative
            // is a NullReferenceException every frame in exactly the runs nobody is watching.
            var kb = Keyboard.current;
            if (kb == null) return;

            if (kb[ToggleKey].wasPressedThisFrame) Visible = !Visible;
            if (_player.Data == null) return;
            // Data is loaded: the no-data override has served its purpose, so the next time the
            // scene is dataless the panel auto-collapses again instead of remembering one click.
            _userExpandedWithoutData = false;
            // Space = play/pause, arrows = step. Only while the panel is up, so a scene that
            // uses these keys for its own controls is unaffected when it is hidden.
            if (!Visible) return;
            if (kb.spaceKey.wasPressedThisFrame) _player.Clock.TogglePlay();
            if (kb.leftArrowKey.wasPressedThisFrame) _player.Clock.Step(-1);
            if (kb.rightArrowKey.wasPressedThisFrame) _player.Clock.Step(1);
        }

        void EnsureStyles()
        {
            if (_header != null) return;
            _header = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = 13 };
            _mono = new GUIStyle(GUI.skin.label) { fontSize = 11, wordWrap = true };
            _small = new GUIStyle(GUI.skin.label) { fontSize = 10, wordWrap = true };
        }

        void OnGUI()
        {
            if (!Visible) return;
            EnsureStyles();

            // While no replay is loaded the panel earns nothing but its title bar: a live sim
            // must not wear a replay UI. Clicking [+] deliberately overrides that for one look
            // (the file picker lives inside), and loading data restores normal behaviour.
            bool effectiveCollapsed = _collapsed ||
                                      (_player.Data == null && !_userExpandedWithoutData);

            float maxH = Mathf.Min(Screen.height - _win.y - 12, 640);
            _win.width = effectiveCollapsed ? 240 : Width;
            _win.height = effectiveCollapsed ? 26 : maxH;
            // Never let a drag park the title bar off screen — an invisible window with the
            // only handle out of reach reads as "the panel is gone".
            _win.x = Mathf.Clamp(_win.x, 0, Mathf.Max(0, Screen.width - 60));
            _win.y = Mathf.Clamp(_win.y, 0, Mathf.Max(0, Screen.height - 26));

            // The collapsed chip must say when a replay is still painting the scene — tracks
            // render from DATA, not from this panel, and an unexplained magenta polyline over a
            // live scene is exactly the confusion this panel exists to prevent.
            string title = _player.Data != null
                ? (effectiveCollapsed ? "DATA CUBE — replay ● LOADED (F9)" : "DATA CUBE — replay")
                : "DATA CUBE — replay (F9)";
            _win = GUI.Window(GetInstanceID(), _win, DrawWindow, title);

            if (Event.current.type == EventType.Repaint)
            {
                try
                {
                    PlayerPrefs.SetFloat(PrefKeyX, _win.x);
                    PlayerPrefs.SetFloat(PrefKeyY, _win.y);
                }
                catch { /* no prefs backend: the layout just does not persist */ }
            }
        }

        void DrawWindow(int _)
        {
            bool effectiveCollapsed = _collapsed ||
                                      (_player.Data == null && !_userExpandedWithoutData);

            // Title-bar controls, drawn over the window chrome: collapse/expand and hide.
            var btn = new Rect(_win.width - 44, 3, 18, 16);
            if (GUI.Button(btn, effectiveCollapsed ? "+" : "–", _small))
            {
                if (effectiveCollapsed) { _collapsed = false; _userExpandedWithoutData = true; }
                else { _collapsed = true; _userExpandedWithoutData = false; }
            }
            if (GUI.Button(new Rect(_win.width - 22, 3, 18, 16), "×", _small))
                Visible = false;              // ToggleKey (F9) brings it back

            if (!effectiveCollapsed)
            {
                _scroll = GUILayout.BeginScrollView(_scroll);
                if (_player.Data == null)
                {
                    DrawNoData();
                }
                else
                {
                    DrawSource();
                    if (GUILayout.Button("⏏ Unload replay — clear its tracks and ghosts from the scene"))
                    {
                        _player.Unload();
                        Refresh();        // the file list is the natural next thing to want
                    }
                    DrawDepthConflict();
                    GUILayout.Space(4);
                    DrawTransport();
                    GUILayout.Space(6);
                    DrawGhosts();
                    GUILayout.Space(4);
                    DrawTracks();
                    DrawWarnings();
                    DrawFilePicker();
                }
                GUILayout.EndScrollView();
            }

            // The whole window is a drag handle where nothing else claims the event —
            // IMGUI gives dragging away for free, and free is the right price for it.
            GUI.DragWindow(new Rect(0, 0, _win.width - 46, 22));
        }

        void DrawNoData()
        {
            GUILayout.Label(_player.LoadError ?? "No replay loaded.", _mono);
            GUILayout.Space(6);
            GUILayout.Label("Export one with:", _small);
            GUILayout.Label("python3 data-cube/scripts/export_bag_for_unity.py <bag-dir>", _small);
            GUILayout.Space(4);
            if (GUILayout.Button("Rescan StreamingAssets/DataCubeReplays")) Refresh();
            foreach (var f in _files)
            {
                if (GUILayout.Button(Path.GetFileName(f))) _player.Load(f);
            }
        }

        void DrawSource()
        {
            var s = _player.Data.Source;
            GUILayout.Label($"{s?.Label ?? s?.Name ?? "(unnamed bag)"}", _mono);

            // Site first: it is the fact that decides whether anything on screen is in the
            // right place at all, and it is cheap to show and expensive to have wrong.
            var site = _player.Data.Site;
            if (site != null && site.Matched)
                GUILayout.Label($"site: {site.Name} · centroid {site.DistanceM:0} m from origin", _small);
            var run = _player.Data.Run;
            if (run != null) GUILayout.Label($"run: {run.Describe()}", _small);

            var bits = new List<string>();
            if (!string.IsNullOrEmpty(s?.Kind)) bits.Add(s.Kind);
            if (s != null && s.MessageCount > 0) bits.Add($"{s.MessageCount:N0} msgs");
            if (s != null && s.Truncated) bits.Add("TRUNCATED RECORDING");
            GUILayout.Label(string.Join(" · ", bits), _small);
        }

        /// <summary>
        /// The depth-sign split, surfaced with its measurement. This is not a footnote: read as
        /// one series the showcase bag's depth topic reports a vehicle changing depth at 48 m/s,
        /// and the only reason it does not look like a broken sensor is that the exporter split
        /// it. Whoever is watching the ghost should be able to see that, and to flip to the
        /// other publisher and check for themselves.
        /// </summary>
        void DrawDepthConflict()
        {
            var dc = _player.Data.DepthConflict;
            if (dc == null || !dc.Detected) return;

            var prev = GUI.color;
            GUI.color = new Color(1f, 0.75f, 0.3f);
            GUILayout.Label("⚠ depth topic has TWO publishers, opposite signs", _mono);
            GUI.color = prev;
            GUILayout.Label($"   {dc.ChainAN} positive-is-down · {dc.ChainBN} negative-is-down · " +
                            $"{dc.FlipFraction * 100:0}% of consecutive samples flip", _small);
            GUILayout.Label($"   split, they agree to {dc.AgreementMedianM:0.000} m; unsplit the " +
                            $"vehicle appears to move {dc.CombinedRateMs:0.#} m/s vertically", _small);

            GUILayout.BeginHorizontal();
            var useAlt = GUILayout.Toggle(_player.UseAlternateDepthChain,
                                          " drive depth from chain B instead", _small);
            if (useAlt != _player.UseAlternateDepthChain) _player.UseAlternateDepthChain = useAlt;
            GUILayout.EndHorizontal();
        }

        void DrawTransport()
        {
            var clock = _player.Clock;

            GUILayout.BeginHorizontal();
            if (GUILayout.Button(clock.Playing ? "❚❚ Pause" : "▶ Play", GUILayout.Width(80)))
                clock.TogglePlay();
            if (GUILayout.Button("⏮", GUILayout.Width(34))) clock.Rewind();
            if (GUILayout.Button("−1s", GUILayout.Width(42))) clock.Step(-1);
            if (GUILayout.Button("+1s", GUILayout.Width(42))) clock.Step(1);
            GUILayout.FlexibleSpace();
            GUILayout.Label(clock.TimeLabel, _mono);
            GUILayout.EndHorizontal();

            // The scrub bar. Dragging it sets bag time directly — every renderer and ghost reads
            // that one value, so the ghost, the track split and any channel readout can never
            // disagree about "when" this is.
            var frac = GUILayout.HorizontalSlider(clock.Fraction, 0f, 1f);
            if (!Mathf.Approximately(frac, clock.Fraction))
                clock.Time = frac * clock.Duration;

            GUILayout.BeginHorizontal();
            GUILayout.Label("speed", _small, GUILayout.Width(38));
            var sp = GUILayout.HorizontalSlider(Mathf.Log10(clock.Speed), -1f, 1.4f,
                                                GUILayout.Width(140));
            clock.Speed = Mathf.Pow(10f, sp);
            GUILayout.Label($"{clock.Speed:0.##}×", _small, GUILayout.Width(46));
            clock.Loop = GUILayout.Toggle(clock.Loop, "loop", _small);
            GUILayout.EndHorizontal();
        }

        void DrawGhosts()
        {
            if (_ghosts == null || _ghosts.Length == 0) return;
            GUILayout.Label("Ghosts", _header);
            foreach (var g in _ghosts)
            {
                if (g == null) continue;
                GUILayout.Label($"• {g.name}", _mono);
                GUILayout.Label("   " + g.StatusLine().Replace("\n", "\n   "), _small);
            }
        }

        void DrawTracks()
        {
            _showTracks = GUILayout.Toggle(_showTracks, $"Tracks ({_player.Data.Tracks.Count})", _header);
            if (!_showTracks) return;

            foreach (var t in _player.Data.Tracks)
            {
                GUILayout.BeginHorizontal();
                if (t.Mappable && _tracks != null)
                {
                    var on = _tracks.IsVisible(t.Key);
                    var col = GUI.color;
                    GUI.color = _tracks.ColourOf(t.Key);
                    var now = GUILayout.Toggle(on, "  ", GUILayout.Width(22));
                    GUI.color = col;
                    if (now != on) _tracks.SetVisible(t.Key, now);
                }
                else
                {
                    GUILayout.Label("  –", _small, GUILayout.Width(22));
                }
                GUILayout.Label($"{t.Label ?? t.Key}", _small);
                GUILayout.FlexibleSpace();
                GUILayout.Label($"{t.Count}", _small, GUILayout.Width(46));
                GUILayout.EndHorizontal();

                if (!t.Mappable)
                {
                    GUILayout.Label($"     not mappable — {t.Note}", _small);
                    continue;
                }
                // The deviation is the whole reason all tracks are drawn: for dr/odom this is
                // dead-reckoning error, for a second publisher it is how far the two disagree.
                if (t.Deviation != null)
                    GUILayout.Label($"     deviates {t.Deviation.MedianM:0.00} m median, " +
                                    $"{t.Deviation.MaxM:0.00} m max vs {t.Deviation.Vs}", _small);
                if (!t.HasDepth)
                    GUILayout.Label("     no depth in bag — drawn at the surface", _small);
            }
        }

        void DrawWarnings()
        {
            var w = _player.Data.Warnings;
            if (w == null || w.Count == 0) return;
            GUILayout.Space(4);
            _showWarnings = GUILayout.Toggle(_showWarnings, $"Warnings ({w.Count})", _header);
            if (!_showWarnings) return;
            foreach (var line in w) GUILayout.Label("• " + line, _small);
        }

        void DrawFilePicker()
        {
            GUILayout.Space(4);
            _showFiles = GUILayout.Toggle(_showFiles, "Load another replay", _header);
            if (!_showFiles) return;
            if (GUILayout.Button("Rescan")) Refresh();
            foreach (var f in _files)
            {
                var isCurrent = f == _player.LoadedPath;
                if (GUILayout.Button((isCurrent ? "● " : "   ") + Path.GetFileName(f)))
                {
                    _player.Load(f);
                    Refresh();
                }
            }
        }
    }
}
