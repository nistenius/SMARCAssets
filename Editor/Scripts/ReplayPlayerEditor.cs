using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using DataCubeViz;
using UnityEditor;
using UnityEngine;

namespace DataCubeViz.EditorTools
{
    /// <summary>
    /// Replaces ReplayPlayer's typed file name with a dropdown of the replays that actually
    /// exist, each shown with the site it belongs to and whether that site matches this scene.
    ///
    /// WHY A DROPDOWN IS NOT JUST A CONVENIENCE HERE
    /// ---------------------------------------------
    /// Ivan, 2026-08-23: "shouldn't that be picked up automatically". Right, and for a stronger
    /// reason than typing effort. A typed file name is a hand-configured constant that nothing
    /// checks, and this project's most expensive bugs are all that shape: a hardcoded station
    /// position read as measured (49.4 m wrong, quality overstated 25x), a `utm_34_V` frame that
    /// made a working sonar look broken, a `timeout_s = 300.0` that made a 130 m mission
    /// impossible before launch. A field you can only fill correctly by remembering something
    /// will eventually be wrong.
    ///
    /// So the list is READ FROM DISK, and every entry carries the two facts that decide whether
    /// it will work: its site, and its distance from this scene's GlobalReferencePoint. A replay
    /// that would be refused at runtime says so BEFORE it is chosen. The refusal itself stays in
    /// ReplayPlayer — **the editor is a convenience and never the guard**, because a check that
    /// only exists at edit time is not a check.
    ///
    /// Files are parsed IN FULL (via the same loader the runtime uses, so there is one parser,
    /// not a second scraping one that can disagree with it) and cached against each file's
    /// write time. Substring-scraping a JSON header was the first version of this and it
    /// undercounted tracks on any file bigger than the read window — a wrong number displayed
    /// confidently, which is the exact failure this whole component exists to prevent.
    /// </summary>
    [CustomEditor(typeof(ReplayPlayer))]
    public class ReplayPlayerEditor : UnityEditor.Editor
    {
        class Entry
        {
            public string Path;
            public string FileName;
            public DateTime Stamp;
            public ReplayFile File;
            public string Error;

            public bool SiteMatched => File?.Site != null && File.Site.Matched;
            public string SiteName => SiteMatched ? File.Site.Name : "unidentified site";
            public int Tracks => File?.Tracks?.Count(t => t.Mappable) ?? 0;

            public double DistanceKmFrom(double lat, double lon)
            {
                if (!SiteMatched) return double.NaN;
                const double mLat = 111320.0;
                double mLon = 111320.0 * Math.Cos(lat * Math.PI / 180.0);
                double dn = (File.Site.Lat - lat) * mLat, de = (File.Site.Lon - lon) * mLon;
                return Math.Sqrt(dn * dn + de * de) / 1000.0;
            }
        }

        static readonly Dictionary<string, Entry> _cache = new();
        static List<Entry> _entries = new();

        public override void OnInspectorGUI()
        {
            var player = (ReplayPlayer)target;
            Rescan(false);

            var globalRef = FindAnyObjectByType<GeoRef.GlobalReferencePoint>();
            DrawSceneLine(globalRef);
            DrawPicker(player, globalRef);

            EditorGUILayout.Space(6);
            DrawDefaultInspector();
        }

        void DrawSceneLine(GeoRef.GlobalReferencePoint globalRef)
        {
            if (globalRef == null)
            {
                EditorGUILayout.HelpBox(
                    "No GlobalReferencePoint in this scene. ReplayPlayer will refuse to run: " +
                    "there is no origin to place a replay against, and it will not guess one. " +
                    "Open a georeferenced scene (Beckholmen / Kristineberg / Asko).",
                    MessageType.Error);
                return;
            }
            EditorGUILayout.LabelField("Scene origin",
                $"{globalRef.Lat:F5}, {globalRef.Lon:F5}", EditorStyles.miniLabel);
        }

        void DrawPicker(ReplayPlayer player, GeoRef.GlobalReferencePoint globalRef)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Replay", GUILayout.Width(EditorGUIUtility.labelWidth - 4));

            var current = player.ReplayFileName;
            var display = string.IsNullOrWhiteSpace(current) ? "(first replay found)" : current;
            if (EditorGUILayout.DropdownButton(new GUIContent(display), FocusType.Keyboard))
                ShowMenu(player, globalRef);
            if (GUILayout.Button("Rescan", GUILayout.Width(58))) Rescan(true);
            EditorGUILayout.EndHorizontal();

            if (_entries.Count == 0)
            {
                EditorGUILayout.HelpBox(
                    $"No *.dcreplay.json in {Rel(ReplayLoader.DefaultDirectory)}.\n\n" +
                    "Export one:\n" +
                    "python3 data-cube/scripts/export_bag_for_unity.py <bag-dir>",
                    MessageType.Info);
                return;
            }

            var sel = _entries.FirstOrDefault(e => Matches(e, current))
                      ?? (string.IsNullOrWhiteSpace(current) ? _entries[0] : null);
            if (sel == null)
            {
                EditorGUILayout.HelpBox($"'{current}' is not in {Rel(ReplayLoader.DefaultDirectory)}. " +
                                        "Pick one from the dropdown, or re-export it.",
                                        MessageType.Warning);
                return;
            }
            if (sel.Error != null) { EditorGUILayout.HelpBox(sel.Error, MessageType.Error); return; }

            var info = $"{sel.Tracks} tracks · {sel.File.DurationS:F0} s · site: " +
                       (sel.SiteMatched ? sel.File.Site.Name : "UNIDENTIFIED");

            if (!sel.SiteMatched)
            {
                EditorGUILayout.HelpBox(
                    info + "\n\nThe exporter could not match this bag to a site in " +
                    "maps/sites.yaml, so ReplayPlayer will refuse to draw it rather than place " +
                    "it at a guessed origin.\n\n" + (sel.File.Site?.Reason ?? ""),
                    MessageType.Error);
                return;
            }

            if (globalRef == null)
            {
                EditorGUILayout.LabelField(" ", info, EditorStyles.miniLabel);
                return;
            }

            var km = sel.DistanceKmFrom(globalRef.Lat, globalRef.Lon);
            if (km * 1000.0 > player.SiteMatchRadiusM)
            {
                EditorGUILayout.HelpBox(
                    info + $"\n\nWILL BE REFUSED AT PLAY: this replay is {km:F1} km from this " +
                    $"scene's origin. Drawn, it would land that far outside the world and simply " +
                    $"look like an empty scene. Open the {sel.File.Site.Name} scene instead.",
                    MessageType.Error);
            }
            else
            {
                EditorGUILayout.LabelField(" ", info + $" · {km * 1000:F0} m from scene origin",
                                           EditorStyles.miniLabel);
                if (sel.File.DepthConflict != null && sel.File.DepthConflict.Detected)
                    EditorGUILayout.HelpBox(
                        "This bag's depth topic carries two publishers with opposite sign " +
                        "conventions; the exporter split them. See the in-scene panel at Play.",
                        MessageType.Warning);
            }
        }

        static bool Matches(Entry e, string current)
        {
            if (string.IsNullOrWhiteSpace(current)) return false;
            return e.FileName == current
                || e.Path == current
                || Path.GetFileNameWithoutExtension(e.FileName) == current
                || e.FileName == current + ".dcreplay.json";
        }

        void ShowMenu(ReplayPlayer player, GeoRef.GlobalReferencePoint globalRef)
        {
            var menu = new GenericMenu();
            menu.AddItem(new GUIContent("(first replay found)"),
                         string.IsNullOrWhiteSpace(player.ReplayFileName),
                         () => Assign(player, ""));
            menu.AddSeparator("");

            foreach (var e in _entries)
            {
                // Grouped by site, and every entry that this scene would refuse says so in its
                // own label — the choice carries its consequence instead of deferring it to Play.
                string suffix;
                if (e.Error != null) suffix = "  [unreadable]";
                else if (!e.SiteMatched) suffix = "  [no site — will be refused]";
                else
                {
                    var km = globalRef == null ? double.NaN : e.DistanceKmFrom(globalRef.Lat, globalRef.Lon);
                    suffix = (!double.IsNaN(km) && km * 1000.0 > player.SiteMatchRadiusM)
                             ? $"  [wrong scene — {km:F0} km]"
                             : $"  ({e.File.DurationS:F0} s)";
                }
                var label = $"{e.SiteName}/{Path.GetFileNameWithoutExtension(e.FileName)}{suffix}";
                var captured = e;
                menu.AddItem(new GUIContent(label), Matches(e, player.ReplayFileName),
                             () => Assign(player, captured.FileName));
            }
            menu.ShowAsContext();
        }

        static void Assign(ReplayPlayer player, string fileName)
        {
            Undo.RecordObject(player, "Set replay file");
            player.ReplayFileName = fileName;
            EditorUtility.SetDirty(player);
        }

        static string Rel(string abs)
        {
            var i = abs.Replace("\\", "/").IndexOf("/Assets/", StringComparison.Ordinal);
            return i >= 0 ? abs.Substring(i + 1) : abs;
        }

        /// <summary>
        /// Reparse only what changed. Keyed on each file's last-write time, so re-exporting a
        /// bag refreshes that entry and nothing else — and an unchanged 7 MB file is never
        /// parsed twice for an inspector redraw.
        /// </summary>
        static void Rescan(bool force)
        {
            if (force) _cache.Clear();

            var found = ReplayLoader.FindReplays();
            var live = new HashSet<string>(found);
            foreach (var gone in _cache.Keys.Where(k => !live.Contains(k)).ToList())
                _cache.Remove(gone);

            foreach (var path in found)
            {
                DateTime stamp;
                try { stamp = System.IO.File.GetLastWriteTimeUtc(path); }
                catch { continue; }

                if (_cache.TryGetValue(path, out var have) && have.Stamp == stamp) continue;

                var e = new Entry { Path = path, FileName = Path.GetFileName(path), Stamp = stamp };
                // One parser for the whole system: the runtime loader, with its format and
                // version refusals, is what decides whether a file is readable here too.
                e.File = ReplayLoader.Load(path, out var err);
                e.Error = err;
                _cache[path] = e;
            }

            _entries = _cache.Values
                             .OrderBy(e => e.SiteName)
                             .ThenBy(e => e.FileName)
                             .ToList();
        }
    }
}
