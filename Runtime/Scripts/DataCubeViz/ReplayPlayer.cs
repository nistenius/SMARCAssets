using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using GeoRef;
using UnityEngine;

namespace DataCubeViz
{
    /// <summary>Which way `depth` counts in the loaded bag. Stated, never inferred silently.</summary>
    public enum DepthSign
    {
        /// <summary>SMaRC convention: `smarc/depth` is positive going down. 2.0 means 2 m under.</summary>
        PositiveIsDown,
        /// <summary>Odometry-z convention: negative going down, as `dr/odom` pose z reports it.</summary>
        NegativeIsDown,
    }

    /// <summary>
    /// Loads a `.dcreplay.json` and turns it into world positions in THIS scene.
    ///
    /// WHAT THIS COMPONENT IS RESPONSIBLE FOR, AND WHAT IT REFUSES
    /// ----------------------------------------------------------
    /// Responsible for: the clock, the file, and lat/lon → Unity X/Z.
    ///
    /// It does that conversion through the scene's own <see cref="GlobalReferencePoint"/> and
    /// nowhere else. That is the whole reason the exporter ships lat/lon instead of metres. The
    /// terrain, the station, and every GeoReference marker in these scenes were placed with that
    /// component's UTM (or Web Mercator) projection; a private equirectangular shortcut here
    /// would be a second projection disagreeing with the first by the meridian convergence —
    /// 5.104° at Kristineberg, ~2.5° at Beckholmen. Over a 620 m mission that is tens of metres
    /// of quiet, plausible error, and it would look like a navigation problem.
    ///
    /// Refuses: to run without a GlobalReferencePoint. There is no fallback origin, because a
    /// guessed origin draws a track that is confidently in the wrong place, and this project has
    /// already paid for one hardcoded position that read as measured. No reference point ⇒ the
    /// component disables itself and says which object is missing.
    /// </summary>
    [AddComponentMenu("DataCube/Replay Player")]
    public class ReplayPlayer : MonoBehaviour
    {
        [Header("Replay file")]
        [Tooltip("File name inside StreamingAssets/DataCubeReplays, or an absolute path. " +
                 "Leave empty to load the first replay found there.")]
        public string ReplayFileName = "";

        [Tooltip("Load on Start. Turn off to load from the panel instead.")]
        public bool LoadOnStart = true;

        [Header("Depth")]
        [Tooltip("Which way the exported depth channel counts. The console prints the observed " +
                 "range at load so you can check this rather than assume it. IGNORED when the " +
                 "exporter already normalised the sign (depth_sign_normalised) — the file's " +
                 "measurement beats this setting.")]
        public DepthSign DepthConvention = DepthSign.PositiveIsDown;

        [Tooltip("When a bag's depth topic carried TWO publishers in opposite sign conventions, " +
                 "drive depth from the SECOND chain instead of the first. Both are normalised " +
                 "to positive-is-down; on the showcase bag they agree to 0.043 m. Use this to " +
                 "see whether the two publishers really do agree, rather than being told.")]
        public bool UseAlternateDepthChain = false;

        [Tooltip("Unity Y of the water surface in this scene. Depth is measured down from here.")]
        public float WaterLevelY = 0f;

        [Header("Site gate")]
        [Tooltip("How far a replay's site may be from this scene's GlobalReferencePoint before " +
                 "the player REFUSES to draw it. 2 km is bounded by how large an operational " +
                 "area actually is (acoustic reach 0.8 km; the showcase run's centroid is 67 m " +
                 "from its origin), NOT by how far apart the registered sites are — that " +
                 "reasoning gave 20 km and labelled a KTH tank test 3.9 km away as a " +
                 "Beckholmen dry-dock run.")]
        public double SiteMatchRadiusM = 2000.0;

        [Header("Playback")]
        public bool AutoPlay = false;
        [Range(0.05f, 20f)] public float Speed = 1f;
        public bool Loop = true;

        public ReplayClock Clock { get; } = new ReplayClock();
        public ReplayFile Data { get; private set; }
        public string LoadError { get; private set; }
        public string LoadedPath { get; private set; }
        public bool Ready => Data != null && _globalRef != null;

        /// <summary>Raised after a successful load, so renderers can rebuild.</summary>
        public event Action<ReplayFile> Loaded;

        GlobalReferencePoint _globalRef;
        readonly Dictionary<string, Vector3[]> _worldCache = new();

        void Awake()
        {
            _globalRef = FindFirstObjectByType<GlobalReferencePoint>();
            if (_globalRef == null)
            {
                LoadError = "No GlobalReferencePoint in this scene. The replay cannot be placed " +
                            "without one and will NOT be drawn at a guessed origin. Open a " +
                            "georeferenced scene (Beckholmen / Kristineberg / Asko), or add a " +
                            "GlobalReferencePoint and set its Lat/Lon.";
                Debug.LogError($"[DataCubeViz] {LoadError}", this);
                enabled = false;
            }
        }

        void Start()
        {
            Clock.Speed = Speed;
            Clock.Loop = Loop;
            if (LoadOnStart) Load(ReplayFileName);
        }

        // Cached world points are only valid for the geometry settings that built them. Toggling
        // DepthConvention or the alternate chain in the Inspector mid-Play must rebuild them, or
        // the scene keeps showing the old interpretation while the Inspector claims the new one
        // — a display that disagrees with its own controls, which is worse than either answer.
        DepthSign _cachedConvention;
        bool _cachedAltChain;
        float _cachedWaterLevel;

        void Update()
        {
            if (_cachedConvention != DepthConvention || _cachedAltChain != UseAlternateDepthChain
                || !Mathf.Approximately(_cachedWaterLevel, WaterLevelY))
            {
                _cachedConvention = DepthConvention;
                _cachedAltChain = UseAlternateDepthChain;
                _cachedWaterLevel = WaterLevelY;
                _worldCache.Clear();
                if (Data != null) Loaded?.Invoke(Data);   // renderers rebuild from new geometry
            }
            Clock.Tick(Time.unscaledDeltaTime);
        }

        // ------------------------------------------------------------------------------------
        // loading
        // ------------------------------------------------------------------------------------

        /// <summary>
        /// Take the replay OUT of the scene: tracks, ghosts, transport — everything.
        ///
        /// Why this exists (Ivan, 2026-08-27): the panel collapsed itself for live sims, but a
        /// previously loaded replay kept its world-space tracks painted over the live scene —
        /// magenta polylines over Beckholmen with nothing on screen saying where they came from.
        /// Rendering follows DATA, not panel visibility, which is correct — so the way to clear
        /// the scene must be to unload the data, not to hide the panel. Firing Loaded(null)
        /// reuses the exact path every renderer already handles for a rebuild: the track
        /// renderer clears its lines on a null file, and a ghost with no track hides itself.
        ///
        /// NOTE: if the component has LoadOnStart + ReplayFileName set in the scene, the next
        /// Play re-loads it. Unload cannot edit the scene for you — clear ReplayFileName (or
        /// untick LoadOnStart) in the Inspector and save the scene to make that stick.
        /// </summary>
        public void Unload()
        {
            _worldCache.Clear();
            Data = null;
            LoadError = null;
            LoadedPath = null;
            Clock.Rewind();
            Loaded?.Invoke(null);
        }

        public bool Load(string fileNameOrPath)
        {
            _worldCache.Clear();
            Data = null;
            LoadError = null;

            string path = ResolvePath(fileNameOrPath, out var resolveError);
            if (path == null)
            {
                LoadError = resolveError;
                Debug.LogError($"[DataCubeViz] {LoadError}", this);
                return false;
            }

            var file = ReplayLoader.Load(path, out var err);
            if (file == null)
            {
                LoadError = err;
                Debug.LogError($"[DataCubeViz] {LoadError}", this);
                return false;
            }

            // THE SITE GATE. Before anything is drawn: is this replay even at this place?
            if (!SiteMatchesScene(file, path, out var siteError))
            {
                LoadError = siteError;
                Debug.LogError($"[DataCubeViz] {LoadError}", this);
                Data = null;              // nothing is drawn, on purpose
                LoadedPath = path;        // but the panel can still say WHICH file was refused
                return false;
            }

            Data = file;
            LoadedPath = path;
            Clock.SetDuration(file.DurationS);
            Clock.Rewind();

            ReportLoad(file);

            if (AutoPlay) Clock.Play();
            Loaded?.Invoke(file);
            return true;
        }

        string ResolvePath(string nameOrPath, out string error)
        {
            error = null;
            if (!string.IsNullOrWhiteSpace(nameOrPath))
            {
                if (Path.IsPathRooted(nameOrPath) && File.Exists(nameOrPath)) return nameOrPath;
                var candidate = Path.Combine(ReplayLoader.DefaultDirectory, nameOrPath);
                if (File.Exists(candidate)) return candidate;
                if (!nameOrPath.EndsWith(".dcreplay.json"))
                {
                    candidate = Path.Combine(ReplayLoader.DefaultDirectory,
                                             nameOrPath + ".dcreplay.json");
                    if (File.Exists(candidate)) return candidate;
                }
                error = $"replay '{nameOrPath}' not found in {ReplayLoader.DefaultDirectory} " +
                        $"and is not an absolute path to an existing file.";
                return null;
            }

            var found = ReplayLoader.FindReplays();
            if (found.Length == 0)
            {
                error = $"no *.dcreplay.json in {ReplayLoader.DefaultDirectory}. Export one with " +
                        $"`python3 data-cube/scripts/export_bag_for_unity.py <bag-dir>`.";
                return null;
            }
            return found[0];
        }

        /// <summary>
        /// Say out loud what was loaded and what it lacks. Every warning the station produced
        /// travels to the console — a caveat that stops at the export boundary is a caveat
        /// nobody reads, and these are the ones that say a bag was truncated or that a db3's
        /// message definitions came from today's source tree rather than the recording.
        /// </summary>
        /// <summary>
        /// Does this replay belong in this scene? Refuse if not, and NAME THE DISTANCE.
        ///
        /// WHY REFUSING BEATS DRAWING
        /// --------------------------
        /// Drawing a foreign replay is not a visible error, it is an invisible one. Measured
        /// 2026-08-23: a Kristineberg bag in the Beckholmen scene is placed at Unity X=308583,
        /// Z=-120156 — the geometry is fine, float32 still resolves 3 cm out there, it is
        /// simply 331 km away. The operator sees an empty dock and a panel claiming seven
        /// tracks loaded. So this does what Mission Control already does when a plan is
        /// measured far from the active site: stop, and say which two places are being
        /// compared and how far apart they are (SYSTEMS_SPEC §8, 2026-08-17).
        ///
        /// The comparison is made in LAT/LON, deliberately, never in the scene's Unity metres.
        /// The reason the mismatch is dangerous in the first place is that the UTM subtraction
        /// silently crosses zones (a zone-32 easting minus a zone-34 easting), so measuring the
        /// error with the same broken arithmetic that causes it would report 331 km for a
        /// 403 km separation. Two lat/lon pairs have no zones.
        /// </summary>
        bool SiteMatchesScene(ReplayFile f, string path, out string error)
        {
            error = null;
            var site = f.Site;

            // Load() is public and the panel can call it, so this can be reached even though
            // Awake() disables the component when there is no reference point. Refuse rather
            // than throw: the message is the useful part either way.
            if (_globalRef == null)
            {
                error = "No GlobalReferencePoint in this scene — there is no origin to check " +
                        "this replay against, and it will not be placed at a guessed one.";
                return false;
            }

            if (site == null || !site.Matched)
            {
                error = $"{Path.GetFileName(path)}: the exporter could not identify which site " +
                        $"this bag is at" + (site?.Reason != null ? $" — {site.Reason}" : "") +
                        ". Refusing to draw it rather than placing it at a guessed origin. " +
                        "Add the site to maps/sites.yaml and re-export.";
                return false;
            }

            double sceneLat = _globalRef.Lat, sceneLon = _globalRef.Lon;
            double mLat = 111320.0;
            double mLon = 111320.0 * Math.Cos(sceneLat * Math.PI / 180.0);
            double dn = (site.Lat - sceneLat) * mLat;
            double de = (site.Lon - sceneLon) * mLon;
            double dist = Math.Sqrt(dn * dn + de * de);

            if (dist <= SiteMatchRadiusM) return true;

            error = $"REFUSED: this replay is at {site.Name} ({site.Lat:F5}, {site.Lon:F5}), but " +
                    $"this scene's GlobalReferencePoint is at ({sceneLat:F5}, {sceneLon:F5}) — " +
                    $"{dist / 1000.0:F1} km apart. Drawn, it would land ~{dist / 1000.0:F0} km " +
                    $"outside the scene and simply look like an empty world, so nothing is drawn. " +
                    $"Open the {site.Name} scene, or load a replay from this site.";
            return false;
        }

        void ReportLoad(ReplayFile f)
        {
            var mappable = f.Tracks.Count(t => t.Mappable);
            Debug.Log($"[DataCubeViz] site {f.Site?.Describe()} · run {f.Run?.Describe()}", this);
            Debug.Log($"[DataCubeViz] loaded {Path.GetFileName(LoadedPath)} — " +
                      $"{f.DurationS:F1} s, {mappable} mappable track(s) of {f.Tracks.Count}, " +
                      $"{f.Channels.Count} channels. Source bag: {f.Source?.Label ?? "?"} " +
                      $"({f.Source?.Kind}{(f.Source != null && f.Source.Truncated ? ", TRUNCATED" : "")})", this);

            foreach (var t in f.Tracks.Where(t => t.Mappable))
            {
                var depth = t.HasDepth ? t.DepthSource : "NONE — will draw at the surface";
                Debug.Log($"[DataCubeViz]   track {t.Key}: position={t.PositionSource}, " +
                          $"depth={depth}, heading={(t.HasHeading ? t.HeadingSource : "NONE")}", this);
            }

            // The depth-sign check: state the observed range so the inspector setting can be
            // verified rather than believed. A tank run near the surface is ambiguous either
            // way and is reported as such instead of being silently accepted.
            var withDepth = f.Tracks.FirstOrDefault(t => t.HasDepth);
            if (withDepth != null)
            {
                var vals = withDepth.DepthM.Where(v => v.HasValue).Select(v => v.Value).ToList();
                if (vals.Count > 0)
                {
                    double lo = vals.Min(), hi = vals.Max();
                    string verdict;
                    if (Math.Abs(hi - lo) < 0.25)
                        verdict = "range too small to tell which sign convention this is — " +
                                  "the vehicle barely changed depth; the setting is unverified";
                    else if (lo >= -0.5 && hi > 0.5)
                        verdict = "values are mostly POSITIVE — consistent with PositiveIsDown";
                    else if (hi <= 0.5 && lo < -0.5)
                        verdict = "values are mostly NEGATIVE — consistent with NegativeIsDown";
                    else
                        verdict = "values straddle zero — check this one by eye";
                    Debug.Log($"[DataCubeViz]   depth from {withDepth.DepthSource}: observed " +
                              $"{lo:F2}..{hi:F2}; setting is {DepthConvention}; {verdict}", this);
                }
            }

            foreach (var w in f.Warnings)
                Debug.LogWarning($"[DataCubeViz] bag warning: {w}", this);
        }

        // ------------------------------------------------------------------------------------
        // geometry
        // ------------------------------------------------------------------------------------

        /// <summary>
        /// All of a track's points in Unity world space, cached. Y comes from depth when the
        /// bag had depth; when it did not, the track sits at the water surface — and the
        /// renderer labels it as having no depth, so a flat line is never read as a flat dive.
        /// </summary>
        public Vector3[] WorldPoints(ReplayTrack track)
        {
            if (track == null || !track.Mappable || _globalRef == null) return Array.Empty<Vector3>();
            if (_worldCache.TryGetValue(track.Key, out var cached)) return cached;

            var pts = new Vector3[track.LonLat.Count];
            for (int i = 0; i < track.LonLat.Count; i++)
            {
                var ll = track.LonLat[i];
                // lon first, lat second -- the exporter writes GeoJSON order.
                var (x, z) = _globalRef.GetUnityXZFromLatLon(ll[1], ll[0]);
                pts[i] = new Vector3(x, DepthToY(track, i), z);
            }
            _worldCache[track.Key] = pts;
            return pts;
        }

        float DepthToY(ReplayTrack track, int i)
        {
            var series = (UseAlternateDepthChain && track.HasAltDepth) ? track.DepthAltM
                                                                       : track.DepthM;
            if (series == null || i >= series.Count) return WaterLevelY;
            var d = series[i];
            // A gap in the record is NOT a surfacing. The point sits at the water line because
            // that is where an unknown depth has to be drawn, and the panel says the samples
            // were missing -- it must never read as "the vehicle came up here".
            if (!d.HasValue) return WaterLevelY;

            // When the exporter split the channel by sign it already normalised both chains to
            // positive-is-down, and it did so from a measurement of the data. The inspector's
            // guess does not get to override a measurement.
            var down = track.DepthSignNormalised
                       || DepthConvention == DepthSign.PositiveIsDown ? d.Value : -d.Value;
            return WaterLevelY - (float)down;
        }

        /// <summary>Interpolated pose at the current clock time. `valid` is false before the
        /// track's first sample — the vehicle has no position yet, which is not the same as
        /// being at the origin.</summary>
        public bool PoseAt(ReplayTrack track, double t, out Vector3 position, out float headingDeg,
                           out bool headingIsCourse)
        {
            position = Vector3.zero;
            headingDeg = 0f;
            headingIsCourse = true;

            var pts = WorldPoints(track);
            if (pts.Length == 0 || track.T.Count == 0) return false;

            int i = ReplayMath.FloorIndex(track.T, t);
            if (i < 0) return false;                       // before the track starts
            if (i >= pts.Length - 1)
            {
                position = pts[Math.Min(i, pts.Length - 1)];
                headingDeg = HeadingAt(track, Math.Min(i, pts.Length - 1), pts, out headingIsCourse);
                return true;
            }

            double t0 = track.T[i], t1 = track.T[i + 1];
            float u = t1 > t0 ? (float)((t - t0) / (t1 - t0)) : 0f;
            position = Vector3.Lerp(pts[i], pts[i + 1], Mathf.Clamp01(u));
            headingDeg = HeadingAt(track, i, pts, out headingIsCourse);
            return true;
        }

        float HeadingAt(ReplayTrack track, int i, Vector3[] pts, out bool isCourse)
        {
            if (track.HasHeading && i < track.HeadingDeg.Count && track.HeadingDeg[i].HasValue)
            {
                isCourse = false;
                return (float)track.HeadingDeg[i].Value;
            }
            // Fall back to course over ground and SAY SO via the out-flag, so the panel can
            // label it. Course is not heading: a vehicle crabbing in a current points one way
            // and travels another, and drawing the ghost as if they were the same hides
            // exactly the thing a crosscurrent investigation is looking for.
            isCourse = true;
            int a = Mathf.Max(0, i - 1), b = Mathf.Min(pts.Length - 1, i + 1);
            var d = pts[b] - pts[a];
            if (d.sqrMagnitude < 1e-6f) return 0f;
            return Mathf.Atan2(d.x, d.z) * Mathf.Rad2Deg;   // Unity: +Z north, +X east
        }

        public ReplayTrack TrackByKey(string key) =>
            Data?.Tracks.FirstOrDefault(t => t.Key == key);

        public IEnumerable<ReplayTrack> MappableTracks =>
            Data?.Tracks.Where(t => t.Mappable) ?? Enumerable.Empty<ReplayTrack>();
    }
}
