using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

namespace Smarc.Environment
{
    /// <summary>
    /// LIVE MARINE TRAFFIC from AIS (Ivan, 2026-09-23): every ship aisstream.io reports around the
    /// site becomes a moving hull SCALED TO ITS AIS DIMENSIONS.
    ///
    /// Source: OCEANVERSE `ovsite.traffic live` writes &lt;bundle&gt;/live/traffic.json every 2 s (the
    /// console's `traffic live`). This component re-reads it when it changes — no network code in
    /// Unity, no API key anywhere near the scene.
    ///
    /// Hull: length = AIS A + B, beam = C + D (metres from the antenna to bow / stern / port /
    /// starboard), draught from the ship's static data; placed so the ANTENNA is at the reported
    /// position (the hull centre is offset by the A/B/C/D asymmetry). A ship that has not sent its
    /// static data yet gets a default size for its type — the file says "assumed", and the hull's
    /// name ends in "(size assumed)". Shape: a box stern, pointed bow over the last 15 %, a
    /// superstructure block on anything ≥ 25 m. Kinematic, with a BoxCollider carrying the Steel
    /// physics material, so sonars and cameras see it (and a vehicle can hit it).
    ///
    /// Motion: between AIS fixes the hull is DEAD-RECKONED along COG at SOG (grid bearings: the
    /// Unity z axis is UTM grid north), for at most MaxDeadReckon_s; corrections are blended, not
    /// jumped. A stale ship (no fix for 3 min) stops; a dropped one (15 min) is removed.
    ///
    /// Time: TIME DRIVES TRAFFIC — while the scene's ScenarioClock is at now (Live mode, or a window
    /// scrubbed to now) the ships are LIVE (aisstream); while it is in the past they are REPLAYED from
    /// the archive (<bundle>/live/replay.json, MarineCadastre, `traffic replay`), interpolated along
    /// their tracks at the clock's time. A future clock has no ships (only a live ship's dead reckoning).
    ///
    /// SCENARIO KNOB, OFF BY DEFAULT: moving steel near the vehicle changes what its sonar sees and
    /// can collide with it — same discipline as CurrentField.
    ///
    /// EDIT-MODE PREVIEW (Ivan, 2026-09-23: "wouldn't it make sense if the traffic is displayed before
    /// play?"): with the knob on, the hulls are shown and updated in the Scene view without Play. They
    /// are temporary (HideFlags.DontSave) — never written into the scene file — and are rebuilt when
    /// Play starts.
    /// </summary>
    [ExecuteAlways]
    [AddComponentMenu("Smarc/Environment/AIS Traffic (live)")]
    public class AisTrafficLive : MonoBehaviour
    {
        [Header("SCENARIO KNOB — off by default")]
        public bool LiveTrafficEnabled = false;
        [Tooltip("<bundle>/live/traffic.json — written by `ovsite.traffic live` (console: traffic live)")]
        public string TrafficFile = "";
        [Min(0.5f)] public float PollSeconds = 2f;
        [Tooltip("Show and update the hulls in the Scene view without Play (temporary objects, never saved)")]
        public bool PreviewInEditMode = true;

        [Header("Motion")]
        public bool DeadReckon = true;
        [Min(0f)] public float MaxDeadReckon_s = 180f;
        [Tooltip("Seconds to blend a hull onto a new fix (larger = smoother, lags more)")]
        [Min(0.1f)] public float CorrectionTau_s = 1.5f;
        [Tooltip("Only ships inside this radius of the scene origin, m (0 = all in the file)")]
        [Min(0f)] public float MaxRange_m = 0f;

        [Header("Physics")]
        public bool Colliders = true;
        public PhysicsMaterial HullPhysicsMaterial;

        [Tooltip("The scene's clock: LIVE ships while it is at now; archived (REPLAY) ships while it is in the past.")]
        public ScenarioClock Clock;
        [Tooltip("<bundle>/live/replay.json — written by `ovsite.traffic replay` (console: traffic replay)")]
        public string ReplayFile = "";

        [Header("NOW (read-only)")]
        public int VesselsShown;
        public string FeedWrittenUtc = "";
        [TextArea(3, 10)] public string StateNow = "";

        // ---- traffic.json (ovsite.traffic, TRAFFIC_VERSION 1.0) ------------------------------
        [Serializable] class XY { public float x, y; }
        [Serializable] class RefJ { public float forward, starboard; }
        [Serializable] class HullJ { public float length_m, beam_m, draught_m; public string size_source, draught_source; public RefJ ref_point_from_centre_m; }
        [Serializable] class VesselJ
        {
            public long mmsi; public string name, type_word, fix_utc, nav_status; public float age_s; public bool stale, inside_site;
            public float sog_ms, cog_grid_deg, heading_grid_deg; public XY enu; public HullJ hull;
        }
        [Serializable] class TrafficJ { public string written_utc, source; public bool connected; public int vessels_count; public VesselJ[] vessels; }

        class Ship
        {
            public GameObject go; public VesselJ v; public DateTime fix; public Vector3 antenna0; public float heading;
            public string sizeKey; public Vector3 shown; public bool hasShown;
        }

        const float Missing = -999f;
        readonly Dictionary<long, Ship> ships = new Dictionary<long, Ship>();
        readonly Dictionary<string, Mesh> meshCache = new Dictionary<string, Mesh>();
        readonly Dictionary<string, Material> matCache = new Dictionary<string, Material>();
        DateTime lastWrite = DateTime.MinValue;
        float nextPoll;
        string feedNote = "";

        // ---- real time that advances in edit mode too (Time.* does not outside Play) ----------
        static float RealTime()
        {
#if UNITY_EDITOR
            if (!Application.isPlaying) return (float)UnityEditor.EditorApplication.timeSinceStartup;
#endif
            return Time.unscaledTime;
        }
        float lastTick = -1f, tickDt;

        static void Kill(UnityEngine.Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) Destroy(o); else DestroyImmediate(o);
        }

        void OnEnable()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.update -= EditorTick;
            UnityEditor.EditorApplication.update += EditorTick;
#endif
            mode = ""; lastWrite = DateTime.MinValue; replayWrite = DateTime.MinValue; nextPoll = 0f; nextReplayCheck = 0f; lastTick = -1f;
            // a preview hull that outlived its component (DontSave objects survive a scene reload): sweep it
            var orphans = new List<GameObject>();
            foreach (Transform c in transform)
                if ((c.gameObject.hideFlags & HideFlags.DontSave) != 0 && c.name.StartsWith("AIS ")) orphans.Add(c.gameObject);
            foreach (var o in orphans) Kill(o);
        }

        void OnDisable()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.update -= EditorTick;
#endif
            Clear();
            foreach (var m in meshCache.Values) Kill(m);
            foreach (var m in matCache.Values) Kill(m);
            meshCache.Clear(); matCache.Clear();
        }

#if UNITY_EDITOR
        // edit mode: Update only runs when something changes — drive the preview from the editor loop
        float nextEditorTick;
        void EditorTick()
        {
            if (Application.isPlaying || this == null || !isActiveAndEnabled) return;
            if (!PreviewInEditMode) { if (ships.Count > 0) { Clear(); StateNow = "edit-mode preview off (Preview In Edit Mode) — press Play"; } return; }
            float now = RealTime();
            if (now < nextEditorTick) return;
            nextEditorTick = now + 0.1f;                            // 10 Hz is plenty for a preview
            Tick();
            if (ships.Count > 0) UnityEditor.SceneView.RepaintAll();
        }
#endif

        void Clear()
        {
            foreach (var s in ships.Values) if (s.go != null) Kill(s.go);
            ships.Clear();
            VesselsShown = 0;
        }

        void Update()
        {
            if (!Application.isPlaying) return;                    // edit mode: EditorTick drives the preview
            Tick();
        }

        void Tick()
        {
            float now = RealTime();
            tickDt = lastTick < 0 ? 0f : Mathf.Clamp(now - lastTick, 0f, 1f);
            lastTick = now;
            if (!LiveTrafficEnabled) { if (ships.Count > 0) Clear(); StateNow = "OFF (tick Live Traffic Enabled)"; return; }
            // TIME DRIVES TRAFFIC: live ships exist only while the scene's clock is at now
            if (Clock == null) Clock = ScenarioClock.Find();
            if (Clock != null && !Clock.IsNow)
            {
                // the clock is in the past (or future): PAST ships from the archive replay, if it covers this time
                if (mode != "replay") { Clear(); mode = "replay"; }
                ReplayTick();
                return;
            }
            if (mode != "live") { Clear(); mode = "live"; lastWrite = DateTime.MinValue; }
            if (RealTime() >= nextPoll) { nextPoll = RealTime() + PollSeconds; Poll(); }
            Move();
        }

        // ---- REPLAY: <bundle>/live/replay.json (ovsite.traffic replay — MarineCadastre archive) ------------
        [Serializable] class WindowJ { public string start_utc, end_utc; }
        [Serializable] class TrackJ { public float[] t_s, x, y, sog_ms, cog_grid_deg, heading_grid_deg; }
        [Serializable] class ReplayVesselJ { public long mmsi; public string name, type_word; public HullJ hull; public TrackJ track; }
        [Serializable] class ReplayJ { public string source, license; public WindowJ window; public float max_track_gap_s = 900f; public int vessels_count; public ReplayVesselJ[] vessels; }

        string mode = "";
        ReplayJ replay;
        DateTime replayWrite = DateTime.MinValue, replayStart;
        float nextReplayCheck;

        void ReplayTick()
        {
            if (RealTime() >= nextReplayCheck)
            {
                nextReplayCheck = RealTime() + 5f;
                if (string.IsNullOrEmpty(ReplayFile) || !File.Exists(ReplayFile)) { replay = null; }
                else
                {
                    var wt = File.GetLastWriteTimeUtc(ReplayFile);
                    if (wt != replayWrite)
                    {
                        replayWrite = wt; Clear();
                        try
                        {
                            replay = JsonUtility.FromJson<ReplayJ>(File.ReadAllText(ReplayFile));
                            if (replay == null || replay.window == null || !ScenarioClock.TryUtc(replay.window.start_utc, out replayStart)) replay = null;
                        }
                        catch (Exception e) { replay = null; feedNote = "unreadable replay.json: " + e.Message; }
                    }
                }
            }
            if (replay == null)
            {
                if (ships.Count > 0) Clear();
                StateNow = $"the ScenarioClock is at {Clock.ScenarioUtc} ({Clock.Epoch}): no live AIS then, and no replay file — " +
                           "for PAST ships run the console's `traffic replay` (MarineCadastre archive, 2024), then export.";
                return;
            }
            double t = (Clock.Utc - replayStart).TotalSeconds;
            int shown = 0;
            foreach (var rv in replay.vessels ?? new ReplayVesselJ[0])
            {
                var tr = rv.track;
                if (tr == null || tr.t_s == null || tr.t_s.Length == 0 || rv.hull == null) continue;
                if (!ReplayPose(tr, (float)t, replay.max_track_gap_s, out var pos, out var hdg))
                {
                    if (ships.TryGetValue(rv.mmsi, out var hid) && hid.go != null) hid.go.SetActive(false);
                    continue;
                }
                if (!ships.TryGetValue(rv.mmsi, out var s))
                {
                    s = new Ship { v = new VesselJ { mmsi = rv.mmsi, name = rv.name, type_word = rv.type_word, hull = rv.hull } };
                    ships[rv.mmsi] = s;
                }
                if (!float.IsNaN(hdg)) s.heading = hdg;
                string key = $"{rv.hull.length_m:F0}x{rv.hull.beam_m:F0}x{rv.hull.draught_m:F1}";
                if (s.go == null || s.sizeKey != key)
                {
                    Rebuild(s, key);
                    s.go.name = $"AIS {rv.mmsi} {(string.IsNullOrEmpty(rv.name) ? "?" : rv.name)} — {rv.type_word} {rv.hull.length_m:F0}x{rv.hull.beam_m:F0} m (REPLAY)" +
                                ((rv.hull.size_source ?? "").StartsWith("assumed") ? " (size assumed)" : "");
                }
                if (!s.go.activeSelf) s.go.SetActive(true);
                s.go.transform.SetPositionAndRotation(pos, Quaternion.Euler(0f, s.heading, 0f));   // antenna = centre (archive has no A/B/C/D)
                shown++;
            }
            VesselsShown = shown;
            StateNow = $"REPLAY {Clock.ScenarioUtc} ({Clock.Epoch}): {shown} of {replay.vessels_count} archived ship(s) at this time — {replay.source}. " +
                       $"Window {replay.window.start_utc} → {replay.window.end_utc}." + (feedNote.Length > 0 ? "\n" + feedNote : "");
        }

        /// Heading if reported, else course over ground, else NaN (grid degrees).
        static float TrackHeading(TrackJ tr, int k)
        {
            if (tr.heading_grid_deg != null && k < tr.heading_grid_deg.Length && tr.heading_grid_deg[k] > Missing + 1) return tr.heading_grid_deg[k];
            if (tr.cog_grid_deg != null && k < tr.cog_grid_deg.Length && tr.cog_grid_deg[k] > Missing + 1) return tr.cog_grid_deg[k];
            return float.NaN;
        }

        /// Position/heading on a track at t (s after the window start): linear between two points no
        /// further apart than maxGap; the nearest point within 60 s at the ends; otherwise not there.
        static bool ReplayPose(TrackJ tr, float t, float maxGap, out Vector3 pos, out float hdg)
        {
            pos = Vector3.zero; hdg = float.NaN;
            var ts = tr.t_s; int n = ts.Length;
            int i = Array.BinarySearch(ts, t);
            if (i < 0) i = ~i;                                  // first index with ts[i] > t
            int a = i - 1, b = i < n ? i : -1;
            if (i < n && ts[i] == t) { a = i; b = i; }
            if (a >= 0 && b >= 0 && ts[b] - ts[a] <= maxGap)
            {
                float f = ts[b] > ts[a] ? (t - ts[a]) / (ts[b] - ts[a]) : 0f;
                pos = new Vector3(Mathf.Lerp(tr.x[a], tr.x[b], f), 0f, Mathf.Lerp(tr.y[a], tr.y[b], f));
                float ha = TrackHeading(tr, a), hb = TrackHeading(tr, b);
                hdg = float.IsNaN(ha) ? hb : float.IsNaN(hb) ? ha : Mathf.Repeat(ha + Mathf.DeltaAngle(ha, hb) * f, 360f);
                return true;
            }
            int k2 = a >= 0 && (b < 0 || t - ts[a] <= ts[b] - t) ? a : b;
            if (k2 >= 0 && Mathf.Abs(ts[k2] - t) <= 60f)
            {
                pos = new Vector3(tr.x[k2], 0f, tr.y[k2]); hdg = TrackHeading(tr, k2);
                return true;
            }
            return false;
        }

        void Poll()
        {
            if (string.IsNullOrEmpty(TrafficFile) || !File.Exists(TrafficFile))
            {
                feedNote = $"no live file at '{TrafficFile}' — start it: site-selector console `traffic live`";
                StateNow = feedNote; return;
            }
            var wt = File.GetLastWriteTimeUtc(TrafficFile);
            double ageFile = (DateTime.UtcNow - wt).TotalSeconds;
            feedNote = ageFile > 30 ? $"FEED STOPPED? the live file is {ageFile:F0} s old (console: traffic)" : "";
            if (wt == lastWrite) { Status(); return; }
            lastWrite = wt;
            string txt;
            try { txt = File.ReadAllText(TrafficFile); } catch (IOException) { return; }   // mid-replace: next poll
            // JsonUtility has no null for numbers: unavailable COG / heading / SOG -> the -999 sentinel
            txt = Regex.Replace(txt, "\"(sog_ms|cog_true_deg|heading_true_deg|cog_grid_deg|heading_grid_deg|type|imo|age_s)\": null", "\"$1\": -999");
            TrafficJ d;
            try { d = JsonUtility.FromJson<TrafficJ>(txt); } catch (Exception e) { feedNote = "unreadable traffic.json: " + e.Message; return; }
            if (d?.vessels == null) return;
            FeedWrittenUtc = d.written_utc;
            var seen = new HashSet<long>();
            foreach (var v in d.vessels)
            {
                if (v.enu == null || v.hull == null) continue;
                var antenna = new Vector3(v.enu.x, 0f, v.enu.y);     // Unity x = ENU east, z = ENU grid north, y = 0 sea surface
                if (MaxRange_m > 0 && new Vector2(antenna.x, antenna.z).magnitude > MaxRange_m) continue;
                seen.Add(v.mmsi);
                if (!ships.TryGetValue(v.mmsi, out var s)) { s = new Ship(); ships[v.mmsi] = s; }
                s.v = v;
                s.antenna0 = antenna;
                s.fix = DateTime.TryParse(v.fix_utc, null, System.Globalization.DateTimeStyles.AdjustToUniversal |
                                          System.Globalization.DateTimeStyles.AssumeUniversal, out var f) ? f : DateTime.UtcNow;
                float hdg = v.heading_grid_deg > Missing + 1 ? v.heading_grid_deg
                          : (v.cog_grid_deg > Missing + 1 && v.sog_ms > 0.5f ? v.cog_grid_deg : s.heading);
                s.heading = hdg;
                string key = $"{v.hull.length_m:F0}x{v.hull.beam_m:F0}x{v.hull.draught_m:F1}";
                if (s.go == null || s.sizeKey != key) Rebuild(s, key);
                s.go.name = $"AIS {v.mmsi} {(string.IsNullOrEmpty(v.name) ? "?" : v.name)} — {v.type_word} {v.hull.length_m:F0}x{v.hull.beam_m:F0} m" +
                            ((v.hull.size_source ?? "").StartsWith("assumed") ? " (size assumed)" : "");
            }
            var gone = new List<long>();
            foreach (var k in ships.Keys) if (!seen.Contains(k)) gone.Add(k);
            foreach (var k in gone) { if (ships[k].go != null) Kill(ships[k].go); ships.Remove(k); }
            VesselsShown = ships.Count;
            Status();
        }

        void Status()
        {
            int assumed = 0, stale = 0;
            foreach (var s in ships.Values) { if ((s.v.hull.size_source ?? "").StartsWith("assumed")) assumed++; if (s.v.stale) stale++; }
            string timeNote = Clock == null ? "\n(no ScenarioClock in the scene: shown regardless of scenario time)" : "";
            StateNow = $"{VesselsShown} ship(s) live ({assumed} with assumed size, {stale} stale), feed {FeedWrittenUtc}" +
                       (feedNote.Length > 0 ? "\n" + feedNote : "") + timeNote;
        }

        void Move()
        {
            var now = DateTime.UtcNow;
            float k = 1f - Mathf.Exp(-tickDt / CorrectionTau_s);
            foreach (var s in ships.Values)
            {
                if (s.go == null) continue;
                var v = s.v;
                float dt = (float)(now - s.fix).TotalSeconds;
                var ant = s.antenna0;
                if (DeadReckon && !v.stale && v.sog_ms > 0.05f && v.cog_grid_deg > Missing + 1)
                {
                    float c = v.cog_grid_deg * Mathf.Deg2Rad;
                    ant += new Vector3(Mathf.Sin(c), 0f, Mathf.Cos(c)) * v.sog_ms * Mathf.Clamp(dt, 0f, MaxDeadReckon_s);
                }
                // the antenna sits A/B/C/D-asymmetrically on the hull: hull centre = antenna - fwd*F - right*S
                float h = s.heading * Mathf.Deg2Rad;
                var fwd = new Vector3(Mathf.Sin(h), 0f, Mathf.Cos(h));
                var right = new Vector3(Mathf.Cos(h), 0f, -Mathf.Sin(h));
                var r = v.hull.ref_point_from_centre_m;
                var centre = ant - fwd * (r != null ? r.forward : 0f) - right * (r != null ? r.starboard : 0f);
                if (!s.hasShown || (centre - s.shown).magnitude > 200f) { s.shown = centre; s.hasShown = true; }
                else s.shown = Vector3.Lerp(s.shown, centre, k);
                s.go.transform.SetPositionAndRotation(s.shown,
                    Quaternion.Slerp(s.go.transform.rotation, Quaternion.Euler(0f, s.heading, 0f), s.hasShown ? k : 1f));
            }
        }

        // ---- the hull -----------------------------------------------------------------------
        void Rebuild(Ship s, string key)
        {
            var v = s.v;
            if (s.go != null) Kill(s.go);
            float L = Mathf.Max(2f, v.hull.length_m), B = Mathf.Max(1f, v.hull.beam_m), T = Mathf.Max(0.3f, v.hull.draught_m);
            float F = Mathf.Clamp(0.08f * L + 0.5f, 0.8f, 14f);                 // freeboard, m (ASSUMED shape rule)
            var go = new GameObject("AIS " + v.mmsi);
            if (!Application.isPlaying) go.hideFlags = HideFlags.DontSave;     // edit-mode preview: never saved into the scene
            go.transform.SetParent(transform, false);
            go.transform.rotation = Quaternion.Euler(0f, s.heading, 0f);
            if (!meshCache.TryGetValue(key, out var mesh)) { mesh = HullMesh(L, B, T, F); mesh.hideFlags = HideFlags.DontSave; meshCache[key] = mesh; }
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = TypeMaterial(v.type_word);
            if (Colliders)
            {
                var rb = go.AddComponent<Rigidbody>();
                rb.isKinematic = true; rb.useGravity = false;
                var bc = go.AddComponent<BoxCollider>();
                bc.center = new Vector3(0f, (F - T) / 2f, 0f);
                bc.size = new Vector3(B, F + T, L);
                if (HullPhysicsMaterial != null) bc.sharedMaterial = HullPhysicsMaterial;   // sharedMaterial: the sonar reads its NAME
            }
            s.go = go; s.sizeKey = key; s.hasShown = false;
        }

        /// Local frame: +z bow, +x starboard, +y up; keel at y = -T, deck at y = +F. Flat-shaded.
        static Mesh HullMesh(float L, float B, float T, float F)
        {
            var verts = new List<Vector3>(); var tris = new List<int>();
            float hb = B / 2f, zs = -L / 2f, zb = L / 2f, zk = zb - 0.15f * L;     // stern, bow tip, where the bow starts
            var ring = new[] { new Vector2(-hb, zs), new Vector2(hb, zs), new Vector2(hb, zk), new Vector2(0f, zb), new Vector2(-hb, zk) };
            Prism(verts, tris, ring, -T, F);
            if (L >= 25f)                                                          // superstructure (bridge house), aft
            {
                float w = 0.7f * hb, z0 = zs + 0.08f * L, z1 = zs + 0.25f * L, hgt = Mathf.Clamp(0.06f * L, 3f, 15f);
                Prism(verts, tris, new[] { new Vector2(-w, z0), new Vector2(w, z0), new Vector2(w, z1), new Vector2(-w, z1) }, F, F + hgt);
            }
            var m = new Mesh { name = $"AIS hull {L:F0}x{B:F0}x{T:F1}" };
            m.SetVertices(verts); m.SetTriangles(tris, 0);
            m.RecalculateNormals(); m.RecalculateBounds();
            return m;
        }

        /// A convex footprint (counter-clockwise seen from above, in x,z) extruded from y0 to y1,
        /// faces wound so Unity renders them from outside.
        static void Prism(List<Vector3> V, List<int> Tr, Vector2[] ring, float y0, float y1)
        {
            int n = ring.Length;
            for (int k = 1; k < n - 1; k++)
            {
                Tri(V, Tr, P(ring[0], y1), P(ring[k], y1), P(ring[k + 1], y1), Vector3.up);
                Tri(V, Tr, P(ring[0], y0), P(ring[k], y0), P(ring[k + 1], y0), Vector3.down);
            }
            var c2 = Vector2.zero; foreach (var p in ring) c2 += p; c2 /= n;
            for (int k = 0; k < n; k++)
            {
                var a = ring[k]; var b = ring[(k + 1) % n];
                var mid = (a + b) / 2f - c2; var outward = new Vector3(mid.x, 0f, mid.y);   // convex: away from the centroid
                Tri(V, Tr, P(a, y0), P(b, y0), P(b, y1), outward);
                Tri(V, Tr, P(a, y0), P(b, y1), P(a, y1), outward);
            }
        }

        static Vector3 P(Vector2 p, float y) { return new Vector3(p.x, y, p.y); }

        static void Tri(List<Vector3> V, List<int> Tr, Vector3 a, Vector3 b, Vector3 c, Vector3 outward)
        {
            if (Vector3.Dot(Vector3.Cross(b - a, c - a), outward) < 0) { var t = b; b = c; c = t; }
            int i = V.Count; V.Add(a); V.Add(b); V.Add(c); Tr.Add(i); Tr.Add(i + 1); Tr.Add(i + 2);
        }

        Material TypeMaterial(string word)
        {
            word = word ?? "unknown type";
            if (matCache.TryGetValue(word, out var m)) return m;
            var sh = Shader.Find("HDRP/Lit") ?? Shader.Find("Standard");
            m = new Material(sh) { name = "AIS " + word, hideFlags = HideFlags.DontSave };
            Color c = word.StartsWith("cargo") ? new Color(0.55f, 0.20f, 0.15f) : word.StartsWith("tanker") ? new Color(0.35f, 0.10f, 0.10f)
                    : word.StartsWith("passenger") ? new Color(0.92f, 0.92f, 0.90f) : word.StartsWith("fishing") ? new Color(0.95f, 0.50f, 0.10f)
                    : word.StartsWith("sailing") ? new Color(0.97f, 0.97f, 0.97f) : word.StartsWith("pleasure") ? new Color(0.80f, 0.82f, 0.85f)
                    : word.StartsWith("pilot") ? new Color(0.10f, 0.10f, 0.10f) : word.StartsWith("towing") ? new Color(0.15f, 0.15f, 0.15f)
                    : new Color(0.50f, 0.52f, 0.55f);
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c); else m.color = c;
            matCache[word] = m;
            return m;
        }
    }
}
