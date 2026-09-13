using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using UnityEngine;

namespace DataCubeViz
{
    /// <summary>
    /// The `.dcreplay.json` payload, as written by `data-cube/scripts/export_bag_for_unity.py`.
    ///
    /// WHY THESE FIELDS AND NOT A ROS MESSAGE MIRROR
    /// ---------------------------------------------
    /// This is deliberately NOT a C# copy of the bag. The station's `bagreader.py` already did
    /// the decoding, the two-publisher split, and the per-frame rigid fit that turns a local
    /// odom frame into lat/lon without assuming plain ENU. Re-deriving any of that here would
    /// be a second implementation of a comparison, which is where divergence lives. What
    /// arrives is a decided, caveated result: positions in lat/lon, with the caveats attached.
    ///
    /// Every track carries `positionSource`. That is not decoration. This project has drawn a
    /// hardcoded constant as a measured fix (49.4 m wrong), and has read Unity's own ground
    /// truth off a topic and reported it as the estimator's. A position that cannot name where
    /// it came from does not get rendered.
    /// </summary>
    [Serializable]
    public class ReplayFile
    {
        [JsonProperty("format")] public string Format;
        [JsonProperty("version")] public int Version;
        [JsonProperty("duration_s")] public double DurationS;
        [JsonProperty("source")] public ReplaySource Source = new();
        /// <summary>Which registered site this bag is AT, measured from its own coordinates
        /// against `maps/sites.yaml` at export time — never typed in.</summary>
        [JsonProperty("site")] public ReplaySite Site = new();
        [JsonProperty("run")] public ReplayRun Run = new();
        /// <summary>Present only when the bag's depth topic carried two publishers in opposite
        /// sign conventions. Carries the measurement, not just the claim.</summary>
        [JsonProperty("depth_conflict")] public DepthConflict DepthConflict;
        [JsonProperty("warnings")] public List<string> Warnings = new();
        [JsonProperty("tracks")] public List<ReplayTrack> Tracks = new();
        [JsonProperty("channels")] public List<ReplayChannel> Channels = new();
        [JsonProperty("events")] public List<ReplayEventLane> Events = new();

        public const string ExpectedFormat = "datacube-replay";
        public const int SupportedVersion = 1;
    }

    [Serializable]
    public class ReplaySource
    {
        [JsonProperty("bag")] public string Bag;
        [JsonProperty("name")] public string Name;
        [JsonProperty("label")] public string Label;
        [JsonProperty("kind")] public string Kind;
        [JsonProperty("truncated")] public bool Truncated;
        [JsonProperty("message_count")] public long MessageCount;
        [JsonProperty("exported_at")] public string ExportedAt;
    }

    [Serializable]
    public class ReplayTrack
    {
        [JsonProperty("key")] public string Key;
        [JsonProperty("label")] public string Label;
        [JsonProperty("kind")] public string Kind;
        [JsonProperty("topic")] public string Topic;
        [JsonProperty("mappable")] public bool Mappable;
        [JsonProperty("count")] public int Count;
        [JsonProperty("position_source")] public string PositionSource;
        [JsonProperty("frame_id")] public string FrameId;
        [JsonProperty("note")] public string Note;
        [JsonProperty("deviation")] public ReplayDeviation Deviation;

        [JsonProperty("t")] public List<double> T = new();
        /// <summary>[lon, lat] pairs — LON FIRST, matching bagreader and GeoJSON. Getting this
        /// backwards puts the track in the Indian Ocean, which is at least obvious.</summary>
        [JsonProperty("lonlat")] public List<List<double>> LonLat = new();

        [JsonProperty("depth_m")] public List<double?> DepthM;
        [JsonProperty("depth_source")] public string DepthSource;
        [JsonProperty("depth_missing")] public int DepthMissing;

        /// <summary>The SECOND depth publisher, when the bag's depth topic carried two of them
        /// in opposite sign conventions (measured 2026-08-23 on the showcase bag: 84% of
        /// consecutive samples flip sign). Both chains arrive normalised to positive-is-down,
        /// so this is directly comparable to <see cref="DepthM"/> — and it is kept rather than
        /// merged, because averaging two publishers is how a disagreement becomes invisible.</summary>
        [JsonProperty("depth_alt_m")] public List<double?> DepthAltM;
        [JsonProperty("depth_alt_source")] public string DepthAltSource;
        [JsonProperty("depth_alt_missing")] public int DepthAltMissing;

        /// <summary>True when the exporter split the depth channel by sign. When true, DepthM
        /// is already positive-is-down and the player's DepthConvention setting is ignored —
        /// the file has a better answer than the inspector does.</summary>
        [JsonProperty("depth_sign_normalised")] public bool DepthSignNormalised;

        public bool HasAltDepth => DepthAltSource != null && DepthAltM != null && DepthAltM.Count > 0;

        [JsonProperty("heading_deg")] public List<double?> HeadingDeg;
        [JsonProperty("heading_source")] public string HeadingSource;
        [JsonProperty("heading_note")] public string HeadingNote;
        [JsonProperty("heading_missing")] public int HeadingMissing;

        public bool HasDepth => DepthSource != null && DepthM != null && DepthM.Count > 0;
        public bool HasHeading => HeadingSource != null && HeadingDeg != null && HeadingDeg.Count > 0;

        /// <summary>One line naming everything this track is and is not. Shown in the panel.</summary>
        public string Provenance()
        {
            var d = HasDepth ? DepthSource : "NO DEPTH — drawn at the surface";
            var h = HasHeading ? HeadingSource : "NO HEADING — facing along track";
            return $"position: {PositionSource}\ndepth: {d}\nheading: {h}";
        }
    }

    /// <summary>
    /// The site a replay belongs to, as measured by the exporter.
    ///
    /// This exists because the failure it prevents is SILENT. Measured 2026-08-23: a
    /// Kristineberg bag loaded into the Beckholmen scene is placed at Unity X=308583,
    /// Z=-120156 — it renders correctly, it is simply 331 km from the camera, so the operator
    /// sees an empty dock while the panel reports seven tracks loaded. (And 331 km is not the
    /// true 403 km separation: CoordinateSharp takes the UTM zone from the point, so a zone-32
    /// easting gets subtracted from a zone-34 one and the difference is meaningless.)
    /// </summary>
    [Serializable]
    public class ReplaySite
    {
        [JsonProperty("matched")] public bool Matched;
        [JsonProperty("id")] public string Id;
        [JsonProperty("name")] public string Name;
        [JsonProperty("lat")] public double Lat;
        [JsonProperty("lon")] public double Lon;
        [JsonProperty("nearest_id")] public string NearestId;
        [JsonProperty("nearest_name")] public string NearestName;
        [JsonProperty("distance_m")] public double DistanceM;
        [JsonProperty("origin_lat")] public double OriginLat;
        [JsonProperty("origin_lon")] public double OriginLon;
        [JsonProperty("reason")] public string Reason;

        public string Describe() =>
            Matched ? $"{Name} ({Id})" : $"unidentified — {Reason}";
    }

    [Serializable]
    public class ReplayRun
    {
        [JsonProperty("run")] public string Run;
        [JsonProperty("vehicle")] public string Vehicle;
        [JsonProperty("campaign")] public string Campaign;
        [JsonProperty("from_campaign_store")] public bool FromCampaignStore;

        public string Describe() =>
            FromCampaignStore ? $"{Campaign} / {Vehicle} / {Run}"
                              : $"{Run} (not in the campaign store — vehicle unknown)";
    }

    [Serializable]
    public class DepthConflict
    {
        [JsonProperty("detected")] public bool Detected;
        [JsonProperty("flip_fraction")] public double FlipFraction;
        [JsonProperty("chain_a_n")] public int ChainAN;
        [JsonProperty("chain_b_n")] public int ChainBN;
        [JsonProperty("agreement_median_m")] public double AgreementMedianM;
        [JsonProperty("combined_rate_ms")] public double CombinedRateMs;
        [JsonProperty("note")] public string Note;
    }

    [Serializable]
    public class ReplayDeviation
    {
        [JsonProperty("median_m")] public double MedianM;
        [JsonProperty("p95_m")] public double P95M;
        [JsonProperty("max_m")] public double MaxM;
        [JsonProperty("vs")] public string Vs;
    }

    [Serializable]
    public class ReplayChannel
    {
        [JsonProperty("key")] public string Key;
        [JsonProperty("label")] public string Label;
        [JsonProperty("unit")] public string Unit;
        [JsonProperty("min")] public double Min;
        [JsonProperty("max")] public double Max;
        [JsonProperty("constant")] public bool Constant;
        [JsonProperty("t")] public List<double> T = new();
        [JsonProperty("v")] public List<double> V = new();

        /// <summary>Nearest sample at bag time t, or null past either end.</summary>
        public double? ValueAt(double t)
        {
            if (T == null || V == null || T.Count == 0) return null;
            int i = ReplayMath.NearestIndex(T, t);
            // T and V are written as a pair by the exporter, but a truncated or hand-edited
            // file would index past the end here and throw inside a draw call, which is a
            // miserable place to find out. Refuse instead.
            if (i < 0 || i >= V.Count) return null;
            return V[i];
        }
    }

    [Serializable]
    public class ReplayEventLane
    {
        [JsonProperty("key")] public string Key;
        [JsonProperty("label")] public string Label;
        [JsonProperty("events")] public List<ReplayEvent> Events = new();
    }

    [Serializable]
    public class ReplayEvent
    {
        [JsonProperty("t")] public double T;
        [JsonProperty("value")] public string Value;
    }

    public static class ReplayMath
    {
        /// <summary>Binary search for the sample nearest t. Returns -1 for an empty series.</summary>
        public static int NearestIndex(List<double> times, double t)
        {
            int n = times.Count;
            if (n == 0) return -1;
            int lo = 0, hi = n - 1;
            while (lo < hi)
            {
                int mid = (lo + hi) / 2;
                if (times[mid] < t) lo = mid + 1; else hi = mid;
            }
            if (lo > 0 && Math.Abs(times[lo - 1] - t) <= Math.Abs(times[lo] - t)) return lo - 1;
            return lo;
        }

        /// <summary>Index of the last sample at or before t — the "where is the vehicle now"
        /// question, which must never jump forward to a sample that has not happened yet.</summary>
        public static int FloorIndex(List<double> times, double t)
        {
            int n = times.Count;
            if (n == 0 || t < times[0]) return -1;
            int lo = 0, hi = n - 1;
            while (lo < hi)
            {
                int mid = (lo + hi + 1) / 2;
                if (times[mid] <= t) lo = mid; else hi = mid - 1;
            }
            return lo;
        }
    }

    public static class ReplayLoader
    {
        /// <summary>Where the exporter writes by default.</summary>
        public static string DefaultDirectory =>
            Path.Combine(Application.streamingAssetsPath, "DataCubeReplays");

        public static string[] FindReplays()
        {
            var dir = DefaultDirectory;
            if (!Directory.Exists(dir)) return Array.Empty<string>();
            return Directory.GetFiles(dir, "*.dcreplay.json");
        }

        /// <summary>
        /// Load and VALIDATE. A wrong-format or wrong-version file is refused by name rather
        /// than half-parsed into a plausible-looking empty scene — a component that cannot do
        /// its job must say so, not return something that reads like success (SETTLED §1b).
        /// </summary>
        public static ReplayFile Load(string path, out string error)
        {
            error = null;
            try
            {
                if (!File.Exists(path)) { error = $"no such file: {path}"; return null; }
                var json = File.ReadAllText(path);
                var f = JsonConvert.DeserializeObject<ReplayFile>(json);
                if (f == null) { error = $"{Path.GetFileName(path)}: parsed to nothing"; return null; }
                if (f.Format != ReplayFile.ExpectedFormat)
                {
                    error = $"{Path.GetFileName(path)}: format is '{f.Format}', expected " +
                            $"'{ReplayFile.ExpectedFormat}' — this is not a DataCube replay file";
                    return null;
                }
                if (f.Version != ReplayFile.SupportedVersion)
                {
                    error = $"{Path.GetFileName(path)}: format version {f.Version}, this build " +
                            $"reads version {ReplayFile.SupportedVersion}. Re-export with the " +
                            $"current scripts/export_bag_for_unity.py.";
                    return null;
                }
                return f;
            }
            catch (Exception e)
            {
                error = $"{Path.GetFileName(path)}: {e.GetType().Name}: {e.Message}";
                return null;
            }
        }
    }
}
