// WaterSurfaceProbe.cs
//
// Measurement instrument for the HDRP water surface, written for the SMARC rig.
//
// It answers three questions in one run:
//   1. Is the CPU water simulation actually available to scripts at all?
//      (FillWaterSearchData returns false when Script Interactions is off -> logged, loudly.)
//   2. What does the queried surface look like as a time series?
//      (elevation at a fixed world point, at the physics rate -> Hs, Tp, patch recurrence)
//   3. Do N points on ONE rigid line agree about ONE plane?
//      (per-point error + step count + residual of a straight-line fit)
//
// It also has a SharedSeedDefectMode toggle that reproduces the failure mode diagnosed on
// 2026-08-18 in HDRPWaterQueryModel.GetWaterLevelAt(): seeding every search from the PREVIOUS
// caller's result instead of from the point's own position, and ignoring the convergence flag.
// Run the same scene twice, once with the toggle on and once off, and diff the CSVs. That is
// the direct measurement of how much of the buoyancy error is the defect and how much is the
// CPU-vs-GPU surface difference.
//
// USAGE
//   1. Drop this file in SMARCAssets/Runtime/Scripts/Diagnostics/ (or any Runtime asmdef folder
//      that already references Unity.Burst, Unity.Collections, Unity.Jobs, Unity.Mathematics and
//      the HDRP package).
//   2. Create an empty GameObject in the scene, name it WaterProbe, place it where you want the
//      probe line to sit (world XZ matters, Y does not).
//   3. Add Component -> Water Surface Probe.
//   4. Drag the scene's Ocean object into Target Surface.
//   5. On the Ocean object: Inspector -> Water Surface -> tick Script Interactions.
//      (HDRP 14-16 also require it in the HDRP Asset: Project Settings -> Graphics -> the HDRP
//      asset -> Rendering -> Water -> Script Interactions. HDRP 17 documents the surface only.)
//   6. Press Play. The CSV lands in Application.persistentDataPath unless OutputPath is set.
//
// COST SWEEP (the question "how expensive is a query" — answered on your machine, not argued):
//      ProbeCount 1, 10, 27, 100, 324, 1000, DurationSeconds 30 each. The CSV trailer prints
//      the mean wall time of the batched job and the per-point cost. 324 = 27 discs x 12 spheres.
//
// RECOMMENDED RUN MATRIX (four runs, ten minutes each, same wind settings):
//      A  full resolution ON,  ripples ON,  defect mode OFF   <- the reference surface
//      B  full resolution OFF, ripples OFF, defect mode OFF   <- HDRP's cheap defaults
//      C  full resolution ON,  ripples ON,  defect mode ON    <- isolates the seed defect
//      D  Ocean transform Y = 1.07, otherwise as A            <- the Kristineberg case
//
// NOTE ON PROPERTY NAMES: the WaterSurface CPU-simulation fields have been renamed between HDRP
// 14, 16 and 17. Rather than hard-code them and break the build, the header dump uses reflection
// and records whatever it finds. Do not "clean this up" into direct field access without checking
// the installed HDRP version.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
// 2026-09-13: `using System.Diagnostics;` together with `using UnityEngine;` makes every
// Debug.Log* call ambiguous (CS0104). Only Stopwatch is needed from it, so alias it.
using Stopwatch = System.Diagnostics.Stopwatch;
using System.Text;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Diagnostics
{
    [DisallowMultipleComponent]
    public class WaterSurfaceProbe : MonoBehaviour
    {
        [Header("Target")]
        [Tooltip("The scene's Ocean object (the one carrying the WaterSurface component).")]
        public WaterSurface TargetSurface;

        [Header("Probe geometry")]
        [Tooltip("Number of sample points along the line. 1 gives a single-point elevation record.")]
        public int ProbeCount = 10;

        [Tooltip("Length of the probe line in metres. 1.33 = SAM's hull length.")]
        public float ProbeSpan = 1.33f;

        [Tooltip("World-space direction of the probe line. Normalised at Start.")]
        public Vector3 LineDirection = Vector3.forward;

        [Header("Search parameters")]
        [Tooltip("Convergence tolerance passed to the search, in metres.")]
        public float SearchError = 0.001f;

        [Tooltip("Iteration budget per point. HDRP's own example uses 8; 16 costs little on 10 points.")]
        public int MaxIterations = 16;

        public bool IncludeDeformation = true;
        public bool ExcludeSimulation = false;

        [Header("Defect reproduction")]
        [Tooltip("ON reproduces HDRPWaterQueryModel's shared-seed behaviour: every point is seeded " +
                 "from the previous point's result instead of from its own position. Leave OFF for " +
                 "correct measurements; turn ON once to quantify the defect.")]
        public bool SharedSeedDefectMode = false;

        [Header("Logging")]
        [Tooltip("Seconds of data to record. 600 s at 50 Hz = 30000 rows.")]
        public float DurationSeconds = 600f;

        [Tooltip("Leave empty to write to Application.persistentDataPath.")]
        public string OutputPath = "";

        [Tooltip("Also log the free-surface normal at each point (3 extra columns per point).")]
        public bool LogNormals = false;

        [Tooltip("Rows buffered before each flush to disk. Keeps file IO out of FixedUpdate.")]
        public int FlushEveryRows = 500;

        // ---- internals -------------------------------------------------------------------

        NativeArray<float3> _targetPositions;
        NativeArray<float3> _startPositions;
        NativeArray<float>  _errors;
        NativeArray<float3> _candidates;
        NativeArray<float3> _projected;
        NativeArray<float3> _normals;
        NativeArray<float3> _directions;
        NativeArray<int>    _stepCounts;

        Stopwatch _sw = new Stopwatch();
        double _usAccum; int _usCount;
        StreamWriter _writer;
        StringBuilder _sb;
        int _rowsSinceFlush;
        float _t0;
        bool _running;
        bool _warnedNoSimData;
        int _failedFillCount;

        void Start()
        {
            if (TargetSurface == null)
            {
                Debug.LogError("[WaterSurfaceProbe] TargetSurface is not assigned. Nothing to probe.");
                enabled = false;
                return;
            }

            ProbeCount = Mathf.Max(1, ProbeCount);
            LineDirection = LineDirection.sqrMagnitude < 1e-9f ? Vector3.forward : LineDirection.normalized;

            _targetPositions = new NativeArray<float3>(ProbeCount, Allocator.Persistent);
            _startPositions  = new NativeArray<float3>(ProbeCount, Allocator.Persistent);
            _errors          = new NativeArray<float>(ProbeCount, Allocator.Persistent);
            _candidates      = new NativeArray<float3>(ProbeCount, Allocator.Persistent);
            _projected       = new NativeArray<float3>(ProbeCount, Allocator.Persistent);
            _normals         = new NativeArray<float3>(ProbeCount, Allocator.Persistent);
            _directions      = new NativeArray<float3>(ProbeCount, Allocator.Persistent);
            _stepCounts      = new NativeArray<int>(ProbeCount, Allocator.Persistent);

            for (int i = 0; i < ProbeCount; ++i)
            {
                float s = ProbeCount == 1 ? 0f : (i / (float)(ProbeCount - 1) - 0.5f) * ProbeSpan;
                Vector3 p = transform.position + LineDirection * s;
                _targetPositions[i] = new float3(p.x, p.y, p.z);
            }

            OpenWriter();
            WriteHeader();

            _t0 = Time.time;
            _running = true;

            // Immediate go/no-go on Script Interactions.
            WaterSimSearchData probeData = new WaterSimSearchData();
            if (!TargetSurface.FillWaterSearchData(ref probeData))
            {
                Debug.LogError("[WaterSurfaceProbe] FillWaterSearchData returned FALSE at Start. " +
                               "The CPU water simulation is NOT available to scripts. Every buoyancy " +
                               "query in this scene is reading a degenerate surface. Enable Script " +
                               "Interactions on the WaterSurface (and, on HDRP 14-16, in the HDRP Asset).");
            }
            else
            {
                Debug.Log("[WaterSurfaceProbe] CPU water simulation available. Recording " +
                          ProbeCount + " points over " + ProbeSpan.ToString("F2") + " m for " +
                          DurationSeconds.ToString("F0") + " s -> " + _pathInUse);
            }
        }

        string _pathInUse;

        void OpenWriter()
        {
            string dir = string.IsNullOrEmpty(OutputPath) ? Application.persistentDataPath : OutputPath;
            Directory.CreateDirectory(dir);
            string name = string.Format("water_probe_{0}_{1}pt_{2}.csv",
                DateTime.Now.ToString("yyyyMMdd_HHmmss"),
                ProbeCount,
                SharedSeedDefectMode ? "defectseed" : "ownseed");
            _pathInUse = Path.Combine(dir, name);
            _writer = new StreamWriter(_pathInUse, false, Encoding.UTF8);
            _sb = new StringBuilder(1 << 16);
        }

        // Reflection dump of every CPU/script-related field on the WaterSurface, so the CSV
        // records the exact configuration that produced it regardless of HDRP version.
        void WriteHeader()
        {
            _writer.WriteLine("# WaterSurfaceProbe");
            _writer.WriteLine("# utc," + DateTime.UtcNow.ToString("o"));
            _writer.WriteLine("# unity," + Application.unityVersion);
            _writer.WriteLine("# scene," + UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
            _writer.WriteLine("# fixedDeltaTime," + Time.fixedDeltaTime.ToString("R", CultureInfo.InvariantCulture));
            _writer.WriteLine("# probeCount," + ProbeCount);
            _writer.WriteLine("# probeSpan_m," + ProbeSpan.ToString("R", CultureInfo.InvariantCulture));
            _writer.WriteLine("# probeOriginWS," + V3(transform.position));
            _writer.WriteLine("# lineDirectionWS," + V3(LineDirection));
            _writer.WriteLine("# searchError_m," + SearchError.ToString("R", CultureInfo.InvariantCulture));
            _writer.WriteLine("# maxIterations," + MaxIterations);
            _writer.WriteLine("# includeDeformation," + IncludeDeformation);
            _writer.WriteLine("# excludeSimulation," + ExcludeSimulation);
            _writer.WriteLine("# sharedSeedDefectMode," + SharedSeedDefectMode);
            _writer.WriteLine("# waterSurfaceTransform," + V3(TargetSurface.transform.position));

            foreach (var kv in DumpSurfaceSettings(TargetSurface))
                _writer.WriteLine("# surface." + kv.Key + "," + kv.Value);

            var cols = new List<string> { "t", "query_us" };   // query_us = wall time of the whole batched search this step
            for (int i = 0; i < ProbeCount; ++i)
            {
                cols.Add("s" + i);          // along-line station, m (constant, kept for the fit)
                cols.Add("y" + i);          // projected surface elevation, world Y
                cols.Add("err" + i);        // residual reported by the search
                cols.Add("steps" + i);      // iterations consumed
                if (LogNormals)
                {
                    cols.Add("nx" + i); cols.Add("ny" + i); cols.Add("nz" + i);
                }
            }
            _writer.WriteLine(string.Join(",", cols));
            _writer.Flush();
        }

        static string V3(Vector3 v)
        {
            return string.Format(CultureInfo.InvariantCulture, "{0:R} {1:R} {2:R}", v.x, v.y, v.z);
        }

        static Dictionary<string, string> DumpSurfaceSettings(WaterSurface s)
        {
            var result = new Dictionary<string, string>();
            var type = s.GetType();
            const BindingFlags flags = BindingFlags.Public | BindingFlags.Instance;

            foreach (var f in type.GetFields(flags))
            {
                string n = f.Name.ToLowerInvariant();
                if (n.Contains("cpu") || n.Contains("script") || n.Contains("resolution") ||
                    n.Contains("ripple") || n.Contains("simulation") || n.Contains("wind") ||
                    n.Contains("chaos") || n.Contains("amplitude") || n.Contains("patch") ||
                    n.Contains("repetition") || n.Contains("surfacetype") || n.Contains("current"))
                {
                    try { result[f.Name] = Convert.ToString(f.GetValue(s), CultureInfo.InvariantCulture); }
                    catch { /* value not printable, skip */ }
                }
            }
            foreach (var p in type.GetProperties(flags))
            {
                if (!p.CanRead || p.GetIndexParameters().Length != 0) continue;
                string n = p.Name.ToLowerInvariant();
                if (n.Contains("cpu") || n.Contains("script") || n.Contains("resolution") || n.Contains("ripple"))
                {
                    try { result[p.Name] = Convert.ToString(p.GetValue(s), CultureInfo.InvariantCulture); }
                    catch { /* getter threw, skip */ }
                }
            }
            return result;
        }

        // Sampling runs in FixedUpdate so the record is at the same rate, and on the same clock,
        // as ForcePoint's buoyancy evaluation. Sampling in Update would measure a different signal.
        void FixedUpdate()
        {
            if (!_running || TargetSurface == null) return;

            float t = Time.time - _t0;
            if (t > DurationSeconds) { Stop("duration reached"); return; }

            WaterSimSearchData simData = new WaterSimSearchData();
            if (!TargetSurface.FillWaterSearchData(ref simData))
            {
                _failedFillCount++;
                if (!_warnedNoSimData)
                {
                    Debug.LogError("[WaterSurfaceProbe] FillWaterSearchData FALSE during run. " +
                                   "Recording gaps; see failedFill count in the trailer.");
                    _warnedNoSimData = true;
                }
                return;
            }

            // Seeding. This is the whole difference between a correct query and the 2026-08-18 defect.
            if (SharedSeedDefectMode)
            {
                // Every point seeded from the PREVIOUS point's converged candidate, one shared chain.
                float3 seed = _candidates.Length > 0 ? _candidates[ProbeCount - 1] : _targetPositions[0];
                for (int i = 0; i < ProbeCount; ++i) { _startPositions[i] = seed; seed = _candidates[i]; }
            }
            else
            {
                // Each point seeded from its own position. Independent, order-invariant, correct.
                for (int i = 0; i < ProbeCount; ++i) _startPositions[i] = _targetPositions[i];
            }

            var job = new WaterSimulationSearchJob
            {
                simSearchData            = simData,
                targetPositionWSBuffer   = _targetPositions,
                startPositionWSBuffer    = _startPositions,
                maxIterations            = MaxIterations,
                error                    = SearchError,
                includeDeformation       = IncludeDeformation,
                excludeSimulation        = ExcludeSimulation,
                errorBuffer              = _errors,
                candidateLocationWSBuffer= _candidates,
                projectedPositionWSBuffer= _projected,
                normalWSBuffer           = _normals,
                directionBuffer          = _directions,
                stepCountBuffer          = _stepCounts,
            };

            // One job for all points. Batch size 1 so Burst spreads them; for 10 points the
            // scheduling overhead dominates either way, but this is the shape the real
            // WaterQueryModel should use for a whole ForcePoint cloud.
            _sw.Restart();
            job.Schedule(ProbeCount, 1).Complete();
            _sw.Stop();
            double us = _sw.Elapsed.TotalMilliseconds * 1000.0;
            _usAccum += us; _usCount++;

            _sb.Append(t.ToString("F5", CultureInfo.InvariantCulture));
            _sb.Append(',').Append(us.ToString("F1", CultureInfo.InvariantCulture));
            for (int i = 0; i < ProbeCount; ++i)
            {
                float s = ProbeCount == 1 ? 0f : (i / (float)(ProbeCount - 1) - 0.5f) * ProbeSpan;
                _sb.Append(',').Append(s.ToString("F5", CultureInfo.InvariantCulture));
                _sb.Append(',').Append(_projected[i].y.ToString("F6", CultureInfo.InvariantCulture));
                _sb.Append(',').Append(_errors[i].ToString("F6", CultureInfo.InvariantCulture));
                _sb.Append(',').Append(_stepCounts[i].ToString(CultureInfo.InvariantCulture));
                if (LogNormals)
                {
                    _sb.Append(',').Append(_normals[i].x.ToString("F5", CultureInfo.InvariantCulture));
                    _sb.Append(',').Append(_normals[i].y.ToString("F5", CultureInfo.InvariantCulture));
                    _sb.Append(',').Append(_normals[i].z.ToString("F5", CultureInfo.InvariantCulture));
                }
            }
            _sb.Append('\n');

            if (++_rowsSinceFlush >= FlushEveryRows)
            {
                _writer.Write(_sb.ToString());
                _writer.Flush();
                _sb.Clear();
                _rowsSinceFlush = 0;
            }
        }

        void Stop(string reason)
        {
            if (!_running) return;
            _running = false;
            if (_sb != null && _sb.Length > 0) { _writer.Write(_sb.ToString()); _sb.Clear(); }
            if (_writer != null)
            {
                _writer.WriteLine("# stopped," + reason);
                _writer.WriteLine("# failedFill," + _failedFillCount);
                if (_usCount > 0)
                    _writer.WriteLine("# meanQuery_us," + (_usAccum / _usCount).ToString("F1", CultureInfo.InvariantCulture)
                                      + " for " + ProbeCount + " points  (" + (_usAccum / _usCount / ProbeCount).ToString("F2", CultureInfo.InvariantCulture) + " us/point)");
                _writer.Flush();
                _writer.Close();
                _writer = null;
            }
            Debug.Log("[WaterSurfaceProbe] Finished (" + reason + "). File: " + _pathInUse
                      + (_usCount > 0 ? "  mean query " + (_usAccum / _usCount).ToString("F1") + " us for " + ProbeCount + " points" : ""));
        }

        void OnDisable() { Stop("disabled"); }

        void OnDestroy()
        {
            Stop("destroyed");
            if (_targetPositions.IsCreated) _targetPositions.Dispose();
            if (_startPositions.IsCreated)  _startPositions.Dispose();
            if (_errors.IsCreated)          _errors.Dispose();
            if (_candidates.IsCreated)      _candidates.Dispose();
            if (_projected.IsCreated)       _projected.Dispose();
            if (_normals.IsCreated)         _normals.Dispose();
            if (_directions.IsCreated)      _directions.Dispose();
            if (_stepCounts.IsCreated)      _stepCounts.Dispose();
        }

        void OnDrawGizmos()
        {
            Vector3 d = LineDirection.sqrMagnitude < 1e-9f ? Vector3.forward : LineDirection.normalized;
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position - d * ProbeSpan * 0.5f,
                            transform.position + d * ProbeSpan * 0.5f);
            int n = Mathf.Max(1, ProbeCount);
            for (int i = 0; i < n; ++i)
            {
                float s = n == 1 ? 0f : (i / (float)(n - 1) - 0.5f) * ProbeSpan;
                Gizmos.DrawSphere(transform.position + d * s, 0.02f);
            }
        }
    }
}
