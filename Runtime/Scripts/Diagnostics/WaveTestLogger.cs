// WaveTestLogger.cs — records what the acceptance gates need, at the physics rate.
//
// Attach to the vehicle root (the object carrying base_link's ArticulationBody). Press Play.
// Writes one CSV row per FixedUpdate to Application.persistentDataPath (or OutputPath):
//
//   t, x, y, z              root position (world, Unity axes: y up, z forward)
//   roll, pitch, yaw        deg, from the root rotation (Unity Euler: x = pitch, y = yaw, z = roll)
//   vx, vy, vz              root velocity
//   eta_cb                  water surface height queried at the CB station (world y)
//   depth_cb                eta_cb − y_cb  (positive = CB below the surface)
//   B_sum                   sum of every ForcePoint's applied buoyancy this step [N]
//   f_mean, f_min, f_max    immersed fractions across the strips (0..1), from AppliedBuoyancyForce / (V·ρ·g)
//   M_pitch                 pitch moment of the buoyancy cloud about the CG [N m]
//   lcg_pct, vbs_pct        actuator states if the Prismatic/VBS components are found
//
// The same file with the WaterSurfaceProbe running at the CB station gives you the sea state
// the vehicle actually saw, so heave/pitch transfer functions can be formed from the two logs.
//
// GATES it feeds (see ADR-011 §Acceptance gates and analyze_wave_test.py):
//   G0 still water:   depth_cb → 66 ± 3 mm, |pitch| < 0.5°, |roll| < 0.5°, no drift in y over 60 s
//   G1 roll decay:    heel 10° by hand, release → damped oscillation at W·BG/(I+A) → f_n
//   G2 submerged:     VBS to neutral, sink to 3 m, waves on → heave RAO ≈ exp(−kd) (parameter-free)
//   G3 surfaced:      waves on → heave RAO → 1 at long periods; pitch amplitude per sam_rao_model
//   G4 no drift:      30 min at 3 m in a 0.5 m sea, VBS held → mean depth change < 5 cm

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using DefaultNamespace.Water;
using Force;
using UnityEngine;
using VehicleComponents.Actuators;

namespace Diagnostics
{
    public class WaveTestLogger : MonoBehaviour
    {
        public float DurationSeconds = 600f;
        public string OutputPath = "";
        public string RunLabel = "run";
        public int FlushEveryRows = 250;

        ArticulationBody _root;
        ForcePoint[] _points;
        float[] _pointBmax;
        Prismatic _lcg;
        VBS _vbs;
        WaterQueryModel _water;
        StreamWriter _w; StringBuilder _sb = new StringBuilder(1 << 16);
        int _rows; float _t0; bool _running;
        string _path;

        void Start()
        {
            _root = GetComponentsInChildren<ArticulationBody>().OrderByDescending(b => b.mass).First();
            _points = GetComponentsInChildren<ForcePoint>(true);
            _pointBmax = _points.Select(p => p.Volume * p.WaterDensity * Mathf.Abs(Physics.gravity.y)).ToArray();
            _lcg = GetComponentsInChildren<Prismatic>(true).FirstOrDefault();
            _vbs = GetComponentsInChildren<VBS>(true).FirstOrDefault();
            _water = WaterQueryModel.GetWaterQueryModel();

            string dir = string.IsNullOrEmpty(OutputPath) ? Application.persistentDataPath : OutputPath;
            Directory.CreateDirectory(dir);
            _path = Path.Combine(dir, $"wavetest_{RunLabel}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
            _w = new StreamWriter(_path, false, Encoding.UTF8);
            _w.WriteLine("# WaveTestLogger");
            _w.WriteLine("# utc," + DateTime.UtcNow.ToString("o"));
            _w.WriteLine("# scene," + UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
            _w.WriteLine("# fixedDeltaTime," + Time.fixedDeltaTime.ToString("R", CultureInfo.InvariantCulture));
            _w.WriteLine("# root," + _root.name + ",mass," + _root.mass.ToString("R", CultureInfo.InvariantCulture));
            _w.WriteLine("# waterQueryModel," + (_water != null ? _water.GetType().Name : "NONE"));
            _w.WriteLine("# forcePoints," + _points.Length + ",stripMode," + _points.Count(p => p.SectionRadius > 0f)
                         + ",sumVolume_L," + (_points.Sum(p => p.Volume) * 1000f).ToString("F3", CultureInfo.InvariantCulture));
            _w.WriteLine("t,x,y,z,roll,pitch,yaw,vx,vy,vz,eta_cb,depth_cb,B_sum,f_mean,f_min,f_max,M_pitch,lcg_pct,vbs_pct");
            _t0 = Time.time; _running = true;
            Debug.Log($"[WaveTestLogger] {_points.Length} ForcePoints ({_points.Count(p => p.SectionRadius > 0f)} strips), " +
                      $"water model {(_water != null ? _water.GetType().Name : "NONE")}, writing {_path}");
        }

        void FixedUpdate()
        {
            if (!_running) return;
            float t = Time.time - _t0;
            if (t > DurationSeconds) { Stop(); return; }

            // Volume-weighted CB from the current point positions.
            Vector3 cb = Vector3.zero; float vsum = 0f;
            for (int i = 0; i < _points.Length; ++i) { cb += _points[i].transform.position * _points[i].Volume; vsum += _points[i].Volume; }
            cb /= Mathf.Max(vsum, 1e-9f);

            Vector3 cg = _root.worldCenterOfMass;
            float eta = _water != null ? _water.GetWaterLevelAt(cb) : float.NaN;

            float bsum = 0f, fmin = 1f, fmax = 0f, fsum = 0f; int nwet = 0; float mpitch = 0f;
            Vector3 fwd = _root.transform.forward;
            for (int i = 0; i < _points.Length; ++i)
            {
                float b = _points[i].AppliedBuoyancyForce.y;
                if (!_points[i].IsUnderwater) b = 0f;
                bsum += b;
                float f = _pointBmax[i] > 0f ? b / _pointBmax[i] : 0f;
                fsum += f; fmin = Mathf.Min(fmin, f); fmax = Mathf.Max(fmax, f); nwet++;
                // moment about CG: lever = along-hull distance from CG, force vertical
                float lever = Vector3.Dot(_points[i].transform.position - cg, fwd);
                mpitch += b * lever;
            }

            Vector3 e = _root.transform.rotation.eulerAngles;
            float Wrap(float a) => a > 180f ? a - 360f : a;
            Vector3 p = _root.transform.position, v = _root.linearVelocity;

            var ci = CultureInfo.InvariantCulture;
            _sb.Append(t.ToString("F4", ci))
               .Append(',').Append(p.x.ToString("F5", ci)).Append(',').Append(p.y.ToString("F5", ci)).Append(',').Append(p.z.ToString("F5", ci))
               .Append(',').Append(Wrap(e.z).ToString("F3", ci)).Append(',').Append(Wrap(e.x).ToString("F3", ci)).Append(',').Append(Wrap(e.y).ToString("F3", ci))
               .Append(',').Append(v.x.ToString("F5", ci)).Append(',').Append(v.y.ToString("F5", ci)).Append(',').Append(v.z.ToString("F5", ci))
               .Append(',').Append(eta.ToString("F5", ci)).Append(',').Append((eta - cb.y).ToString("F5", ci))
               .Append(',').Append(bsum.ToString("F3", ci))
               .Append(',').Append((nwet > 0 ? fsum / nwet : 0f).ToString("F4", ci)).Append(',').Append(fmin.ToString("F4", ci)).Append(',').Append(fmax.ToString("F4", ci))
               .Append(',').Append(mpitch.ToString("F4", ci))
               .Append(',').Append((_lcg != null ? _lcg.percentage : float.NaN).ToString("F1", ci))
               .Append(',').Append((_vbs != null ? _vbs.percentage : float.NaN).ToString("F1", ci))
               .Append('\n');

            if (++_rows % FlushEveryRows == 0) { _w.Write(_sb.ToString()); _sb.Clear(); _w.Flush(); }
        }

        void Stop()
        {
            if (!_running) return;
            _running = false;
            _w.Write(_sb.ToString()); _sb.Clear(); _w.WriteLine("# rows," + _rows); _w.Flush(); _w.Close();
            Debug.Log($"[WaveTestLogger] done, {_rows} rows -> {_path}");
        }

        void OnDisable() { Stop(); }
    }
}
