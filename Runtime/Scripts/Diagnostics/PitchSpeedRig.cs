// PitchSpeedRig.cs — the pitch-moment discriminator: one Play session, one pitch-vs-speed curve.
//
// WHAT QUESTION THIS ANSWERS. The 2026-09-15 diagnosis (data-cube/docs/2026-09-15-pitch-moment-
// diagnosis.md) says the standing +9.76 deg nose-up is NOT an equilibrium set by a mis-aimed
// thrust line -- ThrustTransform is null on sam21.strips, so thrust is applied at a point
// constructed from the CoM ALONG the hull axis and its moment about the CoM is identically zero.
// It says instead that the destabilising couple is LINEAR in the angle (Munk, and the flux
// stations that replace it) while the only thing counteracting it is QUADRATIC (transverse drag
// at a centre of pressure 0.1 m AHEAD of the CoM). A linear destabiliser against a linear
// hydrostatic restoring moment has a THRESHOLD, not a gain:
//
//     U* = sqrt( W * BG_z / C_munk )   ->   0.402 m/s at BG_z 14.6 mm, 0.235 m/s at 5 mm.
//
// Below U* the pitch settles small. Above it the pitch runs away until the quadratic CoP drag
// catches it. So the prediction is a SHARP ONSET near 0.4 m/s and a roughly flat angle above it.
// The competing explanation -- some steady moment that scales with dynamic pressure -- predicts
// pitch growing smoothly with U^2 from zero, no knee. One sweep separates them, and the sweep
// is the whole point of this file.
//
// WHY A RIG AND NOT "PRESS PLAY". Pressing Play gives an UNCOMMANDED vehicle, and that was
// measured on 2026-09-13: VBS sat at its serialized 0 %, the tank was 0.176 kg light, the
// piston was at the end of its stroke, and the vehicle settled 25 deg nose-up at depth -- three
// times the signal we are hunting. Nothing commands speed either, so there is no sweep axis.
// This rig commands VBS and LCG every step, waits for the pistons to physically ARRIVE, drives
// the speed itself, and writes one CSV per case.
//
// THE ONE THING THAT MAKES THIS MEASUREMENT VALID. Both rig forces are applied with
// ArticulationBody.AddForce, which acts AT THE CENTRE OF MASS. A force at the centre of mass
// produces no moment about the centre of mass, at any frequency, at any angle. So neither the
// surge drive nor the depth hold can put a single N.m into the pitch axis: the pitch the CSV
// records is the hull's own, and nothing else's. This is also why the depth hold may stay LIVE
// during the record here, where WaveSweepRig had to freeze its equivalent -- a live depth
// controller would corrupt a wave transfer function, but it cannot corrupt a pitch moment.
//
// AND WHY THE DEPTH HOLD IS PART OF THE PHYSICS, NOT AN ARTEFACT. Holding depth while the hull
// is pitched nose-up is exactly TRIMMED LEVEL FLIGHT: the world-frame heave is driven to zero,
// so the body-frame heave is w = U*sin(theta) -- the angle of attack itself. That is the w the
// Munk term eats (tau[4] += C*u*w). Letting the vehicle climb instead would bleed the angle of
// attack away and hide the very instability we came to measure.
//
// NO THRUSTER, DELIBERATELY. Prop1eRPM/Prop2eRPM/CommandRPM are forced to zero every step and
// the surge comes from the rig. That removes the propeller and its ThrustArm from the picture
// entirely, so the curve cannot be read back as "the thrust line did it".
//
// NO CONTROL SURFACES, DELIBERATELY. Delta1/Delta2 are forced to zero every step. The +9.76 deg
// was recorded with a controller holding the vehicle; a held angle is not an equilibrium and
// carries no speed law. Free sterngear is what has one.
//
// PER-CASE HYDRO SWITCHES. hydro_overrides.txt is read once at Awake, so it can only do one
// configuration per Play session -- and this test lives or dies on A/B pairs against IDENTICAL
// water and identical trim. Each case line therefore carries its own ';'-separated field list,
// applied by REFLECTION so this file needs no reference to the SAM assembly (same reason
// WaveSweepRig kills competing writers by type name). The three that matter:
//   CenterOfPressureOffset=0  the quadratic counterweight removed -> above U* nothing catches it
//   UseSlenderBodyFlux=0      the flux stations off -> the EXPLICIT MunkCoefficient path runs
//   MunkCoefficient=0         the explicit destabiliser removed
// Note that UseSlenderBodyFlux / UseBodyLift / UseRingWing are NOT serialized in
// sam21.strips.prefab, so at Play they are all C#-default TRUE and the explicit Munk term is
// SKIPPED. Every case here records what it actually ran with, in its own header.
//
// Config: <project>/../_logs/pitch/config.txt. Output: the same folder, plus manifest.csv.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using DefaultNamespace.Water;
using Force;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using VehicleComponents.Actuators;

namespace Diagnostics
{
    // RUN BEFORE EVERYTHING ELSE, AND DO THE SILENCING IN Awake. MEASURED ON THE Djuro SCENE
    // 2026-09-15: sam21 there carries SAMTankReplay with RunOnStart ticked, and its Start() calls
    // Begin(), which sets Time.timeScale = 4 AND TeleportRoots the vehicle to the replay's first
    // sample. Neither it nor this file declared an execution order, so which Start() won was luck
    // — and if the replay won, the rig's "home" was the replay's start pose, not the launch point
    // Ivan is looking at in the Inspector, and the whole sweep ran somewhere else at 4x. A
    // MonoBehaviour disabled before its Start() never gets one, so silencing from Awake at -200
    // makes that race structurally impossible instead of usually fine.
    [DefaultExecutionOrder(-200)]
    public class PitchSpeedRig : MonoBehaviour
    {
        [Serializable]
        public class Case
        {
            public string Label = "case";
            public float SpeedMps = 0.5f;
            /// ';'-separated public-field assignments applied to SAMHydrodynamicsV2 by reflection
            /// for THIS case only, e.g. "CenterOfPressureOffset=0;UseSlenderBodyFlux=0".
            public string Hydro = "";
            public float VbsPercent = float.NaN;
            public float LcgPercent = float.NaN;
        }

        [Header("Run")]
        public bool Run = true;
        public float SettleSeconds = 6f;
        [Tooltip("Released and accelerating. Not recorded: the pitch is still a transient.")]
        public float SpinupSeconds = 20f;
        public float RecordSeconds = 25f;
        public bool ExitPlayWhenDone = true;

        [Header("Station")]
        [Tooltip("Depth below still water the rig holds. Must clear the seabed AND the surface " +
                 "for the whole run; the vehicle travels speed*(spinup+record) metres downrange.")]
        public float DepthM = 3f;
        [Tooltip("Compass-style heading for the run, degrees clockwise from world +Z. Point it at " +
                 "open water. Left alone, the rig uses the heading the vehicle is ALREADY parked " +
                 "at in the scene, which at a curated site is the one the site build measured.")]
        public float HeadingDeg = 0f;
        public bool UseSceneHeading = true;

        [Header("Water — flat on purpose, waves are a different experiment")]
        public bool ForceCalmWater = true;
        public float WindKmh = 0.5f;
        public float Chaos = 0.1f;
        public bool Ripples = false;

        [Header("Actuators — commanded every step, never left to the serialized value")]
        public float NeutralVbsPercent = 50f;
        public float NeutralLcgPercent = 50f;
        public float ArrivedTolerance = 1.0f;

        [Header("Surge drive (axial force AT THE CoM -> zero pitch moment by construction)")]
        public float SurgeKp = 60f;
        public float SurgeKi = 25f;
        public float MaxSurgeForce = 120f;

        [Header("Depth hold (vertical force AT THE CoM -> zero pitch moment by construction)")]
        public float DepthKp = 35f;
        public float DepthKi = 4f;
        public float DepthKd = 45f;
        public float MaxHoldForce = 200f;

        [Header("Safety")]
        [Tooltip("Free-flight settling time after release before the abort checks arm.")]
        public float SettleAfterReleaseSeconds = 3f;
        public float AbortDepthError = 2.5f;
        public float AbortPitchDeg = 80f;
        public float AbortSpeed = 12f;

        [Header("Targets (auto-found if left empty)")]
        public GameObject Vehicle;
        public WaterSurface Surface;

        [Header("Read-only")]
        public int CaseIndex;
        public string Phase = "idle";
        public float SurgeForceN, HoldForceN, SpeedU, PitchDeg;
        public string LastAbort = "";

        readonly List<Case> _cases = new List<Case>();
        ArticulationBody _root;
        ForcePoint[] _points;
        VBS _vbs; Prismatic _lcg;
        WaterQueryModel _water;
        MonoBehaviour _hydro;                       // SAMHydrodynamicsV2, reached by reflection
        readonly Dictionary<string, FieldInfo> _hf = new Dictionary<string, FieldInfo>();
        readonly Dictionary<string, object> _hydroOriginal = new Dictionary<string, object>();
        Vector3 _home, _homeAwake; Quaternion _heading; Vector3 _axisLocal = Vector3.forward;
        float _stillWaterY;

        string _dir, _outDir, _runUtc = "";
        StreamWriter _w; readonly StringBuilder _sb = new StringBuilder(1 << 16);
        int _phase; float _tPhase; float _tRec; int _rows;
        float _surgeI, _depthI;
        bool _done, _pendingFirstCase;
        string _abort;
        readonly List<string> _summary = new List<string>();

        // ---------------------------------------------------------------- setup

        void Awake()
        {
            _dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "_logs", "pitch"));
            Directory.CreateDirectory(_dir);

            if (!ReadConfig()) { enabled = false; return; }
            if (!Run) { Debug.Log("[PitchSpeedRig] run=0, standing down."); enabled = false; return; }

            if (Vehicle == null)
            {
                var fp = FindObjectsByType<ForcePoint>(FindObjectsSortMode.None).FirstOrDefault();
                if (fp != null) Vehicle = fp.GetComponentInParent<ArticulationBody>()?.transform.root.gameObject;
            }
            if (Vehicle == null) { Debug.LogError("[PitchSpeedRig] no vehicle found."); enabled = false; return; }

            BindHydro();
            SilenceCompetingWriters();

            // SAMTankReplay.Begin() leaves Time.timeScale at its own value and only restores it on
            // Stop(). If it ever got that far, put it back: a sweep is a measurement, not a demo.
            if (!Mathf.Approximately(Time.timeScale, 1f))
            {
                Debug.LogWarning($"[PitchSpeedRig] Time.timeScale was {Time.timeScale} — reset to 1.");
                Time.timeScale = 1f;
            }

            // The pose the Inspector shows, recorded before anything else's Start() could move it.
            // Only a cross-check: the real _home is the ARTICULATION ROOT's pose, and ArticulationBody
            // cannot be asked which body that is until Start (see the note on FindRoot).
            _homeAwake = Vehicle.transform.position;
        }

        /// ARTICULATION BODIES DO NOT KNOW THEIR OWN TOPOLOGY IN Awake. MEASURED 2026-09-15 on the
        /// first Djurö run: `isRoot` was false for every body, so the root fell through to "heaviest
        /// link", and PhysX answered every call with "Only the root body of this articulation can be
        /// teleported" / `set_immovable` — the vehicle was never placed at depth, the depth-hold seed
        /// came out at exactly -W (16.85 kg × 9.81 = 165.3 N, i.e. a subtree with no mass in it), and
        /// every case aborted on the resulting 2.20 m depth error inside one step. `WaveSweepRig` uses
        /// the same `isRoot` query and works because it runs entirely in Start.
        ///
        /// So: resolution happens in Start, and the topology is derived from TRANSFORMS (the body with
        /// no ArticulationBody above it), which is true whenever it is asked. `isRoot` is then only a
        /// cross-check, and a disagreement is reported rather than silently tolerated.
        static ArticulationBody FindRoot(ArticulationBody b)
        {
            for (Transform t = b.transform.parent; t != null; t = t.parent)
            {
                var a = t.GetComponent<ArticulationBody>();
                if (a != null) return FindRoot(a);
            }
            return b;
        }

        void Start()
        {
            var bodies = Vehicle.GetComponentsInChildren<ArticulationBody>(true);
            if (bodies.Length == 0) { Debug.LogError("[PitchSpeedRig] no ArticulationBody under the vehicle."); enabled = false; return; }
            // Several articulations can live under one vehicle (attached sensors get their own).
            // Take the one whose subtree carries the mass, not whichever appears first.
            _root = bodies.Select(FindRoot).Distinct()
                          .OrderByDescending(r => r.GetComponentsInChildren<ArticulationBody>(true).Sum(x => x.mass))
                          .First();
            if (!_root.isRoot)
                Debug.LogError($"[PitchSpeedRig] '{_root.name}' has no ArticulationBody above it but reports " +
                               $"isRoot = false. Teleport and immovable will be refused by PhysX; stop and fix this " +
                               $"before trusting anything this run writes.");
            _points = Vehicle.GetComponentsInChildren<ForcePoint>(true);
            if (_points.Length == 0)
                Debug.LogError("[PitchSpeedRig] no ForcePoints under the vehicle — buoyancy reads as 0 and the " +
                               "depth-hold seed will be the full weight, downward.");
            _vbs = Vehicle.GetComponentsInChildren<VBS>(true).FirstOrDefault();
            _lcg = Vehicle.GetComponentsInChildren<Prismatic>(true).FirstOrDefault();

            _water = WaterQueryModel.GetWaterQueryModel();
            if (Surface == null) Surface = FindObjectsByType<WaterSurface>(FindObjectsSortMode.None).FirstOrDefault();
            if (_water == null) { Debug.LogError("[PitchSpeedRig] no WaterQueryModel in the scene."); enabled = false; return; }
            if (Surface == null) { Debug.LogError("[PitchSpeedRig] no HDRP WaterSurface in the scene."); enabled = false; return; }
            if (!Surface.scriptInteractions) { Surface.scriptInteractions = true; Debug.LogWarning("[PitchSpeedRig] WaterSurface.scriptInteractions was OFF — turned on."); }

            _stillWaterY = Surface.transform.position.y;
            _runUtc = DateTime.UtcNow.ToString("o");

            // EVERY SWEEP GETS ITS OWN FOLDER, NAMED FOR ITS COEFFICIENT SET. Case files are keyed on
            // the case label alone, so a second sweep used to overwrite the first one case by case --
            // MEASURED 2026-09-15, when a repeat run destroyed two cases of a 14-minute baseline before
            // it was noticed, and would have destroyed all twenty. Worse, a run that stopped early left
            // the UNVISITED cases behind as files from the previous run: same names, same columns,
            // different physics, and nothing to tell them apart. Putting the tag in the folder name
            // also means a `bare25` sweep and a `tankcfg` sweep cannot land on top of each other, which
            // is the exact comparison this rig exists to make.
            string tag = HGet<string>("OverrideTag");
            _outDir = Path.Combine(_dir, DateTime.UtcNow.ToString("yyyyMMdd-HHmmss") + "__" +
                                         (string.IsNullOrEmpty(tag) ? "prefab" : tag));
            Directory.CreateDirectory(_outDir);

            // Home and heading are the ARTICULATION ROOT's, because TeleportRoot moves that body and
            // not the GameObject the Inspector happens to show.
            _home = _root.transform.position;
            if (UseSceneHeading)
            {
                HeadingDeg = _root.transform.rotation.eulerAngles.y;
                Debug.Log($"[PitchSpeedRig] heading taken from the scene: {HeadingDeg:F2} deg " +
                          $"(set `heading=` in the config to override).");
            }
            _heading = Quaternion.Euler(0f, HeadingDeg, 0f);

            // Belt and braces on the execution-order note above: if something still moved the
            // vehicle during its own Start, say so rather than sweeping at the wrong station.
            // Against the ROOT BODY, not the GameObject: SAMTankReplay calls BaseLink.TeleportRoot,
            // which moves the articulation without moving the parent transform, so a check on
            // Vehicle.transform would have seen nothing. 5 m, because base_link legitimately sits a
            // little off the vehicle object's origin.
            float moved = (_root.transform.position - _homeAwake).magnitude;
            if (moved > 5f)
                Debug.LogWarning($"[PitchSpeedRig] '{_root.name}' is {moved:F1} m from where the vehicle was in " +
                                 $"Awake — something teleported the articulation. Find out what, before trusting " +
                                 $"this run.");

            if (ForceCalmWater)
            {
                Surface.largeWindSpeed = WindKmh; Surface.largeChaos = Chaos; Surface.ripples = Ripples;
                Debug.Log($"[PitchSpeedRig] water forced calm: wind {WindKmh} km/h, chaos {Chaos}, ripples {Ripples}.");
            }

            float longest = _cases.Max(c => c.SpeedMps) * (SpinupSeconds + RecordSeconds);
            Debug.Log($"[PitchSpeedRig] {_cases.Count} cases | vehicle {_root.name} | depth {DepthM} m | " +
                      $"heading {HeadingDeg} deg | longest downrange run {longest:F0} m | " +
                      $"hydro {(_hydro == null ? "NOT FOUND" : _hydro.GetType().Name)} | out {_outDir}");
            if (_hydro == null)
                Debug.LogWarning("[PitchSpeedRig] no SAMHydrodynamicsV2 on this vehicle: per-case hydro " +
                                 "switches and the force-channel columns will be blank, and the rig cannot " +
                                 "guarantee the thruster is off. The pitch column is still valid.");

            WriteManifestHeader();
            _pendingFirstCase = true;      // let every other Start() (SAMBallastTrim at order 110) run first
        }

        /// SAMHydrodynamicsV2 lives in the SAM assembly; this file deliberately does not reference
        /// it (same reasoning as WaveSweepRig.SilenceCompetingWriters). Bind the public fields we
        /// read and write by name, once, and tolerate every one of them being absent.
        void BindHydro()
        {
            _hydro = Vehicle.GetComponentsInChildren<MonoBehaviour>(true)
                            .FirstOrDefault(m => m != null && m.GetType().Name == "SAMHydrodynamicsV2");
            if (_hydro == null) return;
            foreach (var f in _hydro.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance))
                _hf[f.Name] = f;
            var la = HGet<Vector3>("LongAxis");
            if (la.sqrMagnitude > 1e-6f) _axisLocal = la.normalized;
        }

        T HGet<T>(string name)
        {
            if (_hydro == null || !_hf.TryGetValue(name, out var f) || !typeof(T).IsAssignableFrom(f.FieldType))
                return default;
            return (T)f.GetValue(_hydro);
        }

        bool HSet(string name, string value)
        {
            if (_hydro == null) return false;
            FieldInfo f;
            if (!_hf.TryGetValue(name, out f))
            {
                // case-insensitive second chance, so a config typo in capitalisation still lands
                var k = _hf.Keys.FirstOrDefault(x => string.Equals(x, name, StringComparison.OrdinalIgnoreCase));
                if (k == null) { Debug.LogWarning($"[PitchSpeedRig] hydro field '{name}' does not exist — ignored."); return false; }
                f = _hf[k];
            }
            if (!_hydroOriginal.ContainsKey(f.Name)) _hydroOriginal[f.Name] = f.GetValue(_hydro);
            var ci = CultureInfo.InvariantCulture;
            try
            {
                if (f.FieldType == typeof(float)) f.SetValue(_hydro, float.Parse(value, NumberStyles.Float, ci));
                else if (f.FieldType == typeof(int)) f.SetValue(_hydro, int.Parse(value, ci));
                else if (f.FieldType == typeof(bool)) f.SetValue(_hydro, value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase));
                else { Debug.LogWarning($"[PitchSpeedRig] hydro field '{f.Name}' is {f.FieldType.Name}, not settable from config."); return false; }
            }
            catch (Exception e) { Debug.LogWarning($"[PitchSpeedRig] hydro '{f.Name}={value}': {e.Message}"); return false; }
            return true;
        }

        /// Put every field any case has touched back to what the prefab had, before applying the
        /// next case's own list. Without this an A/B pair leaks in one direction only.
        void RestoreHydro()
        {
            foreach (var kv in _hydroOriginal)
                if (_hf.TryGetValue(kv.Key, out var f)) f.SetValue(_hydro, kv.Value);
        }

        void SilenceCompetingWriters()
        {
            string[] kill = { "Teleporter_Sub", "SAMKeyboardControl", "SAMTankReplay", "SAMDofTest",
                              "Actuator_Sub", "WaveSweepRig", "WaveTestLogger", "SAMReplay", "SAMMissionReplay" };
            int n = 0;
            foreach (var mb in Vehicle.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb == null || mb == this) continue;
                if (kill.Contains(mb.GetType().Name) && mb.enabled) { mb.enabled = false; n++; }
            }
            if (n > 0) Debug.Log($"[PitchSpeedRig] disabled {n} competing writer(s) for the run.");
        }

        bool ReadConfig()
        {
            string path = Path.Combine(_dir, "config.txt");
            if (!File.Exists(path)) { Debug.LogError($"[PitchSpeedRig] no config at {path}"); return false; }
            var ci = CultureInfo.InvariantCulture;
            foreach (var raw in File.ReadAllLines(path))
            {
                string line = raw.Trim();
                if (line.Length == 0 || line.StartsWith("#")) continue;
                int eq = line.IndexOf('='); if (eq < 0) continue;
                string k = line.Substring(0, eq).Trim().ToLowerInvariant(), v = line.Substring(eq + 1).Trim();
                float f; int i; bool b;
                switch (k)
                {
                    case "run": if (bool.TryParse(v, out b)) Run = b; else if (int.TryParse(v, out i)) Run = i != 0; break;
                    case "settle": if (float.TryParse(v, NumberStyles.Float, ci, out f)) SettleSeconds = f; break;
                    case "spinup": if (float.TryParse(v, NumberStyles.Float, ci, out f)) SpinupSeconds = f; break;
                    case "record": if (float.TryParse(v, NumberStyles.Float, ci, out f)) RecordSeconds = f; break;
                    case "depth": if (float.TryParse(v, NumberStyles.Float, ci, out f)) DepthM = f; break;
                    case "heading":
                        if (v.Equals("scene", StringComparison.OrdinalIgnoreCase)) UseSceneHeading = true;
                        else if (float.TryParse(v, NumberStyles.Float, ci, out f)) { HeadingDeg = f; UseSceneHeading = false; }
                        break;
                    case "calm_water": if (int.TryParse(v, out i)) ForceCalmWater = i != 0; break;
                    case "wind_kmh": if (float.TryParse(v, NumberStyles.Float, ci, out f)) WindKmh = f; break;
                    case "chaos": if (float.TryParse(v, NumberStyles.Float, ci, out f)) Chaos = f; break;
                    case "ripples": if (int.TryParse(v, out i)) Ripples = i != 0; break;
                    case "neutral_vbs": if (float.TryParse(v, NumberStyles.Float, ci, out f)) NeutralVbsPercent = f; break;
                    case "neutral_lcg": if (float.TryParse(v, NumberStyles.Float, ci, out f)) NeutralLcgPercent = f; break;
                    case "surge_kp": if (float.TryParse(v, NumberStyles.Float, ci, out f)) SurgeKp = f; break;
                    case "surge_ki": if (float.TryParse(v, NumberStyles.Float, ci, out f)) SurgeKi = f; break;
                    case "depth_kp": if (float.TryParse(v, NumberStyles.Float, ci, out f)) DepthKp = f; break;
                    case "depth_ki": if (float.TryParse(v, NumberStyles.Float, ci, out f)) DepthKi = f; break;
                    case "depth_kd": if (float.TryParse(v, NumberStyles.Float, ci, out f)) DepthKd = f; break;
                    case "settle_after_release": if (float.TryParse(v, NumberStyles.Float, ci, out f)) SettleAfterReleaseSeconds = f; break;
                    case "abort_depth_error": if (float.TryParse(v, NumberStyles.Float, ci, out f)) AbortDepthError = f; break;
                    case "abort_pitch_deg": if (float.TryParse(v, NumberStyles.Float, ci, out f)) AbortPitchDeg = f; break;
                    case "abort_speed": if (float.TryParse(v, NumberStyles.Float, ci, out f)) AbortSpeed = f; break;
                    case "exit_play": if (bool.TryParse(v, out b)) ExitPlayWhenDone = b; break;
                    case "case":
                    {
                        var p = v.Split(',');
                        if (p.Length < 2) { Debug.LogWarning($"[PitchSpeedRig] bad case line: {line}"); break; }
                        _cases.Add(new Case
                        {
                            Label = p[0].Trim(),
                            SpeedMps = float.Parse(p[1], NumberStyles.Float, ci),
                            Hydro = p.Length > 2 ? p[2].Trim() : "",
                            VbsPercent = p.Length > 3 && p[3].Trim().Length > 0 ? float.Parse(p[3], NumberStyles.Float, ci) : float.NaN,
                            LcgPercent = p.Length > 4 && p[4].Trim().Length > 0 ? float.Parse(p[4], NumberStyles.Float, ci) : float.NaN,
                        });
                        break;
                    }
                    default: Debug.LogWarning($"[PitchSpeedRig] unknown config key '{k}' — ignored."); break;
                }
            }
            if (_cases.Count == 0) { Debug.LogError("[PitchSpeedRig] config has no case= lines."); return false; }
            return true;
        }

        // ---------------------------------------------------------------- cases

        float CaseVbs() => float.IsNaN(_cases[CaseIndex].VbsPercent) ? NeutralVbsPercent : _cases[CaseIndex].VbsPercent;
        float CaseLcg() => float.IsNaN(_cases[CaseIndex].LcgPercent) ? NeutralLcgPercent : _cases[CaseIndex].LcgPercent;

        void BeginCase(int idx)
        {
            CaseIndex = idx;
            var c = _cases[idx];

            RestoreHydro();
            int applied = 0;
            foreach (var item in c.Hydro.Split(';').Select(x => x.Trim()).Where(x => x.Length > 0))
            {
                int e = item.IndexOf('='); if (e < 0) continue;
                if (HSet(item.Substring(0, e).Trim(), item.Substring(e + 1).Trim())) applied++;
            }

            _root.immovable = true;
            _root.TeleportRoot(new Vector3(_home.x, _stillWaterY - DepthM, _home.z), _heading);
            _root.linearVelocity = Vector3.zero;
            _root.angularVelocity = Vector3.zero;

            _surgeI = 0f; _depthI = 0f; SurgeForceN = 0f; HoldForceN = 0f; _abort = "";
            _phase = 0; _tPhase = 0f; _tRec = 0f; _rows = 0;
            CommandActuators();

            OpenWriter(c, applied);
            Debug.Log($"[PitchSpeedRig] case {idx + 1}/{_cases.Count} '{c.Label}': U {c.SpeedMps} m/s, " +
                      $"VBS {CaseVbs()}%, LCG {CaseLcg()}%, hydro [{(c.Hydro.Length == 0 ? "prefab" : c.Hydro)}] " +
                      $"({applied} applied)");
        }

        void OpenWriter(Case c, int hydroApplied)
        {
            var ci = CultureInfo.InvariantCulture;
            _w = new StreamWriter(Path.Combine(_outDir, $"pitch_{c.Label}.csv"), false, Encoding.UTF8);
            _w.WriteLine("# PitchSpeedRig");
            _w.WriteLine("# utc," + DateTime.UtcNow.ToString("o"));
            // The RUN's identity, stamped once in Start and repeated in every case. `utc` above is
            // when THIS case opened, which is 14 minutes of distinct timestamps over one sweep --
            // useless for telling one run from another, which is exactly what a reader needs to know
            // before it compares block A from one sweep with block C from the next.
            _w.WriteLine("# run_utc," + _runUtc);
            _w.WriteLine("# scene," + UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
            _w.WriteLine("# case," + c.Label);
            _w.WriteLine("# target_speed_mps," + c.SpeedMps.ToString("R", ci));
            _w.WriteLine("# target_depth_m," + DepthM.ToString("R", ci) + ",heading_deg," + HeadingDeg.ToString("R", ci));
            _w.WriteLine("# commanded_vbs_pct," + CaseVbs().ToString("R", ci) + ",commanded_lcg_pct," + CaseLcg().ToString("R", ci));
            _w.WriteLine("# fixedDeltaTime," + Time.fixedDeltaTime.ToString("R", ci) + ",still_water_y," + _stillWaterY.ToString("R", ci));
            _w.WriteLine("# vehicle," + _root.name + ",total_mass_kg," + LiveTotalMassKg().ToString("R", ci));
            _w.WriteLine("# hydro_case_overrides," + (c.Hydro.Length == 0 ? "none" : c.Hydro.Replace(',', '|')) + ",applied," + hydroApplied);
            // The physics this case ACTUALLY ran with, read back off the component after the
            // overrides landed. A case that only records what it ASKED for cannot be trusted:
            // three of these switches are not serialized in the prefab at all and default true.
            foreach (var n in new[] { "MunkCoefficient", "CenterOfPressureOffset", "UseSlenderBodyFlux",
                                      "UseBodyLift", "UseRingWing", "UseDeflectedRingLift", "UseAddedMass",
                                      "TailEffectiveness", "ThrustArm", "Mass", "Length", "Diameter",
                                      "WaterDensity", "FluxStations" })
                if (_hf.TryGetValue(n, out var f))
                    _w.WriteLine($"# hydro,{n}," + Convert.ToString(f.GetValue(_hydro), ci));
            _w.WriteLine("# hydro_thrust_transform," + (HGet<Transform>("ThrustTransform") == null ? "null" : HGet<Transform>("ThrustTransform").name));
            // THE COEFFICIENT SET, BY NAME AND BY VALUE. MEASURED 2026-09-15: the first complete sweep
            // ran with hydro_overrides.txt tag 'bare25' live -- 11 coefficients replaced at Awake,
            // including Zww 140 against the prefab's 50 -- and NONE of it reached the header, because
            // the header logged the scalars this rig switches and not the arrays a file can rewrite.
            // A run that cannot name its own coefficients is not reproducible, whatever else it records.
            _w.WriteLine("# hydro_override_tag," + (string.IsNullOrEmpty(HGet<string>("OverrideTag")) ? "none" : HGet<string>("OverrideTag"))
                         + ",file," + (string.IsNullOrEmpty(HGet<string>("OverrideFile")) ? "none" : HGet<string>("OverrideFile")));
            WriteArray("DLin"); WriteArray("DQuad"); WriteArray("AddedMass"); WriteArray("LongAxis");
            _w.WriteLine("# rig_forces_at_com,1,pitch_moment_injected_by_rig_Nm,0");
            // FRAME. u_ax is the speed along the hull axis (LongAxis), which is the only velocity
            // component whose meaning is basis-independent. v_bx/w_by and p_bz/q_bx/r_by are the
            // raw body-LOCAL UNITY components, not Fossen's (nu1..nu6): the hydro builds its own
            // (axisN, t1, t2) basis at Awake from LongAxis, and duplicating that construction here
            // is how the two silently disagree about a sign. With LongAxis = +Z, p_bz is roll.
            // tau_l* is LastTorqueLocal, ALSO in the body-local Unity basis, and it carries only
            // the damping + explicit-Munk torque -- the flux stations, body lift and ring fin are
            // applied as forces at stations and appear in flux_N / bodylift_N / ringfin_N instead.
            _w.WriteLine("# frame,u_ax_along_LongAxis,body_local_unity_axes_for_v_w_p_q_r_and_tau");
            _w.WriteLine(string.Join(",", new[] {
                "t","phase","x","y","z","depth","roll","pitch","yaw",
                "u_ax","v_bx","w_by","p_bz","q_bx","r_by","alpha_deg","speed_world",
                "F_surge","F_hold","vbs_pct","lcg_pct","lcg_actual",
                "sub_frac","spd_water","thrust_N","flux_N","bodylift_N","ringfin_N","ringctl_N",
                "tau_lx","tau_ly","tau_lz" }));
        }

        /// One header line per coefficient array, straight off the component after every override has
        /// landed. Vector3 and float[] both, so LongAxis travels with the damping it multiplies.
        void WriteArray(string name)
        {
            if (_hydro == null || !_hf.TryGetValue(name, out var f)) return;
            var ci = CultureInfo.InvariantCulture;
            object v = f.GetValue(_hydro);
            if (v is float[] a) _w.WriteLine($"# hydro_array,{name}," + string.Join(",", a.Select(x => x.ToString("R", ci))));
            else if (v is Vector3 q) _w.WriteLine($"# hydro_array,{name},{q.x.ToString("R", ci)},{q.y.ToString("R", ci)},{q.z.ToString("R", ci)}");
        }

        void CloseWriter(Case c)
        {
            if (_w == null) return;
            var ci = CultureInfo.InvariantCulture;
            _w.Write(_sb.ToString()); _sb.Clear();
            _w.WriteLine("# rows," + _rows);
            _w.WriteLine("# abort," + (_abort == "" ? "none" : _abort));
            _w.Flush(); _w.Close(); _w = null;
            _summary.Add($"{c.Label},{c.SpeedMps.ToString("R", ci)}," +
                         $"{SpeedU.ToString("F4", ci)},{PitchDeg.ToString("F3", ci)}," +
                         $"{HoldForceN.ToString("F3", ci)},{SurgeForceN.ToString("F3", ci)},{_rows}," +
                         (_abort == "" ? "ok" : "\"" + _abort + "\"") + "," +
                         (c.Hydro.Length == 0 ? "prefab" : "\"" + c.Hydro + "\""));
            Debug.Log($"[PitchSpeedRig] case '{c.Label}' done: {_rows} rows, final U {SpeedU:F3} m/s, pitch {PitchDeg:+0.00;-0.00} deg");
        }

        // ---------------------------------------------------------------- loop

        void FixedUpdate()
        {
            if (_done) return;
            if (_pendingFirstCase) { _pendingFirstCase = false; BeginCase(0); return; }
            if (_w == null) return;

            var c = _cases[CaseIndex];
            float dt = Time.fixedDeltaTime;
            _tPhase += dt;

            CommandActuators();
            SilenceThrusterAndFins();

            Vector3 axisW = _root.transform.TransformDirection(_axisLocal).normalized;
            Vector3 vel = _root.linearVelocity;
            SpeedU = Vector3.Dot(vel, axisW);
            PitchDeg = Wrap(_root.transform.rotation.eulerAngles.x);
            float yTarget = _stillWaterY - DepthM;

            switch (_phase)
            {
                case 0:
                    Phase = "SET";
                    if (_tPhase >= 0.5f) { _phase = 1; _tPhase = 0f; }
                    break;

                case 1:
                {
                    Phase = "SETTLE";
                    float vbsNow = VbsActual(), lcgNow = LcgActual();
                    bool there = (float.IsNaN(vbsNow) || Mathf.Abs(vbsNow - CaseVbs()) <= ArrivedTolerance)
                              && (float.IsNaN(lcgNow) || Mathf.Abs(lcgNow - CaseLcg()) <= ArrivedTolerance);
                    if (_tPhase >= SettleSeconds && (there || _tPhase >= SettleSeconds + 30f))
                    {
                        if (!there)
                            Debug.LogWarning($"[PitchSpeedRig] '{c.Label}': pistons never arrived " +
                                             $"(VBS {vbsNow:F1}/{CaseVbs():F1}, LCG {lcgNow:F1}/{CaseLcg():F1}) — continuing.");
                        // Seed the depth integrator with the static imbalance the strips report,
                        // so it does not spend the spin-up winding up from zero.
                        float wN = LiveTotalMassKg() * Mathf.Abs(Physics.gravity.y), bN = LastBuoyancySum();
                        _depthI = Mathf.Clamp(wN - bN, -MaxHoldForce, MaxHoldForce);
                        _root.immovable = false;
                        _phase = 2; _tPhase = 0f;
                        // W and B, not just their difference. MEASURED 2026-09-15: the first Djurö run
                        // seeded -165.30 N on every case -- exactly -W, the signature of a subtree with
                        // no mass in it -- and drove the vehicle onto the seabed. Printed as a
                        // difference alone, that number says nothing; printed as W and B it names the
                        // broken one immediately.
                        Debug.Log($"[PitchSpeedRig] '{c.Label}': released. VBS {vbsNow:F1}%, LCG {lcgNow:F1}%, " +
                                  $"W {wN:F2} N ({LiveTotalMassKg():F3} kg over {_root.GetComponentsInChildren<ArticulationBody>(true).Length} links), " +
                                  $"B {bN:F2} N ({_points.Length} ForcePoints) -> depth-hold seed {_depthI:F2} N.");
                        if (wN < 1f || bN < 1f)
                            Debug.LogError($"[PitchSpeedRig] W or B is zero — the rig is not holding the body it " +
                                           $"thinks it is. Stop; this run will drive the vehicle into the seabed.");
                    }
                    break;
                }

                case 2:
                    Phase = "SPINUP";
                    DriveAndHold(c, axisW, yTarget, dt);
                    if (_tPhase >= SpinupSeconds) { _phase = 3; _tPhase = 0f; }
                    break;

                case 3:
                    Phase = "RECORD";
                    DriveAndHold(c, axisW, yTarget, dt);
                    _tRec += dt;
                    break;
            }

            CheckSanity(c, yTarget);
            LogRow(c, axisW, vel);

            if (_abort != "" || (_phase == 3 && _tRec >= RecordSeconds))
            {
                CloseWriter(c);
                if (CaseIndex + 1 < _cases.Count) BeginCase(CaseIndex + 1);
                else Finish();
            }
        }

        /// The two rig forces. BOTH go through ArticulationBody.AddForce, which applies at the
        /// centre of mass — so the pitch axis sees nothing from either of them. Do not "improve"
        /// this into AddForceAtPosition; that would silently make the whole measurement circular.
        void DriveAndHold(Case c, Vector3 axisW, float yTarget, float dt)
        {
            float eU = c.SpeedMps - SpeedU;
            _surgeI = Mathf.Clamp(_surgeI + SurgeKi * eU * dt, -MaxSurgeForce, MaxSurgeForce);
            SurgeForceN = Mathf.Clamp(SurgeKp * eU + _surgeI, -MaxSurgeForce, MaxSurgeForce);
            _root.AddForce(axisW * SurgeForceN);

            float eY = yTarget - _root.transform.position.y;
            _depthI = Mathf.Clamp(_depthI + DepthKi * eY * dt, -MaxHoldForce, MaxHoldForce);
            HoldForceN = Mathf.Clamp(DepthKp * eY + _depthI - DepthKd * _root.linearVelocity.y,
                                     -MaxHoldForce, MaxHoldForce);
            _root.AddForce(Vector3.up * HoldForceN);
        }

        void CommandActuators()
        {
            if (_vbs != null) _vbs.percentage = CaseVbs();
            if (_lcg != null) _lcg.percentage = CaseLcg();
        }

        /// No propeller, no sterngear. A thruster would reintroduce the arm this test exists to
        /// rule out; a deflected ring would hold the angle instead of letting it find itself.
        void SilenceThrusterAndFins()
        {
            if (_hydro == null) return;
            SetF("Prop1eRPM", 0f); SetF("Prop2eRPM", 0f); SetF("CommandRPM", 0f);
            SetF("Delta1", 0f); SetF("Delta2", 0f);
            if (_hf.TryGetValue("UseCommandRPM", out var u)) u.SetValue(_hydro, false);
            // ThrustSource = ExternalERPM (0). On PropellerCommandRPM the hydro reads the rpm off
            // the Propeller components and the fields zeroed above are never consulted — the
            // thruster would still be running while this file claimed it was off.
            if (_hf.TryGetValue("ThrustSource", out var ts) && ts.FieldType.IsEnum)
                ts.SetValue(_hydro, Enum.ToObject(ts.FieldType, 0));
        }

        void SetF(string n, float v) { if (_hf.TryGetValue(n, out var f) && f.FieldType == typeof(float)) f.SetValue(_hydro, v); }

        float LiveTotalMassKg()
        {
            float m = 0f;
            foreach (var b in _root.GetComponentsInChildren<ArticulationBody>(true)) m += b.mass;
            return m;
        }

        /// Non-finite ForcePoint samples are real (water queries made before HDRP's CPU simulation
        /// is up return error = FLT_MAX). One summed in is enough to turn the seed into NaN.
        float LastBuoyancySum()
        {
            float b = 0f;
            for (int i = 0; i < _points.Length; ++i)
            {
                if (!_points[i].IsUnderwater) continue;
                float v = _points[i].AppliedBuoyancyForce.y;
                if (float.IsNaN(v) || float.IsInfinity(v)) continue;
                b += v;
            }
            return b;
        }

        float VbsActual() { if (_vbs == null) return CaseVbs(); try { return _vbs.GetCurrentValue(); } catch { return float.NaN; } }
        float LcgActual() { if (_lcg == null) return CaseLcg(); try { return _lcg.GetCurrentValue(); } catch { return float.NaN; } }

        static float Wrap(float a) => a > 180f ? a - 360f : a;

        static bool IsFinite(Vector3 q) =>
            !(float.IsNaN(q.x) || float.IsNaN(q.y) || float.IsNaN(q.z) ||
              float.IsInfinity(q.x) || float.IsInfinity(q.y) || float.IsInfinity(q.z));

        void CheckSanity(Case c, float yTarget)
        {
            // Not before release, and not in the first SettleAfterReleaseSeconds of free flight: the
            // step the body is let go on has a transient in it, and killing a case on that transient
            // is how a whole matrix comes back empty.
            if (_phase < 2 || _abort != "" || (_phase == 2 && _tPhase < SettleAfterReleaseSeconds)) return;
            Vector3 p = _root.transform.position, v = _root.linearVelocity;
            string why = null;
            if (!IsFinite(p) || !IsFinite(v)) why = "non-finite state";
            else if (Mathf.Abs(p.y - yTarget) > AbortDepthError) why = $"depth error {p.y - yTarget:F2} m";
            else if (Mathf.Abs(PitchDeg) > AbortPitchDeg) why = $"pitch {PitchDeg:F1} deg";
            else if (v.magnitude > AbortSpeed) why = $"speed {v.magnitude:F2} m/s";
            if (why == null) return;

            _abort = why; LastAbort = $"{c.Label}: {why}";
            _root.immovable = true;
            _root.linearVelocity = Vector3.zero;
            _root.angularVelocity = Vector3.zero;
            // A pitch abort is a RESULT here, not a failure: above U* the angle is supposed to run.
            // Name the phase from _phase, not the Phase string: the string is only rewritten at the
            // top of the next step's switch, so on a transition step it still says the old one, and
            // the first Djurö run reported "stopped in SETTLE" for an abort that happened in SPINUP.
            string ph = _phase == 2 ? "SPINUP" : _phase == 3 ? "RECORD" : Phase;
            Debug.LogWarning($"[PitchSpeedRig] case '{c.Label}' stopped in {ph} at t={(_phase == 3 ? _tRec : _tPhase):F1}s: {why}");
        }

        void LogRow(Case c, Vector3 axisW, Vector3 vel)
        {
            var ci = CultureInfo.InvariantCulture;
            var tr = _root.transform;
            Vector3 vb = tr.InverseTransformDirection(vel);          // body frame, Unity axes
            Vector3 wb = tr.InverseTransformDirection(_root.angularVelocity);
            Vector3 p = tr.position, e = tr.rotation.eulerAngles;
            Vector3 tau = HGet<Vector3>("LastTorqueLocal");
            // angle of attack in the vertical plane: the ANGLE the hull actually flies at, which
            // is the quantity the Munk couple multiplies. Depth-held level flight makes it equal
            // to the pitch angle; logging both means that identity is checked, not assumed.
            float alpha = Mathf.Abs(SpeedU) < 1e-3f ? 0f
                        : Mathf.Atan2(-Vector3.Dot(vel, tr.up), Mathf.Abs(SpeedU)) * Mathf.Rad2Deg;

            _sb.Append((_phase == 3 ? _tRec : -_tPhase).ToString("F4", ci))
               .Append(',').Append(_phase)
               .Append(',').Append(p.x.ToString("F4", ci))
               .Append(',').Append(p.y.ToString("F5", ci))
               .Append(',').Append(p.z.ToString("F4", ci))
               .Append(',').Append((_stillWaterY - p.y).ToString("F5", ci))
               .Append(',').Append(Wrap(e.z).ToString("F3", ci))
               .Append(',').Append(Wrap(e.x).ToString("F3", ci))
               .Append(',').Append(Wrap(e.y).ToString("F3", ci))
               .Append(',').Append(SpeedU.ToString("F5", ci))
               .Append(',').Append(vb.x.ToString("F5", ci))
               .Append(',').Append(vb.y.ToString("F5", ci))
               .Append(',').Append(wb.z.ToString("F5", ci))
               .Append(',').Append(wb.x.ToString("F5", ci))
               .Append(',').Append(wb.y.ToString("F5", ci))
               .Append(',').Append(alpha.ToString("F4", ci))
               .Append(',').Append(vel.magnitude.ToString("F5", ci))
               .Append(',').Append(SurgeForceN.ToString("F3", ci))
               .Append(',').Append(HoldForceN.ToString("F3", ci))
               .Append(',').Append(VbsActual().ToString("F2", ci))
               .Append(',').Append(CaseLcg().ToString("F2", ci))
               .Append(',').Append(LcgActual().ToString("F3", ci))
               .Append(',').Append(HGet<float>("SubmergedFraction").ToString("F4", ci))
               .Append(',').Append(HGet<float>("SpeedThroughWater").ToString("F4", ci))
               .Append(',').Append(HGet<float>("ThrustForce").ToString("F4", ci))
               .Append(',').Append(HGet<float>("FluxForceN").ToString("F4", ci))
               .Append(',').Append(HGet<float>("BodyLiftForceN").ToString("F4", ci))
               .Append(',').Append(HGet<float>("RingFinForceN").ToString("F4", ci))
               .Append(',').Append(HGet<float>("RingControlForceN").ToString("F4", ci))
               .Append(',').Append(tau.x.ToString("F5", ci))
               .Append(',').Append(tau.y.ToString("F5", ci))
               .Append(',').Append(tau.z.ToString("F5", ci))
               .Append('\n');

            if (++_rows % 250 == 0) { _w.Write(_sb.ToString()); _sb.Clear(); _w.Flush(); }
        }

        // ---------------------------------------------------------------- end

        void WriteManifestHeader()
        {
            var ci = CultureInfo.InvariantCulture;
            using (var m = new StreamWriter(Path.Combine(_outDir, "manifest.csv"), false, Encoding.UTF8))
            {
                m.WriteLine("# PitchSpeedRig manifest, " + DateTime.UtcNow.ToString("o"));
                m.WriteLine("# scene," + UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
                m.WriteLine("# depth," + DepthM.ToString("R", ci) + ",heading," + HeadingDeg.ToString("R", ci) +
                            ",settle," + SettleSeconds.ToString("R", ci) + ",spinup," + SpinupSeconds.ToString("R", ci) +
                            ",record," + RecordSeconds.ToString("R", ci) +
                            ",neutral_vbs," + NeutralVbsPercent.ToString("R", ci) +
                            ",neutral_lcg," + NeutralLcgPercent.ToString("R", ci));
                m.WriteLine("label,target_u_mps,final_u_mps,final_pitch_deg,hold_force_N,surge_force_N,rows,status,hydro");
            }
        }

        void Finish()
        {
            _done = true; Phase = "done";
            RestoreHydro();
            using (var m = new StreamWriter(Path.Combine(_outDir, "manifest.csv"), true, Encoding.UTF8))
                foreach (var s in _summary) m.WriteLine(s);
            Debug.Log($"[PitchSpeedRig] SWEEP COMPLETE — {_summary.Count} cases -> {_outDir}");
#if UNITY_EDITOR
            if (ExitPlayWhenDone) UnityEditor.EditorApplication.isPlaying = false;
#endif
        }

        void OnDisable()
        {
            if (_w != null && !_done) CloseWriter(_cases[CaseIndex]);
            if (_hydro != null) RestoreHydro();
        }
    }
}
