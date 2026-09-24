// WaveRigSetup — puts the 2026-09-23 wave instruments into the OPEN scene, in one menu item, so a
// measurement session is a repeatable act and not a sequence of clicks.
//
//   SMARC / Wave / Set Up Wave Rig In This Scene
//
// What it does, all of it logged and all of it reversible by not saving the scene:
//   1. finds the scene's SiteEnvironment + WaterSurface and reports the water the physics will read
//      (script interactions, ripple band, SiteWater density);
//   2. raises Repetition Size if the longest case's wavelength would not fit in the patch — HDRP's
//      FFT patch is tiled, so a 264 m swell in a 250 m patch is not a swell, it is a ripple;
//   3. creates `WaveRig` at the vehicle's station carrying SeaStateVerifier (the sweep) and
//      SurfaceParityMarkers (the physics-vs-pixels gate);
//   4. quiets the wiring that would otherwise fight or spam the run: ROS publishers/subscribers on
//      the vehicle (there is no endpoint in a measurement session). EnvironmentTimeline and live AIS
//      are left alone here — SeaStateVerifier suspends those itself, for the duration of the run
//      only, so the scene you keep is the scene you had.
//
// It never saves the scene and never touches a prefab asset.

using System.Collections.Generic;
using System.Linq;
using Diagnostics;
using Smarc.Environment;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace SmarcEditor
{
    public static class WaveRigSetup
    {
        const string RigName = "WaveRig";

        [MenuItem("SMARC/Wave/Set Up Wave Rig In This Scene")]
        public static void SetUp()
        {
            var log = new List<string>();

            var env = Object.FindFirstObjectByType<SiteEnvironment>();
            if (env == null) { EditorUtility.DisplayDialog("Wave rig", "No SiteEnvironment in this scene.\n\nIt lives on the world's Ocean and is written by the site builder.", "OK"); return; }
            var ocean = env.Ocean != null ? env.Ocean : env.GetComponent<WaterSurface>();
            if (ocean == null) { EditorUtility.DisplayDialog("Wave rig", "SiteEnvironment has no Ocean (WaterSurface).", "OK"); return; }

            // --- the water the PHYSICS will read -------------------------------------------------
            Undo.RecordObject(ocean, "wave rig");
            if (!ocean.scriptInteractions) { ocean.scriptInteractions = true; log.Add("Ocean: scriptInteractions was OFF -> on (the physics had no waves at all)"); }
            if (ocean.ripples && !ocean.cpuEvaluateRipples) { ocean.cpuEvaluateRipples = true; log.Add("Ocean: cpuEvaluateRipples was OFF -> on (the rendered ripple band was not in the queried surface)"); }
            var sw = ocean.GetComponent<SiteWater>();
            log.Add($"water density: {(sw != null ? $"{sw.Density_kgm3:F1} kg/m3 ({sw.Source})" : "NO SiteWater on the Ocean — everything will default to 997 fresh")}");

            // --- the patch must hold the longest wave ---------------------------------------------
            var verifierExisting = Object.FindFirstObjectByType<SeaStateVerifier>();
            float longestTp = LongestTp(verifierExisting != null ? verifierExisting.Cases : DefaultCases());
            float lambda = 9.81f * longestTp * longestTp / (2f * Mathf.PI);
            float needed = Mathf.Clamp(Mathf.Ceil(2.5f * lambda / 50f) * 50f, 250f, 5000f);
            if (ocean.repetitionSize < needed)
            {
                log.Add($"Ocean: Repetition Size {ocean.repetitionSize:F0} -> {needed:F0} m (the longest case is Tp {longestTp:F1} s = {lambda:F0} m; a tiled patch shorter than ~2.5 wavelengths truncates the spectral peak)");
                ocean.repetitionSize = needed;
            }
            else log.Add($"Ocean: Repetition Size {ocean.repetitionSize:F0} m holds the longest case (Tp {longestTp:F1} s, {lambda:F0} m)");
            EditorUtility.SetDirty(ocean);

            // --- the vehicle -----------------------------------------------------------------------
            GameObject vehicle = Object.FindObjectsByType<Force.ForcePoint>(FindObjectsSortMode.None)
                .Select(p => p.transform.root.gameObject).Distinct().FirstOrDefault();
            if (vehicle == null) log.Add("NO VEHICLE with ForcePoints found — the sea will be measured, the response will not");
            else
            {
                int killed = 0;
                foreach (var mb in vehicle.GetComponentsInChildren<MonoBehaviour>(true))
                {
                    if (mb == null) continue;
                    string t = mb.GetType().Name;
                    bool wire = t.EndsWith("_Pub") || t.EndsWith("_Sub") || t.StartsWith("ROS") || t.Contains("Publisher") || t.Contains("Subscriber");
                    if (wire && mb.enabled) { Undo.RecordObject(mb, "wave rig"); mb.enabled = false; killed++; }
                }
                log.Add($"vehicle '{vehicle.name}' at {vehicle.transform.position}: disabled {killed} ROS publisher/subscriber component(s) (no endpoint in a measurement session)");
                var replay = vehicle.GetComponentInChildren<Force.SAMTankReplay>(true);
                if (replay != null && replay.enabled && replay.RunOnStart)
                { Undo.RecordObject(replay, "wave rig"); replay.enabled = false; log.Add("vehicle: SAMTankReplay was ENABLED and would have replayed a KTH tank run over this site — disabled"); }
            }

            // --- the rig ---------------------------------------------------------------------------
            var rigGo = GameObject.Find(RigName);
            if (rigGo == null) { rigGo = new GameObject(RigName); Undo.RegisterCreatedObjectUndo(rigGo, "wave rig"); log.Add($"created '{RigName}'"); }
            Vector3 station = vehicle != null ? vehicle.transform.position : Vector3.zero;
            rigGo.transform.position = new Vector3(station.x, 0f, station.z);

            var ver = rigGo.GetComponent<SeaStateVerifier>() ?? Undo.AddComponent<SeaStateVerifier>(rigGo);
            Undo.RecordObject(ver, "wave rig");
            ver.Environment = env;
            ver.Vehicle = vehicle;
            ver.ExitPlayWhenDone = false;
            var markers = rigGo.GetComponent<SurfaceParityMarkers>() ?? Undo.AddComponent<SurfaceParityMarkers>(rigGo);
            Undo.RecordObject(markers, "wave rig");
            markers.Count = 25; markers.Span_m = 30f; markers.Diameter_m = 0.25f;
            EditorUtility.SetDirty(ver); EditorUtility.SetDirty(markers);
            log.Add($"rig at ({rigGo.transform.position.x:F1}, 0, {rigGo.transform.position.z:F1}): SeaStateVerifier ({ver.Cases.Split('\n').Count(l => l.Trim().Length > 0 && !l.TrimStart().StartsWith("#"))} cases) + {markers.Count} parity markers over {markers.Span_m:F0} m");

            // --- what the sea state will be ----------------------------------------------------------
            env.Apply();
            log.Add($"SiteEnvironment now: drive {env.Drive}, realised Hs {env.RealisedHsPhysics_m:F3} m (physics) / {env.RealisedHsVisual_m:F3} m (visual), Tp {env.RealisedTp_s:F2} s, lambda_p {env.PeakWavelength_m:F0} m");

            Debug.Log("[WaveRigSetup] scene prepared (NOT saved):\n  - " + string.Join("\n  - ", log) +
                      "\n\nPress Play. The sweep writes _logs/wave/seastate/<utc>/. Nothing here is saved to disk or to any prefab.");
            Selection.activeGameObject = rigGo;
        }

        static string DefaultCases() => "hs18_tp13,seastate,1.8,13";

        static float LongestTp(string cases)
        {
            float tp = 4f;
            foreach (var raw in cases.Split('\n'))
            {
                string l = raw.Trim(); if (l.Length == 0 || l.StartsWith("#")) continue;
                var f = l.Split(','); if (f.Length < 4) continue;
                if (f[1].Trim().ToLowerInvariant() != "seastate") continue;
                if (float.TryParse(f[3].Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float v))
                    tp = Mathf.Max(tp, v);
            }
            return tp;
        }
    }
}
