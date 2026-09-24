// SiteWater — the density of the water at this site, owned in ONE place (Ivan, 2026-09-23: "I'd like
// the VBS to use the density at the site").
//
// Before this, the density lived in six places that did not agree: VBS.density (997, the mass of the
// water the tank takes in), every ForcePoint.WaterDensity (997), SAMBuoyancyTrim and SAMBallastTrim
// (each stamping its own value onto the ForcePoints), SAMHydrodynamics (1026) and SAMHydrodynamicsV2
// (997) — and a global StreamingAssets/SAMReplay/dof/config.txt `ballast_site=tank` that forced tank
// water into EVERY scene, Askö and Kristineberg included.
//
// Now: one SiteWater on each world prefab's Ocean says what the water is. Everything that needs a
// density asks SiteWater.Density() — VBS (the ballast water it takes in), ForcePoint (buoyancy),
// SAMBallastTrim / SAMBuoyancyTrim (their reports), SAMHydrodynamics / V2 (added mass, drag, thrust).
//
// THE ONE EXCEPTION IS A TANK REPLAY. A recorded KTH-tank run is fresh water wherever the scene is
// (the replay rig lives in AskoWaveTest), so while an enabled SAMTankReplay with RunOnStart is in the
// scene, the replay config's `ballast_site` / `ballast_density` wins and the log says so. Nowhere else
// does that file set the density any more.
//
// No SiteWater in the scene: 997 kg/m3 (fresh, KTH tank at ~24 °C) with a warning.

using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Smarc.Environment
{
    [DefaultExecutionOrder(-500)]
    [AddComponentMenu("Smarc/Environment/Site Water (density)")]
    public class SiteWater : MonoBehaviour
    {
        public const float FreshTankDensity = 997f;

        [Tooltip("Water density at this site, kg/m3. KTH tank 997 (fresh, ~24 °C, measured 997.17 on 2026-09-17); " +
                 "Baltic / Asko / Djuro / Beckholmen 1005 (brackish); Kristineberg / open ocean 1025.")]
        public float Density_kgm3 = FreshTankDensity;
        [Tooltip("Where the number comes from: measured, or assumed from the salinity class.")]
        [TextArea(1, 3)] public string Source = "";

        [Header("Resolved at Play (read-only)")]
        public float ResolvedDensity_kgm3;
        public string ResolvedFrom = "";

        static bool _resolved;
        static float _rho = FreshTankDensity;
        static string _from = "";

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetStatics()
        {
            _resolved = false;
            SceneManager.sceneLoaded -= OnSceneLoaded;
            SceneManager.sceneLoaded += OnSceneLoaded;
        }
        static void OnSceneLoaded(Scene s, LoadSceneMode m) { if (m == LoadSceneMode.Single) _resolved = false; }

        /// The density every component in this scene must use, kg/m3.
        public static float Density() => Density(out _);

        public static float Density(out string from)
        {
            if (!_resolved) Resolve();
            from = _from;
            return _rho;
        }

        static void Resolve()
        {
            var sites = FindObjectsByType<SiteWater>(FindObjectsSortMode.None);
            SiteWater site = sites.Length > 0 ? sites[0] : null;
            if (sites.Length > 1)
                Debug.LogWarning($"[SiteWater] {sites.Length} SiteWater components in the scene; using '{site.name}' ({site.Density_kgm3:F1}). " +
                                 "One site, one water — remove the others.");
            float rho; string from;
            if (site != null && site.Density_kgm3 > 900f && site.Density_kgm3 < 1100f)
            { rho = site.Density_kgm3; from = $"SiteWater on '{site.name}'" + (string.IsNullOrEmpty(site.Source) ? "" : $" ({site.Source})"); }
            else
            {
                rho = FreshTankDensity;
                from = site == null ? "no SiteWater in the scene — fresh-water default" : $"SiteWater on '{site.name}' has an implausible {site.Density_kgm3} — fresh-water default";
                Debug.LogWarning("[SiteWater] " + from + $" {rho:F0} kg/m3.");
            }

            if (TankReplayActive() && ReplayConfigDensity(out float cfgRho, out string cfgFrom))
            { rho = cfgRho; from = $"TANK REPLAY running — {cfgFrom} (the site's water is ignored for the replay)"; }

            _rho = rho; _from = from; _resolved = true;
            foreach (var s in sites) { s.ResolvedDensity_kgm3 = rho; s.ResolvedFrom = from; }
            Debug.Log($"[SiteWater] water density {rho:F1} kg/m3 — {from}. VBS, ForcePoints and hydrodynamics use it.");
        }

        static bool TankReplayActive()
        {
            foreach (var r in FindObjectsByType<Force.SAMTankReplay>(FindObjectsSortMode.None))
                if (r.enabled && r.isActiveAndEnabled && r.RunOnStart) return true;
            return false;
        }

        static bool ReplayConfigDensity(out float rho, out string from)
        {
            rho = FreshTankDensity; from = "";
            string cfg = Path.Combine(Application.streamingAssetsPath, "SAMReplay", "dof", "config.txt");
            if (!File.Exists(cfg)) { from = "no replay config — fresh"; return true; }
            bool found = false;
            foreach (string line in File.ReadAllLines(cfg))
            {
                string t = line.Trim(); if (t.Length == 0 || t.StartsWith("#")) continue;
                int eq = t.IndexOf('='); if (eq < 0) continue;
                string k = t.Substring(0, eq).Trim().ToLowerInvariant(), v = t.Substring(eq + 1).Trim();
                if (k == "ballast_site")
                {
                    switch (v.ToLowerInvariant())
                    {
                        case "tank": case "kth": case "fresh": rho = 997f; found = true; break;
                        case "asko": case "baltic": case "brackish": rho = 1005f; found = true; break;
                        case "kristineberg": case "ocean": case "westcoast": rho = 1025f; found = true; break;
                    }
                    from = $"replay config ballast_site={v}";
                }
                else if (k == "ballast_density" && float.TryParse(v, System.Globalization.NumberStyles.Float,
                             System.Globalization.CultureInfo.InvariantCulture, out float f))
                { rho = f; found = true; from = $"replay config ballast_density={v}"; }
            }
            if (!found) from = "replay config names no site — fresh";
            return true;
        }
    }
}
