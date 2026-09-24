using System;
using UnityEngine;

namespace Smarc.Environment
{
    /// <summary>
    /// The SUN from the ScenarioClock: its direction is COMPUTED from the clock's time and the site's
    /// latitude / longitude (NOAA Solar Calculator equations — the same as ovsite.timeline.solar_position,
    /// ~0.01°, refraction-corrected), so it is right at any time, inside the data window or not.
    ///
    /// Direction: azimuth is from TRUE north; Unity +z is UTM GRID north, so the clock's grid
    /// convergence is subtracted (the Beckholmen 2.2° lesson, 2026-08-10).
    /// Strength (optional): the light's intensity as built is taken as clear-sky high sun; it is
    /// scaled by sin(elevation) and by cloud cover (Kasten & Czeplak 1980: 1 - 0.75·(cover)^3.4),
    /// reading EnvironmentTimeline.CloudCoverNow_pct. Below the horizon the sun is off (civil
    /// twilight fades it out between 0° and -6°).
    /// </summary>
    [ExecuteAlways]
    [RequireComponent(typeof(Light))]
    [AddComponentMenu("Smarc/Environment/Sun From Clock")]
    public class SunFromClock : MonoBehaviour
    {
        public ScenarioClock Clock;
        public EnvironmentTimeline Weather;
        [Tooltip("Rotate this directional light to the computed sun direction.")]
        public bool ApplyDirection = true;
        [Tooltip("Scale the intensity with elevation and cloud cover (off = keep the intensity as built).")]
        public bool ApplyIntensity = true;
        [Tooltip("Intensity at high sun under a clear sky, in the light's own unit (captured from the light by the builder).")]
        public float ClearSkyIntensity = -1f;

        [Header("NOW (read-only)")]
        public float Elevation_deg;
        public float AzimuthTrue_deg;
        public float AzimuthGrid_deg;
        public float Intensity;
        public string StateNow = "";

        Light sun;
        DateTime last = DateTime.MinValue;

        void OnEnable() { sun = GetComponent<Light>(); if (ClearSkyIntensity < 0 && sun != null) ClearSkyIntensity = sun.intensity; last = DateTime.MinValue; }
        void OnValidate() { last = DateTime.MinValue; }

        void Update()
        {
            if (Clock == null) Clock = ScenarioClock.Find();
            if (Clock == null || sun == null) { StateNow = "no ScenarioClock in the scene"; return; }
            if (Math.Abs((Clock.Utc - last).TotalSeconds) < 10 && last != DateTime.MinValue) return;   // the sun moves 0.04° in 10 s
            last = Clock.Utc;
            SolarPosition(Clock.Utc, Clock.Latitude, Clock.Longitude, out var el, out var az);
            Elevation_deg = (float)el; AzimuthTrue_deg = (float)az;
            AzimuthGrid_deg = Mathf.Repeat(AzimuthTrue_deg - Clock.GridConvergence_deg, 360f);
            if (ApplyDirection)
            {
                float e = Elevation_deg * Mathf.Deg2Rad, a = AzimuthGrid_deg * Mathf.Deg2Rad;
                var toSun = new Vector3(Mathf.Sin(a) * Mathf.Cos(e), Mathf.Sin(e), Mathf.Cos(a) * Mathf.Cos(e));
                transform.rotation = Quaternion.LookRotation(-toSun, Vector3.up);         // a light shines along its +z
            }
            float cover = Weather != null && !float.IsNaN(Weather.CloudCoverNow_pct) ? Mathf.Clamp01(Weather.CloudCoverNow_pct / 100f) : 0f;
            float cloud = 1f - 0.75f * Mathf.Pow(cover, 3.4f);
            float geom = Elevation_deg > 0 ? Mathf.Sin(Elevation_deg * Mathf.Deg2Rad)
                       : Mathf.Clamp01((Elevation_deg + 6f) / 6f) * 0.02f;               // twilight glow, then night
            Intensity = ApplyIntensity && ClearSkyIntensity > 0 ? ClearSkyIntensity * Mathf.Clamp01(geom) * cloud : sun.intensity;
            if (ApplyIntensity && ClearSkyIntensity > 0) sun.intensity = Intensity;
            StateNow = $"{Clock.ScenarioUtc}: sun {Elevation_deg:F1}° above the horizon, azimuth {AzimuthTrue_deg:F1}° true ({AzimuthGrid_deg:F1}° grid)" +
                       (Weather != null && !float.IsNaN(Weather.CloudCoverNow_pct) ? $", cloud {Weather.CloudCoverNow_pct:F0} % (x{cloud:F2})" : ", cloud unknown (clear sky assumed)") +
                       (Elevation_deg < -6 ? " — NIGHT" : Elevation_deg < 0 ? " — twilight" : "");
        }

        /// NOAA Solar Calculator (Meeus) — identical to ovsite.timeline.solar_position.
        public static void SolarPosition(DateTime utc, double lat, double lon, out double elevation, out double azimuth)
        {
            double jd = (utc - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds / 86400.0 + 2440587.5;
            double T = (jd - 2451545.0) / 36525.0;
            double L0 = (280.46646 + T * (36000.76983 + T * 0.0003032)) % 360.0;
            double M = 357.52911 + T * (35999.05029 - 0.0001537 * T);
            double ecc = 0.016708634 - T * (0.000042037 + 0.0000001267 * T);
            double Mr = D2R(M);
            double C = Math.Sin(Mr) * (1.914602 - T * (0.004817 + 0.000014 * T)) + Math.Sin(2 * Mr) * (0.019993 - 0.000101 * T)
                       + Math.Sin(3 * Mr) * 0.000289;
            double omega = 125.04 - 1934.136 * T;
            double lam = L0 + C - 0.00569 - 0.00478 * Math.Sin(D2R(omega));
            double eps0 = 23.0 + (26.0 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60.0) / 60.0;
            double eps = eps0 + 0.00256 * Math.Cos(D2R(omega));
            double decl = R2D(Math.Asin(Math.Sin(D2R(eps)) * Math.Sin(D2R(lam))));
            double y = Math.Pow(Math.Tan(D2R(eps / 2)), 2);
            double L0r = D2R(L0);
            double eqt = 4 * R2D(y * Math.Sin(2 * L0r) - 2 * ecc * Math.Sin(Mr) + 4 * ecc * y * Math.Sin(Mr) * Math.Cos(2 * L0r)
                                 - 0.5 * y * y * Math.Sin(4 * L0r) - 1.25 * ecc * ecc * Math.Sin(2 * Mr));
            double minutes = utc.Hour * 60 + utc.Minute + utc.Second / 60.0;
            double tst = ((minutes + eqt + 4 * lon) % 1440.0 + 1440.0) % 1440.0;
            double ha = tst / 4 < 0 ? tst / 4 + 180 : tst / 4 - 180;
            double la = D2R(lat), de = D2R(decl), h = D2R(ha);
            double cz = Math.Max(-1, Math.Min(1, Math.Sin(la) * Math.Sin(de) + Math.Cos(la) * Math.Cos(de) * Math.Cos(h)));
            double zen = R2D(Math.Acos(cz));
            double el = 90.0 - zen, r;
            if (el > 85) r = 0;
            else if (el > 5) { double te = Math.Tan(D2R(el)); r = 58.1 / te - 0.07 / Math.Pow(te, 3) + 0.000086 / Math.Pow(te, 5); }
            else if (el > -0.575) r = 1735 + el * (-518.2 + el * (103.4 + el * (-12.79 + el * 0.711)));
            else r = -20.772 / Math.Tan(D2R(el));
            elevation = el + r / 3600.0;
            double sz = Math.Sin(D2R(zen));
            if (Math.Abs(sz) < 1e-9) azimuth = lat > decl ? 180.0 : 0.0;
            else
            {
                double ca = Math.Max(-1, Math.Min(1, (Math.Sin(la) * cz - Math.Sin(de)) / (Math.Cos(la) * sz)));
                double az = R2D(Math.Acos(ca));
                azimuth = ha > 0 ? (az + 180.0) % 360.0 : (540.0 - az) % 360.0;
            }
        }

        static double D2R(double d) { return d * Math.PI / 180.0; }
        static double R2D(double r) { return r * 180.0 / Math.PI; }
    }
}
