// HdrpSeaStateModel — what sea state an HDRP WaterSurface actually makes, computed from HDRP's own
// spectrum, so the Ocean can be DRIVEN to a stated Hs / Tp instead of a wind speed (Ivan,
// 2026-09-23: "drive the parameters to the same statistical measures").
//
// WHY THIS CAN BE EXACT. HDRP's ocean is not a JONSWAP sea and not a black box. Each swell band is
// an N x N grid of Fourier components (N = the HDRP asset's water simulation resolution, 256 on
// the High asset) with
//
//     h0(k) = E(k) * sqrt( 0.2 * Phillips(k; V, chaos, orientation) / patch^2 )
//     Phillips = exp(-1 / (k^2 L^2)) / k^4 * wk^2 * (wk < 0 ? chaos : 1),   L = V^2 / g,
//     wk = lerp(dot(k^, -dir), 0.5, chaos)
//
// and E a complex Gaussian drawn from a FIXED integer hash of the grid cell (HDRP's
// WaterHashFunctionUInt4, identical in WaterSimulation.compute and the CPU port). The inverse FFT
// is unnormalised and the surface is the real part of sum h0 e^{i(k.x + w t)}, so the spatial
// variance of the band is EXACTLY 0.5 * sum |h0|^2 * amplitudeMultiplier^2, the same every run.
// Nothing here is fitted: the formula is transcribed from
//   com.unity.render-pipelines.high-definition/Runtime/Water/HDRenderPipeline.WaterSystem.SimulationCPU.cs
//   (PhillipsSpectrumInitialization, WaterHashFunctionUInt4, GaussianDis, k_PhillipsAmplitudeScalar)
//   ...WaterSurface.Simulation.cs (EvaluateSpectrumParams: band patch sizes, wind per band)
//   ...HDRenderPipeline.WaterSystem.Utilities.cs (EvaluateSwellSecondPatchSize, noise sample offset)
// Checked against the 2026-09-13 Asko measurements before a line of Unity code used it:
//   10 km/h / 250 m: model Hs 0.162 m, measured 0.134;  30 km/h / 500 m: 1.522 vs 1.459;
//   30 km/h / 1500 m: 1.476 vs 1.417  -> measured/model 0.83 / 0.96 / 0.96.
//
// PEAK PERIOD. The omnidirectional frequency spectrum of a Phillips sea peaks at
// k_p = sqrt(2/2.5) / L, so Tp = 2 pi V / (g * 0.9457) = 0.6775 V (V in m/s). Measured 09-13:
// 10 km/h -> 1.82 s (model 1.88); 30 km/h -> 5.12 s (model 5.65; 60 s records quantise Tp to
// 5.12 / 6.00 s at that frequency). Tp therefore sets the wind; Hs then sets the band amplitude
// multipliers, which scale height without moving the period.
//
// Deliberately free of UnityEngine so it compiles and is tested outside Unity (mcs).

using System;

namespace Smarc.Environment
{
    public static class HdrpSeaStateModel
    {
        public const double G = 9.81;                 // HDRP k_EarthGravity (not 9.80665)
        public const double PhillipsAmplitude = 0.2;  // HDRP k_PhillipsAmplitudeScalar
        public const double RipplesPatch_m = 10.0;    // HDRP WaterConsts.k_RipplesBandSize
        public const double TpPerWindSpeed = 2.0 * Math.PI / (G * 0.9457416090031758); // s per (m/s) = 0.6775
        public const double MaxSwellWind_ms = 250.0 / 3.6;
        const int NoiseOffset = 64;                   // HDRP k_NoiseFunctionOffset / NOISE_FUNCTION_OFFSET

        /// HDRP EvaluateSwellSecondPatchSize: second swell band patch = repetition / ratio.
        public static double SecondBandPatch(double repetition_m)
        {
            double t = (repetition_m - 250.0) / (5000.0 - 250.0);
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            return repetition_m / (5.0 + t * 45.0);
        }

        public static int NoiseSampleOffset(int n) => n == 128 ? 64 : (n == 64 ? 96 : 0);

        static void Hash(uint x, uint y, uint z, out double r0, out double r1, out double r2, out double r3)
        {
            unchecked
            {
                uint a = x, b = y, c = z, d = z;                     // x.xyzz
                uint na = ((a >> 16) ^ b) * 0x45d9f3bu;              // ^ x.yzxy
                uint nb = ((b >> 16) ^ c) * 0x45d9f3bu;
                uint nc = ((c >> 16) ^ a) * 0x45d9f3bu;
                uint nd = ((d >> 16) ^ b) * 0x45d9f3bu;
                a = na; b = nb; c = nc; d = nd;
                na = ((a >> 16) ^ b) * 0x45d9f3bu;                   // ^ x.yzxz
                nb = ((b >> 16) ^ c) * 0x45d9f3bu;
                nc = ((c >> 16) ^ a) * 0x45d9f3bu;
                nd = ((d >> 16) ^ c) * 0x45d9f3bu;
                a = na; b = nb; c = nc; d = nd;
                na = ((a >> 16) ^ b) * 0x45d9f3bu;                   // ^ x.yzxx
                nb = ((b >> 16) ^ c) * 0x45d9f3bu;
                nc = ((c >> 16) ^ a) * 0x45d9f3bu;
                nd = ((d >> 16) ^ a) * 0x45d9f3bu;
                const double inv = 1.0 / 4294967295.0;
                r0 = na * inv; r1 = nb * inv; r2 = nc * inv; r3 = nd * inv;
            }
        }

        static double Gaussian(double u, double v) => Math.Sqrt(-2.0 * Math.Log(Math.Max(u, 1e-6))) * Math.Cos(Math.PI * v);

        /// Expected (ensemble) Phillips density at one grid wavenumber, HDRP's units.
        static double Phillips(double kx, double ky, double dirX, double dirY, double V, double chaos, double patch)
        {
            double kk = kx * kx + ky * ky;
            if (kk == 0.0 || V <= 0.0) return 0.0;
            double L = V * V / G;
            double kl = Math.Sqrt(kk);
            double wk = (kx / kl * dirX + ky / kl * dirY) * (1.0 - chaos) + 0.5 * chaos;
            double p = Math.Exp(-1.0 / (kk * L * L)) / (kk * kk) * wk * wk;
            if (wk < 0.0) p *= chaos;
            return PhillipsAmplitude * p / (patch * patch);
        }

        /// REALISED variance of one band (m^2) with amplitude multiplier 1: 0.5 * sum |h0|^2 over the
        /// grid, with HDRP's own noise. Exactly what the surface (and the physics query) carries.
        public static double BandVariance(double patch_m, double wind_ms, double chaos, double orientationDeg,
                                          int sliceIndex, int n)
        {
            if (patch_m <= 0 || wind_ms <= 0) return 0.0;
            double th = orientationDeg * Math.PI / 180.0;
            double dirX = -Math.Cos(th), dirY = -Math.Sin(th);   // -OrientationToDirection(orientation)
            int off = NoiseSampleOffset(n);
            double sum = 0.0;
            for (int y = 0; y < n; ++y)
            {
                double ky = 2.0 * Math.PI * (y - n * 0.5) / patch_m;
                for (int x = 0; x < n; ++x)
                {
                    double kx = 2.0 * Math.PI * (x - n * 0.5) / patch_m;
                    double P = Phillips(kx, ky, dirX, dirY, wind_ms, chaos, patch_m);
                    if (P == 0.0) continue;
                    Hash((uint)(x + off + NoiseOffset), (uint)(y + off + NoiseOffset), (uint)(sliceIndex + NoiseOffset),
                         out double r0, out double r1, out double r2, out double r3);
                    double er = 0.7071067811865476 * Gaussian(r0, r1);
                    double ei = 0.7071067811865476 * Gaussian(r2, r3);
                    sum += (er * er + ei * ei) * P;
                }
            }
            return 0.5 * sum;
        }

        public struct Inputs
        {
            public double Repetition_m, SwellWind_ms, Chaos, OrientationDeg, Band0Multiplier, Band1Multiplier;
            public bool Ripples; public double RippleWind_ms, RippleChaos, RippleOrientationDeg;
            public bool RipplesInPhysics;   // WaterSurface.cpuEvaluateRipples
            public int Resolution;
        }

        public struct Result
        {
            public double HsVisual_m, HsPhysics_m, Tp_s, PeakWavelength_m, SwellVarPerUnitMult, RippleVar;
            public double Var0, Var1;
            public bool PeakFitsPatch;
        }

        public static Result Evaluate(Inputs i)
        {
            int n = i.Resolution > 0 ? i.Resolution : 256;
            double p1 = SecondBandPatch(i.Repetition_m);
            var r = new Result();
            r.Var0 = BandVariance(i.Repetition_m, i.SwellWind_ms, i.Chaos, i.OrientationDeg, 0, n);
            r.Var1 = BandVariance(p1, i.SwellWind_ms, i.Chaos, i.OrientationDeg, 1, n);
            r.RippleVar = i.Ripples ? BandVariance(RipplesPatch_m, i.RippleWind_ms, i.RippleChaos, i.RippleOrientationDeg, 2, n) : 0.0;
            double swell = r.Var0 * i.Band0Multiplier * i.Band0Multiplier + r.Var1 * i.Band1Multiplier * i.Band1Multiplier;
            r.SwellVarPerUnitMult = r.Var0 + r.Var1;
            r.HsVisual_m = 4.0 * Math.Sqrt(swell + r.RippleVar);
            r.HsPhysics_m = 4.0 * Math.Sqrt(swell + (i.RipplesInPhysics ? r.RippleVar : 0.0));
            r.Tp_s = TpPerWindSpeed * i.SwellWind_ms;
            r.PeakWavelength_m = G * r.Tp_s * r.Tp_s / (2.0 * Math.PI);
            r.PeakFitsPatch = r.PeakWavelength_m <= 0.5 * i.Repetition_m;
            return r;
        }

        /// Wind speed (m/s) that puts HDRP's spectral peak at Tp.
        public static double WindForPeakPeriod(double tp_s)
        {
            double v = tp_s / TpPerWindSpeed;
            return v < 0 ? 0 : (v > MaxSwellWind_ms ? MaxSwellWind_ms : v);
        }

        /// Band multiplier (applied to both swell bands) that makes the PHYSICS surface's Hs equal
        /// target, given the ripple variance the physics also carries. hsCorrection = measured/model
        /// from a probe sweep (1 = trust the model).
        public static double MultiplierForHs(double targetHs_m, Result atUnitMultiplier, bool ripplesInPhysics, double hsCorrection)
        {
            if (atUnitMultiplier.SwellVarPerUnitMult <= 0) return 0.0;
            double c = hsCorrection > 0 ? hsCorrection : 1.0;
            double want = (targetHs_m / (4.0 * c)); want *= want;
            if (ripplesInPhysics) want -= atUnitMultiplier.RippleVar;
            if (want <= 0) return 0.0;
            return Math.Sqrt(want / atUnitMultiplier.SwellVarPerUnitMult);
        }
    }
}
