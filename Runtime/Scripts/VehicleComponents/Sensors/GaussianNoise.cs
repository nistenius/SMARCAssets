using System;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// Seedable Gaussian sampler (Box-Muller over System.Random), independent of
    /// UnityEngine.Random so each sensor can have its own deterministic stream.
    /// Seed 0 means "random seed each run"; any other seed gives a repeatable
    /// sequence for regression runs.
    ///
    /// Defaults across the sensors using this class were measured from real SAM
    /// tank-test rosbags (2025-08 MPC tests, _example_data_sets/SAM) — see
    /// data-cube/docs/sim-fidelity-ground-truth-and-noise.md.
    /// </summary>
    public class GaussianNoise
    {
        Random rng;
        bool hasSpare = false;
        double spare;

        public GaussianNoise(int seed)
        {
            rng = seed == 0 ? new Random() : new Random(seed);
        }

        /// <summary>Sample from N(0, sigma^2).</summary>
        public double Sample(double sigma)
        {
            if (sigma <= 0.0) return 0.0;
            if (hasSpare)
            {
                hasSpare = false;
                return spare * sigma;
            }
            double u, v, s;
            do
            {
                u = 2.0 * rng.NextDouble() - 1.0;
                v = 2.0 * rng.NextDouble() - 1.0;
                s = u * u + v * v;
            } while (s >= 1.0 || s == 0.0);
            s = Math.Sqrt(-2.0 * Math.Log(s) / s);
            spare = v * s;
            hasSpare = true;
            return u * s * sigma;
        }

        public float Samplef(float sigma) => (float)Sample(sigma);

        /// <summary>Uniform in [0,1). For dropout draws etc.</summary>
        public double NextDouble() => rng.NextDouble();

        /// <summary>Uniform in [min,max).</summary>
        public double Range(double min, double max) => min + rng.NextDouble() * (max - min);
    }
}
