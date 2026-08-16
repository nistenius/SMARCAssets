using System.Collections.Generic;
using UnityEngine;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// The seabed AHEAD of the vehicle, sampled from the forward-looking 3D sonar's field of view.
    ///
    /// Data Cube 2026-08-15, Ivan: bottom tracking "should then also be looking ahead using the 3D
    /// sonar to plan better the path to follow to track a varying bottom contour".
    ///
    /// WHY THIS EXISTS SEPARATELY FROM THE DVL AND FROM Sonar.cs
    /// ---------------------------------------------------------
    /// The DVL/altimeter reads the seabed DIRECTLY BELOW. By the time a rising slope is under the
    /// vehicle, the vehicle is already too low over it, and an altimeter-only loop is permanently
    /// behind on any real terrain. Sonar.cs produces a full point cloud, which is the right
    /// instrument for mapping and the wrong one for a control loop: it is heavy, and it answers a
    /// different question ("what is out there") from the one bottom-following asks ("how deep is
    /// the seabed at each distance along my track").
    ///
    /// So this samples a thin vertical fan along the vehicle's heading and reports, per range bin,
    /// the depth of the seabed. That is exactly the input bottom_follow.plan_depth() consumes, and
    /// nothing more. Publishing the smallest sufficient thing keeps the acoustic and control paths
    /// separable, and means a real hull can feed the same topic from whatever forward sonar it
    /// actually carries.
    ///
    /// GROUND TRUTH VS MEASUREMENT — read this before trusting a run.
    /// Raycasts against the Unity seabed collider ARE ground truth. That is legitimate for
    /// developing and testing the policy, and it is NOT a sonar model: no beam width, no grazing
    /// angle dropout, no multipath, no silt. `isGroundTruth` is published on every message and
    /// must stay true here, because a component that quietly hands ground truth to a controller
    /// wearing an estimate's label is precisely the failure that hid a dead state_estimator for a
    /// whole session (spec invariant 11). Degradations are opt-in below, and named.
    ///
    /// NOT FLOWN. Built off-rig 2026-08-15.
    /// </summary>
    [AddComponentMenu("Smarc/Sensor/ForwardBottomProfiler")]
    public class ForwardBottomProfiler : Sensor
    {
        [Header("Fan geometry")]
        [Tooltip("How far ahead to probe, metres. Should exceed the controller's lookahead window.")]
        public float maxRangeM = 40f;

        [Tooltip("Number of range bins reported along track. Each is one seabed depth.")]
        public int numBins = 16;

        [Tooltip("Downward tilt of the fan's near edge, degrees below horizontal.")]
        public float nearTiltDeg = 60f;

        [Tooltip("Downward tilt of the fan's far edge, degrees below horizontal. Smaller = looks further.")]
        public float farTiltDeg = 5f;

        [Tooltip("Half-width of the fan across track, degrees. A narrow fan follows the track; a wide one is safer near a side wall and noisier.")]
        public float halfWidthDeg = 10f;

        [Tooltip("Rays across track per range bin. 1 = a single vertical slice.")]
        public int raysAcross = 3;

        [Header("Honesty")]
        [Tooltip("Raycasts against the seabed collider are GROUND TRUTH, not a sonar model. Leave this on; it is published so no consumer can mistake one for the other.")]
        public bool isGroundTruth = true;

        [Tooltip("Drop returns whose grazing angle is shallower than this (degrees). The one real-sonar effect modelled by default, because a flat seabed seen edge-on genuinely returns nothing and a profiler that pretends otherwise invents terrain.")]
        public float minGrazingAngleDeg = 8f;

        [Header("Optional degradation (opt-in, so a clean run stays clean)")]
        public bool enableNoise = false;
        [Tooltip("Depth noise sigma, metres.")]
        public float depthSigmaM = 0.05f;
        public bool enableDropout = false;
        [Tooltip("Chance per bin per ping that the bin returns nothing.")]
        public float dropoutProbPerBin = 0.02f;

        [Header("Current values (read-only)")]
        public int validBins;
        public float shallowestSeabedDepthM = -1f;
        public float shallowestAtRangeM = -1f;

        /// <summary>Range ahead of each bin, metres. Parallel to <see cref="seabedDepths"/>.</summary>
        [HideInInspector] public float[] rangesAhead;

        /// <summary>Seabed depth below the surface at that range, metres, positive down.
        /// NaN where nothing was returned — never 0, which would read as "the seabed is at the
        /// surface" and is the most dangerous possible encoding of missing data.</summary>
        [HideInInspector] public float[] seabedDepths;

        [Header("Debug")]
        public bool drawFan = false;

        LayerMask _mask;
        System.Random _rng;

        // `Start`, matching DVL/GPS/IMU and the rest of this folder. The base Sensor has no Init()
        // to override -- an earlier version declared `public override bool Init()` and did not
        // compile.
        void Start()
        {
            Allocate();
        }

        void Allocate()
        {
            numBins = Mathf.Max(1, numBins);
            raysAcross = Mathf.Max(1, raysAcross);
            rangesAhead = new float[numBins];
            seabedDepths = new float[numBins];
            // Everything except the vehicle itself; the hull must not occlude its own profiler.
            _mask = ~0;
            if (_rng == null) _rng = new System.Random();
        }

        /// <summary>Depth below the water surface of a world point, positive down.
        /// Unity's y is up and the water plane is y = 0 in every SMARC scene, which is the one
        /// assumption here worth stating out loud rather than leaving in the arithmetic.</summary>
        static float DepthOf(Vector3 worldPoint) => -worldPoint.y;

        /// <summary>
        /// Returns whether this tick produced data worth publishing.
        ///
        /// TRUE EVEN WHEN NOTHING WAS SEEN, deliberately, and unlike DVL — which returns false on
        /// a lost bottom lock because a velocity it does not have is not a reading. Here "I looked
        /// and the seabed is not in view" IS the reading: the controller must know the difference
        /// between a fresh no-bottom report and a profiler that has stopped running, because the
        /// first means hold the leg's target depth and the second means something is broken. A
        /// silence that could mean either is what the reason string exists to prevent.
        /// The one false is a genuinely uninitialised component.
        /// </summary>
        public override bool UpdateSensor(double deltaTime)
        {
            if (rangesAhead == null || rangesAhead.Length != numBins) Allocate();
            validBins = 0;
            shallowestSeabedDepthM = -1f;
            shallowestAtRangeM = -1f;
            float shallowest = float.MaxValue;

            Vector3 origin = transform.position;
            // Heading only: a pitching vehicle must not swing its own lookahead window up and
            // down the seabed. The fan's tilt is measured from HORIZONTAL, not from the hull.
            Vector3 fwd = transform.forward;
            fwd.y = 0f;
            if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
            fwd.Normalize();
            Vector3 right = Vector3.Cross(Vector3.up, fwd);

            for (int b = 0; b < numBins; b++)
            {
                // Bins are spaced by TILT, not by range: a fixed tilt step gives finer resolution
                // close in (where the vehicle must react soonest) and coarser far out, which is
                // the right way round and falls out of the geometry for free.
                float t = numBins == 1 ? 0f : (float)b / (numBins - 1);
                float tilt = Mathf.Lerp(nearTiltDeg, farTiltDeg, t);

                float best = float.NaN;
                float bestRange = float.NaN;

                for (int a = 0; a < raysAcross; a++)
                {
                    float ta = raysAcross == 1 ? 0f : ((float)a / (raysAcross - 1)) * 2f - 1f;
                    float yaw = ta * halfWidthDeg;
                    Vector3 dir = Quaternion.AngleAxis(yaw, Vector3.up)
                                * Quaternion.AngleAxis(tilt, right)
                                * fwd;

                    if (!Physics.Raycast(origin, dir, out RaycastHit hit, maxRangeM, _mask)) continue;
                    if (hit.transform.IsChildOf(transform.root)) continue;   // our own hull

                    // Grazing angle against the surface the ray struck. A near-parallel hit on a
                    // flat seabed returns nothing in the water, and accepting it here would
                    // manufacture terrain far beyond where a real sonar can see.
                    float grazing = 90f - Vector3.Angle(-dir, hit.normal);
                    if (grazing < minGrazingAngleDeg) continue;

                    if (enableDropout && _rng.NextDouble() < dropoutProbPerBin) continue;

                    float depth = DepthOf(hit.point);
                    if (enableNoise) depth += (float)NextGaussian() * depthSigmaM;

                    // Horizontal distance along track, which is what the controller reasons in.
                    Vector3 flat = hit.point - origin;
                    flat.y = 0f;
                    float range = flat.magnitude;

                    // SHALLOWEST WITHIN THE BIN, matching the controller's own rule. Taking the
                    // mean across the fan would average a ridge away with the water beside it.
                    if (float.IsNaN(best) || depth < best) { best = depth; bestRange = range; }
                }

                rangesAhead[b] = float.IsNaN(bestRange) ? float.NaN : bestRange;
                seabedDepths[b] = best;
                if (!float.IsNaN(best))
                {
                    validBins++;
                    if (best < shallowest) { shallowest = best; shallowestAtRangeM = rangesAhead[b]; }
                }

                if (drawFan)
                {
                    // A hit is drawn to where the seabed actually is (world y = -depth); a miss is
                    // drawn out to max range in grey, so "looked and saw nothing" is visible in
                    // the scene view rather than being an absent line you have to notice.
                    Vector3 end = float.IsNaN(best)
                        ? origin + Quaternion.AngleAxis(tilt, right) * fwd * maxRangeM
                        : new Vector3(origin.x + fwd.x * rangesAhead[b], -best,
                                      origin.z + fwd.z * rangesAhead[b]);
                    Debug.DrawLine(origin, end, float.IsNaN(best) ? Color.grey : Color.cyan);
                }
            }

            if (validBins > 0) shallowestSeabedDepthM = shallowest;
            return true;
        }

        double NextGaussian()
        {
            double u1 = 1.0 - _rng.NextDouble(), u2 = 1.0 - _rng.NextDouble();
            return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Sin(2.0 * System.Math.PI * u2);
        }

        /// <summary>The profile as (range, depth) pairs, skipping bins that returned nothing.
        /// Consumed by the publisher; kept separate so the geometry above can be tested without
        /// a ROS connection.</summary>
        public List<Vector2> ValidProfile()
        {
            var outp = new List<Vector2>();
            for (int i = 0; i < numBins; i++)
            {
                if (float.IsNaN(seabedDepths[i]) || float.IsNaN(rangesAhead[i])) continue;
                outp.Add(new Vector2(rangesAhead[i], seabedDepths[i]));
            }
            return outp;
        }
    }
}
