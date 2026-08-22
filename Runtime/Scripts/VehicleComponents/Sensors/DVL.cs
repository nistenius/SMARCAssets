using UnityEngine;

namespace VehicleComponents.Sensors
{
    [AddComponentMenu("Smarc/Sensor/DVL")]
    public class DVL: Sensor
    {
        [Header("DVL")]
        public int numBeams = 4;
        public int minHitsToReport = 3;
        public float maxRange = 50f;
        public float minRange = 0.05f;
        public float angleFromVertical = 22.5f;
        public float rotationOffset = 135f;
        public float verticalEmitOffset = -0.01f;
        public bool invertBeamOrder = true;
        public bool drawBeams = true;

        [Header("Noise (Waterlinked A50, measured in SAM tank tests 2025-08)")]
        [Tooltip("Add noise to the published values. Covariances are populated either way. Disable for deterministic regression runs.")]
        public bool enableNoise = true;
        [Tooltip("0 = new random seed every run. Any other value = repeatable noise sequence.")]
        public int noiseSeed = 0;
        [Tooltip("Velocity sigma floor per axis, m/s. Measured ~0.01 m/s at standstill on /sam/core/dvl.")]
        public float velocitySigmaFloor = 0.01f;
        [Tooltip("Velocity sigma as percent of current speed (A50 long-term accuracy is +/-1.01%).")]
        public float velocitySigmaPercent = 1.0f;
        [Tooltip("Altitude sigma, m.")]
        public float altitudeSigma = 0.02f;

        [Header("Dropout injection")]
        [Tooltip("Randomly lose bottom lock, like the real A50 during manoeuvres. Tank bags show gaps in nearly every run, some 20-30 s.")]
        public bool enableDropout = false;
        [Tooltip("Chance per ping to start a dropout.")]
        public float dropoutProbPerPing = 0.005f;
        public float dropoutMinDuration = 0.5f;
        public float dropoutMaxDuration = 10f;
        [Tooltip("Hold true to force a dropout right now (deliberate injection during an experiment).")]
        public bool forceDropout = false;

        [Header("Current values")]
        public bool bottomLock;
        public Vector3 velocity;
        [Tooltip("Height above the seabed, m. -1 while the measurement is invalid (dropout / no lock), " +
                 "per the same smarc_msgs/DVL convention as velocityCovariance. NEVER leave a stale " +
                 "reading here on lost lock: a consumer cannot tell a stale altitude from a live one.")]
        public float altitude = -1f;
        public float[] ranges;
        public int numHits;
        [Tooltip("Row-major xyz. Diagonal set from the noise model; all -1 while measurement is invalid (dropout / no lock), per smarc_msgs/DVL convention.")]
        public double[] velocityCovariance = new double[9];

        GaussianNoise noise;
        float dropoutRemaining = 0f;

        void Start()
        {
            ranges = new float[numBeams];
            noise = new GaussianNoise(noiseSeed);
        }

        void SetCovarianceInvalid()
        {
            for(int i=0; i<9; i++) velocityCovariance[i] = -1.0;
        }

        void SetCovarianceValid()
        {
            float sigma = velocitySigmaFloor + velocitySigmaPercent * 0.01f * velocity.magnitude;
            double var = sigma * sigma;
            for(int i=0; i<9; i++) velocityCovariance[i] = 0.0;
            velocityCovariance[0] = var;
            velocityCovariance[4] = var;
            velocityCovariance[8] = var;
        }

        public override bool UpdateSensor(double deltaTime)
        {
            if (noise == null) noise = new GaussianNoise(noiseSeed);

            // Dropout injection: lose bottom lock deliberately or by chance.
            if (dropoutRemaining > 0f) dropoutRemaining -= (float)deltaTime;
            if (enableDropout && dropoutRemaining <= 0f && !forceDropout
                && noise.NextDouble() < dropoutProbPerPing)
            {
                dropoutRemaining = (float)noise.Range(dropoutMinDuration, dropoutMaxDuration);
            }
            if (forceDropout || dropoutRemaining > 0f)
            {
                bottomLock = false;
                velocity = Vector3.zero;
                altitude = -1f;
                SetCovarianceInvalid();
                return false;
            }

            // Base directions depending on the pose of the DVL
            Vector3 right = transform.TransformDirection(Vector3.right);
            Vector3 down = transform.TransformDirection(-Vector3.up);
            Vector3 source = transform.position + transform.TransformDirection(Vector3.up)*verticalEmitOffset;
            // Start looking down
            Vector3 direction = transform.TransformDirection(-Vector3.up);
            // Tilt forward first
            direction = Quaternion.AngleAxis(angleFromVertical, right) * direction;
            // Rotate around vertical for the offset
            direction = Quaternion.AngleAxis(rotationOffset, down) * direction;

            var angleAroundVertical = 360/numBeams;
            if(invertBeamOrder) angleAroundVertical*=-1;

            numHits = 0;
            for(int i=0;i < numBeams; i++)
            {
                // Then rotate around vertical each beam
                // according to their index
                direction = Quaternion.AngleAxis(angleAroundVertical, down) * direction;
                // draw the first 4 beams with colors getting hotter
                if(drawBeams)
                {
                    Color c = Color.Lerp(Color.red, Color.green, (i+1)/(float)numBeams);
                    Debug.DrawLine(source, source + direction, c, 0.5f);
                }
                RaycastHit beamHit;
                if(Physics.Raycast(source, direction, out beamHit, maxRange))
                {
                    if(beamHit.distance >= minRange)
                    {
                        // finally, its a valid hit
                        if(drawBeams) Debug.DrawLine(source, beamHit.point, Color.yellow, 0.5f);
                        ranges[i] = beamHit.distance;
                        numHits++;
                    }
                }
            }

            bottomLock = numHits >= minHitsToReport;
            // If not enough hits, no velocity or altitude or anything...
            if(!bottomLock)
            {
                altitude = -1f;
                SetCovarianceInvalid();
                return false;
            }

            velocity = mixedBody.transform.InverseTransformVector(mixedBody.velocity);
            SetCovarianceValid();
            if (enableNoise)
            {
                float sigma = velocitySigmaFloor + velocitySigmaPercent * 0.01f * velocity.magnitude;
                velocity += new Vector3(noise.Samplef(sigma), noise.Samplef(sigma), noise.Samplef(sigma));
            }

            // Altitude is a little trickier since we're faking it
            // rather than doing the whole beams thing...
            // So instead, we just do a raycast straight down
            // and use that as our altitude
            // This should be about the same as getting beam distances, their angles and calcing
            // the distance a "straight down" beam would produce from those.
            RaycastHit altHit;
            if(Physics.Raycast(source, -Vector3.up, out altHit, maxRange))
            {
                altitude = altHit.distance;
                if (enableNoise) altitude += noise.Samplef(altitudeSigma);
            }
            else
            {
                // Beams found the bottom but the straight-down ray did not (steep attitude, a hole
                // in the mesh, bottom beyond maxRange). Report invalid rather than the previous
                // ping's value -- a stale altitude is indistinguishable from a live one downstream.
                altitude = -1f;
            }

            return true;

        }

    }
}