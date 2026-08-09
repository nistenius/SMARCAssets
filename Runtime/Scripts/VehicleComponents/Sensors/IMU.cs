using UnityEngine;


namespace VehicleComponents.Sensors
{
    [AddComponentMenu("Smarc/Sensor/IMU")]
    public class IMU : Sensor
    {
        // Mostly copied from https://github.com/MARUSimulator/marus-core/blob/21c003a384335777b9d9fb6805eeab1cdb93b2f0/Scripts/Sensors/Primitive/ImuSensor.cs
        // Thank you guys <3
        [Header("IMU")]
        public bool withGravity = true;

        [Header("Noise (STIM300, measured in SAM tank tests 2025-08)")]
        [Tooltip("Add noise to the published values. Covariances are populated either way. Disable for deterministic regression runs.")]
        public bool enableNoise = true;
        [Tooltip("0 = new random seed every run. Any other value = repeatable noise sequence.")]
        public int noiseSeed = 0;
        [Tooltip("Accelerometer white-noise density in (m/s^2)/sqrt(Hz). Per-sample sigma = density * sqrt(frequency). Measured 0.016 m/s^2 @ 125 Hz on /sam/core/imu.")]
        public double accelNoiseDensity = 0.0014;
        [Tooltip("Gyro white-noise density in (rad/s)/sqrt(Hz). Measured 0.001 rad/s @ 125 Hz on /sam/core/imu.")]
        public double gyroNoiseDensity = 9e-5;
        [Tooltip("Roll/pitch sigma of the (AHRS-like) orientation output, rad.")]
        public double orientationSigmaRollPitch = 0.0035;
        [Tooltip("Yaw/heading sigma of the orientation output, rad. Heading is worse than roll/pitch on a real AHRS.")]
        public double orientationSigmaYaw = 0.0175;

        [Header("Heading drift (what a compass is for)")]
        [Tooltip("Heading random-walk in deg/sqrt(min). A gyro-only unit (STIM300) has NO magnetometer, so its heading drifts without bound -- that is the error a compass-aided unit (SBG) bounds. Set >0 on the STIM instance, 0 on the SBG instance.\n\nWith both at 0 (the old behaviour) the sim's heading is perfect and no compass factor can ever be shown to help.")]
        public double yawDriftDegPerSqrtMin = 0.0;
        [Tooltip("Optional cap on the accumulated heading bias, deg. 0 = uncapped random walk. A compass-aided unit should use 0 drift instead of a cap.")]
        public double yawDriftCapDeg = 0.0;

        [Tooltip("Current accumulated heading bias, deg. Read-only; useful for plotting against the estimator's yaw error.")]
        public double yawBiasDeg = 0.0;

        [Header("Current values")]
        public Vector3 localVelocity;
        public Vector3 linearAcceleration;
        public double[] linearAccelerationCovariance = new double[9];

        public Vector3 angularVelocity;
        public double[] angularVelocityCovariance = new double[9];

        public Vector3 eulerAngles;
        public Quaternion orientation;
        public double[] orientationCovariance = new double[9];

        private Vector3 lastVelocity = Vector3.zero;
        private GaussianNoise noise;

        void Start()
        {
            noise = new GaussianNoise(noiseSeed);
        }

        void PopulateCovariances()
        {
            // Per-sample variances from the continuous-time noise densities at
            // the configured sensor rate. Published even when noise is disabled,
            // so the estimator's use_sensor_covariance stays meaningful.
            double accelVar = accelNoiseDensity * accelNoiseDensity * frequency;
            double gyroVar = gyroNoiseDensity * gyroNoiseDensity * frequency;
            double rpVar = orientationSigmaRollPitch * orientationSigmaRollPitch;
            double yawVar = orientationSigmaYaw * orientationSigmaYaw;
            linearAccelerationCovariance[0] = accelVar;
            linearAccelerationCovariance[4] = accelVar;
            linearAccelerationCovariance[8] = accelVar;
            angularVelocityCovariance[0] = gyroVar;
            angularVelocityCovariance[4] = gyroVar;
            angularVelocityCovariance[8] = gyroVar;
            orientationCovariance[0] = rpVar;
            orientationCovariance[4] = rpVar;
            orientationCovariance[8] = yawVar;
        }

        public override bool UpdateSensor(double deltaTime)
        {
            if (!mixedBody.isValid)
            {
                Debug.LogError("No valid body found for IMU!");
                return false;
            }

            // Use MixedBody to handle both Rigidbody and ArticulationBody
            localVelocity = mixedBody.localVelocity;

            if (deltaTime > 0)
            {
                Vector3 deltaLinearAcceleration = localVelocity - lastVelocity;
                linearAcceleration = deltaLinearAcceleration / (float)deltaTime;
            }

            angularVelocity = -mixedBody.transform.InverseTransformVector(mixedBody.angularVelocity);
            eulerAngles = mixedBody.transform.rotation.eulerAngles;
            orientation = Quaternion.Euler(eulerAngles);

            lastVelocity = localVelocity;

            if (withGravity)
            {
                // Find the global gravity in the local frame and add to the computed linear acceleration
                Vector3 localGravity = mixedBody.transform.InverseTransformDirection(Physics.gravity);
                linearAcceleration += localGravity;
            }

            PopulateCovariances();

            if (enableNoise)
            {
                if (noise == null) noise = new GaussianNoise(noiseSeed);
                float accelSigma = (float)(accelNoiseDensity * System.Math.Sqrt(frequency));
                float gyroSigma = (float)(gyroNoiseDensity * System.Math.Sqrt(frequency));
                linearAcceleration += new Vector3(
                    noise.Samplef(accelSigma),
                    noise.Samplef(accelSigma),
                    noise.Samplef(accelSigma));
                angularVelocity += new Vector3(
                    noise.Samplef(gyroSigma),
                    noise.Samplef(gyroSigma),
                    noise.Samplef(gyroSigma));
                // Unity euler: x/z are roll/pitch-like, y is heading.
                eulerAngles += new Vector3(
                    noise.Samplef((float)orientationSigmaRollPitch) * Mathf.Rad2Deg,
                    noise.Samplef((float)orientationSigmaYaw) * Mathf.Rad2Deg,
                    noise.Samplef((float)orientationSigmaRollPitch) * Mathf.Rad2Deg);
            }

            // Heading random walk, applied whether or not white noise is on: this is the
            // systematic error a magnetometer-aided unit does not have, and the reason the
            // estimator needs a compass to stay observable on a long dive.
            if (yawDriftDegPerSqrtMin > 0.0)
            {
                if (noise == null) noise = new GaussianNoise(noiseSeed);
                double dtMin = deltaTime / 60.0;
                yawBiasDeg += noise.Sample(yawDriftDegPerSqrtMin * System.Math.Sqrt(dtMin));
                if (yawDriftCapDeg > 0.0)
                    yawBiasDeg = System.Math.Max(-yawDriftCapDeg,
                                 System.Math.Min(yawDriftCapDeg, yawBiasDeg));
                eulerAngles.y += (float)yawBiasDeg;
            }

            if (enableNoise || yawDriftDegPerSqrtMin > 0.0)
            {
                orientation = Quaternion.Euler(eulerAngles);
            }

            return true;
        }
    }
}
