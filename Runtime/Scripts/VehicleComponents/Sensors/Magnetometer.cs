using UnityEngine;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// Three-axis magnetometer, in tesla, in the sensor's body frame.
    ///
    /// Why this exists (Ivan, 2026-08-08): the STIM300 is a gyro-only unit — excellent
    /// short-term stability, but its heading drifts without bound. The SBG is
    /// magnetometer-aided: noisier moment to moment, but its heading does not run away.
    /// On a real dive it is the compass that keeps yaw observable; the sim published no
    /// magnetic field at all, so nothing could bound heading and the estimator's yaw was
    /// free to wander (which is exactly how it diverged on 2026-08-08).
    ///
    /// The field is the local Earth field rotated into the body frame, plus optional
    /// hard-iron bias (a constant body-frame offset, e.g. the vehicle's own magnets) and
    /// white noise. Declination is applied so that "magnetic north" is not Unity north.
    /// </summary>
    [AddComponentMenu("Smarc/Sensor/Magnetometer")]
    public class Magnetometer : Sensor
    {
        [Header("Earth field")]
        [Tooltip("Total field strength in microtesla. ~50.7 uT around Stockholm.")]
        public float fieldStrengthMicroTesla = 50.7f;
        [Tooltip("Inclination (dip) in degrees, positive downward. ~72.5 deg at Stockholm's latitude.")]
        public float inclinationDeg = 72.5f;
        [Tooltip("Magnetic declination in degrees east of true north. ~+7 deg around Stockholm.")]
        public float declinationDeg = 7.0f;

        [Header("Errors")]
        [Tooltip("Add noise and bias to the published field. Covariance is published either way.")]
        public bool enableNoise = true;
        [Tooltip("0 = new random seed every run. Any other value = repeatable noise sequence.")]
        public int noiseSeed = 0;
        [Tooltip("White noise sigma per axis, microtesla.")]
        public float noiseSigmaMicroTesla = 0.3f;
        [Tooltip("Hard-iron bias in the BODY frame, microtesla. Constant offset from the vehicle's own magnetics; this is what a figure-of-eight calibration removes. Leave zero for a calibrated vehicle.")]
        public Vector3 hardIronBiasMicroTesla = Vector3.zero;

        [Header("Current values")]
        [Tooltip("Field in the sensor body frame, tesla (ROS units).")]
        public Vector3 magneticField;
        [Tooltip("Heading implied by the field, deg. Diagnostic only — the estimator should use the field, not this.")]
        public float impliedHeadingDeg;
        public double[] magneticFieldCovariance = new double[9];

        GaussianNoise noise;

        void Start()
        {
            noise = new GaussianNoise(noiseSeed);
        }

        public override bool UpdateSensor(double deltaTime)
        {
            if (noise == null) noise = new GaussianNoise(noiseSeed);

            // Earth field in world (Unity) coordinates: magnetic north rotated by declination
            // from Unity's +Z ("north"), dipping by the inclination.
            float inc = inclinationDeg * Mathf.Deg2Rad;
            float dec = declinationDeg * Mathf.Deg2Rad;
            float horizontal = fieldStrengthMicroTesla * Mathf.Cos(inc);
            float vertical = fieldStrengthMicroTesla * Mathf.Sin(inc);
            // Unity: +Z north, +X east, +Y up. Dip points into the ground => -Y.
            Vector3 worldField = new Vector3(
                horizontal * Mathf.Sin(dec),
                -vertical,
                horizontal * Mathf.Cos(dec));

            // Into the sensor's body frame.
            Vector3 bodyField = transform.InverseTransformDirection(worldField);

            if (enableNoise)
            {
                bodyField += hardIronBiasMicroTesla;
                bodyField += new Vector3(
                    noise.Samplef(noiseSigmaMicroTesla),
                    noise.Samplef(noiseSigmaMicroTesla),
                    noise.Samplef(noiseSigmaMicroTesla));
            }

            // Diagnostic heading from the horizontal components, in Unity's frame.
            impliedHeadingDeg = Mathf.Repeat(
                Mathf.Atan2(bodyField.x, bodyField.z) * Mathf.Rad2Deg, 360f);

            // Publish in tesla, ROS convention.
            magneticField = bodyField * 1e-6f;

            double var = (noiseSigmaMicroTesla * 1e-6) * (noiseSigmaMicroTesla * 1e-6);
            magneticFieldCovariance[0] = var;
            magneticFieldCovariance[4] = var;
            magneticFieldCovariance[8] = var;

            return true;
        }
    }
}
