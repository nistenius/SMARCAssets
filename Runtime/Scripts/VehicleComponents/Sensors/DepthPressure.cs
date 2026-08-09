using UnityEngine;
using DefaultNamespace.Water; // WaterQueryModel

namespace VehicleComponents.Sensors
{
    [AddComponentMenu("Smarc/Sensor/DepthPressure")]
    public class DepthPressure: Sensor
    {
        [Header("Depth-Pressure")]
        public float maxDepth;
        public bool includeAtmosphericPressure;
        public float pressure;

        [Header("Noise (measured 1.8 Pa on /sam/core/depth20_pressure, tank tests 2025-08)")]
        [Tooltip("Add noise to the published value. Variance is published either way. Disable for deterministic regression runs.")]
        public bool enableNoise = true;
        [Tooltip("0 = new random seed every run. Any other value = repeatable noise sequence.")]
        public int noiseSeed = 0;
        [Tooltip("Pressure sigma in Pa. The real 20 bar sensor is heavily filtered: ~2 Pa (~0.2 mm of water).")]
        public float pressureSigmaPa = 2.0f;

        private GaussianNoise noise;
        private WaterQueryModel _waterModel;

        void Start()
        {
            noise = new GaussianNoise(noiseSeed);
            var waterModels = FindObjectsByType<WaterQueryModel>(FindObjectsSortMode.None);
            if(waterModels.Length > 0) _waterModel = waterModels[0];
            else 
            {
                Debug.LogWarning("DepthPressure: No WaterQueryModel found in the scene, disabling sensor");
                enabled = false;
            }

        }


        public override bool UpdateSensor(double deltaTime)
        {
            var waterSurfaceLevel = _waterModel.GetWaterLevelAt(transform.position);
            float depth = waterSurfaceLevel - transform.position.y;
            if (includeAtmosphericPressure) pressure = 101325.0f;
            else pressure = 0;

            // 1m water = 9806.65 Pa
            if (depth > maxDepth) return false;
            else
            {
                pressure += depth * 9806.65f;
                if (enableNoise)
                {
                    if (noise == null) noise = new GaussianNoise(noiseSeed);
                    pressure += noise.Samplef(pressureSigmaPa);
                }
                return true;
            }
            
        }
    }
}