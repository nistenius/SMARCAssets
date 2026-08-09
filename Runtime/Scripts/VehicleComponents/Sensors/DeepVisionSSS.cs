using System.Reflection;
using UnityEngine;
using ROS.Core;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// Deep Vision side scan sonar (DE340/DE680 family, as on SAM 2.2): ONE transducer,
    /// switchable between two operating frequencies. Selecting the mode applies the
    /// matching range/resolution/ping-rate preset to the Sonar component (and the
    /// publisher rate). Interferometric in both modes -> SidescanMsg angle bytes are
    /// populated and point clouds can be built downstream.
    ///
    ///   LF 340 kHz: 15-200 m/side, 20 cm bins  — wide-area search      (~3 Hz ping)
    ///   HF 680 kHz: 10-100 m/side,  5 cm bins  — high-res imaging      (~7 Hz ping)
    ///
    /// The mode takes effect when entering Play (the Sonar sizes its bucket arrays in
    /// Awake); switching mid-run needs a Stop/Play, same as re-configuring the real
    /// unit's recording session.
    /// </summary>
    [DefaultExecutionOrder(-100)] // apply the preset before Sonar.Awake sizes its arrays
    [RequireComponent(typeof(Sonar))]
    [AddComponentMenu("Smarc/Sensor/DeepVisionSSS")]
    public class DeepVisionSSS : MonoBehaviour
    {
        public enum FrequencyMode { LF340, HF680 }

        [Tooltip("Operating frequency of the (single) transducer. LF340: 200 m/side wide-area. HF680: 100 m/side high-res. Applied at Play start.")]
        public FrequencyMode Mode = FrequencyMode.HF680;

        void OnValidate() { Apply(); }
        void Awake() { Apply(); }

        public void Apply()
        {
            var sonar = GetComponent<Sonar>();
            if (sonar == null) return;

            float pingHz;
            if (Mode == FrequencyMode.LF340)
            {
                sonar.MaxRange = 200f;
                sonar.NumBucketsPerBeam = 1000; // 20 cm range bins
                sonar.NumRaysPerBeam = 256;
                pingHz = 3f; // two-way travel at 200 m ~ 0.27 s
            }
            else
            {
                sonar.MaxRange = 100f;
                sonar.NumBucketsPerBeam = 2000; // 5 cm range bins
                sonar.NumRaysPerBeam = 512;
                pingHz = 7f;
            }
            sonar.isInterferometric = true;
            sonar.frequency = pingHz;

            // Match the SSS publisher rate to the ping rate. Reflection because the
            // publisher classes are internal; ROSPublisher<T>.frequency is public.
            foreach (var mb in GetComponents<MonoBehaviour>())
            {
                if (mb == null || mb == this || !(mb is ROSBehaviour rosb)) continue;
                if (!rosb.topic.Contains("sidescan")) continue;
                var f = mb.GetType().GetField("frequency", BindingFlags.Public | BindingFlags.Instance);
                if (f != null && f.FieldType == typeof(float)) f.SetValue(mb, pingHz);
            }
        }
    }
}
