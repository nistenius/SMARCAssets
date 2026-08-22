using System;
using UnityEngine;

namespace VehicleComponents.Actuators
{
    [AddComponentMenu("Smarc/Actuator/VBS")]
    public class VBS : LinkAttachment, IPercentageActuator
    {
        // IDLE AND RESET MEAN EMPTY, NOT HALF FULL (Ivan, 2026-08-17).
        // `resetValue` is what Actuator_Sub's watchdog commands when VBS messages stop
        // arriving (< expectedFrequency, default 2 Hz) -- i.e. it IS the vehicle's
        // no-commander state, and it is re-commanded every FixedUpdate for as long as the
        // silence lasts. The real vehicle's own emergency table settles what that state
        // must be: "Jetson level (Jetson off -> Teensy nodes handle: VBS -> 0 and
        // thrusters -> 0)" [wiki:sam:software:emergency_management, quoted in
        // data-cube/docs/SAM_VEHICLE_SPECS.md]. An uncommanded AUV empties its tank and
        // goes up; it does not hold half a tank and hover. Half-full also costs the GPS:
        // gps_link sits only 0.071 m above base_link, so a vehicle that will not float
        // never gets its antenna dry and never gets a fix.
        // NOTE these are defaults for NEW components only -- a prefab that serialised 42
        // or 50 keeps its own value (SETTLED.md sec.5, "serialized prefab values override
        // script defaults"), so the prefabs must be fixed in the Inspector as well.
        [Header("VBS")] [Range(0, 100)] public float percentage = 0f;

        [Tooltip("Commanded when the command stream stops (watchdog). MUST be 0: an uncommanded vehicle surfaces.")]
        [Range(0, 100)] public float resetValue = 0f;

        public float maxVolume_l = 0.250f;
        public float density = 997f; //kg/m3

        private float _initialMass;
        private float _maximumPos;
        private float _minimumPos;

        public new void Awake()
        {
            base.Awake();
            var xDrive = parentMixedBody.xDrive;
            //   _initialMass = parentArticulationBody.mass;
            _initialMass = density / 1000 * maxVolume_l;
            _minimumPos = xDrive.upperLimit;
            _maximumPos = xDrive.lowerLimit;
        }

        public void SetPercentage(float newValue)
        {
            percentage = Mathf.Clamp(newValue, 0, 100);
        }

        public float GetResetValue()
        {
            return resetValue;
        }

        public float GetCurrentValue()
        {
            return (1 - (mixedBody.jointPosition[0] - _minimumPos) / (_maximumPos - _minimumPos)) * 100;
        }

        public bool HasNewData()
        {
            return true;
        }

        new public void FixedUpdate()
        {
            base.FixedUpdate();
            if (attachedLink == null) return; // LinkAttachment may still be retrying
            if (Physics.simulationMode == SimulationMode.FixedUpdate) DoUpdate();
        }

        public void DoUpdate()
        {
            mixedBody.mass = 0.300f + _initialMass * GetCurrentValue() / 100; // Piston weight + water weight
            var computeTargetValue = ComputeTargetValue(percentage);
            mixedBody.SetDriveTarget(ArticulationDriveAxis.X, computeTargetValue);
        }

        public float ComputeTargetValue(float target)
        {
            return Mathf.Lerp(_maximumPos, _minimumPos, target / 100);
        }
    }
}