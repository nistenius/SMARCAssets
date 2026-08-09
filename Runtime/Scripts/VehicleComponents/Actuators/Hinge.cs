using UnityEngine;
using ROS.Core;

namespace VehicleComponents.Actuators
{
    [AddComponentMenu("Smarc/Actuator/Hinge")]
    public class Hinge : LinkAttachment, IROSPublishable
    {
        [Header("Hinge")] public float angle;
        public float AngleMax = 0.2f;
        public bool reverse = false;


        void OnValidate()
        {
            if (angle > AngleMax) angle = AngleMax;
            if (angle < -AngleMax) angle = -AngleMax;
        }

        public void SetAngle(float a)
        {
            angle = Mathf.Clamp(a, -AngleMax, AngleMax);
        }

        new void FixedUpdate()
        {
            base.FixedUpdate();
            if (attachedLink == null) return; // LinkAttachment may still be retrying
            if (Physics.simulationMode == SimulationMode.FixedUpdate) DoUpdate();
        }

        public void DoUpdate()
        {
            int direction = reverse ? -1 : 1;
            parentMixedBody.SetDriveTarget(ArticulationDriveAxis.X, direction * angle * Mathf.Rad2Deg);
        }

        public bool HasNewData()
        {
            return true;
        }
    }
}