using System;
using System.Linq;
using DefaultNamespace.Water;
using ROS.Core;
using Unity.Robotics.Core;
using UnityEngine;


namespace Force
{
    public class ForcePoint : MonoBehaviour
    {
        [Header("Connected Body")] public ArticulationBody ConnectedArticulationBody;
        public Rigidbody ConnectedRigidbody;


        [Header("Buoyancy")]
        [Tooltip("Because HDRP water level queries are expensive at high frequency, you might want to limit it to something managable. 50 is a good starting point. -1 to query every fixed update.")]
        public float WaterQueryFrequency = -1f;

        [Tooltip("GameObject that we will calculate the volume of. Set volume below to 0 to use.")]
        public GameObject VolumeObject;

        [Tooltip("If the gameObject above has many meshes, set the one to use for volume calculations here.")]
        public Mesh VolumeMesh;

        [Tooltip("If not zero, will be used for buoyancy calculations. If zero, the volumeObject/Mesh above will be used to calculate.")]
        public float Volume;

        public float WaterDensity = 997; // kg/m3

        [Tooltip("How deep should the point be to apply the entire buoyancy force. Force is applied proportionally.")]
        public float DepthBeforeSubmerged = 0.03f;

        [Tooltip("Maximum force applied by buoyancy. Nice to keep things from going to space :)")]
        public float MaxBuoyancyForce = 1000f;

        [Header("Strip mode (surface-piercing hull sections)")]
        [Tooltip("Local hull radius at this point [m]. If > 0 the point represents a horizontal circular " +
                 "section (a strip of a cylindrical hull) and the applied fraction follows the circular-segment " +
                 "area of that section versus the water level, instead of the linear DepthBeforeSubmerged ramp. " +
                 "The point sits on the hull AXIS; the section is wet from axis depth -SectionRadius upward.")]
        public float SectionRadius = 0f;

        [Tooltip("If true, Volume is THIS point's own displaced volume (e.g. its strip) and is applied as-is. " +
                 "If false (legacy), Volume is the whole body's volume and is divided equally among all " +
                 "ForcePoints on the body.")]
        public bool VolumeIsPerPoint = false;
        [Tooltip("De-bounce factor. How many physics frames of look-ahead do we consider 'fast enough to jump out, dont apply buoyancy' when we are underwater? Helps prevent objects shooting to the moon if they are buoyant AND deep. Set to 0 to disable.")]
        public int BuoyancyDebounceFrames = 3;


        [Header("Underwater/Air Drag")] [Tooltip("Linear Drag applied while underwater. Sets the connected body's drag/linearDamping value when underwater. Set to -1 to use the starting drag value of the body for this.")]
        public float UnderwaterDrag = -1f;

        [Tooltip("Angular Drag applied while underwater. Sets the connected body's drag/linearDamping value when underwater. Set to -1 to use the starting drag value of the body for this.")]
        public float UnderwaterAngularDrag = -1f;

        [Tooltip("Linear Drag applied while underwater. Sets the connected body's drag/linearDamping value when above water. Set to -1 to use the starting drag value of the body for this.")]
        public float AirDrag = -1f;

        [Tooltip("Angular Drag applied while underwater. Sets the connected body's drag/linearDamping value when above water. Set to -1 to use the starting drag value of the body for this.")]
        public float AirAngularDrag = -1f;




        [Header("Current state wrt water level")]
        public bool IsUnderwater = false;
        public bool IsSubmerged = false;
        public float CurrentDepth;




        [Header("Gravity")] [Tooltip("Do we over-ride the gravity of the connected body?")]
        public bool AddGravity = false;

        [Tooltip("If true, calculates center of gravity from all the ForcePoints on the body and overrides the body's centerOfMass, otherwise the centerOfMass of the connected body is used.")]
        public bool AutomaticCenterOfGravity = false;

        [Tooltip("If not zero, will be used for gravity force. If zero, the connected body's mass will be used instead.")]
        public float Mass;


        [Header("Debug")] public bool DrawForces = false;
        public Vector3 AppliedBuoyancyForce, AppliedGravityForce;

        [Header("Instrumentation")]
        [Tooltip("The buoyancy force handed to AddForceAtPosition on THIS physics step, zero on a step " +
                 "where none was. AppliedBuoyancyForce is sticky -- it keeps the last value applied and " +
                 "a reader cannot tell a fresh application from a repeat of an old one, which is why a " +
                 "whole evening went into a 0.47 N that the log said was being applied every step. " +
                 "2026-09-14.")]
        public Vector3 AppliedBuoyancyThisStep;
        [Tooltip("DoUpdate calls, and the subset of them on which buoyancy was actually handed to the " +
                 "solver. If these differ, the reported force is not the time-average applied force.")]
        public int StepCount, BuoyancyApplyCount;
        [Tooltip("Buoyancy this point COMPUTED this step, before the MaxBuoyancyForce clamp and before " +
                 "the willSurfaceSoon scale -- so 'computed vs applied' can be differenced per point.")]
        public float ComputedBuoyancyN, ClampedOffN, ScaledOffN;
        public bool ApplyCustomForce = false;
        public Vector3 CustomForce = Vector3.zero;


        private MixedBody body;
        private WaterQueryModel waterModel;
        private FrequencyTimer waterQueryTimer;
        private ForcePoint[] RelatedForcePoints;


        public Vector3 ApplyForce(Vector3 force, bool onlyUnderWater = false, bool onlyAboveWater = false)
        {
            Vector3 appliedForce = Vector3.zero;
            bool enabled = true;
            if (onlyAboveWater) enabled = !IsSubmerged;
            if (onlyUnderWater) enabled = IsUnderwater;
            if (onlyAboveWater && onlyUnderWater) enabled = false;

            if (enabled)
            {
                appliedForce = force / RelatedForcePoints.Length;
                body.AddForceAtPosition
                (
                    appliedForce,
                    transform.position,
                    ForceMode.Force
                );
            }

            return appliedForce;
        }


        /// <summary>
        /// Fraction of a circle of radius r that lies below a horizontal line at height h above the
        /// circle's lowest point (h in [0, 2r]). 0 at h=0, 0.5 at h=r, 1 at h=2r. This is the exact
        /// immersed-area law of a horizontal cylinder section, so summing strips along a cylindrical
        /// hull reproduces the true waterplane and the true righting behaviour when surface-piercing.
        /// </summary>
        public static float SubmergedSectionFraction(float h, float r)
        {
            if (r <= 0f || h <= 0f) return 0f;
            if (h >= 2f * r) return 1f;
            float a = Mathf.Clamp((r - h) / r, -1f, 1f);      // signed distance centre->chord, in radii
            float theta = 2f * Mathf.Acos(a);                  // central angle of the immersed segment
            return (theta - Mathf.Sin(theta)) / (2f * Mathf.PI);
        }

        /// <summary>Apply a force at this point as-is (no division among sibling ForcePoints).</summary>
        public Vector3 ApplyForceUndivided(Vector3 force)
        {
            body.AddForceAtPosition(force, transform.position, ForceMode.Force);
            return force;
        }

        public void Awake()
        {
            body = new MixedBody(ConnectedArticulationBody, ConnectedRigidbody);

            if (!body.isValid)
            {
                Debug.LogWarning($"{gameObject.name} requires at least one of ConnectedArticulationBody or ConnectedRigidBody to be set!");
                return;
            }

            if (AirDrag == -1) AirDrag = body.drag;
            if (AirAngularDrag == -1) AirAngularDrag = body.angularDrag;
            if (UnderwaterDrag == -1) UnderwaterDrag = body.drag;
            if (UnderwaterAngularDrag == -1) UnderwaterAngularDrag = body.angularDrag;

            // If the force point is doing the gravity, disable the body's own
            if (AddGravity)
            {
                body.useGravity = false;
                if (Mass == 0) Mass = body.mass;
            }

            waterModel = WaterQueryModel.GetWaterQueryModel();
            
            RelatedForcePoints = body.gameObject.GetComponentsInChildren<ForcePoint>();
            // only consider points that are connected to the same body so that
            // FPs can be grouped together, spread out as wanted, and allow "partial" forces on different
            // parts of an articulation chain for example.
            RelatedForcePoints = RelatedForcePoints.Where(p => p.ConnectedArticulationBody == ConnectedArticulationBody || p.ConnectedRigidbody == ConnectedRigidbody).ToArray();
            if (AutomaticCenterOfGravity)
            {
                body.automaticCenterOfMass = false;
                var centerOfMass = RelatedForcePoints.Select(point => point.transform.localPosition).Aggregate(new Vector3(0, 0, 0), (s, v) => s + v);
                body.centerOfMass = centerOfMass / RelatedForcePoints.Length;
            }

            if (VolumeMesh == null && VolumeObject != null) VolumeMesh = VolumeObject.GetComponent<MeshFilter>().mesh;
            if (Volume == 0 && VolumeMesh != null) Volume = MeshVolume.CalculateVolumeOfMesh(VolumeMesh, VolumeObject.transform.lossyScale);

            waterQueryTimer = new FrequencyTimer(WaterQueryFrequency);
        }

        void UpdateDepth()
        {
            if (waterModel == null)
            {
                waterModel = WaterQueryModel.GetWaterQueryModel();
                if (waterModel == null) return;
            }

            float waterSurfaceLevel = waterModel.GetWaterLevelAt(transform.position);
            CurrentDepth = waterSurfaceLevel - transform.position.y;
            if (SectionRadius > 0f)
            {
                IsSubmerged = CurrentDepth >= SectionRadius;      // whole section under
                IsUnderwater = CurrentDepth > -SectionRadius;     // any of the section under
            }
            else
            {
                IsSubmerged = CurrentDepth >= DepthBeforeSubmerged;
                IsUnderwater = CurrentDepth > 0;
            }
        }

        // Volume * Density * Gravity
        void FixedUpdate()
        {
            if (Physics.simulationMode == SimulationMode.FixedUpdate) DoUpdate();
        }

        public void DoUpdate()
        {
            StepCount++;
            AppliedBuoyancyThisStep = Vector3.zero;
            ComputedBuoyancyN = ClampedOffN = ScaledOffN = 0f;
            var forcePointPosition = transform.position;
            if (AddGravity)
            {
                AppliedGravityForce = ApplyForce(Mass * Physics.gravity);

                if (DrawForces) Debug.DrawLine(forcePointPosition, forcePointPosition + AppliedGravityForce, Color.red, 0.1f);
            }

            // Only update water level related stuff at a limited frequency to avoid performance hits
            if (waterQueryTimer.ExhaustTicks(Clock.Now)) // returns true if any ticks were exhausted or if frequency is <= 0
            {
                UpdateDepth();
                // the forces applied need to be scaled up by the ratio of fixedupdate rate to water query rate
                // since AddForceAtPosition assumes the force is per fixedupdate
                // otherwise, if the water query rate is lower than fixedupdate rate, the forces will
                // be under-applied
                float waterForceScale = WaterQueryFrequency > 0 ? 1f / Time.fixedDeltaTime / WaterQueryFrequency : 1f;
                if (IsUnderwater)
                {
                    // before apply any buoyancy, check if our current upward speed would already take us out of the water
                    // so that we dont go to the moon. The correct way to do this would be to be able to _set_ the position
                    // of the body OR increase sim frequency, but articulations remove the first option and increasing sim
                    // frequency is expensive, so here we are.
                    float upwardSpeed = Vector3.Dot(body.velocity, Vector3.up);
                    float dx = upwardSpeed * Time.fixedDeltaTime * BuoyancyDebounceFrames;
                    bool willSurfaceSoon = CurrentDepth - dx <= 0;
                    // dx is exactly 0 whenever the body is at rest, which makes this 0/0 -> NaN, and
                    // PhysX then REJECTS the whole force ("assign attempt ... is not valid") so the
                    // strip's buoyancy is silently dropped for that step.  MEASURED 2026-09-13 on the
                    // still-water mass audit: 27-36 non-finite point samples per case.  Guard it: when
                    // dx is negligible the point is not about to leave the water on this step anyway,
                    // and the immersion law already gives the right (half-immersed) force at depth 0.
                    if (willSurfaceSoon && dx > 1e-6f) waterForceScale *= CurrentDepth / dx;

                    float displacementMultiplier = SectionRadius > 0f
                        ? SubmergedSectionFraction(CurrentDepth + SectionRadius, SectionRadius)
                        : Mathf.Clamp01(CurrentDepth / DepthBeforeSubmerged);
                    var buoyancyForceMag = Volume * WaterDensity * Math.Abs(Physics.gravity.y) * displacementMultiplier;
                    ComputedBuoyancyN = buoyancyForceMag;
                    buoyancyForceMag = Mathf.Min(MaxBuoyancyForce, buoyancyForceMag);
                    ClampedOffN = ComputedBuoyancyN - buoyancyForceMag;
                    ScaledOffN = buoyancyForceMag * (1f - waterForceScale);
                    var buoyancyForce = new Vector3(0, buoyancyForceMag, 0);

                    AppliedBuoyancyForce = VolumeIsPerPoint
                        ? ApplyForceUndivided(waterForceScale * buoyancyForce)
                        : ApplyForce(waterForceScale * buoyancyForce, onlyUnderWater: true);
                    AppliedBuoyancyThisStep = AppliedBuoyancyForce;
                    BuoyancyApplyCount++;

                    if (DrawForces) Debug.DrawLine(forcePointPosition, forcePointPosition + AppliedBuoyancyForce, Color.blue, 0.1f);
                }

                // change the drag of the body to underwater if any is point is. This is a ad-hoc way to 
                // simulate the sticktion water usually applies to objects
                // also, some objects might need to be useful under AND over water (like ropes...)
                // and their drag really should reflect where they are moment to moment
                // yes, all of the points will do the same thing. but this makes it so we dont need
                // a central forcepoint controller or sth
                var anyUnderwater = RelatedForcePoints.Select(p => p.IsUnderwater).Aggregate(false, (s, v) => s || v);
                body.drag = anyUnderwater ? UnderwaterDrag : AirDrag;
                body.angularDrag = anyUnderwater ? UnderwaterAngularDrag : AirAngularDrag;
            }



            // And lastly, whatever custom force was set.
            if (ApplyCustomForce) ApplyForce(CustomForce);
        }
    }
}