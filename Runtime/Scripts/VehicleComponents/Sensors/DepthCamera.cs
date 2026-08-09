using UnityEngine;
using Unity.Collections;

namespace VehicleComponents.Sensors
{
    /// <summary>
    /// Ground-truth depth camera implemented with physics raycasts through a pinhole
    /// projection model. Render-pipeline independent (works the same in HDRP/URP/built-in),
    /// consistent with how Sonar.cs measures the world (colliders, not visual meshes).
    ///
    /// Intended as a development aid mimicking a RealSense depth stream: one ray per pixel,
    /// depth = z-depth along the camera forward axis (not ray length), 0 where invalid
    /// (no hit, closer than MinRange, farther than MaxRange) - same convention as the
    /// RealSense, which reports 0 for invalid pixels.
    /// </summary>
    [AddComponentMenu("Smarc/Sensor/DepthCamera")]
    public class DepthCamera : Sensor
    {
        [Header("Depth Camera")]
        [Tooltip("Depth image width in pixels. One raycast per pixel: keep modest (e.g. 106x60 = 6360 rays).")]
        public int textureWidth = 106;
        [Tooltip("Depth image height in pixels.")]
        public int textureHeight = 60;
        [Tooltip("Horizontal field of view in degrees. D435i depth: 87.")]
        public float HFOVDeg = 87f;
        [Tooltip("Vertical field of view in degrees. D435i depth: 58.")]
        public float VFOVDeg = 58f;
        [Tooltip("Minimum valid depth in meters. D435i: ~0.3 m.")]
        public float MinRange = 0.3f;
        [Tooltip("Maximum valid depth in meters.")]
        public float MaxRange = 10f;

        // Row-major, top-left origin, meters. 0 = invalid.
        [HideInInspector] public float[] Depths;

        [Header("Load")]
        public float TimeShareInFixedUpdate;

        Vector3[] localDirs;   // per-pixel unit direction in sensor-local frame
        float[] localZ;        // z (forward) component of each unit direction, for range->z-depth conversion
        int numPixels;

        new protected void OnValidate()
        {
            base.OnValidate();
            if (textureWidth < 2) textureWidth = 2;
            if (textureHeight < 2) textureHeight = 2;
            if (MaxRange <= MinRange) MaxRange = MinRange + 1f;
        }

        new protected void Awake()
        {
            base.Awake();
            InitRays();
        }

        void InitRays()
        {
            numPixels = textureWidth * textureHeight;
            Depths = new float[numPixels];
            localDirs = new Vector3[numPixels];
            localZ = new float[numPixels];

            // Pinhole intrinsics from the FOVs
            float fx = textureWidth  / (2f * Mathf.Tan(HFOVDeg * Mathf.Deg2Rad / 2f));
            float fy = textureHeight / (2f * Mathf.Tan(VFOVDeg * Mathf.Deg2Rad / 2f));
            float cx = textureWidth  / 2f;
            float cy = textureHeight / 2f;

            for (int v = 0; v < textureHeight; v++)
            {
                for (int u = 0; u < textureWidth; u++)
                {
                    int i = v * textureWidth + u;
                    // Image convention: u right, v down. Unity: x right, y up, z forward.
                    var d = new Vector3(
                        (u + 0.5f - cx) / fx,
                        -(v + 0.5f - cy) / fy,
                        1f);
                    d.Normalize();
                    localDirs[i] = d;
                    localZ[i] = d.z;
                }
            }
        }

        public override bool UpdateSensor(double deltaTime)
        {
            var t0 = Time.realtimeSinceStartup;

            // Re-init if the user changed resolution/FOV in the inspector at runtime
            if (localDirs == null || localDirs.Length != textureWidth * textureHeight) InitRays();

            var results = new NativeArray<RaycastHit>(numPixels, Allocator.TempJob);
            var commands = new NativeArray<RaycastCommand>(numPixels, Allocator.TempJob);

            var pos = transform.position;
            var rot = transform.rotation;
            for (int i = 0; i < numPixels; i++)
            {
                commands[i] = new RaycastCommand(pos, rot * localDirs[i], QueryParameters.Default, MaxRange);
            }

            var handle = RaycastCommand.ScheduleBatch(commands, results, 64);
            handle.Complete();

            for (int i = 0; i < numPixels; i++)
            {
                var hit = results[i];
                if (hit.collider == null) // no hit (main thread, so the managed accessor is fine)
                {
                    Depths[i] = 0f;
                    continue;
                }
                float z = hit.distance * localZ[i];
                Depths[i] = (z < MinRange || z > MaxRange) ? 0f : z;
            }

            results.Dispose();
            commands.Dispose();

            var t1 = Time.realtimeSinceStartup;
            TimeShareInFixedUpdate = (t1 - t0) / Time.fixedDeltaTime;
            if (TimeShareInFixedUpdate > 0.5f) Debug.LogWarning($"DepthCamera in {transform.parent?.name}/{transform.name} took more than half the time in a fixedUpdate!");

            return true;
        }
    }
}
