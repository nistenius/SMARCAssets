// DockLaunchBox.cs — seeded start poses for the docking runs (2026-09-23, docking work order R1).
// BUILT-UNFLOWN.
//
// Subscribes /<ns>/dock/reset (std_msgs/Int32 = the seed k). On a message, teleports the
// vehicle's ROOT ArticulationBody (base_link) to start pose k of the box, zeroes every body's
// linear/angular velocity and joint velocity, re-arms the gates and contact zones, and publishes
// a {"gate":"start",...} event with the pose it put the hull at. 10 seeded runs need no hand
// placement and no Stop/Play (RIG_STARTUP: ONE Play per session; resets are teleports).
//
// THE BOX (work order §2), base_link in D (origin throat centre, x into the tube, FRD):
//   x_D = -(ConeLength + StartFromMouth) = -6.0 m with the defaults,
//   y_D uniform +-LateralHalf (0.5), z_D uniform +-DepthHalf (0.2), psi_D uniform +-HeadingHalfDeg (10),
//   level (pitch = roll = 0).
// DETERMINISTIC: SplitMix64, draws in the order lateral, depth, heading — bit-for-bit the python
// in sam_docking/start_box.py (the scorer and the command sheet print from that). Pinned values:
//   seed 0: y -0.27263174124438183  z  0.10055178332290296   psi  2.3592543209909023
//   seed 1: y  0.42623392566506524  z -0.010497006442916713  psi  7.908771775084098
//   seed 2: y  0.18736721333848505  z  0.0023210355115992343 psi -5.909176647420644
// The Console prints the drawn triple on every reset — compare it with these once.
//
// DIFFERENCE FROM Teleporter_Sub, ON PURPOSE: joint POSITIONS are NOT reset (Teleporter's
// ResetArticulationBody zeroes them), so the VBS piston and the LCG battery stay where the idle
// trim hold put them. Only velocities are zeroed.

using System.Globalization;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Docking
{
    [AddComponentMenu("Smarc/Docking/Dock Launch Box")]
    public class DockLaunchBox : MonoBehaviour
    {
        public DockStation Station;
        [Header("Start box (base_link, in D) [m, deg]")]
        public float StartFromMouth = 5.5f;
        public float LateralHalf = 0.5f;
        public float DepthHalf = 0.2f;
        public float HeadingHalfDeg = 10f;

        [Header("Debug: teleport to this seed on the next FixedUpdate (Inspector only)")]
        public bool DebugReset = false;
        public int DebugSeed = 0;
        [Tooltip("R1 gate: teleport ON the axis (y = z = psi = 0) at x_D = DebugOnAxisX, for a hand-driven straight shot through the mouth. Published as seed -1.")]
        public bool DebugOnAxis = false;
        public float DebugOnAxisX = -3.0f;

        public int LastSeed { get; private set; } = -1;

        int pendingSeed = -1;
        int releaseStage = 2;
        ROSConnection ros;

        void Start()
        {
            if (Station == null) Station = GetComponentInParent<DockStation>();
            if (Station == null || string.IsNullOrEmpty(Station.Namespace))
            {
                Debug.LogError("[DockLaunchBox] no DockStation / namespace — reset topic not subscribed.");
                return;
            }
            ros = ROSConnection.GetOrCreateInstance();
            string topic = $"/{Station.Namespace}/dock/reset";
            ros.Subscribe<Int32Msg>(topic, m => { pendingSeed = m.data; });
            Debug.Log($"[DockLaunchBox] listening on {topic} (std_msgs/Int32 seed)");
        }

        // ---- the generator (mirror of start_box.py) ------------------------------------------
        static ulong SplitMix(ref ulong state)
        {
            unchecked
            {
                state += 0x9E3779B97F4A7C15UL;
                ulong z = state;
                z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
                z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
                return z ^ (z >> 31);
            }
        }

        static double UniformPm1(ref ulong state)
        {
            ulong z = SplitMix(ref state);
            double u = (z >> 11) * (1.0 / 9007199254740992.0);   // 2^53
            return 2.0 * u - 1.0;
        }

        public static void Draw(int seed, double lat, double dep, double hdg, out double y, out double z, out double psiDeg)
        {
            ulong st;
            unchecked { st = (ulong)(long)seed * 0x632BE59BD9B4E019UL + 0x1234567UL; }
            y = lat * UniformPm1(ref st);
            z = dep * UniformPm1(ref st);
            psiDeg = hdg * UniformPm1(ref st);
        }

        void FixedUpdate()
        {
            if (DebugReset) { DebugReset = false; pendingSeed = DebugSeed; }
            if (DebugOnAxis && Station != null && Station.VehicleRoot != null)
            {
                DebugOnAxis = false;
                Place(-1, DebugOnAxisX, 0.0, 0.0, 0.0);
                return;
            }

            // release the root one physics step after the teleport (Teleporter_Sub's pattern)
            if (releaseStage == 0) releaseStage = 1;
            else if (releaseStage == 1 && Station != null && Station.VehicleRoot != null)
            {
                Station.VehicleRoot.immovable = false;
                releaseStage = 2;
            }

            if (pendingSeed < 0 || Station == null || Station.VehicleRoot == null) return;
            int seed = pendingSeed;
            pendingSeed = -1;
            Teleport(seed);
        }

        void Teleport(int seed)
        {
            Draw(seed, LateralHalf, DepthHalf, HeadingHalfDeg, out double y, out double z, out double psiDeg);
            Place(seed, -(Station.ConeLength + StartFromMouth), y, z, psiDeg);
        }

        void Place(int seed, float xD, double y, double z, double psiDeg)
        {            // D -> dock-local Unity: local.x = y_D, local.y = -z_D, local.z = x_D
            Vector3 world = Station.transform.TransformPoint(new Vector3((float)y, (float)-z, xD));
            // Unity yaw about +y is clockwise seen from above = nose RIGHT = psi_D positive
            Quaternion rot = Station.transform.rotation * Quaternion.Euler(0f, (float)psiDeg, 0f);

            var root = Station.VehicleRoot;
            root.immovable = true;
            root.TeleportRoot(world, rot);
            foreach (var ab in Station.Vehicle.GetComponentsInChildren<ArticulationBody>(true))
            {
                ab.linearVelocity = Vector3.zero;
                ab.angularVelocity = Vector3.zero;
                switch (ab.dofCount)
                {
                    case 1: ab.jointVelocity = new ArticulationReducedSpace(0f); break;
                    case 2: ab.jointVelocity = new ArticulationReducedSpace(0f, 0f); break;
                    case 3: ab.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f); break;
                }
            }
            releaseStage = 0;
            LastSeed = seed;
            Station.ArmGates();

            string json = string.Format(CultureInfo.InvariantCulture,
                "{{\"gate\":\"start\",\"t\":{0:F4},\"seed\":{1},\"x_D\":{2:F4},\"y_D\":{3:F6},\"z_D\":{4:F6},\"psi_deg\":{5:F4}}}",
                DockStation.SimTime, seed, xD, y, z, psiDeg);
            Debug.Log($"[DockLaunchBox] seed {seed}: y {y:R} z {z:R} psi {psiDeg:R} -> world {world.ToString("F3")}");
            Station.PublishEvent(json);
        }
    }
}
