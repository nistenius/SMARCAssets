// DockStation.cs — the docking station's scene-side truth (2026-09-23, docking work order R1).
// BUILT-UNFLOWN: written off-rig, never compiled or played by its author.
//
// THE STATION IS RIGID AND STATIC (Ivan, 2026-09-23): no Rigidbody, no ArticulationBody on this
// object or its children. The hull cannot displace it; a hull-dock contact is a scored EVENT
// (DockContact.cs), never a station displacement. Start() refuses (logs an error and disables
// ROS output) if anything under the station carries a Rigidbody.
//
// THIS TRANSFORM IS THE DOCK FRAME. Its position is the THROAT ENTRANCE centre and its Unity
// forward (+z) is the axis INTO the tube. The dock frame D of sam_docking (FRD) is therefore
//     x_D = local.z,  y_D = local.x,  z_D = -local.y      (ToDock / DirToDock below)
//
// PUBLISHES (namespace = the VEHICLE GameObject's name — the same rule ROSBehaviour applies,
// `topic = $"/{robotGO.name}/{topic}"`; never a second copy of the name):
//   /<ns>/dock/gt_pose     geometry_msgs/PoseStamped, 1 Hz, frame "unity_origin",
//                          position = transform.position.To<ENU>(),
//                          orientation = transform.rotation.To<ENU>()  — EXACTLY GT_Odom_Pub's
//                          convention (that publisher is `smarc/odom` on the vehicle), so the dock
//                          and the vehicle ground truth live in one frame. ROS-TCP-Connector's
//                          To<ENU>() quaternion pre-rotates -90 deg yaw then maps to FLU, i.e. the
//                          published pose is a PROPER ROS pose: world ENU, body x = Unity forward.
//                          (Pinned by sam_docking/test/test_dock_frame.py::test_unity_*.)
//   /<ns>/dock/gate_event  std_msgs/String JSON — DockGate / DockContact / DockLaunchBox events
//   /<ns>/dock/info        std_msgs/String JSON, every InfoPeriodS — the geometry as built, so the
//                          controller yaml can be checked against the scene (one fact, compared).
//
// COLLIDER SELF-TEST (the silent-dead MeshCollider trap): on Start, two raycasts against the
// station's own colliders (one at the cone's outer skin from the side, one into the mouth at the
// inner cone wall) — PASS/FAIL in the Console. A FAIL means the FLS cannot see the dock and the
// hull will pass through it; do not fly.

using System.Globalization;
using System.Text;
using DefaultNamespace;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Docking
{
    [AddComponentMenu("Smarc/Docking/Dock Station")]
    [DisallowMultipleComponent]
    public class DockStation : MonoBehaviour
    {
        [Header("Geometry as BUILT [m] (DockSceneBuilder writes these; editing them here does NOT rebuild the mesh)")]
        public float MouthDiameter = 0.60f;
        public float ThroatDiameter = 0.21f;   // round dock v1 (2026-09-23); the builder overwrites this with its as-built value
        public float ConeLength = 0.50f;
        public float TubeLength = 0.80f;
        public float WallThickness = 0.01f;
        public float AxisHeightAboveFloor = 0.35f;
        public float FloorWorldY = float.NaN;

        [Header("Vehicle — its GameObject name IS the ROS namespace")]
        [Tooltip("The #robot-tagged vehicle root (sam21). Empty = find the single #robot in the scene (refuses if 0 or >1).")]
        public GameObject Vehicle;

        [Header("ROS")]
        public bool PublishRos = true;
        public float GtPoseHz = 1f;
        public float InfoPeriodS = 5f;
        public string WorldFrame = "unity_origin";

        public string Namespace { get; private set; }
        public ArticulationBody VehicleRoot { get; private set; }
        public Transform VehicleBaseLink { get; private set; }

        ROSConnection ros;
        string tGt, tEvent, tInfo;
        float nextGt, nextInfo;
        bool rosReady;

        void Awake()
        {
            ResolveVehicle();
        }

        public bool ResolveVehicle()
        {
            if (Vehicle == null)
            {
                var robots = GameObject.FindGameObjectsWithTag("robot");
                if (robots.Length != 1)
                {
                    Debug.LogError($"[DockStation] Vehicle not set and {robots.Length} #robot objects in the scene — refusing to guess the namespace.");
                    return false;
                }
                Vehicle = robots[0];
            }
            Namespace = Vehicle.name;
            var bl = Utils.FindDeepChildWithName(Vehicle, "base_link");
            VehicleBaseLink = bl != null ? bl.transform : null;
            VehicleRoot = VehicleBaseLink != null ? VehicleBaseLink.GetComponent<ArticulationBody>() : null;
            if (VehicleRoot == null || !VehicleRoot.isRoot)
            {
                Debug.LogError($"[DockStation] '{Vehicle.name}' has no root ArticulationBody on base_link — gates and the launch box cannot work.");
                return false;
            }
            return true;
        }

        void Start()
        {
            if (GetComponentsInChildren<Rigidbody>(true).Length > 0 || GetComponentsInChildren<ArticulationBody>(true).Length > 0)
                Debug.LogError("[DockStation] a Rigidbody/ArticulationBody is under the station. The station must be STATIC (Ivan 2026-09-23). Remove it.");
            SelfTest();
            if (!PublishRos || string.IsNullOrEmpty(Namespace)) return;
            ros = ROSConnection.GetOrCreateInstance();
            tGt = $"/{Namespace}/dock/gt_pose";
            tEvent = $"/{Namespace}/dock/gate_event";
            tInfo = $"/{Namespace}/dock/info";
            ros.RegisterPublisher<PoseStampedMsg>(tGt);
            ros.RegisterPublisher<StringMsg>(tEvent);
            ros.RegisterPublisher<StringMsg>(tInfo);
            rosReady = true;
            Debug.Log($"[DockStation] publishing {tGt} ({GtPoseHz} Hz, frame {WorldFrame}), {tEvent}, {tInfo}. Throat at {transform.position.ToString("F3")}, axis {transform.forward.ToString("F3")}");
        }

        void FixedUpdate()
        {
            if (!rosReady) return;
            float now = Time.time;
            if (GtPoseHz > 0f && now >= nextGt)
            {
                nextGt = now + 1f / GtPoseHz;
                var msg = new PoseStampedMsg
                {
                    header = new HeaderMsg { stamp = new TimeStamp(Clock.time), frame_id = WorldFrame },
                    pose = new PoseMsg { position = transform.position.To<ENU>(), orientation = transform.rotation.To<ENU>() }
                };
                ros.Publish(tGt, msg);
            }
            if (InfoPeriodS > 0f && now >= nextInfo)
            {
                nextInfo = now + InfoPeriodS;
                ros.Publish(tInfo, new StringMsg(InfoJson()));
            }
        }

        public string InfoJson()
        {
            var c = CultureInfo.InvariantCulture;
            return string.Format(c,
                "{{\"mouth_d\":{0:F4},\"throat_d\":{1:F4},\"cone_len\":{2:F4},\"tube_len\":{3:F4},\"wall\":{4:F4}," +
                "\"axis_height\":{5:F4},\"floor_y\":{6:F4},\"mouth_x_D\":{7:F4},\"throat_world\":[{8:F4},{9:F4},{10:F4}],\"ns\":\"{11}\"}}",
                MouthDiameter, ThroatDiameter, ConeLength, TubeLength, WallThickness, AxisHeightAboveFloor, FloorWorldY,
                -ConeLength, transform.position.x, transform.position.y, transform.position.z, Namespace);
        }

        public void PublishEvent(string json)
        {
            Debug.Log($"[DockStation] event {json}");
            if (rosReady) ros.Publish(tEvent, new StringMsg(json));
        }

        public static double SimTime => Clock.time;

        // ---- frame D helpers -------------------------------------------------------------------
        public Vector3 ToDock(Vector3 world)
        {
            var l = transform.InverseTransformPoint(world);
            return new Vector3(l.z, l.x, -l.y);
        }

        public Vector3 DirToDock(Vector3 worldDir)
        {
            var l = transform.InverseTransformDirection(worldDir);
            return new Vector3(l.z, l.x, -l.y);
        }

        /// <summary>psi (nose right +) and theta (nose up +) of a world direction, in D (FRD).</summary>
        public void Angles(Vector3 worldForward, out float psi, out float theta)
        {
            var f = DirToDock(worldForward);
            psi = Mathf.Atan2(f.y, f.x);
            theta = Mathf.Atan2(-f.z, Mathf.Sqrt(f.x * f.x + f.y * f.y));
        }

        /// <summary>Where the vehicle's longitudinal axis pierces the plane x_D = planeX, in D.
        /// Returns false if the axis is parallel to the plane.</summary>
        public bool AxisPierce(float planeX, out Vector3 hitD)
        {
            hitD = Vector3.zero;
            if (VehicleBaseLink == null) return false;
            Vector3 p = ToDock(VehicleBaseLink.position), f = DirToDock(VehicleBaseLink.forward);
            if (Mathf.Abs(f.x) < 1e-4f) return false;
            float s = (planeX - p.x) / f.x;
            hitD = p + s * f;
            return true;
        }

        public void ArmGates()
        {
            foreach (var g in GetComponentsInChildren<DockGate>(true)) g.Arm();
            foreach (var c in GetComponentsInChildren<DockContact>(true)) c.Arm();
        }

        void SelfTest()
        {
            var cols = GetComponentsInChildren<Collider>(true);
            var sb = new StringBuilder("[DockStation] collider self-test: ");
            sb.Append(Probe("outer skin from the side", transform.TransformPoint(new Vector3(1.5f, 0f, -0.25f)), -transform.right, cols));
            sb.Append(" | ");
            sb.Append(Probe("inner cone through the mouth", transform.TransformPoint(new Vector3(0.20f, 0f, -2.0f)),
                            (transform.TransformPoint(new Vector3(0.20f, 0f, -0.30f)) - transform.TransformPoint(new Vector3(0.20f, 0f, -2.0f))).normalized, cols));
            Debug.Log(sb.ToString());
        }

        static string Probe(string what, Vector3 origin, Vector3 dir, Collider[] mine)
        {
            var hits = Physics.RaycastAll(origin, dir, 5f, ~0, QueryTriggerInteraction.Ignore);
            float best = float.PositiveInfinity; Collider bc = null;
            foreach (var h in hits) if (h.distance < best) { best = h.distance; bc = h.collider; }
            bool ours = false;
            foreach (var c in mine) if (c == bc) ours = true;
            return ours ? $"PASS {what} ({bc.name} at {best:F3} m)"
                        : $"FAIL {what} (first hit: {(bc == null ? "nothing" : bc.name)})";
        }
    }
}
