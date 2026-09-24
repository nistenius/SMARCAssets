// DockGate.cs — a thin trigger in the mouth or throat plane that records the crossing with the
// GROUND-TRUTH errors at that instant (2026-09-23, docking work order R1). BUILT-UNFLOWN.
//
// This is the SCORER's truth (docking_score.py). It is NOT fed to any controller.
//
// On the FIRST hull collider entering the trigger after Arm() (DockLaunchBox arms on every reset;
// Play start arms too), publishes on /<ns>/dock/gate_event (via DockStation) a JSON object:
//   {"gate":"mouth"|"throat", "t": sim s, "seed": k,
//    "e_y","e_z":   where the hull's LONGITUDINAL AXIS pierces this plane, in D [m] (FRD:
//                   e_y starboard +, e_z down +). The axis-plane intersection, not base_link's
//                   offset: at the instant of crossing that is the number the cone clearance
//                   is judged against;
//    "psi_err","theta_err": hull axis vs dock axis [rad] (nose right +, nose up +);
//    "speed": |v| [m/s], "u_axis": velocity along the dock axis [m/s];
//    "x_base","y_base","z_base": base_link in D [m]; "by": the collider that entered}
//
// The trigger lives on the "Ignore Raycast" layer so GT altitude / generic raycasts do not see
// it; the FLS already skips triggers (Sonar.cs: hitTriggers false).

using System.Globalization;
using UnityEngine;

namespace Docking
{
    [AddComponentMenu("Smarc/Docking/Dock Gate")]
    [RequireComponent(typeof(BoxCollider))]
    public class DockGate : MonoBehaviour
    {
        public string GateName = "mouth";
        [Tooltip("Plane position in D (x_D) — set by the builder: -ConeLength for the mouth, 0 for the throat.")]
        public float PlaneX = 0f;
        public DockStation Station;

        bool armed = true;
        public void Arm() { armed = true; }

        void Reset()
        {
            var bc = GetComponent<BoxCollider>();
            bc.isTrigger = true;
        }

        void Awake()
        {
            if (Station == null) Station = GetComponentInParent<DockStation>();
            var bc = GetComponent<BoxCollider>();
            if (!bc.isTrigger) { bc.isTrigger = true; Debug.LogWarning($"[DockGate {GateName}] collider was not a trigger — fixed at runtime; rebuild the scene."); }
        }

        void OnTriggerEnter(Collider other)
        {
            if (!armed || Station == null || Station.Vehicle == null) return;
            if (!other.transform.IsChildOf(Station.Vehicle.transform)) return;
            if (other.isTrigger) return;                      // the hull's own trigger spheres do not count
            armed = false;

            var c = CultureInfo.InvariantCulture;
            Station.AxisPierce(PlaneX, out Vector3 hit);
            Station.Angles(Station.VehicleBaseLink.forward, out float psi, out float theta);
            Vector3 v = Station.VehicleRoot.linearVelocity;
            Vector3 vD = Station.DirToDock(v);
            Vector3 b = Station.ToDock(Station.VehicleBaseLink.position);
            var launch = Station.GetComponentInChildren<DockLaunchBox>(true);
            int seed = launch != null ? launch.LastSeed : -1;
            string json = string.Format(c,
                "{{\"gate\":\"{0}\",\"t\":{1:F4},\"seed\":{2},\"e_y\":{3:F5},\"e_z\":{4:F5},\"psi_err\":{5:F5},\"theta_err\":{6:F5}," +
                "\"speed\":{7:F4},\"u_axis\":{8:F4},\"x_base\":{9:F4},\"y_base\":{10:F5},\"z_base\":{11:F5},\"by\":\"{12}\"}}",
                GateName, DockStation.SimTime, seed, hit.y, hit.z, psi, theta, v.magnitude, vD.x, b.x, b.y, b.z, other.name);
            Station.PublishEvent(json);
        }
    }
}
