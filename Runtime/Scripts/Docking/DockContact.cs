// DockContact.cs — hull-to-station contact as a scored EVENT (2026-09-23, docking work order R1).
// BUILT-UNFLOWN. The station is static, so a contact can never move it; what matters for the
// score is WHERE the hull touched and HOW HARD.
//
// Sits on each station collider (shell, marker board). On collision ENTER with the vehicle,
// publishes /<ns>/dock/gate_event {"gate":"contact","zone": board|rim|cone|tube|endstop,
// "t","seed","x_D","r_D","impulse","rel_speed"}. zone is from the first contact point in D:
// the board and the mouth rim are "before the cone" (the work order's failure condition); cone
// wall contact is the funnel doing its job; tube contact and the end stop happen after the throat.
// One event per zone per arming (DockLaunchBox re-arms on reset), plus a running count in the log.
//
// Unity 6 PhysX: a static non-convex MeshCollider against the hull's colliders. Since 2026-09-23 the
// hull carries TIGHT convex colliders from SAM_HULL.dae (SamTightColliderBuilder; the old capsule
// r 0.0696 is disabled). Round dock v1, throat Ø 0.210: clearance top rail 9.7 mm (binding, upward),
// side-scan pods 28 mm, bare hull 42 mm. A "tube" contact therefore means the real shape touched the
// bore; docking_score.py counts it as a failure.

using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace Docking
{
    [AddComponentMenu("Smarc/Docking/Dock Contact")]
    public class DockContact : MonoBehaviour
    {
        public DockStation Station;
        public bool IsBoard = false;
        readonly HashSet<string> seen = new HashSet<string>();
        int count;

        public void Arm() { seen.Clear(); count = 0; }

        void Awake()
        {
            if (Station == null) Station = GetComponentInParent<DockStation>();
        }

        void OnCollisionEnter(Collision col)
        {
            if (Station == null || Station.Vehicle == null) return;
            if (!col.transform.IsChildOf(Station.Vehicle.transform)) return;
            count++;
            var cp = col.contactCount > 0 ? col.GetContact(0).point : col.transform.position;
            Vector3 d = Station.ToDock(cp);
            float r = Mathf.Sqrt(d.y * d.y + d.z * d.z);
            string zone = IsBoard ? "board"
                        : d.x < -Station.ConeLength + 0.02f ? "rim"
                        : d.x < 0f ? "cone"
                        : d.x < Station.TubeLength - 0.01f ? "tube" : "endstop";
            if (!seen.Add(zone)) return;
            var launch = Station.GetComponentInChildren<DockLaunchBox>(true);
            string json = string.Format(CultureInfo.InvariantCulture,
                "{{\"gate\":\"contact\",\"zone\":\"{0}\",\"t\":{1:F4},\"seed\":{2},\"x_D\":{3:F4},\"r_D\":{4:F4},\"impulse\":{5:F4},\"rel_speed\":{6:F4},\"n\":{7}}}",
                zone, DockStation.SimTime, launch != null ? launch.LastSeed : -1, d.x, r, col.impulse.magnitude,
                col.relativeVelocity.magnitude, count);
            Station.PublishEvent(json);
        }
    }
}
