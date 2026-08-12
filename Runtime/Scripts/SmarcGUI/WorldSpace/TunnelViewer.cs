using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;   // PathMsg
using RosMessageTypes.Std;   // Float32MultiArrayMsg

namespace SmarcGUI.WorldSpace
{
    /// <summary>
    /// Tunnel viewer — renders the planned path-to-follow as a 3D corridor ahead
    /// of the vehicle (strategy §5a).
    ///
    /// Two topics, deliberately separate so either can be missing:
    ///   nav/tunnel          nav_msgs/Path            smooth centerline (§5f)
    ///   nav/tunnel_width    Float32MultiArray        half-width per centerline
    ///                                                sample [m], same length
    ///
    /// Drawn as a ladder of rings along the centerline plus rails down each
    /// side — cheap, reads clearly through murky water, and (unlike a solid
    /// tube) does not hide the sonar returns behind it. Ring colour carries the
    /// squeeze: green where the corridor is wide, red where it closes to the
    /// margin floor, so a narrow passage is visible before the vehicle is in it.
    ///
    /// Positions arrive in the ROS odom/map frame. The transform this component
    /// sits on is used as the ROS-origin anchor: park it on the same object the
    /// rest of the ROS world hangs from (utm/map root), NOT on the vehicle.
    /// </summary>
    [AddComponentMenu("Smarc/GUI/TunnelViewer")]
    public class TunnelViewer : MonoBehaviour
    {
        [Tooltip("Robot whose plan to display (absolute topics /<RobotName>/nav/...).")]
        public string RobotName = "sam_auv_v1";

        [Tooltip("Draw a ring every Nth centerline sample. Higher = sparser ladder.")]
        public int RingStride = 3;

        [Tooltip("Points per ring. 12 is plenty for a readable hoop.")]
        public int RingSegments = 12;

        [Tooltip("Corridor half-width at or below which a ring is drawn red — keep in " +
                 "step with the governor's margin0 so the display and the controller " +
                 "agree about what 'tight' means.")]
        public float TightHalfWidth = 2.5f;

        [Tooltip("Line width [m].")]
        public float LineWidth = 0.06f;

        [Tooltip("Seconds without a plan before the tunnel is hidden. A stale corridor " +
                 "is a lie about where the vehicle intends to go.")]
        public float StaleSec = 5f;

        readonly List<Vector3> centre = new List<Vector3>();
        readonly List<Quaternion> orient = new List<Quaternion>();
        float[] halfWidth = new float[0];
        float lastPathTime = -999f;
        readonly List<LineRenderer> rings = new List<LineRenderer>();
        LineRenderer railL, railR, spine;
        Material mat;

        void Start()
        {
            mat = new Material(Shader.Find("Sprites/Default"));
            spine = MakeLine("spine");
            railL = MakeLine("rail_port");
            railR = MakeLine("rail_stbd");

            var ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<PathMsg>($"/{RobotName}/nav/tunnel", m =>
            {
                centre.Clear(); orient.Clear();
                foreach (var ps in m.poses)
                {
                    var p = ps.pose.position; var q = ps.pose.orientation;
                    // ROS ENU (x east, y north, z up) -> Unity (x right, y up, z fwd)
                    centre.Add(new Vector3((float)p.x, (float)p.z, (float)p.y));
                    orient.Add(new Quaternion((float)q.x, (float)q.z, (float)q.y, -(float)q.w));
                }
                lastPathTime = Time.time;
            });
            ros.Subscribe<Float32MultiArrayMsg>($"/{RobotName}/nav/tunnel_width",
                                                m => halfWidth = m.data ?? new float[0]);
        }

        LineRenderer MakeLine(string name)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.material = mat;
            lr.widthMultiplier = LineWidth;
            lr.useWorldSpace = false;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.numCornerVertices = 2;
            return lr;
        }

        static Color Squeeze(float halfW, float tight)
        {
            if (halfW <= tight)          return new Color(1.0f, 0.2f, 0.1f, 0.9f);
            if (halfW <= 2f * tight)     return new Color(1.0f, 0.75f, 0.0f, 0.8f);
            return new Color(0.25f, 0.8f, 1.0f, 0.6f);
        }

        float HalfWidthAt(int i) =>
            (halfWidth != null && i < halfWidth.Length) ? halfWidth[i] : 1.5f;

        void Update()
        {
            bool fresh = Time.time - lastPathTime < StaleSec && centre.Count >= 2;
            spine.enabled = railL.enabled = railR.enabled = fresh;
            foreach (var r in rings) r.enabled = fresh;
            if (!fresh) return;

            spine.positionCount = centre.Count;
            spine.SetPositions(centre.ToArray());
            spine.startColor = spine.endColor = new Color(1f, 1f, 1f, 0.35f);

            // rails: centerline offset by the local half-width, left and right of
            // the tangent — this is the corridor the vehicle must stay inside
            var left = new Vector3[centre.Count];
            var right = new Vector3[centre.Count];
            for (int i = 0; i < centre.Count; i++)
            {
                Vector3 fwd = (i + 1 < centre.Count ? centre[i + 1] - centre[i]
                                                    : centre[i] - centre[i - 1]);
                fwd.y = 0f;
                if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
                Vector3 side = Vector3.Cross(Vector3.up, fwd.normalized);
                float w = HalfWidthAt(i);
                left[i] = centre[i] + side * w;
                right[i] = centre[i] - side * w;
            }
            railL.positionCount = left.Length;  railL.SetPositions(left);
            railR.positionCount = right.Length; railR.SetPositions(right);
            var cMid = Squeeze(HalfWidthAt(centre.Count / 2), TightHalfWidth);
            railL.startColor = railL.endColor = railR.startColor = railR.endColor = cMid;

            // ladder of rings
            int want = Mathf.Max(0, centre.Count / Mathf.Max(RingStride, 1));
            while (rings.Count < want) rings.Add(MakeLine($"ring_{rings.Count:000}"));
            for (int r = 0; r < rings.Count; r++)
            {
                int i = r * RingStride;
                var lr = rings[r];
                if (i >= centre.Count) { lr.enabled = false; continue; }
                lr.enabled = true;
                Vector3 fwd = (i + 1 < centre.Count ? centre[i + 1] - centre[i]
                                                    : centre[i] - centre[Mathf.Max(i - 1, 0)]);
                if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
                fwd.Normalize();
                Vector3 side = Vector3.Cross(Vector3.up, fwd).normalized;
                Vector3 up = Vector3.Cross(fwd, side).normalized;
                float w = HalfWidthAt(i);
                lr.positionCount = RingSegments + 1;
                for (int k = 0; k <= RingSegments; k++)
                {
                    float a = k * Mathf.PI * 2f / RingSegments;
                    lr.SetPosition(k, centre[i] + (side * Mathf.Cos(a) + up * Mathf.Sin(a)) * w);
                }
                var c = Squeeze(w, TightHalfWidth);
                lr.startColor = lr.endColor = c;
            }
        }
    }
}
