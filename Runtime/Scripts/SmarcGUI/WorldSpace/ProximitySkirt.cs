using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;   // Float32MultiArrayMsg

namespace SmarcGUI.WorldSpace
{
    /// <summary>
    /// Proximity skirt — the "parking sensor" view of the margin rose
    /// (strategy §5b). A translucent ring of wedges around the hull, one per
    /// 10 deg sector, fed by perception/margin_rose:
    ///
    ///   green   margin >= 2x margin0      comfortable
    ///   yellow  margin0 .. 2x margin0     closing
    ///   red     margin <= margin0         at the floor / governor at zero
    ///   grey    UNKNOWN (NaN)             no evidence in that sector
    ///
    /// The grey is the point of the whole display. HT1 measured that a
    /// forward-only sonar plus memory knows a median 28% of the 36 sectors, so
    /// most of the skirt is honestly grey most of the time. Painting unknown
    /// as green would make this instrument dangerous — an operator would read
    /// "clear astern" from an absence of data.
    ///
    /// Wedge OPACITY carries confidence: solid for live evidence, fading with
    /// the age (metres travelled) since the sector was last seen, so you can
    /// watch the belief decay behind the vehicle.
    ///
    /// Subscriber-driven and standalone: drop the prefab in any scene, set
    /// RobotName, point Follow at the vehicle. Nothing is authored in the GUI.
    /// </summary>
    [AddComponentMenu("Smarc/GUI/ProximitySkirt")]
    public class ProximitySkirt : MonoBehaviour
    {
        [Tooltip("Robot whose rose to display (absolute topic /<RobotName>/perception/margin_rose).")]
        public string RobotName = "sam_auv_v1";

        [Tooltip("Transform the skirt follows (usually the vehicle's base_link).")]
        public Transform Follow;

        [Tooltip("Metres above/below the vehicle to draw the skirt.")]
        public float VerticalOffset = 0f;

        [Tooltip("Clamp for how far a wedge is drawn; keeps the display readable in open water.")]
        public float MaxDrawRange = 8f;

        [Tooltip("Margin floor used for the colour bands — keep in step with the governor's margin0.")]
        public float Margin0 = 2.5f;

        [Tooltip("Wedge opacity for fresh evidence; memory fades from here.")]
        [Range(0.05f, 1f)] public float MaxAlpha = 0.55f;

        [Tooltip("Metres of travel after which memory evidence is drawn at its faintest.")]
        public float FadeOverMetres = 8f;

        [Tooltip("Seconds without a rose message before the skirt hides itself. A stale " +
                 "safety display is worse than none.")]
        public float StaleSec = 3f;

        const int N = 36;                  // must match sonar_slam.rose.N_SECTORS
        float[] margin = new float[N], sigma = new float[N], age = new float[N];
        float lastMsgTime = -999f;
        readonly List<GameObject> wedges = new List<GameObject>();
        Material mat;

        void Start()
        {
            for (int i = 0; i < N; i++) margin[i] = float.NaN;
            mat = new Material(Shader.Find("Sprites/Default"));  // unlit, alpha-blended
            for (int i = 0; i < N; i++) wedges.Add(MakeWedge(i));

            var ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<Float32MultiArrayMsg>($"/{RobotName}/perception/margin_rose", m =>
            {
                if (m.data == null || m.data.Length < N * 3) return;
                for (int i = 0; i < N; i++)
                {
                    margin[i] = m.data[i * 3 + 0];
                    sigma[i]  = m.data[i * 3 + 1];
                    age[i]    = m.data[i * 3 + 2];
                }
                lastMsgTime = Time.time;
            });
        }

        /// <summary>One flat triangle fan spanning 10 deg, in the skirt's local frame.</summary>
        GameObject MakeWedge(int i)
        {
            var go = new GameObject($"wedge_{i:00}");
            go.transform.SetParent(transform, false);
            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            mr.material = mat;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mr.receiveShadows = false;
            mf.mesh = new Mesh();
            return go;
        }

        /// <summary>ROS sector bearing (0 = bow, +CCW/port) -> Unity local direction.
        /// Unity is left-handed with +Z forward and +X starboard, so a ROS bearing
        /// of b maps to (sin(-b), 0, cos(-b)) — get this wrong and the skirt is
        /// mirrored, which is the one bug that would make it lie about which side
        /// the wall is on.</summary>
        static Vector3 Dir(float bearingDeg)
        {
            float r = -bearingDeg * Mathf.Deg2Rad;
            return new Vector3(Mathf.Sin(r), 0f, Mathf.Cos(r));
        }

        void Update()
        {
            bool fresh = Time.time - lastMsgTime < StaleSec;
            if (Follow != null)
            {
                transform.position = Follow.position + Vector3.up * VerticalOffset;
                // yaw-follow only: the skirt is a horizontal margin, it should not
                // roll and pitch with the hull
                transform.rotation = Quaternion.Euler(0f, Follow.eulerAngles.y, 0f);
            }

            for (int i = 0; i < N; i++)
            {
                var go = wedges[i];
                if (!fresh) { go.SetActive(false); continue; }
                go.SetActive(true);

                float d = margin[i];
                bool known = !float.IsNaN(d) && !float.IsInfinity(d);
                float draw = known ? Mathf.Min(d, MaxDrawRange) : MaxDrawRange;

                Color c;
                if (!known)                 c = new Color(0.55f, 0.55f, 0.55f, MaxAlpha * 0.25f);
                else if (d <= Margin0)      c = new Color(1.00f, 0.15f, 0.10f, MaxAlpha);
                else if (d <= 2f * Margin0) c = new Color(1.00f, 0.70f, 0.00f, MaxAlpha * 0.9f);
                else                        c = new Color(0.20f, 0.85f, 0.35f, MaxAlpha * 0.8f);

                if (known && age[i] > 0.01f)      // memory fades with distance travelled
                    c.a *= Mathf.Lerp(1f, 0.3f, Mathf.Clamp01(age[i] / FadeOverMetres));

                BuildWedgeMesh(go.GetComponent<MeshFilter>().mesh, i, draw);
                var mr = go.GetComponent<MeshRenderer>();
                mr.material.color = c;
            }
        }

        void BuildWedgeMesh(Mesh mesh, int i, float radius)
        {
            const float half = 360f / N / 2f - 0.5f;   // small gap between wedges
            float centre = (i + 0.5f) * (360f / N);
            var v = new Vector3[5];
            v[0] = Vector3.zero;
            for (int k = 0; k < 4; k++)
                v[k + 1] = Dir(centre - half + k * (2 * half / 3f)) * radius;
            mesh.Clear();
            mesh.vertices = v;
            mesh.triangles = new[] { 0, 1, 2, 0, 2, 3, 0, 3, 4 };
            mesh.RecalculateNormals();
        }
    }
}
