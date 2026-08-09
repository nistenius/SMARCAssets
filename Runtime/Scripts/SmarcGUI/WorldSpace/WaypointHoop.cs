using UnityEngine;

namespace SmarcGUI.WorldSpace
{
    /// <summary>
    /// A "hula hoop" torus visualizing the acceptance region of a waypoint:
    /// hoop diameter = 2 x waypoint tolerance, so passing anywhere inside the
    /// hoop means the WP counts as reached.
    /// The hoop lies in the plane perpendicular to its local +Z axis; use
    /// SetDirection() to face it along the path leg so the vehicle flies through it.
    /// </summary>
    public class WaypointHoop : MonoBehaviour
    {
        [Tooltip("Radius of the tube of the torus itself, purely visual.")]
        public float TubeRadius = 0.08f;
        public int RingSegments = 48;
        public int TubeSegments = 10;
        public Color HoopColor = new Color(1f, 0.6f, 0f, 1f); // orange

        [Tooltip("Give the hoop a MeshCollider so physics raycasts hit it — makes it visible to the 3D sonar (and a physical gate the vehicle could clip).")]
        public bool Collidable = true;

        float radius = -1f;
        MeshFilter meshFilter;
        MeshRenderer meshRenderer;
        MeshCollider meshCollider;

        void Awake()
        {
            meshFilter = gameObject.GetComponent<MeshFilter>();
            if (meshFilter == null) meshFilter = gameObject.AddComponent<MeshFilter>();
            meshRenderer = gameObject.GetComponent<MeshRenderer>();
            if (meshRenderer == null) meshRenderer = gameObject.AddComponent<MeshRenderer>();

            // HDRP project; fall back for other pipelines just in case.
            var shader = Shader.Find("HDRP/Unlit");
            if (shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            var mat = new Material(shader);
            // HDRP/Unlit uses _UnlitColor, the others use _Color / _BaseColor.
            if (mat.HasProperty("_UnlitColor")) mat.SetColor("_UnlitColor", HoopColor);
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", HoopColor);
            if (mat.HasProperty("_Color")) mat.color = HoopColor;
            meshRenderer.material = mat;
            meshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        /// <summary>Set hoop radius = waypoint tolerance (so diameter = 2x tolerance).</summary>
        public void SetRadius(float r)
        {
            if (r <= 0f) return;
            if (Mathf.Approximately(r, radius)) return;
            radius = r;
            var mesh = BuildTorus(radius, TubeRadius, RingSegments, TubeSegments);
            meshFilter.mesh = mesh;
            UpdateCollider(mesh);
        }

        void UpdateCollider(Mesh mesh)
        {
            if (Collidable)
            {
                if (meshCollider == null) meshCollider = gameObject.GetComponent<MeshCollider>();
                if (meshCollider == null) meshCollider = gameObject.AddComponent<MeshCollider>();
                meshCollider.sharedMesh = mesh; // non-convex: raycasts (sonar) see the actual ring
                meshCollider.enabled = true;
            }
            else if (meshCollider != null)
            {
                meshCollider.enabled = false;
            }
        }

        /// <summary>Face the hoop opening along dir (the direction of travel through the WP).</summary>
        public void SetDirection(Vector3 dir)
        {
            if (dir.sqrMagnitude < 1e-6f) return;
            transform.rotation = Quaternion.LookRotation(dir.normalized, Vector3.up);
        }

        static Mesh BuildTorus(float ringRadius, float tubeRadius, int ringSegs, int tubeSegs)
        {
            var mesh = new Mesh { name = "WaypointHoopTorus" };
            int vertCount = ringSegs * tubeSegs;
            var verts = new Vector3[vertCount];
            var normals = new Vector3[vertCount];
            var tris = new int[ringSegs * tubeSegs * 6];

            for (int i = 0; i < ringSegs; i++)
            {
                float theta = 2f * Mathf.PI * i / ringSegs;
                // ring lies in local XY plane, opening along +Z
                var ringDir = new Vector3(Mathf.Cos(theta), Mathf.Sin(theta), 0f);
                var center = ringDir * ringRadius;

                for (int j = 0; j < tubeSegs; j++)
                {
                    float phi = 2f * Mathf.PI * j / tubeSegs;
                    // tube circle spans ringDir (outward) and Z (through-axis)
                    var normal = ringDir * Mathf.Cos(phi) + Vector3.forward * Mathf.Sin(phi);
                    int vi = i * tubeSegs + j;
                    verts[vi] = center + normal * tubeRadius;
                    normals[vi] = normal;
                }
            }

            int t = 0;
            for (int i = 0; i < ringSegs; i++)
            {
                int iNext = (i + 1) % ringSegs;
                for (int j = 0; j < tubeSegs; j++)
                {
                    int jNext = (j + 1) % tubeSegs;
                    int a = i * tubeSegs + j;
                    int b = iNext * tubeSegs + j;
                    int c = iNext * tubeSegs + jNext;
                    int d = i * tubeSegs + jNext;
                    tris[t++] = a; tris[t++] = b; tris[t++] = c;
                    tris[t++] = a; tris[t++] = c; tris[t++] = d;
                }
            }

            mesh.vertices = verts;
            mesh.normals = normals;
            mesh.triangles = tris;
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
