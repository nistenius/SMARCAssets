using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace SmarcGUI.WorldSpace
{
    /// <summary>
    /// A "hula hoop" torus visualizing the acceptance region of a waypoint:
    /// hoop diameter = 2 x waypoint tolerance, so passing anywhere inside the
    /// hoop means the WP counts as reached.
    /// The hoop lies in the plane perpendicular to its local +Z axis; use
    /// SetDirection() to face it along the path leg so the vehicle flies through it.
    ///
    /// RENDERING (2026-08-21, the §3o open defect: hoops were not visible below the surface).
    /// The material is built in a fixed order, and every rung says out loud which one it took —
    /// the whole reason this was unresolvable for three sessions is that a hoop that is not
    /// there and a hoop that is not rendering look identical:
    ///
    ///   1. `HoopMaterial`, a SERIALIZED material asset. This is the intended path.
    ///      `Shader.Find` was the previous path and it returns null in player builds and after
    ///      any render-pipeline change, and a material made with `new Material(shader)` carries
    ///      none of HDRP's keywords — HDRP requires `HDMaterial.ValidateMaterial` after the
    ///      properties are set, which nothing was calling.
    ///   2. `UseEmissive`: an emissive HDRP material survives the water's absorption. This is
    ///      not cosmetic. `baltic` water absorbs fully at ~6 m, so a purely reflective hoop at
    ///      10 m is gone by construction — no rendering fix can bring it back, and making the
    ///      water clearer to compensate would be faking the water instead of lighting the hoop.
    ///   3. `RenderQueueOverride`: the last rung, for a material that is being sorted against
    ///      the water surface or the underwater pass. -1 leaves the material's own queue alone.
    ///
    /// `LogRenderDiagnostics` prints, one frame after the hoop is built: which rung was used,
    /// the shader name, the render queue, the world position, the radius, and
    /// `MeshRenderer.isVisible`. That last flag separates "culled / off camera" from
    /// "drawn and invisible", which the gizmo ring cannot do from the Game view.
    /// </summary>
    public class WaypointHoop : MonoBehaviour
    {
        [Tooltip("Radius of the tube of the torus itself, purely visual.")]
        public float TubeRadius = 0.08f;
        public int RingSegments = 48;
        public int TubeSegments = 10;
        public Color HoopColor = new Color(1f, 0.6f, 0f, 1f); // orange

        [Header("Rendering (see the class comment — SETTLED §3o)")]
        [Tooltip("Material asset for the hoop. ASSIGN THIS: SMARC/Video/1 creates WaypointHoop.mat. Left empty, the component falls back to Shader.Find(\"HDRP/Unlit\") and says so — that path is null in player builds and produces a material with no HDRP keywords.")]
        public Material HoopMaterial;

        [Tooltip("Drive the material's emissive colour from HoopColor. Baltic water absorbs fully at ~6 m, so a non-emissive hoop 10 m away is gone whatever else is fixed.")]
        public bool UseEmissive = true;

        [Tooltip("Emissive multiplier. 1 is barely lit, 3-6 reads well through murky water, above ~12 blooms.")]
        public float EmissiveIntensity = 4f;

        [Tooltip("Force the material's render queue. -1 leaves it alone. Rung 3 of the fix ladder: 2000 = plain opaque geometry, 3000 = transparent (drawn after the water).")]
        public int RenderQueueOverride = -1;

        [Tooltip("Print which material path was taken, the shader, the queue, the position, the radius and MeshRenderer.isVisible one frame after the hoop is built.")]
        public bool LogRenderDiagnostics = false;

        // DEFAULT OFF SINCE 2026-08-18. A collidable hoop is a synthetic sonar target sitting
        // exactly where the vehicle is trying to navigate. That was arguably tolerable when one
        // hoop existed at a time; now that the whole plan stays on screen it would put a torus
        // at EVERY waypoint into the 3D sonar's returns at once. The algae-farm mission exists
        // to classify real returns into ropes, buoys and anchors, and SETTLED §3g already records
        // this project losing time to "the sonar used to echo off trigger volumes". A
        // visualisation aid must not be a sonar target. Turn it on deliberately when you want a
        // physical gate to exercise obstacle avoidance.
        [Tooltip("Give the hoop a MeshCollider so physics raycasts hit it — makes it visible to the 3D sonar (and a physical gate the vehicle could clip). Leave OFF unless you are deliberately testing obstacle avoidance: it injects synthetic geometry into the sonar.")]
        public bool Collidable = false;

        float radius = -1f;
        MeshFilter meshFilter;
        MeshRenderer meshRenderer;
        MeshCollider meshCollider;
        Material mat;
        string materialPath = "(not built)";   // which rung of the ladder produced `mat`

        void Awake()
        {
            meshFilter = gameObject.GetComponent<MeshFilter>();
            if (meshFilter == null) meshFilter = gameObject.AddComponent<MeshFilter>();
            meshRenderer = gameObject.GetComponent<MeshRenderer>();
            if (meshRenderer == null) meshRenderer = gameObject.AddComponent<MeshRenderer>();

            mat = BuildMaterial();
            meshRenderer.material = mat;
            meshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            // The hoop is a marker, not scenery: it must be readable even when the camera is
            // outside the light probe volume, and it must never be culled for being small.
            meshRenderer.receiveShadows = false;
            meshRenderer.allowOcclusionWhenDynamic = false;
            ApplyColor();

            if (LogRenderDiagnostics) StartCoroutine(ReportNextFrame());
        }

        Material BuildMaterial()
        {
            // Rung 1: a serialized material asset. Correct keywords, correct queue, survives a
            // player build. This is the path that is meant to run.
            if (HoopMaterial != null)
            {
                materialPath = $"serialized asset '{HoopMaterial.name}'";
                return new Material(HoopMaterial);
            }

            // Fallback. Named loudly, because Shader.Find is exactly what SETTLED §3o suspected.
            var shader = Shader.Find("HDRP/Unlit");
            if (shader == null) shader = Shader.Find("HDRP/Lit");
            if (shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            if (shader == null)
            {
                materialPath = "NONE — Shader.Find returned null for every candidate";
                Debug.LogError("[WaypointHoop] Shader.Find found no usable shader and HoopMaterial is empty. " +
                               "The hoop will render with Unity's error material or not at all. " +
                               "Run SMARC/Video/1 - Create video materials and assign WaypointHoop.mat on MissionWPHoop.");
                return null;
            }

            materialPath = $"Shader.Find(\"{shader.name}\") FALLBACK";
            Debug.LogWarning($"[WaypointHoop] no HoopMaterial assigned — built one from Shader.Find(\"{shader.name}\"). " +
                             "This is the path SETTLED §3o suspected: it is null in player builds and carries no HDRP " +
                             "keywords until ValidateMaterial runs. Assign WaypointHoop.mat on the MissionWPHoop object.");
            var m = new Material(shader);
            return m;
        }

        /// <summary>
        /// Recolour an existing hoop. Used to distinguish the waypoint being flown right now
        /// from the ones already passed, once the whole plan is on screen at once.
        /// </summary>
        public void SetColor(Color c)
        {
            HoopColor = c;
            ApplyColor();
        }

        void ApplyColor()
        {
            if (mat == null) return;
            // HDRP/Unlit uses _UnlitColor, HDRP/Lit uses _BaseColor, the others use _Color. Set
            // whichever exists rather than assuming a pipeline: this scene is HDRP but the
            // component is shared, and a hoop that renders black is worse than one that renders
            // wrong.
            if (mat.HasProperty("_UnlitColor")) mat.SetColor("_UnlitColor", HoopColor);
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", HoopColor);
            if (mat.HasProperty("_Color")) mat.color = HoopColor;

            if (UseEmissive) ApplyEmissive();

            // A material built at runtime has no HDRP keywords until this runs; a material whose
            // emissive properties were just written needs it again. Cheap, and the single most
            // likely reason a script-made HDRP material draws as nothing.
            HDMaterial.ValidateMaterial(mat);

            if (RenderQueueOverride >= 0) mat.renderQueue = RenderQueueOverride;
        }

        void ApplyEmissive()
        {
            if (mat == null) return;
            var e = new Color(HoopColor.r, HoopColor.g, HoopColor.b, 1f);
            if (mat.HasProperty("_EmissiveColorLDR")) mat.SetColor("_EmissiveColorLDR", e);
            if (mat.HasProperty("_EmissiveIntensity")) mat.SetFloat("_EmissiveIntensity", EmissiveIntensity);
            if (mat.HasProperty("_EmissiveIntensityUnit")) mat.SetFloat("_EmissiveIntensityUnit", 0f); // Nits
            if (mat.HasProperty("_UseEmissiveIntensity")) mat.SetFloat("_UseEmissiveIntensity", 1f);
            // HDRP reads _EmissiveColor (the resolved HDR value) at shading time.
            if (mat.HasProperty("_EmissiveColor")) mat.SetColor("_EmissiveColor", e * Mathf.Max(0f, EmissiveIntensity));
            mat.EnableKeyword("_EMISSIVE_COLOR_MAP");
            mat.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
        }

        /// <summary>
        /// One frame after the hoop exists, say what it actually is. `isVisible` is the field the
        /// gizmo ring cannot give you: it distinguishes "off camera / culled" from "drawn and
        /// invisible", and those two have been indistinguishable in every previous attempt at
        /// this defect.
        /// </summary>
        System.Collections.IEnumerator ReportNextFrame()
        {
            yield return null;
            string shaderName = mat != null && mat.shader != null ? mat.shader.name : "<null>";
            int queue = mat != null ? mat.renderQueue : -1;
            Debug.Log($"[WaypointHoop] {name}: material via {materialPath}; shader '{shaderName}'; queue {queue}; " +
                      $"emissive {(UseEmissive ? EmissiveIntensity.ToString("F1") : "off")}; " +
                      $"pos {transform.position} radius {radius:F2} m; " +
                      $"renderer enabled {(meshRenderer != null && meshRenderer.enabled)}; " +
                      $"isVisible {(meshRenderer != null && meshRenderer.isVisible)}; " +
                      $"collidable {Collidable}.");
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

        /// <summary>
        /// A wire ring drawn as a GIZMO, so the hoop is locatable in the Scene view even when the
        /// shaded mesh is not.
        ///
        /// WHY (2026-08-18): hoops were invisible below the surface in both Game and Scene view,
        /// and there was no way to tell "the hoop is not there" from "the hoop is there and not
        /// rendering". Toggling the water off is NOT the test -- that removes WaterQueryModel,
        /// which ForcePoints query for buoyancy, so the vehicle simply sinks and floods the
        /// console. Gizmos ignore materials, shaders, fog and the water surface, so:
        ///
        ///   gizmo ring visible, shaded torus not  -> the hoop EXISTS; a rendering problem
        ///   neither visible                       -> no hoop was ever created; a data problem
        ///
        /// That is the discriminating measurement, and it costs one wire circle.
        /// </summary>
        void OnDrawGizmos()
        {
            if (radius <= 0f) return;
            Gizmos.color = HoopColor;
            Gizmos.matrix = transform.localToWorldMatrix;
            const int segs = 48;
            var prev = new Vector3(radius, 0f, 0f);
            for (int i = 1; i <= segs; i++)
            {
                float th = 2f * Mathf.PI * i / segs;
                var cur = new Vector3(Mathf.Cos(th) * radius, Mathf.Sin(th) * radius, 0f);
                Gizmos.DrawLine(prev, cur);
                prev = cur;
            }
            // A short spike along +Z marks the direction the vehicle should fly THROUGH the hoop.
            Gizmos.DrawLine(Vector3.zero, new Vector3(0f, 0f, radius * 0.5f));
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
