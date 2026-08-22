using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace SmarcGUI.Water
{
    /// <summary>
    /// "Marine snow": suspended particles in a box that travels with the active camera, so the
    /// camera always flies through a fresh volume of them, and they only exist below the water
    /// line.
    ///
    /// THE BOX FOLLOWS THE CAMERA, THE PARTICLES DO NOT. The shape module is parented to the
    /// camera but the simulation space is WORLD, so already-spawned particles stay put in the
    /// scene while new ones are born ahead of the lens. That is what makes a dolly move read as
    /// motion instead of as a screen-space overlay.
    ///
    /// IT MUST NOT BECOME A SONAR TARGET (SETTLED §3o, §3g). No collider is ever created and the
    /// ParticleSystem's collision and trigger modules are explicitly disabled: the 3D sonar
    /// raycasts the physics scene, and the algae-farm work exists to classify real returns.
    ///
    /// THE UNDERWATER TEST DOES NOT CALL GetWaterLevelAt, ON PURPOSE. `HDRPWaterQueryModel` is a
    /// single shared instance that seeds each search from the PREVIOUS caller's result
    /// (SETTLED §3s). Asking it where the water is at the camera — which during a fly-in is
    /// hundreds of metres from the hull — would hand the vehicle's ForcePoints a poisoned seed on
    /// their next query. So this component reads the still-water plane straight off the
    /// WaterSurface transform, which is pinned at Y = 0 anyway and is exactly the number a
    /// swell-free pool surface would return.
    /// </summary>
    [AddComponentMenu("Smarc/Water/Underwater Particles")]
    public class UnderwaterParticles : MonoBehaviour
    {
        [Header("Where")]
        [Tooltip("Camera the emission box follows. Left empty, the enabled camera with the highest depth is used and re-checked every RecheckCameraSec.")]
        public Camera TargetCamera;
        [Tooltip("Seconds between re-checks for the active camera while TargetCamera is empty.")]
        public float RecheckCameraSec = 0.5f;
        [Tooltip("WaterSurface whose transform Y is the still-water plane. Left empty, the first one in the scene is used.")]
        public WaterSurface Surface;

        [Header("Emission volume (metres, camera-local)")]
        [Tooltip("Width and height of the box in front of the lens.")]
        public Vector2 BoxWidthHeight = new Vector2(24f, 16f);
        [Tooltip("Depth of the box along the view direction.")]
        public float BoxDepth = 26f;
        [Tooltip("How far in front of the lens the box centre sits. Keep at least a couple of metres so particles are not born inside the near plane.")]
        public float BoxForwardOffset = 9f;

        [Header("Look")]
        [Tooltip("Particles per cubic metre. 0.02 is a clean sea, 0.15 reads as thick Baltic summer water. Total particle count is this times the box volume, capped by MaxParticles.")]
        public float ParticlesPerCubicMetre = 0.06f;
        [Tooltip("Hard cap on live particles, whatever the density says.")]
        public int MaxParticles = 3000;
        public Vector2 SizeRange = new Vector2(0.010f, 0.045f);
        [Tooltip("Sink rate (m/s). Marine snow falls; a positive number here is downward.")]
        public float SinkSpeed = 0.012f;
        [Tooltip("Random horizontal drift (m/s).")]
        public float DriftSpeed = 0.02f;
        [Tooltip("Seconds a particle lives. Long lives plus world space means a slow camera passes the same flecks twice, which looks right.")]
        public Vector2 LifetimeRange = new Vector2(18f, 40f);
        public Color ParticleColor = new Color(0.85f, 0.88f, 0.78f, 0.55f);

        [Header("Material")]
        [Tooltip("Transparent unlit material for the flecks. ASSIGN THIS — SMARC/Video/1 creates MarineSnow.mat for it. Left empty the component builds one from Shader.Find(\"HDRP/Unlit\"), which returns null in a player build and after a pipeline change, and it will say so.")]
        public Material ParticleMaterial;
        [Tooltip("Generate a soft round dot texture at Play and put it in the material's colour map, so no texture asset has to be shipped or wired.")]
        public bool GenerateDotTexture = true;

        [Header("Gating")]
        [Tooltip("Master switch — the CinematicDirector drives this per shot.")]
        public bool Enabled = true;
        [Tooltip("Only emit while the camera is below the still-water plane. Off means the flecks also hang in the air, which looks like dust and not like water.")]
        public bool UnderwaterOnly = true;
        [Tooltip("Metres below the water plane the camera must be before emission starts. A small number keeps the flecks from popping on at the exact water line during the surfacing shot.")]
        public float UnderwaterMargin = 0.15f;
        [Tooltip("Clear live particles the moment the camera leaves the water, instead of letting them fade out. On for the surfacing shot: flecks hanging in the sky are the one way this effect gives itself away.")]
        public bool ClearOnSurface = true;

        ParticleSystem ps;
        ParticleSystemRenderer psr;
        Material runtimeMat;
        Texture2D dot;
        float nextCameraCheck;
        bool emitting;

        void Start()
        {
            ResolveSurface();
            Build();
        }

        void ResolveSurface()
        {
            if (Surface != null) return;
            var all = FindObjectsByType<WaterSurface>(FindObjectsSortMode.None);
            if (all.Length > 0) Surface = all[0];
            if (Surface == null)
                Debug.LogWarning("[UnderwaterParticles] no WaterSurface in the scene — the underwater gate cannot be evaluated, so particles will be emitted everywhere.");
        }

        /// <summary>Still-water plane. See the class comment for why this is not GetWaterLevelAt.</summary>
        public float WaterPlaneY => Surface != null ? Surface.transform.position.y : 0f;

        void Build()
        {
            var go = new GameObject("MarineSnow");
            go.transform.SetParent(transform, false);
            ps = go.AddComponent<ParticleSystem>();
            psr = go.GetComponent<ParticleSystemRenderer>();

            var main = ps.main;
            main.loop = true;
            main.playOnAwake = false;
            // WORLD, so the flecks stay in the scene while the emission box moves with the lens.
            main.simulationSpace = ParticleSystemSimulationSpace.World;
            main.startLifetime = new ParticleSystem.MinMaxCurve(LifetimeRange.x, LifetimeRange.y);
            main.startSize = new ParticleSystem.MinMaxCurve(SizeRange.x, SizeRange.y);
            main.startSpeed = 0f;
            main.startColor = ParticleColor;
            main.gravityModifier = 0f;          // sink handled by velocity, so it stays terminal-velocity slow
            main.maxParticles = MaxParticles;
            main.cullingMode = ParticleSystemCullingMode.PauseAndCatchup;

            var shape = ps.shape;
            shape.enabled = true;
            shape.shapeType = ParticleSystemShapeType.Box;
            shape.scale = new Vector3(BoxWidthHeight.x, BoxWidthHeight.y, BoxDepth);
            shape.position = new Vector3(0f, 0f, BoxForwardOffset);

            var vel = ps.velocityOverLifetime;
            vel.enabled = true;
            vel.space = ParticleSystemSimulationSpace.World;
            vel.x = new ParticleSystem.MinMaxCurve(-DriftSpeed, DriftSpeed);
            vel.y = new ParticleSystem.MinMaxCurve(-SinkSpeed - DriftSpeed * 0.25f, -SinkSpeed + DriftSpeed * 0.25f);
            vel.z = new ParticleSystem.MinMaxCurve(-DriftSpeed, DriftSpeed);

            var rot = ps.rotationOverLifetime;
            rot.enabled = true;
            rot.z = new ParticleSystem.MinMaxCurve(-0.6f, 0.6f);

            var alpha = ps.colorOverLifetime;
            alpha.enabled = true;
            var grad = new Gradient();
            grad.SetKeys(
                new[] { new GradientColorKey(Color.white, 0f), new GradientColorKey(Color.white, 1f) },
                new[] { new GradientAlphaKey(0f, 0f), new GradientAlphaKey(1f, 0.12f), new GradientAlphaKey(1f, 0.85f), new GradientAlphaKey(0f, 1f) });
            alpha.color = new ParticleSystem.MinMaxGradient(grad);

            // A VISUALISATION AID MUST NOT BE A SONAR TARGET, and must not be a physics body.
            var col = ps.collision; col.enabled = false;
            var trig = ps.trigger; trig.enabled = false;
            var lights = ps.lights; lights.enabled = false;

            psr.renderMode = ParticleSystemRenderMode.Billboard;
            psr.alignment = ParticleSystemRenderSpace.View;
            psr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            psr.receiveShadows = false;
            psr.sortMode = ParticleSystemSortMode.Distance;

            runtimeMat = BuildMaterial();
            psr.material = runtimeMat;

            ApplyEmissionRate();
            ps.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
        }

        Material BuildMaterial()
        {
            Material m;
            if (ParticleMaterial != null)
            {
                m = new Material(ParticleMaterial);
            }
            else
            {
                var shader = Shader.Find("HDRP/Unlit");
                if (shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
                if (shader == null) shader = Shader.Find("Unlit/Transparent");
                if (shader == null)
                {
                    Debug.LogError("[UnderwaterParticles] no unlit shader found by name and no ParticleMaterial assigned. " +
                                   "Run SMARC/Video/1 - Create video materials and assign MarineSnow.mat.");
                    return null;
                }
                Debug.LogWarning($"[UnderwaterParticles] no ParticleMaterial assigned; falling back to Shader.Find(\"{shader.name}\"). " +
                                 "Shader.Find returns null in player builds and after a pipeline change — assign MarineSnow.mat instead.");
                m = new Material(shader);
                MakeTransparentUnlit(m);
            }

            if (m.HasProperty("_UnlitColor")) m.SetColor("_UnlitColor", ParticleColor);
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", ParticleColor);
            if (m.HasProperty("_Color")) m.color = ParticleColor;

            if (GenerateDotTexture)
            {
                dot = MakeDotTexture(32);
                if (m.HasProperty("_UnlitColorMap")) m.SetTexture("_UnlitColorMap", dot);
                else if (m.HasProperty("_BaseColorMap")) m.SetTexture("_BaseColorMap", dot);
                else if (m.HasProperty("_MainTex")) m.SetTexture("_MainTex", dot);
            }
            return m;
        }

        /// <summary>
        /// Turn a freshly-created HDRP material into a transparent, non-depth-writing one.
        /// A material created with `new Material(shader)` carries none of HDRP's keywords, which
        /// is why ValidateMaterial has to be called after the properties are set — the same class
        /// of defect as the waypoint hoop's runtime material.
        /// </summary>
        public static void MakeTransparentUnlit(Material m)
        {
            if (m == null) return;
            if (m.HasProperty("_SurfaceType")) m.SetFloat("_SurfaceType", 1f);      // Transparent
            if (m.HasProperty("_BlendMode")) m.SetFloat("_BlendMode", 0f);          // Alpha
            if (m.HasProperty("_ZWrite")) m.SetFloat("_ZWrite", 0f);
            if (m.HasProperty("_TransparentZWrite")) m.SetFloat("_TransparentZWrite", 0f);
            if (m.HasProperty("_AlphaCutoffEnable")) m.SetFloat("_AlphaCutoffEnable", 0f);
            if (m.HasProperty("_SrcBlend")) m.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
            if (m.HasProperty("_DstBlend")) m.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            HDMaterial.ValidateMaterial(m);
        }

        static Texture2D MakeDotTexture(int size)
        {
            var t = new Texture2D(size, size, TextureFormat.RGBA32, false, true) { wrapMode = TextureWrapMode.Clamp, name = "MarineSnowDot" };
            var px = new Color[size * size];
            float r = size * 0.5f;
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    float dx = (x + 0.5f) - r, dy = (y + 0.5f) - r;
                    float d = Mathf.Sqrt(dx * dx + dy * dy) / r;
                    float a = Mathf.Clamp01(1f - d);
                    a = a * a;                       // soft falloff, no hard rim
                    px[y * size + x] = new Color(1f, 1f, 1f, a);
                }
            t.SetPixels(px);
            t.Apply(false, false);
            return t;
        }

        void ApplyEmissionRate()
        {
            if (ps == null) return;
            float volume = Mathf.Max(0.001f, BoxWidthHeight.x * BoxWidthHeight.y * BoxDepth);
            int wanted = Mathf.Clamp(Mathf.RoundToInt(volume * ParticlesPerCubicMetre), 1, MaxParticles);
            float meanLife = Mathf.Max(0.1f, (LifetimeRange.x + LifetimeRange.y) * 0.5f);

            var main = ps.main;
            main.maxParticles = MaxParticles;

            var em = ps.emission;
            em.enabled = true;
            // Steady state: rate * lifetime = population.
            em.rateOverTime = wanted / meanLife;
        }

        Camera ResolveCamera()
        {
            if (TargetCamera != null) return TargetCamera;
            if (Time.unscaledTime < nextCameraCheck) return cachedCamera;
            nextCameraCheck = Time.unscaledTime + Mathf.Max(0.05f, RecheckCameraSec);

            Camera best = null;
            foreach (var c in Camera.allCameras)
            {
                if (!c.enabled || !c.gameObject.activeInHierarchy) continue;
                if (c.targetTexture != null) continue;                 // not what the Game view shows
                if (best == null || c.depth > best.depth) best = c;
            }
            cachedCamera = best;
            return best;
        }
        Camera cachedCamera;

        /// <summary>Called by the CinematicDirector when it takes over rendering.</summary>
        public void SetCamera(Camera cam)
        {
            TargetCamera = cam;
            cachedCamera = cam;
        }

        void LateUpdate()
        {
            if (ps == null) return;

            var cam = ResolveCamera();
            if (cam == null)
            {
                SetEmitting(false);
                return;
            }

            // Box rides with the lens; particles stay in the world.
            transform.SetPositionAndRotation(cam.transform.position, cam.transform.rotation);

            bool under = !UnderwaterOnly || (cam.transform.position.y < WaterPlaneY - UnderwaterMargin);
            SetEmitting(Enabled && under);
        }

        void SetEmitting(bool on)
        {
            if (on == emitting && ps.isPlaying == on) return;
            emitting = on;
            if (on)
            {
                ApplyEmissionRate();
                ps.Play(true);
            }
            else
            {
                ps.Stop(true, ClearOnSurface
                    ? ParticleSystemStopBehavior.StopEmittingAndClear
                    : ParticleSystemStopBehavior.StopEmitting);
            }
        }

        /// <summary>Push Inspector edits into the live system without restarting Play.</summary>
        public void RefreshFromInspector()
        {
            if (ps == null) return;
            var main = ps.main;
            main.startLifetime = new ParticleSystem.MinMaxCurve(LifetimeRange.x, LifetimeRange.y);
            main.startSize = new ParticleSystem.MinMaxCurve(SizeRange.x, SizeRange.y);
            main.startColor = ParticleColor;
            var shape = ps.shape;
            shape.scale = new Vector3(BoxWidthHeight.x, BoxWidthHeight.y, BoxDepth);
            shape.position = new Vector3(0f, 0f, BoxForwardOffset);
            var vel = ps.velocityOverLifetime;
            vel.x = new ParticleSystem.MinMaxCurve(-DriftSpeed, DriftSpeed);
            vel.y = new ParticleSystem.MinMaxCurve(-SinkSpeed - DriftSpeed * 0.25f, -SinkSpeed + DriftSpeed * 0.25f);
            vel.z = new ParticleSystem.MinMaxCurve(-DriftSpeed, DriftSpeed);
            ApplyEmissionRate();
            if (runtimeMat != null)
            {
                if (runtimeMat.HasProperty("_UnlitColor")) runtimeMat.SetColor("_UnlitColor", ParticleColor);
                if (runtimeMat.HasProperty("_BaseColor")) runtimeMat.SetColor("_BaseColor", ParticleColor);
            }
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.color = new Color(0.6f, 0.9f, 1f, 0.5f);
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawWireCube(new Vector3(0f, 0f, BoxForwardOffset), new Vector3(BoxWidthHeight.x, BoxWidthHeight.y, BoxDepth));
        }

        void OnDestroy()
        {
            if (runtimeMat != null) Destroy(runtimeMat);
            if (dot != null) Destroy(dot);
        }
    }
}
