using System;
using System.Collections.Generic;

using UnityEngine;

using Sonar = VehicleComponents.Sensors.Sonar;

namespace Visualizers
{

    public class RayViewer : MonoBehaviour
    {
        Sonar sonar;

        [Header("Rays")]
        [Tooltip("Draw rays in the scene view as lines?")]
        public bool DrawRays = false;
        public Material RayMaterial;
        public float RayThickness = 0.05f;

        [Header("Hits")]
        [Tooltip("Just draw the hit points as particles?")]
        public bool DrawHits = false;
        [Tooltip("Drawing hits every single frame can create A LOT of points. If you want to visualize a large area fast, you maybe don't need 1mm density :)")]
        public int DrawEveryNthFrame = 10;
        [Tooltip("Assign a mesh object to be drawn at every hit point. Assign something with few verts, like a quad or triangle, a sphere at most.")]
        public bool UseRainbow = false;
        public float HitsSize = 0.1f;
        [Tooltip("How many seconds should the hits be drawn? Limited by MaxParticlesMultiplier too.")]
        public float HitsLifetime = 1f;
        [Tooltip("How many sets of rays should we allow to be drawn? Limited by lifetime too.")]
        public int MaxParticlesMultiplier = 10;

        GameObject RayDrawer;
        LineRenderer RaysLR;


        GameObject HitsDrawer;
        ParticleSystem HitsParticleSystem;
        ParticleSystem.EmitParams[] HitsEmitParams;
        int HitsSkipped = 0;


        public static Color Rainbow(float progress)
        {
            float div = Math.Abs(progress % 1) * 6;
            int ascending = (int) ((div % 1) * 255);
            int descending = 255 - ascending;

            static Color FromArgb (int alpha, int red, int green, int blue)
            {
                float fa = ((float)alpha) / 255.0f;
                float fr = ((float)red)   / 255.0f;
                float fg = ((float)green) / 255.0f;
                float fb = ((float)blue)  / 255.0f;
                return new Color(fr,fg,fb,fa);
            }

            switch ((int) div)
            {
                case 0:
                    return FromArgb(255, 255, ascending, 0);
                case 1:
                    return FromArgb(255, descending, 255, 0);
                case 2:
                    return FromArgb(255, 0, 255, ascending);
                case 3:
                    return FromArgb(255, 0, descending, 255);
                case 4:
                    return FromArgb(255, ascending, 0, 255);
                default: // case 5:
                    return FromArgb(255, 255, 0, descending);
            }
        }

        /// <summary>
        /// Turn the hit particles on or off AT RUNTIME, building the drawer if Start() did not.
        ///
        /// WHY THIS EXISTS (2026-08-21): `Start()` only creates `HitsDrawer`/`RayDrawer` when the
        /// corresponding bool was already true, so flipping `DrawHits` in the Inspector mid-Play
        /// used to do nothing at all and looked like the toggle was ignored. The CinematicDirector
        /// turns the beams on per shot — that is what "the beams drape the dock" means on camera —
        /// so the toggle has to be real.
        ///
        /// Nothing here creates a collider. A visualisation must never be a sonar target (§3o/§3g),
        /// and these are ParticleSystem particles and a LineRenderer: no physics at all.
        /// </summary>
        public void SetHitsVisible(bool on)
        {
            DrawHits = on;
            if (on && HitsParticleSystem == null) BuildHitsDrawer();
            if (HitsDrawer != null) HitsDrawer.SetActive(on);
            if (!on && HitsParticleSystem != null)
                HitsParticleSystem.Clear(true);
        }

        /// <summary>Same for the ray lines. Off by default: 2550 lines is a wall, not a beam.</summary>
        public void SetRaysVisible(bool on)
        {
            DrawRays = on;
            if (on && RaysLR == null) BuildRayDrawer();
            if (RaysLR != null) RaysLR.enabled = on;
        }

        void BuildRayDrawer()
        {
            if (RayDrawer != null) return;
            RayDrawer = new GameObject("RayDrawer");
            RayDrawer.transform.SetParent(transform);
            RaysLR = RayDrawer.AddComponent<LineRenderer>();
            RaysLR.material = RayMaterial;
            RaysLR.startWidth = RayThickness;
            RaysLR.endWidth = RayThickness;
            RaysLR.receiveShadows = false;
            RaysLR.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        void BuildHitsDrawer()
        {
            if (HitsDrawer != null) return;
            if (sonar == null) sonar = GetComponent<Sonar>();
            if (sonar == null)
            {
                Debug.LogWarning($"[RayViewer] {name} has no Sonar on the same GameObject — nothing to draw.");
                return;
            }

            HitsDrawer = new GameObject("HitsDrawer");
            HitsDrawer.transform.SetParent(transform);
            HitsParticleSystem = HitsDrawer.AddComponent<ParticleSystem>();

            var rendererModule = HitsDrawer.GetComponent<ParticleSystemRenderer>();
            var mat = Shader.Find("Particles/Standard Unlit");
            if(mat == null)
            {
                DrawHits = false;
                Debug.Log($"Material 'Particles/Standard Unlit' not found, sonar hits wont be drawn. Found: {mat}");
                HitsDrawer.SetActive(false);
                return;
            }

            rendererModule.material = new Material(mat);
            rendererModule.alignment = ParticleSystemRenderSpace.World;
            rendererModule.sortMode = ParticleSystemSortMode.YoungestInFront;
            rendererModule.renderMode = ParticleSystemRenderMode.Mesh;
            rendererModule.mesh = Resources.GetBuiltinResource<Mesh>("Quad.fbx");

            var mainModule = HitsParticleSystem.main;
            mainModule.startSpeed = 0f;
            mainModule.playOnAwake = false;
            mainModule.maxParticles = sonar.TotalRayCount * MaxParticlesMultiplier;
            mainModule.startColor = Color.red;
            mainModule.startSize = HitsSize;
            mainModule.startLifetime = HitsLifetime;
            mainModule.simulationSpace = ParticleSystemSimulationSpace.World;
            mainModule.ringBufferMode = ParticleSystemRingBufferMode.PauseUntilReplaced;

            var emissionModule = HitsParticleSystem.emission;
            emissionModule.enabled = false;
            var shapeModule = HitsParticleSystem.shape;
            shapeModule.enabled = false;

            HitsEmitParams = new ParticleSystem.EmitParams[sonar.TotalRayCount];
        }

        void Start()
        {
            // ONE construction path, called from here and from the runtime toggles. The previous
            // version had the whole ParticleSystem setup written out twice-over-time (Start built
            // it, nothing else could) which is how a toggle ends up silently doing nothing.
            sonar = GetComponent<Sonar>();
            if(DrawRays) BuildRayDrawer();
            if(DrawHits) BuildHitsDrawer();
        }

        void UpdateRays()
        {
            if(RaysLR != null)
            {
                if (!DrawRays) RaysLR.enabled = false;
                else
                {
                    RaysLR.enabled = true;
                    // the pattern is [sonar, hit0 hit1 sonar, hit2 hit3 sonar, hit4 hit5 sonar]
                    var positions = new List<Vector3>();
                    positions.Add(sonar.transform.position);

                    for (int i=0; i<sonar.SonarHits.Length-1; i+=2)
                    {
                        var hit0 = sonar.SonarHits[i].Hit.point;
                        if (hit0 == Vector3.zero) positions.Add(sonar.transform.position);
                        else positions.Add(hit0);
                        var hit1 = sonar.SonarHits[i+1].Hit.point;
                        if (hit1 == Vector3.zero) positions.Add(sonar.transform.position);
                        else positions.Add(hit1);
                        positions.Add(sonar.transform.position);
                    }

                    RaysLR.positionCount = positions.Count;
                    RaysLR.material = RayMaterial;
                    RaysLR.startWidth = RayThickness;
                    RaysLR.endWidth = RayThickness;
                    RaysLR.SetPositions(positions.ToArray());
                }
            }
        }

        void UpdateHits()
        {
            if (!DrawHits || HitsParticleSystem == null || HitsEmitParams == null) return;
            
            for (int i = 0; i < sonar.TotalRayCount; i++)
            {
                var emitParams = HitsEmitParams[i];
                var hitPoint = sonar.SonarHits[i].Hit.point;
                var surfaceNormal = sonar.SonarHits[i].Hit.normal;
                if(surfaceNormal == null) continue;
                if(surfaceNormal == Vector3.zero) surfaceNormal = Vector3.up;

                float normalizedZ = Mathf.InverseLerp(sonar.HitsMaxHeight, sonar.HitsMinHeight, hitPoint.y);
                if (UseRainbow) emitParams.startColor = Rainbow(normalizedZ);
                else emitParams.startColor = Color.red;
                emitParams.position = hitPoint + 0.03f*surfaceNormal;
                emitParams.rotation3D = Quaternion.LookRotation(-surfaceNormal).eulerAngles;
                emitParams.startSize = HitsSize;
                emitParams.startLifetime = HitsLifetime;
                
                HitsParticleSystem.Emit(emitParams, 1);
            }
                
        }

        void Update()
        {
            UpdateRays();

            // FIXED 2026-08-21: `HitsSkipped` started at 0 and the `else` branch was the only thing
            // that ever advanced it, so the counter was pinned at 0 and the hits were emitted EVERY
            // frame — DrawEveryNthFrame did nothing. With 2550 rays, a 3 s lifetime and
            // ringBufferMode = PauseUntilReplaced, the buffer filled in ten frames and the display
            // then stopped updating, which is the opposite of what the field is for.
            int every = Mathf.Max(1, DrawEveryNthFrame);
            if (HitsSkipped == 0) UpdateHits();
            HitsSkipped = (HitsSkipped + 1) % every;
        }

        
    }

}
