using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace SmarcGUI.Water
{
    /// <summary>
    /// Named APPEARANCE presets for the scene's existing HDRP WaterSurface — "what does this
    /// water look like", and nothing else.
    ///
    /// WHY THIS IS AN APPEARANCE COMPONENT AND NOT A SECOND WATER OBJECT (SETTLED §3s):
    /// `HDRPWaterQueryModel.GetWaterLevelAt()` seeds every search from the PREVIOUS caller's
    /// result and discards the convergence bool, so the moment the water plane is anywhere but
    /// world zero, two ForcePoints 1.1 m apart disagree about the water level by 0.36 m,
    /// buoyancy becomes a torque, and the vehicle leaves the scene at 67 m/s. Beckholmen has
    /// only ever been stable because its Water sits at Y = 0. Therefore:
    ///
    ///   * this component NEVER writes the water's Transform,
    ///   * it never creates a second WaterSurface,
    ///   * it never disables the Water GameObject (that deletes WaterQueryModel and the vehicle
    ///     sinks — SETTLED §3o; the safe render toggle is WaterRenderToggle.ToggleWaterRender),
    ///   * it never touches `scriptInteractions` (the CPU water simulation the query model reads).
    ///
    /// It reads the transform once, and says so loudly if someone has moved it.
    ///
    /// HOW TO READ THE NUMBERS. HDRP's `absorptionDistance` is the metres of water after which
    /// light is fully absorbed, so it is the field that behaves like "visibility". Beckholmen
    /// shipped at 56.3 m, which is a swimming-pool look; Baltic coastal water in summer is a
    /// Secchi depth of roughly 3-6 m. The two shipped presets are therefore:
    ///
    ///   clear_demo   absorption 56.3 m — today's look, captured field-for-field from the scene
    ///   baltic        absorption  6.0 m — green-brown, murky, still readable at ~10 m
    ///   baltic_murky  absorption  3.5 m — the honest summer Secchi number; close-up shots only
    ///
    /// `baltic` is the shooting preset. `baltic_murky` is more truthful and photographs worse:
    /// at 3.5 m a waypoint hoop 10 m away is gone, which is why the hoop is rendered EMISSIVE
    /// (see WaypointHoop.UseEmissive) rather than by making the water clearer than it is.
    ///
    /// THE UNDERWATER LOOK IS THIS COMPONENT'S JOB, NOT THE FOG VOLUME'S. HDRP's underwater
    /// rendering is driven by the WaterSurface's own absorption/scattering plus
    /// `absorptionDistanceMultiplier`, not by the scene's Sky and Fog Global Volume. Putting a
    /// Fog override on that volume would tint the ABOVE-water shots too and would edit a profile
    /// asset shared with the other scenes, so it is deliberately not done here.
    ///
    /// PRESETS ARE A SAVED SCENE STATE. Apply from the Inspector button in EDIT mode, then save
    /// the scene. A Play-mode apply is lost on Stop (2026-08-18 lesson: the station's measured
    /// position was a Play-mode Inspector edit and vanished).
    /// </summary>
    [AddComponentMenu("Smarc/Water/Baltic Water Preset")]
    [DisallowMultipleComponent]
    public class BalticWaterPreset : MonoBehaviour
    {
        [Serializable]
        public class WaterLook
        {
            public string Name = "preset";

            [Header("Transparency and colour")]
            [Tooltip("HDRP 'Absorption Distance' (m). The field that reads as visibility: light is fully absorbed after this many metres of water. 56.3 = the shipped pool look, 6 = murky Baltic that still photographs, 3.5 = the honest summer Secchi depth.")]
            public float AbsorptionDistance = 56.3f;
            [Tooltip("HDRP 'Scattering Color'. The colour of the water body itself. Also drives the UNDERWATER colour while Scattering Color Mode is 'Scattering Color' (the scene's setting).")]
            public Color ScatteringColor = new Color(0f, 0.21404114f, 0.31854683f, 1f);
            [Tooltip("HDRP 'Refraction Color'.")]
            public Color RefractionColor = new Color(0.033104762f, 0.26327342f, 0.26327342f, 1f);
            [Tooltip("HDRP 'Max Refraction Distance' (m).")]
            public float MaxRefractionDistance = 1.926f;

            [Header("Scattering terms")]
            public float AmbientScattering = 0.597f;
            public float HeightScattering = 0f;
            public float DisplacementScattering = 0f;
            public float DirectLightTipScattering = 0.2f;
            public float DirectLightBodyScattering = 0.2f;

            [Header("Caustics — turbid water has almost none")]
            public bool Caustics = true;
            public float CausticsIntensity = 0.5f;

            [Header("Underwater")]
            public bool UnderWater = true;
            [Tooltip("Multiplies AbsorptionDistance for the UNDERWATER view only. < 1 = murkier when you are in it than when you look into it.")]
            public float AbsorptionDistanceMultiplier = 1f;
            public float UnderWaterAmbientProbeContribution = 1f;
            [Tooltip("Only used when the surface's 'Scattering Color Mode' is Custom. The scene ships in 'Scattering Color' mode, where ScatteringColor above is what you see underwater; this is set anyway so flipping the mode by hand gives the same look.")]
            public Color UnderWaterScatteringColor = new Color(0f, 0.27f, 0.23f, 1f);
            [Tooltip("Leave OFF unless you want the water line to refract. Costs performance and reads as a wobble on the surfacing shot.")]
            public bool UnderWaterRefraction = false;

            [Header("Surface")]
            public float StartSmoothness = 0.88905823f;
            public float EndSmoothness = 0.7890582f;
            public float RipplesWindSpeed = 2.1f;
        }

        [Tooltip("The WaterSurface to drive. Left empty, the one on this GameObject is used.")]
        public WaterSurface Surface;

        [Tooltip("Name of the preset that is currently meant to be applied. The Inspector's Apply button writes this preset onto the surface and marks the scene dirty.")]
        public string ActivePreset = "clear_demo";

        [Tooltip("Apply ActivePreset at Play as well. OFF by default and it should normally stay off: a preset is a SAVED SCENE STATE, and applying at Play hides the fact that the saved scene still says something else.")]
        public bool ApplyOnStart = false;

        [Tooltip("Log every field written, with its old and new value. Useful the first time; noise afterwards.")]
        public bool VerboseApply = false;

        public List<WaterLook> Presets = new List<WaterLook>();

        // ---------------------------------------------------------------- defaults

        void Reset()
        {
            Surface = GetComponent<WaterSurface>();
            SeedDefaultPresets();
        }

        /// <summary>Adds the shipped presets that are not present yet. Never overwrites an edited one.</summary>
        public void SeedDefaultPresets()
        {
            if (Presets == null) Presets = new List<WaterLook>();

            AddIfMissing(new WaterLook
            {
                // Field-for-field the values serialized in BeckholmenWorld.prefab on 2026-08-21.
                Name = "clear_demo",
                AbsorptionDistance = 56.3f,
                ScatteringColor = new Color(0f, 0.21404114f, 0.31854683f, 1f),
                RefractionColor = new Color(0.033104762f, 0.26327342f, 0.26327342f, 1f),
                MaxRefractionDistance = 1.926f,
                AmbientScattering = 0.597f,
                HeightScattering = 0f,
                DisplacementScattering = 0f,
                DirectLightTipScattering = 0.2f,
                DirectLightBodyScattering = 0.2f,
                Caustics = true,
                CausticsIntensity = 0.5f,
                UnderWater = true,
                AbsorptionDistanceMultiplier = 1f,
                UnderWaterAmbientProbeContribution = 1f,
                UnderWaterScatteringColor = new Color(0f, 0.27f, 0.23f, 1f),
                UnderWaterRefraction = false,
                StartSmoothness = 0.88905823f,
                EndSmoothness = 0.7890582f,
                RipplesWindSpeed = 2.1f
            });

            AddIfMissing(new WaterLook
            {
                Name = "baltic",
                AbsorptionDistance = 6.0f,
                ScatteringColor = new Color(0.105f, 0.205f, 0.150f, 1f),   // green-brown
                RefractionColor = new Color(0.090f, 0.180f, 0.140f, 1f),
                MaxRefractionDistance = 1.1f,
                AmbientScattering = 0.78f,       // more of it: turbid water glows rather than clears
                HeightScattering = 0.20f,
                DisplacementScattering = 0.18f,
                DirectLightTipScattering = 0.35f,
                DirectLightBodyScattering = 0.35f,
                Caustics = true,
                CausticsIntensity = 0.12f,       // turbidity kills caustics; not zero, so the floor still shimmers
                UnderWater = true,
                AbsorptionDistanceMultiplier = 1f,
                UnderWaterAmbientProbeContribution = 0.85f,
                UnderWaterScatteringColor = new Color(0.085f, 0.175f, 0.130f, 1f),
                UnderWaterRefraction = false,
                StartSmoothness = 0.82f,
                EndSmoothness = 0.72f,
                RipplesWindSpeed = 2.6f
            });

            AddIfMissing(new WaterLook
            {
                Name = "baltic_murky",
                AbsorptionDistance = 3.5f,       // the honest summer Secchi number
                ScatteringColor = new Color(0.115f, 0.190f, 0.130f, 1f),
                RefractionColor = new Color(0.095f, 0.165f, 0.120f, 1f),
                MaxRefractionDistance = 0.8f,
                AmbientScattering = 0.88f,
                HeightScattering = 0.25f,
                DisplacementScattering = 0.22f,
                DirectLightTipScattering = 0.4f,
                DirectLightBodyScattering = 0.4f,
                Caustics = true,
                CausticsIntensity = 0.05f,
                UnderWater = true,
                AbsorptionDistanceMultiplier = 0.85f,
                UnderWaterAmbientProbeContribution = 0.7f,
                UnderWaterScatteringColor = new Color(0.095f, 0.160f, 0.115f, 1f),
                UnderWaterRefraction = false,
                StartSmoothness = 0.78f,
                EndSmoothness = 0.68f,
                RipplesWindSpeed = 3.0f
            });
        }

        void AddIfMissing(WaterLook look)
        {
            if (Find(look.Name) == null) Presets.Add(look);
        }

        public WaterLook Find(string name)
        {
            if (Presets == null) return null;
            foreach (var p in Presets)
                if (p != null && p.Name == name) return p;
            return null;
        }

        public string[] PresetNames()
        {
            if (Presets == null) return new string[0];
            var names = new string[Presets.Count];
            for (int i = 0; i < Presets.Count; i++) names[i] = Presets[i] == null ? "<null>" : Presets[i].Name;
            return names;
        }

        void Start()
        {
            if (ApplyOnStart) ApplyPreset(ActivePreset);
        }

        // ---------------------------------------------------------------- apply

        /// <summary>
        /// Write a preset onto the WaterSurface. Returns false and NAMES the reason rather than
        /// half-applying. Nothing here writes the transform, the enabled flag, or scriptInteractions.
        /// </summary>
        public bool ApplyPreset(string name)
        {
            var surface = ResolveSurface();
            if (surface == null)
            {
                Debug.LogError($"[BalticWaterPreset] no WaterSurface on '{this.name}' and none assigned — nothing applied.");
                return false;
            }

            var look = Find(name);
            if (look == null)
            {
                Debug.LogError($"[BalticWaterPreset] no preset named '{name}'. Known: {string.Join(", ", PresetNames())}");
                return false;
            }

            WarnIfWaterMoved(surface);

            if (VerboseApply)
                Debug.Log($"[BalticWaterPreset] applying '{look.Name}': absorption {surface.absorptionDistance} -> {look.AbsorptionDistance} m, " +
                          $"scattering {surface.scatteringColor} -> {look.ScatteringColor}, caustics {surface.causticsIntensity} -> {look.CausticsIntensity}");

            surface.absorptionDistance = look.AbsorptionDistance;
            surface.scatteringColor = look.ScatteringColor;
            surface.refractionColor = look.RefractionColor;
            surface.maxRefractionDistance = look.MaxRefractionDistance;

            surface.ambientScattering = look.AmbientScattering;
            surface.heightScattering = look.HeightScattering;
            surface.displacementScattering = look.DisplacementScattering;
            surface.directLightTipScattering = look.DirectLightTipScattering;
            surface.directLightBodyScattering = look.DirectLightBodyScattering;

            surface.caustics = look.Caustics;
            surface.causticsIntensity = look.CausticsIntensity;

            surface.underWater = look.UnderWater;
            surface.absorptionDistanceMultiplier = look.AbsorptionDistanceMultiplier;
            surface.underWaterAmbientProbeContribution = look.UnderWaterAmbientProbeContribution;
            surface.underWaterScatteringColor = look.UnderWaterScatteringColor;
            surface.underWaterRefraction = look.UnderWaterRefraction;

            surface.startSmoothness = look.StartSmoothness;
            surface.endSmoothness = look.EndSmoothness;
            surface.ripplesWindSpeed = look.RipplesWindSpeed;

            ActivePreset = look.Name;

            if (look.UnderWater && surface.volumeBounds == null)
                Debug.LogWarning("[BalticWaterPreset] underWater is on but the surface has no Volume Bounds BoxCollider — " +
                                 "HDRP has no region to render the underwater view in, so in-water shots will look like above-water ones.");

            Debug.Log($"[BalticWaterPreset] '{look.Name}' applied to '{surface.name}' " +
                      $"(absorption {look.AbsorptionDistance} m, underwater x{look.AbsorptionDistanceMultiplier}). " +
                      "Transform, enabled flag and scriptInteractions untouched.");
            return true;
        }

        /// <summary>
        /// Read the surface's CURRENT values back into a preset, so today's look can be kept
        /// before it is overwritten. Creates the preset if it does not exist.
        /// </summary>
        public void CaptureIntoPreset(string name)
        {
            var surface = ResolveSurface();
            if (surface == null)
            {
                Debug.LogError("[BalticWaterPreset] nothing to capture from — no WaterSurface.");
                return;
            }
            var look = Find(name);
            if (look == null)
            {
                look = new WaterLook { Name = name };
                Presets.Add(look);
            }

            look.AbsorptionDistance = surface.absorptionDistance;
            look.ScatteringColor = surface.scatteringColor;
            look.RefractionColor = surface.refractionColor;
            look.MaxRefractionDistance = surface.maxRefractionDistance;
            look.AmbientScattering = surface.ambientScattering;
            look.HeightScattering = surface.heightScattering;
            look.DisplacementScattering = surface.displacementScattering;
            look.DirectLightTipScattering = surface.directLightTipScattering;
            look.DirectLightBodyScattering = surface.directLightBodyScattering;
            look.Caustics = surface.caustics;
            look.CausticsIntensity = surface.causticsIntensity;
            look.UnderWater = surface.underWater;
            look.AbsorptionDistanceMultiplier = surface.absorptionDistanceMultiplier;
            look.UnderWaterAmbientProbeContribution = surface.underWaterAmbientProbeContribution;
            look.UnderWaterScatteringColor = surface.underWaterScatteringColor;
            look.UnderWaterRefraction = surface.underWaterRefraction;
            look.StartSmoothness = surface.startSmoothness;
            look.EndSmoothness = surface.endSmoothness;
            look.RipplesWindSpeed = surface.ripplesWindSpeed;

            Debug.Log($"[BalticWaterPreset] captured the surface's current look into preset '{name}'.");
        }

        public WaterSurface ResolveSurface()
        {
            if (Surface != null) return Surface;
            Surface = GetComponent<WaterSurface>();
            return Surface;
        }

        /// <summary>
        /// The one check that matters. SETTLED §3s: a non-zero water Y turns buoyancy into a
        /// torque and the vehicle leaves the scene. This component cannot cause that — it never
        /// writes the transform — but it is the component people will be looking at when it
        /// happens, so it says so.
        /// </summary>
        void WarnIfWaterMoved(WaterSurface surface)
        {
            float y = surface.transform.position.y;

            // THE ONE SANCTIONED EXCEPTION (2026-08-21). `DockDrainDirector` lowers the water on
            // purpose to show the dry dock at the end of a take, and it is allowed to because it
            // disables every ForcePoint in the scene and freezes their bodies BEFORE the water
            // moves, restores both on exit, and refuses to run outside Play mode — so the §3s
            // mechanism (a ForcePoint querying a water plane off world zero) has no path to run.
            // This is a NAMED exception, not a suppression: if the flag is set, the drain said so
            // in the Console, and if it is not, the error below stands unchanged.
            if (DockDrainDirector.DrainInProgress)
            {
                Debug.LogWarning($"[BalticWaterPreset] the Water transform is at Y = {y:F3} because a " +
                                 "DockDrainDirector drain is RUNNING — the sanctioned SETTLED §3s exception. " +
                                 "Every ForcePoint is disabled and every body frozen for the duration. " +
                                 "Applying a look now is fine; do not SAVE the scene until the drain restores.");
                return;
            }

            if (Mathf.Abs(y) > 1e-4f)
                Debug.LogError($"[BalticWaterPreset] the Water transform is at Y = {y:F3}, not 0. SETTLED §3s: " +
                               "HDRPWaterQueryModel shares one search seed across all callers and discards the " +
                               "convergence bool, so a non-zero water plane gives ForcePoints divergent water " +
                               "levels, differential buoyancy, and a vehicle at 67 m/s. Move the TERRAIN, not the " +
                               "water. (This preset changes appearance only and did not move anything.)");

            if (surface.scriptInteractions)
                Debug.LogWarning("[BalticWaterPreset] the surface has CPU 'Script Interactions' ON. That is the mode " +
                                 "HDRPWaterQueryModel actually searches in, and it has never been the shipped " +
                                 "Beckholmen setting (serialized: off). If the vehicle starts behaving differently " +
                                 "after a look change, this — not the colours — is the difference.");
        }

        // ---------------------------------------------------------------- misc

        /// <summary>
        /// Set an enum-typed field by its integer value without naming the enum type at compile
        /// time. Used for `underWaterScatteringColorMode`, whose nested enum name is the one
        /// piece of this file that could differ between HDRP minor versions; a missing field is
        /// reported, never silently skipped.
        /// </summary>
        public bool TrySetEnumField(string fieldName, int value)
        {
            var surface = ResolveSurface();
            if (surface == null) return false;
            var f = typeof(WaterSurface).GetField(fieldName, BindingFlags.Public | BindingFlags.Instance);
            if (f == null || !f.FieldType.IsEnum)
            {
                Debug.LogWarning($"[BalticWaterPreset] WaterSurface has no public enum field '{fieldName}' in this HDRP version — not set.");
                return false;
            }
            f.SetValue(surface, Enum.ToObject(f.FieldType, value));
            return true;
        }
    }
}
