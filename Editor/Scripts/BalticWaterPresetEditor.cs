using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

using SmarcGUI.Water;

/// <summary>
/// Inspector buttons for BalticWaterPreset.
///
/// The buttons exist so that applying a look is an EDIT-MODE action that dirties the scene. The
/// component can apply a preset at runtime too, and that is exactly the thing not to rely on:
/// a Play-mode Inspector edit is discarded on Stop, and the record already contains a station
/// position that read "MEASURED ±0.014 m" and was a Play-mode edit nobody saved (SETTLED §3s2).
/// While Play is running the buttons say so instead of pretending.
/// </summary>
[CustomEditor(typeof(BalticWaterPreset))]
// NOTE: `UnityEditor.Editor` in full, deliberately. This assembly already declares a global
// namespace called `Editor` (Editor/Scripts/MaterialShaderChanger.cs etc.), so a bare `Editor`
// here binds to THAT namespace and the compiler says CS0118 "'Editor' is a namespace but is used
// like a type". Do not shorten it.
public class BalticWaterPresetEditor : UnityEditor.Editor
{
    string captureName = "clear_demo";

    public override void OnInspectorGUI()
    {
        var preset = (BalticWaterPreset)target;

        EditorGUILayout.HelpBox(
            "A preset is a SAVED SCENE STATE. Apply in Edit mode, then save the scene.\n" +
            "This component never moves the Water transform, never disables the Water object and " +
            "never touches Script Interactions (SETTLED §3s / §3o).",
            MessageType.Info);

        if (Application.isPlaying)
            EditorGUILayout.HelpBox("Play is running: anything applied now is lost on Stop.", MessageType.Warning);

        var surface = preset.ResolveSurface();
        if (surface != null)
        {
            float y = surface.transform.position.y;
            if (Mathf.Abs(y) > 1e-4f)
                EditorGUILayout.HelpBox($"The Water transform is at Y = {y:F3}, not 0. SETTLED §3s: this makes " +
                                        "ForcePoints disagree about the water level and the vehicle leaves the " +
                                        "scene at 67 m/s. Move the terrain, not the water.", MessageType.Error);
            EditorGUILayout.LabelField("Live on the surface",
                $"absorption {surface.absorptionDistance:F1} m · underwater x{surface.absorptionDistanceMultiplier:F2} · " +
                $"caustics {surface.causticsIntensity:F2}");
        }

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Apply", EditorStyles.boldLabel);
        var names = preset.PresetNames();
        using (new EditorGUILayout.HorizontalScope())
        {
            foreach (var n in names)
            {
                if (!GUILayout.Button(n)) continue;
                if (surface == null)
                {
                    Debug.LogError("[BalticWaterPreset] no WaterSurface resolved — nothing applied.");
                    continue;
                }
                Undo.RecordObject(surface, "Apply water preset");
                Undo.RecordObject(preset, "Apply water preset");
                if (preset.ApplyPreset(n))
                {
                    EditorUtility.SetDirty(surface);
                    EditorUtility.SetDirty(preset);
                    PrefabUtility.RecordPrefabInstancePropertyModifications(surface);
                    PrefabUtility.RecordPrefabInstancePropertyModifications(preset);
                    if (!Application.isPlaying) EditorSceneManager.MarkSceneDirty(preset.gameObject.scene);
                }
            }
        }

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Capture the surface's current look into a preset", EditorStyles.boldLabel);
        using (new EditorGUILayout.HorizontalScope())
        {
            captureName = EditorGUILayout.TextField(captureName);
            if (GUILayout.Button("Capture", GUILayout.Width(90)))
            {
                Undo.RecordObject(preset, "Capture water preset");
                preset.CaptureIntoPreset(captureName);
                EditorUtility.SetDirty(preset);
                if (!Application.isPlaying) EditorSceneManager.MarkSceneDirty(preset.gameObject.scene);
            }
        }

        if (GUILayout.Button("Re-seed the shipped presets (never overwrites an edited one)"))
        {
            Undo.RecordObject(preset, "Seed water presets");
            preset.SeedDefaultPresets();
            EditorUtility.SetDirty(preset);
        }

        EditorGUILayout.Space();
        DrawDefaultInspector();
    }
}
