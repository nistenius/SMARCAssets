using UnityEngine;
using UnityEditor;

using VehicleComponents.Comms;

// NOTE: `UnityEditor.Editor` in full, deliberately, and the same everywhere in this folder.
// This assembly already declares a top-level namespace literally called `Editor`
// (`namespace Editor.Scripts` in GlobalReferencePointEditor.cs, MaterialShaderChanger.cs, …), so a
// bare `Editor` base class binds to THAT NAMESPACE and the compiler stops with CS0118, "'Editor'
// is a namespace but is used like a type". It cost a compile cycle on 2026-08-21; every custom
// editor in this folder writes it out in full, and BalticWaterPresetEditor.cs carries the same
// note. Do not shorten it.
namespace SMARC.Editor
{
    /// <summary>
    /// The Scene-view readout for <see cref="DeployedTransducer"/> — the cheap discriminator
    /// between "configured 0.5 m" and "actually 0.5 m".
    ///
    /// The gizmo LINES live in the component (`OnDrawGizmos`) so they draw with no selection at
    /// all; the numeric LABEL lives here because `Handles` only exists in the editor assembly.
    /// `[DrawGizmo]` with both selection states is what makes the label appear whether or not the
    /// station is selected — Ivan clicks the station ROOT, which never selects `acoustic_link`, so
    /// an `OnSceneGUI`-only label would be invisible exactly when it is wanted.
    /// </summary>
    [CustomEditor(typeof(DeployedTransducer))]
    public class DeployedTransducerEditor : UnityEditor.Editor
    {
        // `UnityEditor.GizmoType` in full, deliberately — the same shape as the `Editor` note
        // above, and found the same way. This project declares its OWN global-namespace
        // `public enum GizmoType` (Runtime/Scripts/SimpleGizmo.cs: Position / CenterOfMass /
        // CenterOfProps), and a type in the global namespace beats one pulled in by `using
        // UnityEditor;`. So a bare `GizmoType` binds to SimpleGizmo's enum and the compiler says
        // CS0117, "does not contain a definition for 'InSelectionHierarchy'" — which reads as a
        // Unity API that has moved, and is not. Do not shorten it.
        [DrawGizmo(UnityEditor.GizmoType.InSelectionHierarchy | UnityEditor.GizmoType.NotInSelectionHierarchy)]
        static void DrawLabel(DeployedTransducer t, UnityEditor.GizmoType gizmoType)
        {
            if (t == null) return;

            Vector3 at = t.transform.position + Vector3.up * 0.15f;

            if (!t.TryGetWaterPlaneY(out float planeY))
            {
                // "in THIS scene", because that is now the rule and the difference is actionable:
                // an Ocean in another loaded scene is deliberately not used (SETTLED §3s8).
                Label(at, "no WaterSurface in this scene — depth unknown", new Color(1f, 0.4f, 0.3f));
                return;
            }

            float actual = planeY - t.transform.position.y;
            bool wet = actual > 0f;

            // The ACTUAL depth first, because that is the number in question. The configured one
            // follows in brackets so a disagreement is readable at a glance instead of requiring
            // the Inspector.
            string label = wet
                ? $"transducer {actual:F2} m below water line  (set {t.DipDepthM:F2})"
                : $"DRY — {-actual:F2} m ABOVE water line  (set {t.DipDepthM:F2})";

            Label(at, label, wet ? new Color(0.4f, 0.95f, 1f) : new Color(1f, 0.4f, 0.3f));

            // The water line, marked once more with a flat disc so it reads as a plane rather than
            // as two crossed sticks when the camera is low.
            Handles.color = new Color(0.6f, 0.85f, 1f, 0.35f);
            Handles.DrawWireDisc(new Vector3(t.transform.position.x, planeY, t.transform.position.z),
                                 Vector3.up, 0.8f);
        }

        static GUIStyle labelStyle;

        /// <summary>
        /// Coloured scene label that degrades to an uncoloured one instead of throwing.
        ///
        /// `[DrawGizmo]` callbacks run during gizmo rendering, NOT inside an OnGUI pass, and
        /// `EditorStyles.*` is null outside a GUI context — touching it there is an exception per
        /// frame per station, which in this project's idiom is a console nobody reads any more. So
        /// the style is built once, lazily, behind a null check, and if it cannot be built the
        /// label still prints. The verdict is in the TEXT ("DRY — …") as well as the colour,
        /// precisely so losing the colour loses nothing that matters.
        /// </summary>
        static void Label(Vector3 at, string text, Color color)
        {
            // try/catch and not just a null check: depending on the Unity version, EditorStyles
            // outside a GUI context either RETURNS null or THROWS. Both end at the same fallback.
            if (labelStyle == null)
            {
                try
                {
                    var s = EditorStyles.boldLabel;
                    if (s != null) labelStyle = new GUIStyle(s);
                }
                catch (System.Exception) { labelStyle = null; }
            }

            if (labelStyle == null) { Handles.Label(at, text); return; }

            labelStyle.normal.textColor = color;
            Handles.Label(at, text, labelStyle);
        }

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var t = (DeployedTransducer)target;
            EditorGUILayout.Space();

            // THE READOUTS ARE DRAWN HERE BECAUSE THEY ARE NOT SERIALISED (2026-08-21). They used
            // to be ordinary public fields and the default drawer showed them — which also meant
            // Unity could WRITE them into the station prefab asset, where `Deployed: 1` and
            // `ActualDepthM: 0.5` duly appeared, in a file that contains no water. A live readout
            // that can be baked is a readout that can outlive what it describes (§3s, the stale
            // AppliedBuoyancyForce). Non-serialised fields are invisible to DrawDefaultInspector,
            // so they are printed explicitly, and disabled so nobody mistakes them for settings.
            using (new EditorGUI.DisabledScope(true))
            {
                EditorGUILayout.LabelField("State (live, never saved)", EditorStyles.boldLabel);
                EditorGUILayout.Toggle("Deployed", t.Deployed);
                EditorGUILayout.TextField("Actual Depth M",
                    float.IsNaN(t.ActualDepthM) ? "unknown (no water plane)" : t.ActualDepthM.ToString("F3"));
                EditorGUILayout.TextField("Refusal Reason", t.RefusalReason ?? "");
            }

            if (!t.CanDeployHere(out string whereWhy, out _))
            {
                EditorGUILayout.HelpBox("Not deploying here: " + whereWhy, MessageType.Info);
                EditorGUILayout.Space();
                return;
            }

            if (t.TryGetWaterPlaneY(out float planeY, out string waterWhy))
            {
                EditorGUILayout.LabelField("Water plane Y", planeY.ToString("F3") + " m  (WaterSurface transform, never GetWaterLevelAt)");
                float actual = planeY - t.transform.position.y;
                EditorGUILayout.LabelField("Actual depth", actual.ToString("F3") + " m");
                if (actual <= 0f)
                    EditorGUILayout.HelpBox("transducer above the water line — not deployed. " +
                                            "The acoustic link will report exactly that, by name.",
                                            MessageType.Error);
                else if (Mathf.Abs(actual - t.DipDepthM) > 0.01f)
                    EditorGUILayout.HelpBox($"Deployed at {actual:F2} m but configured for {t.DipDepthM:F2} m. " +
                                            "Press Deploy now, or turn Auto Deploy In Editor on.",
                                            MessageType.Warning);
            }
            else
            {
                // The reason comes from the component, in its own words, rather than being
                // re-worded here: "no Ocean in this scene" and "the Ocean you assigned is in
                // ANOTHER scene" need different actions, and a single sentence covering both is a
                // refusal reason that refuses to be acted on.
                EditorGUILayout.HelpBox(waterWhy + "\n\nIt will NOT assume Y = 0 — an assumed water " +
                                        "level is an invented measurement.", MessageType.Error);
            }

            EditorGUILayout.Space();
            if (GUILayout.Button("Deploy now (snap to water plane)"))
            {
                Undo.RecordObject(t.transform, "Deploy transducer");
                t.Deploy();
                EditorUtility.SetDirty(t.transform);
            }
        }
    }
}
