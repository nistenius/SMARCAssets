// WaveTestSceneBuilder.cs — builds the wave-response test scene from AskoCurated, by script.
//
// WHY BY SCRIPT. The measurement has to be repeatable and its setup has to be reviewable. A
// scene assembled by hand in the Inspector is neither: nobody can diff it, and the next person
// cannot tell which of forty objects mattered. This is one menu item that always produces the
// same scene from the same source, and says in the console exactly what it turned off.
//
// WHAT IT BUILDS. AskoWaveTest.unity — the Askö world (its terrain, its Ocean pinned at
// (0,0,0), its Baltic water preset) with:
//   * the OLD sam2.2 instance (scene-renamed `sam_auv_v1`, 1064 property overrides, the legacy
//     two-column 10-point buoyancy cloud) DISABLED, not deleted, so the scene still diffs
//     cleanly against AskoCurated and Ivan can re-enable it for an A/B;
//   * a fresh `sam2.2.strips` instance — 27 on-axis strips + the ballast links — at the same
//     station, on the surface;
//   * every ROS publisher/subscriber on that new vehicle disabled. A physics measurement does
//     not need the wire, and the publishers were flooding the console with "cannot sustain
//     10 Hz" at 10 lines a second, which is both noise and load;
//   * the site furniture (GUI, station, Milou, MMTMini, hoop, video rig, sonar accumulator)
//     disabled — none of it bears on buoyancy and all of it costs frame time;
//   * a `WaveSweepRig` object driving the matrix.
//
// AskoCurated itself is never modified. Menu: SMARC ▸ Wave Test ▸ Build Asko Wave Test Scene.

using System.Collections.Generic;
using System.IO;
using System.Linq;
using Diagnostics;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace SmarcEditor
{
    public static class WaveTestSceneBuilder
    {
        const string SourceScene = "Assets/Scenes/AskoCurated.unity";
        const string TargetScene = "Assets/Scenes/AskoWaveTest.unity";
        const string StripsPrefab = "Packages/com.smarc.assets/Runtime/Prefabs/sam2.2.strips.prefab";

        static readonly string[] DisableRoots =
        {
            "GUI", "datacube_station_01", "Milou", "MMTMiniCooper",
            "SonarMapAccumulator", "MissionWPHoop", "AskoVideoRig",
        };

        [MenuItem("SMARC/Wave Test/Build Asko Wave Test Scene")]
        public static void Build()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;

            var scene = EditorSceneManager.OpenScene(SourceScene, OpenSceneMode.Single);
            if (!scene.IsValid()) { Debug.LogError($"[WaveTestSceneBuilder] could not open {SourceScene}"); return; }

            var roots = scene.GetRootGameObjects().ToList();
            var log = new List<string>();

            // 1. the old vehicle — disabled, not deleted
            var oldSam = roots.FirstOrDefault(r => r.GetComponentInChildren<Force.ForcePoint>(true) != null);
            if (oldSam != null) { oldSam.SetActive(false); log.Add($"disabled old vehicle '{oldSam.name}'"); }
            else log.Add("WARNING: no existing vehicle with ForcePoints found");

            // 2. site furniture
            foreach (var n in DisableRoots)
            {
                var go = roots.FirstOrDefault(r => r.name == n);
                if (go != null && go.activeSelf) { go.SetActive(false); log.Add($"disabled '{n}'"); }
            }

            // 3. the new vehicle
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(StripsPrefab)
                         ?? AssetDatabase.LoadAssetAtPath<GameObject>(FindStripsPrefab());
            if (prefab == null) { Debug.LogError("[WaveTestSceneBuilder] sam2.2.strips.prefab not found."); return; }

            var sam = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
            Vector3 home = oldSam != null ? oldSam.transform.position : new Vector3(-151.14f, 0f, -210.67f);
            sam.transform.position = new Vector3(home.x, 0f, home.z);
            sam.transform.rotation = Quaternion.identity;
            sam.name = "sam2.2.strips";
            log.Add($"instantiated sam2.2.strips at ({home.x:F2}, 0.00, {home.z:F2})");

            int killed = 0;
            foreach (var mb in sam.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb == null) continue;
                string t = mb.GetType().Name;
                bool wire = t.EndsWith("_Pub") || t.EndsWith("_Sub") || t.StartsWith("ROS")
                            || t.Contains("Publisher") || t.Contains("Subscriber") || t == "TFPub" || t == "TF_Pub";
                if (wire && mb.enabled) { mb.enabled = false; killed++; }
            }
            log.Add($"disabled {killed} ROS publisher/subscriber component(s) on the new vehicle");

            // 4. the rig
            var rigGo = new GameObject("WaveSweepRig");
            var rig = rigGo.AddComponent<WaveSweepRig>();
            rig.Vehicle = sam;
            rig.Surface = Object.FindObjectsByType<UnityEngine.Rendering.HighDefinition.WaterSurface>(FindObjectsSortMode.None).FirstOrDefault();
            SceneManager.MoveGameObjectToScene(rigGo, scene);
            log.Add($"added WaveSweepRig (surface '{(rig.Surface != null ? rig.Surface.name : "NOT FOUND")}')");

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, TargetScene, true);
            EditorSceneManager.OpenScene(TargetScene, OpenSceneMode.Single);

            Debug.Log("[WaveTestSceneBuilder] built " + TargetScene + "\n  - " + string.Join("\n  - ", log));
        }

        static string FindStripsPrefab()
        {
            foreach (var guid in AssetDatabase.FindAssets("sam2.2.strips t:Prefab"))
            {
                var p = AssetDatabase.GUIDToAssetPath(guid);
                if (Path.GetFileName(p) == "sam2.2.strips.prefab") return p;
            }
            return "";
        }
    }
}
