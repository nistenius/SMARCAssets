// SMARC -> Check ROS Namespace Consistency  (added 2026-09-15, with the sam_auv_v1 -> sam21 rename)
//
// WHY THIS EXISTS
// ---------------
// ROSBehaviour derives the ROS namespace from the SCENE, not from any setting:
//
//     GetRobotGO()  -> the component's own GameObject if it is tagged "robot",
//                      otherwise the nearest PARENT tagged "robot"
//     OnEnable()    -> string robot_name = robotGO.name;
//                      topic = $"/{robot_name}/{topic}";      (for non-global topics)
//     ROSPublisher  -> robot_name = robotGO.name;             (used for tf frames, "{robot_name}/{link}")
//
// So THE GAMEOBJECT NAME IS THE NAMESPACE. Rename the robot root in a scene and every topic
// and every tf frame moves with it, silently, with no compile error and no inspector warning.
// Meanwhile a dozen components carry their OWN copy of that name in a serialized string field
// (VehicleDashboard.RobotName, CinematicDirector.VehicleName, SonarMapAccumulator.RobotName,
// MissionWPHoop_Sub.RobotName, DockDrainDirector.VehicleName, TunnelViewer.RobotName,
// ProximitySkirt.RobotName, ...). Those fields are how those components FIND the robot, or how
// they build an absolute topic. When they disagree with the root object's name the scene does
// not fail: it half-works. The GUI binds to nothing, the hoop subscribes to a namespace nobody
// publishes on, the cinematic director aims at a vehicle that is not there.
//
// A C# default only applies to a NEWLY added component. Every component already serialized in a
// scene or prefab keeps the value it was saved with -- which is exactly why the 2026-09-15
// rename had to touch nine stored values in five assets on top of nine source literals.
//
// THIS TOOL READS. IT NEVER FIXES.
// An auto-fixer would be a second writer to the scene (SETTLED §3k: Unity is already one, and
// a careless rewrite is how a scene full of NREs happened). Reporting is the whole job; the
// human decides which side of a disagreement is wrong.
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.SceneManagement;
using ROS.Core;   // ROSBehaviour

public static class NamespaceConsistencyCheck
{
    const string RobotTag = "robot";

    /// <summary>Serialized string fields that are meant to hold a ROS namespace / robot root name.</summary>
    static readonly string[] NameFields = { "RobotName", "VehicleName" };

    [MenuItem("SMARC/Check ROS Namespace Consistency")]
    public static void Check()
    {
        var scene = SceneManager.GetActiveScene();
        if (!scene.IsValid() || !scene.isLoaded)
        {
            Debug.LogWarning("[NamespaceCheck] No loaded scene to check.");
            return;
        }

        GameObject[] roots = scene.GetRootGameObjects();
        bool robotTagExists = InternalEditorUtility.tags.Contains(RobotTag);
        if (!robotTagExists)
        {
            Debug.LogError(
                $"[NamespaceCheck] The tag \"{RobotTag}\" is not defined in this project. " +
                "ROSBehaviour.GetRobotGO looks for it to find the vehicle root, so NOTHING in this " +
                "scene can namespace a topic. Fix the tag before reading anything below.");
        }

        // ---------------------------------------------------------------- 1. the namespaces
        // A "robot" here is what ROSBehaviour would resolve to: the tagged object a ROS component
        // sits on or under. That -- not the prefab, not the asset file -- is the namespace.
        var robotNames = new List<string>();
        var robotOwners = new Dictionary<string, GameObject>();
        int rosComponentCount = 0;
        int orphanCount = 0;

        foreach (GameObject root in roots)
        {
            foreach (ROSBehaviour rb in root.GetComponentsInChildren<ROSBehaviour>(true))
            {
                rosComponentCount++;
                if (rb.NotARobot) continue;

                GameObject robotGO = ResolveRobotGO(rb.gameObject, robotTagExists);
                if (robotGO == null)
                {
                    orphanCount++;
                    Debug.LogError(
                        $"[NamespaceCheck] {rb.GetType().Name} on '{PathOf(rb.gameObject)}' (topic " +
                        $"'{rb.topic}') has NO self/parent tagged \"{RobotTag}\". ROSBehaviour.OnEnable " +
                        "disables a component in this state -- it cannot namespace its topic.",
                        rb);
                    continue;
                }

                string n = robotGO.name;
                if (!robotOwners.ContainsKey(n))
                {
                    robotOwners[n] = robotGO;
                    robotNames.Add(n);
                }
                else if (robotOwners[n] != robotGO)
                {
                    Debug.LogError(
                        $"[NamespaceCheck] TWO DIFFERENT objects are both named '{n}' and both act as a " +
                        $"robot root: '{PathOf(robotOwners[n])}' and '{PathOf(robotGO)}'. They share one " +
                        "ROS namespace and one tf frame tree; which one a component binds to is an " +
                        "accident of scene order.",
                        robotGO);
                }
            }
        }

        var sb = new StringBuilder();
        sb.AppendLine($"[NamespaceCheck] scene '{scene.name}': {roots.Length} root objects, " +
                      $"{rosComponentCount} ROSBehaviour components, {robotNames.Count} robot namespace(s).");
        foreach (string n in robotNames)
        {
            sb.AppendLine($"    namespace '/{n}'   <- GameObject '{PathOf(robotOwners[n])}'   " +
                          $"(tf frames '{n}/<link>')");
            if (n.Contains("."))
            {
                Debug.LogWarning(
                    $"[NamespaceCheck] Robot root '{PathOf(robotOwners[n])}' has a '.' in its name. If any " +
                    "ROS component under it ever publishes, '/" + n + "/...' is NOT a legal ROS 2 name " +
                    "([A-Za-z_][A-Za-z0-9_]*). Offline / non-ROS rigs may keep the dotted name on purpose.",
                    robotOwners[n]);
            }
        }
        Debug.Log(sb.ToString().TrimEnd());

        // ---------------------------------------------------------------- 2. the stored copies
        int fieldsFound = 0, mismatches = 0, empties = 0;
        foreach (GameObject root in roots)
        {
            foreach (MonoBehaviour mb in root.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (mb == null) continue;   // missing script
                foreach (FieldInfo fi in SerializedStringFields(mb.GetType()))
                {
                    if (Array.IndexOf(NameFields, fi.Name) < 0) continue;
                    fieldsFound++;

                    string value = fi.GetValue(mb) as string;
                    string where = $"{mb.GetType().Name}.{fi.Name} on '{PathOf(mb.gameObject)}'";

                    if (string.IsNullOrEmpty(value))
                    {
                        empties++;
                        Debug.LogWarning(
                            $"[NamespaceCheck] {where} is EMPTY. It will not resolve to any robot; " +
                            "whatever it drives is off rather than wrong.", mb);
                        continue;
                    }

                    if (robotNames.Contains(value))
                    {
                        Debug.Log($"[NamespaceCheck] OK  {where} = '{value}'", mb);
                        continue;
                    }

                    mismatches++;
                    string known = robotNames.Count == 0
                        ? "(this scene has NO robot root at all)"
                        : "'" + string.Join("', '", robotNames) + "'";
                    Debug.LogError(
                        $"[NamespaceCheck] MISMATCH  {where} = '{value}', but the robot root object(s) in " +
                        $"this scene are named {known}. The GameObject name IS the ROS namespace " +
                        "(ROSBehaviour: topic = \"/{robot_name}/{topic}\", robot_name = robotGO.name), so " +
                        "this field points at a namespace nothing publishes on. Either rename the object " +
                        "or fix the field -- this tool will not guess which.",
                        mb);
                }
            }
        }

        Debug.Log($"[NamespaceCheck] done: {fieldsFound} name field(s) inspected, {mismatches} mismatch(es), " +
                  $"{empties} empty, {orphanCount} ROS component(s) with no robot root. Nothing was modified.");
    }

    /// <summary>Public instance string fields plus [SerializeField] private ones -- i.e. exactly what
    /// Unity stores in the scene, which is what can disagree with the object name.</summary>
    static IEnumerable<FieldInfo> SerializedStringFields(Type t)
    {
        const BindingFlags Flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
        foreach (FieldInfo fi in t.GetFields(Flags))
        {
            if (fi.FieldType != typeof(string)) continue;
            if (fi.IsPublic || fi.GetCustomAttribute<SerializeField>() != null) yield return fi;
        }
    }

    /// <summary>The same walk ROSBehaviour.GetRobotGO does: self if tagged, else nearest tagged parent.</summary>
    static GameObject ResolveRobotGO(GameObject go, bool tagExists)
    {
        if (!tagExists) return null;
        for (Transform t = go.transform; t != null; t = t.parent)
        {
            if (t.gameObject.CompareTag(RobotTag)) return t.gameObject;
        }
        return null;
    }

    static string PathOf(GameObject go)
    {
        if (go == null) return "(null)";
        var parts = new List<string>();
        for (Transform t = go.transform; t != null; t = t.parent) parts.Add(t.name);
        parts.Reverse();
        return string.Join("/", parts);
    }
}
