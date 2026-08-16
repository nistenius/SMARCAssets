using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

using ROS.Subscribers;   // MissionWPHoop_Sub

/// <summary>
/// Drops the MissionWPHoop prefab into a scene — the "hula hoop" that shows the
/// waypoint the vehicle is CURRENTLY flying to, used at Beckholmen since 2026-08-09
/// and wanted at every site (Ivan, 2026-08-17).
///
/// The hoop is SITE-AGNOSTIC by construction: MissionWPHoop_Sub subscribes to
/// /&lt;RobotName&gt;/mission/last_wp and converts the waypoint's lat/lon through
/// whatever GlobalReferencePoint the scene has. It carries no site constants, so
/// nothing here needs to know about UTM zones, origins or terrain — which is why
/// this is a two-line addition and not a port.
///
/// Two entry points on purpose:
///   * the menu item, for a scene that already exists (adding a hoop must not
///     require rebuilding a 2049^2 heightmap);
///   * Ensure(), called from the site builders so a freshly built scene has one.
///
/// Both are IDEMPOTENT — a second run finds the existing hoop and leaves it alone.
/// Two MissionWPHoop_Sub in one scene would give two subscribers on one topic and
/// two hoops on one waypoint, which is the duplicate-publisher confusion of
/// 2026-08-15 in visual form.
/// </summary>
public static class MissionWPHoopSetup
{
    public const string PrefabPath =
        "Packages/com.smarc.assets/Runtime/Prefabs/Components/MissionWPHoop.prefab";

    [MenuItem("SMARC/Add Mission WP Hoop to Open Scene")]
    public static void AddToOpenScene()
    {
        var go = Ensure(null);
        if (go != null)
        {
            Selection.activeGameObject = go;
            EditorSceneManager.MarkSceneDirty(go.scene);
        }
    }

    /// <summary>
    /// Ensure the open scene has exactly one mission WP hoop.
    /// robotName: whose mission to mirror. Null/empty keeps the prefab's own value
    /// (sam_auv_v1) rather than overwriting it with a guess — a wrong RobotName is
    /// silent, because it produces a subscription to a topic nobody publishes and a
    /// hoop that simply never appears.
    /// </summary>
    public static GameObject Ensure(string robotName)
    {
        var existing = Object.FindFirstObjectByType<MissionWPHoop_Sub>(FindObjectsInactive.Include);
        if (existing != null)
        {
            Debug.Log($"[WPHoop] already present ({existing.name}, RobotName={existing.RobotName}) — left alone");
            return existing.gameObject;
        }

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null)
        {
            Debug.LogWarning("[WPHoop] prefab not found: " + PrefabPath);
            return null;
        }

        var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        go.name = "MissionWPHoop";
        // The hoop positions itself in world coordinates from each waypoint's lat/lon,
        // so the parent's transform is presentation only. Origin keeps it out of the way.
        go.transform.position = Vector3.zero;

        var sub = go.GetComponent<MissionWPHoop_Sub>();
        if (sub == null)
        {
            Debug.LogWarning("[WPHoop] prefab has no MissionWPHoop_Sub — nothing will subscribe");
            return go;
        }
        if (!string.IsNullOrEmpty(robotName) && robotName != sub.RobotName)
        {
            Debug.Log($"[WPHoop] RobotName {sub.RobotName} -> {robotName} (from the scene's vehicle)");
            sub.RobotName = robotName;
            PrefabUtility.RecordPrefabInstancePropertyModifications(sub);
        }

        Debug.Log($"[WPHoop] added — subscribes /{sub.RobotName}/mission/last_wp. "
                + "Needs a GlobalReferencePoint in the scene (it disables itself and says so if absent).");
        return go;
    }
}
