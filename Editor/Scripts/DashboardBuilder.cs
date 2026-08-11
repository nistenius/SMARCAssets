// SMARC -> Build Vehicle Dashboard (2026-08-10).
// Adds to Runtime/Prefabs/SmarcGUI/GUI.prefab, idempotently:
//   - Canvas-Top/DashboardPanel: transparent-black panel + TMP text block,
//     driven by VehicleDashboard (setpoints, wp, action, health, obstacle).
//   - GUIDarkTheme on the GUI root (transparent-black restyle, disable to revert).
// Rerun after tweaking constants; it removes and rebuilds its own objects only.
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using SmarcGUI;

public static class DashboardBuilder
{
    const string PrefabPath = "Packages/com.smarc.assets/Runtime/Prefabs/SmarcGUI/GUI.prefab";
    const string PrefabPathAssets = "Assets/SMARCAssets/Runtime/Prefabs/SmarcGUI/GUI.prefab";

    [MenuItem("SMARC/Build Vehicle Dashboard")]
    public static void Build()
    {
        string path = System.IO.File.Exists(PrefabPath.Replace("Packages/com.smarc.assets/", "Packages/com.smarc.assets/")) ? PrefabPath : PrefabPathAssets;
        // Robust load: try both package and Assets layouts.
        GameObject root = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (root == null) { root = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPathAssets); path = PrefabPathAssets; }
        else path = PrefabPath;
        if (root == null)
        {
            // Last resort: find it by name anywhere in the project.
            foreach (var guid in AssetDatabase.FindAssets("GUI t:Prefab"))
            {
                var p = AssetDatabase.GUIDToAssetPath(guid);
                if (p.EndsWith("/SmarcGUI/GUI.prefab")) { path = p; break; }
            }
            root = AssetDatabase.LoadAssetAtPath<GameObject>(path);
        }
        if (root == null) { Debug.LogError("DashboardBuilder: GUI.prefab not found"); return; }

        var contents = PrefabUtility.LoadPrefabContents(path);
        try
        {
            var canvasTop = FindChild(contents.transform, "Canvas-Top");
            var parent = canvasTop != null ? canvasTop : contents.transform;

            // Idempotency: rebuild our panel from scratch.
            var old = FindChild(parent, "DashboardPanel");
            if (old != null) Object.DestroyImmediate(old.gameObject);

            var panelGO = new GameObject("DashboardPanel", typeof(RectTransform), typeof(Image));
            panelGO.transform.SetParent(parent, false);
            var rt = panelGO.GetComponent<RectTransform>();
            rt.anchorMin = new Vector2(0f, 1f);
            rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f);
            rt.anchoredPosition = new Vector2(8f, -64f);   // just under the top banner
            rt.sizeDelta = new Vector2(560f, 78f);
            panelGO.GetComponent<Image>().color = new Color(0f, 0f, 0f, 0.55f);

            var textGO = new GameObject("DashboardText", typeof(RectTransform));
            textGO.transform.SetParent(panelGO.transform, false);
            var trt = textGO.GetComponent<RectTransform>();
            trt.anchorMin = Vector2.zero;
            trt.anchorMax = Vector2.one;
            trt.offsetMin = new Vector2(10f, 6f);
            trt.offsetMax = new Vector2(-10f, -6f);
            var tmp = textGO.AddComponent<TextMeshProUGUI>();
            tmp.fontSize = 16f;
            tmp.richText = true;
            tmp.color = Color.white;
            tmp.text = "dashboard: waiting for data...";

            var dash = panelGO.AddComponent<VehicleDashboard>();
            dash.DashboardText = tmp;
            dash.RobotName = "sam_auv_v1";

            // Small controller annotations injected into the existing banner fields
            // (Ivan, 2026-08-10): yaw err on Compass, obstacle on Alt, depth ref+err
            // on Depth, rpm+vbs on Speed.
            dash.SmallCompassText = InjectSmallLabel(contents.transform, "Compass");
            dash.SmallAltText     = InjectSmallLabel(contents.transform, "Altitude");
            dash.SmallDepthText   = InjectSmallLabel(contents.transform, "Depth");
            dash.SmallSpeedText   = InjectSmallLabel(contents.transform, "Speed");

            // Dark theme on the whole GUI (disable the component to revert).
            if (contents.GetComponent<GUIDarkTheme>() == null)
                contents.AddComponent<GUIDarkTheme>();

            PrefabUtility.SaveAsPrefabAsset(contents, path);
            Debug.Log($"DashboardBuilder: DashboardPanel + GUIDarkTheme written to {path}");
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(contents);
        }
    }

    static TextMeshProUGUI InjectSmallLabel(Transform root, string fieldName)
    {
        var field = FindChild(root, fieldName);
        if (field == null) { Debug.LogWarning($"DashboardBuilder: banner field '{fieldName}' not found"); return null; }

        var old = field.Find("CtrlSmall");
        if (old != null) Object.DestroyImmediate(old.gameObject);

        var go = new GameObject("CtrlSmall", typeof(RectTransform));
        go.transform.SetParent(field, false);
        // The banner fields live in a LayoutGroup — without this the label is
        // treated as a layout element and squeezed into a vertical 1-char column.
        var le = go.AddComponent<UnityEngine.UI.LayoutElement>();
        le.ignoreLayout = true;
        var rt = go.GetComponent<RectTransform>();
        rt.anchorMin = new Vector2(0f, 1f);
        rt.anchorMax = new Vector2(1f, 1f);
        rt.pivot = new Vector2(0.5f, 1f);
        rt.anchoredPosition = new Vector2(0f, 0f);
        rt.sizeDelta = new Vector2(0f, 14f);

        var tmp = go.AddComponent<TextMeshProUGUI>();
        tmp.fontSize = 10f;
        tmp.color = Color.white;
        tmp.alignment = TextAlignmentOptions.TopRight;
        tmp.margin = new Vector4(2f, 1f, 4f, 0f);
        tmp.raycastTarget = false;
        tmp.text = "";
        return tmp;
    }

    static Transform FindChild(Transform t, string name)
    {
        if (t.name == name) return t;
        foreach (Transform c in t)
        {
            var r = FindChild(c, name);
            if (r != null) return r;
        }
        return null;
    }
}
