using UnityEngine;
using UnityEditor;

using VehicleComponents.Sensors;
using ROS.Publishers;
using Visualizers;

/// <summary>
/// Builds the SAM v2 perception setup:
///  - Sonar3D15.prefab           : WaterLinked Sonar 3D-15 (90x40 deg FOV, 15 m) as an FLS-type Sonar + PointCloud2 pub
///  - RealSenseD435i.prefab      : stereo pair (left/right CameraImage) + ground-truth depth raycast cam
///  - SAMSensorsV2.prefab        : SAMSensors minus (CameraDown, CameraStrb, CameraPort, MultiBeamSonar) plus the two above
///  - sam_auv_v2.prefab          : sam_auv_v1 with new nose links (sonar3d_link, cam_realsense_left/right_link)
///                                 and SAMSensorsV2 instead of SAMSensors.
/// Run from menu: SMARC -> Build SAM v2 Perception Prefabs
/// Safe to re-run; overwrites the generated prefabs.
/// </summary>
public static class SamV2PerceptionBuilder
{
    const string PKG = "Packages/com.smarc.assets/Runtime/Prefabs";
    const string SonarPrefabPath = PKG + "/Components/Sonar3D15.prefab";
    const string RealSensePrefabPath = PKG + "/Components/RealSenseD435i.prefab";
    const string SensorsV2PrefabPath = PKG + "/Components/SAMSensorsV2.prefab";
    const string SamV1PrefabPath = PKG + "/sam_auv_v1.prefab";
    // Named sam2.2 (SAM class 2, version 2) so multiple versions/instances of the class
    // can coexist. NOTE: ROS names cannot contain '.', and topics/frames are prefixed
    // with the robot GameObject's name at runtime — name scene instances sam2_2, sam0 etc.
    const string SamV2PrefabPath = PKG + "/sam2.2.prefab";
    const string RayMaterialGUID = "4c69fe22859e53c60b1e6d411903d798"; // same mat the old MBES RayViewer used

    // ---- WL Sonar 3D-15 (real: 90x40 deg, 15 m, ~16k beams/ping; sim uses fewer rays, tune as needed)
    const float SonarHFOV = 90f;
    const float SonarVFOV = 40f;
    const float SonarMaxRange = 15f;
    const int SonarNumBeamsHorizontal = 91;   // ~1 deg horizontal spacing
    const int SonarNumRaysPerBeam = 41;       // ~1 deg vertical spacing -> 3731 rays/ping
    const float SonarFrequency = 5f;          // update rate not in public datasheet; tunable

    // ---- RealSense D435i (depth/IR FOV 87x58 deg, baseline 50 mm)
    const int CamWidth = 848;
    const int CamHeight = 480;
    const float CamFocalLength_mm = 1.93f;
    const float CamHFOV = 87f;
    const float Baseline = 0.05f;
    // Sensor render rate. Publishers are slower: two raw 848x480 rgb8 streams at 10 Hz
    // saturated the ROS TCP bridge ("Queue full! Messages are getting dropped") in the
    // 2026-08-09 test — and bridge overload both sheds core sensor messages (health
    // checker sees rate faults) and can shut the socket (see live-sim-runbook traps).
    const float CamFrequency = 10f;
    const float CamRawPubFrequency = 2f;         // raw stream is for stereo dev, not for streaming
    const float CamCompressedPubFrequency = 5f;  // human/preview stream
    const float CamInfoPubFrequency = 5f;

    // ---- Mounting (base_link frame, Unity axes: x right, y up, z forward)
    // SAM nose is around z=0.70 (URDF x). Sonar on the centerline at the nose,
    // RealSense on top of the sonar housing (~60 mm tall). Both pitched 20 deg down.
    static readonly Vector3 SonarLinkPos = new Vector3(0f, 0.03f, 0.70f);
    static readonly Vector3 CamLeftLinkPos = new Vector3(-Baseline / 2f, 0.09f, 0.70f);
    static readonly Vector3 CamRightLinkPos = new Vector3(Baseline / 2f, 0.09f, 0.70f);
    const float MountPitchDeg = 20f;

    [MenuItem("SMARC/Build SAM v2 Perception Prefabs")]
    public static void BuildAll()
    {
        BuildSonarPrefab();
        BuildRealSensePrefab();
        BuildSensorsV2Prefab();
        BuildSamV2Prefab();
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log("[SamV2PerceptionBuilder] Done. Built:\n" +
                  $"  {SonarPrefabPath}\n  {RealSensePrefabPath}\n  {SensorsV2PrefabPath}\n  {SamV2PrefabPath}");
    }

    static void BuildSonarPrefab()
    {
        var go = new GameObject("Sonar3D15");

        var sonar = go.AddComponent<Sonar>();
        sonar.linkName = "sonar3d_link";
        sonar.Type = SonarType.FLS;
        sonar.NumBeams = SonarNumBeamsHorizontal;
        sonar.NumRaysPerBeam = SonarNumRaysPerBeam;
        sonar.BeamBreadthDeg = SonarVFOV;
        sonar.FLSFOVDeg = SonarHFOV;
        // FLS rays span [Tilt, Tilt+BeamBreadth] below the link's forward axis.
        // The link itself is pitched down 20 deg (the physical mount), so center
        // the fan on the link axis: [-20, +20].
        sonar.TiltAngleDeg = -SonarVFOV / 2f;
        sonar.MaxRange = SonarMaxRange;
        sonar.BeamBreadth3DecibelsDeg = 60f;
        sonar.frequency = SonarFrequency;

        var pub = go.AddComponent<SonarPointCloud_Pub>();
        pub.topic = "payload/sonar3d/points";
        pub.frequency = SonarFrequency;

        var viewer = go.AddComponent<RayViewer>();
        viewer.DrawRays = false; // 3.7k rays; flip on for debugging
        viewer.DrawHits = true;
        viewer.UseRainbow = true;
        viewer.HitsSize = 0.1f;
        viewer.HitsLifetime = 3f;
        viewer.DrawEveryNthFrame = 10;
        var rayMatPath = AssetDatabase.GUIDToAssetPath(RayMaterialGUID);
        if (!string.IsNullOrEmpty(rayMatPath))
            viewer.RayMaterial = AssetDatabase.LoadAssetAtPath<Material>(rayMatPath);

        PrefabUtility.SaveAsPrefabAsset(go, SonarPrefabPath);
        Object.DestroyImmediate(go);
    }

    static void BuildRealSensePrefab()
    {
        var root = new GameObject("RealSenseD435i");

        MakeStereoCamera(root, "CamLeft", "cam_realsense_left_link", "payload/realsense/left", 0f);
        MakeStereoCamera(root, "CamRight", "cam_realsense_right_link", "payload/realsense/right", Baseline);

        // Ground-truth depth, raycast-based (pipeline independent). RealSense depth
        // is registered to the left imager, so attach to the left link.
        var depthGO = new GameObject("DepthGT");
        depthGO.transform.SetParent(root.transform, false);
        var depth = depthGO.AddComponent<DepthCamera>();
        depth.linkName = "cam_realsense_left_link";
        depth.textureWidth = 106;  // 1/8 of 848x480; one raycast per pixel
        depth.textureHeight = 60;
        depth.HFOVDeg = 87f;
        depth.VFOVDeg = 58f;
        depth.MinRange = 0.3f;
        depth.MaxRange = 10f;
        depth.frequency = 5f;
        var depthPub = depthGO.AddComponent<DepthImage_Pub>();
        depthPub.topic = "payload/realsense/depth/image_raw";
        depthPub.frequency = 5f;

        PrefabUtility.SaveAsPrefabAsset(root, RealSensePrefabPath);
        Object.DestroyImmediate(root);
    }

    static void MakeStereoCamera(GameObject parent, string name, string linkName, string topicBase, float baselineFromLeft)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent.transform, false);

        var camImage = go.AddComponent<CameraImage>(); // RequireComponent adds the Camera
        camImage.linkName = linkName;
        camImage.textureWidth = CamWidth;
        camImage.textureHeight = CamHeight;
        camImage.frequency = CamFrequency;
        camImage.viewCam = false;

        // Match D435i geometry via physical camera params. Square pixels, HFOV=87
        // -> fx = fy = 848 / (2 tan(43.5 deg)) ~= 446.6, VFOV ~= 56.6 (spec 58).
        var cam = go.GetComponent<Camera>();
        float fpx = CamWidth / (2f * Mathf.Tan(CamHFOV * Mathf.Deg2Rad / 2f));
        cam.usePhysicalProperties = true;
        cam.focalLength = CamFocalLength_mm;
        cam.sensorSize = new Vector2(
            CamWidth * CamFocalLength_mm / fpx,
            CamHeight * CamFocalLength_mm / fpx);
        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 300f;

        var imgPub = go.AddComponent<CameraImage_Pub>();
        imgPub.topic = $"{topicBase}/image_raw";
        imgPub.frequency = CamRawPubFrequency;

        var infoPub = go.AddComponent<CameraInfo_Pub>();
        infoPub.topic = $"{topicBase}/camera_info";
        infoPub.frequency = CamInfoPubFrequency;
        // K is computed live from the Camera by CameraInfo_Pub; P we fill here.
        // Stereo: right camera P has Tx = -fx * baseline.
        infoPub.k1 = 0; infoPub.k2 = 0; infoPub.t1 = 0; infoPub.t2 = 0; infoPub.k3 = 0;
        infoPub.fxp = fpx; infoPub.fyp = fpx;
        infoPub.cxp = CamWidth / 2f; infoPub.cyp = CamHeight / 2f;
        infoPub.Tx = -fpx * baselineFromLeft;
        infoPub.Ty = 0;

        var compPub = go.AddComponent<CameraImageCompressed_Pub>();
        compPub.topic = $"{topicBase}/image_raw/compressed";
        compPub.frequency = CamCompressedPubFrequency;
    }

    static void BuildSensorsV2Prefab()
    {
        var src = AssetDatabase.LoadAssetAtPath<GameObject>(PKG + "/Components/SAMSensors.prefab");
        if (src == null) { Debug.LogError("SAMSensors.prefab not found!"); return; }

        var root = (GameObject)PrefabUtility.InstantiatePrefab(src);
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        root.name = "SAMSensorsV2";

        // Not on the next SAM: the three hull cameras and the MBES (never really sat on SAM).
        // The sidescan stays (interferometric flag can be toggled on its Sonar component).
        string[] toRemove = { "CameraDown", "CameraStrb", "CameraPort", "MultiBeamSonar" };
        foreach (var name in toRemove)
        {
            var t = FindDeep(root.transform, name);
            if (t != null) Object.DestroyImmediate(t.gameObject);
            else Debug.LogWarning($"[SamV2] Expected child {name} not found in SAMSensors copy.");
        }

        // New nose package, as nested prefab instances.
        foreach (var path in new[] { SonarPrefabPath, RealSensePrefabPath })
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            inst.transform.SetParent(root.transform, false);
        }

        PrefabUtility.SaveAsPrefabAsset(root, SensorsV2PrefabPath);
        Object.DestroyImmediate(root);
    }

    static void BuildSamV2Prefab()
    {
        var src = AssetDatabase.LoadAssetAtPath<GameObject>(SamV1PrefabPath);
        if (src == null) { Debug.LogError("sam_auv_v1.prefab not found!"); return; }

        var root = (GameObject)PrefabUtility.InstantiatePrefab(src);
        PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        root.name = "sam2.2";

        // 1) Swap the sensor suite.
        var oldSensors = FindDeep(root.transform, "SAMSensors");
        Transform sensorsParent = root.transform;
        int sensorsSiblingIndex = 0;
        if (oldSensors != null)
        {
            sensorsParent = oldSensors.parent;
            sensorsSiblingIndex = oldSensors.GetSiblingIndex();
            Object.DestroyImmediate(oldSensors.gameObject);
        }
        else Debug.LogWarning("[SamV2] SAMSensors instance not found in sam_auv_v1 copy; adding V2 at root.");

        var sensorsV2Prefab = AssetDatabase.LoadAssetAtPath<GameObject>(SensorsV2PrefabPath);
        var sensorsV2 = (GameObject)PrefabUtility.InstantiatePrefab(sensorsV2Prefab);
        sensorsV2.transform.SetParent(sensorsParent, false);
        sensorsV2.transform.SetSiblingIndex(sensorsSiblingIndex);

        // 2) Add the new nose links under base_link.
        var baseLink = FindDeep(root.transform, "base_link");
        if (baseLink == null)
        {
            Debug.LogError("[SamV2] base_link not found; cannot add sensor links.");
            Object.DestroyImmediate(root);
            return;
        }
        AddLink(baseLink, "sonar3d_link", SonarLinkPos, MountPitchDeg);
        AddLink(baseLink, "cam_realsense_left_link", CamLeftLinkPos, MountPitchDeg);
        AddLink(baseLink, "cam_realsense_right_link", CamRightLinkPos, MountPitchDeg);

        PrefabUtility.SaveAsPrefabAsset(root, SamV2PrefabPath);
        Object.DestroyImmediate(root);
    }

    static void AddLink(Transform parent, string name, Vector3 localPos, float pitchDeg)
    {
        var existing = FindDeep(parent, name);
        if (existing != null) Object.DestroyImmediate(existing.gameObject);
        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPos;
        go.transform.localRotation = Quaternion.Euler(pitchDeg, 0f, 0f);
    }

    static Transform FindDeep(Transform parent, string name)
    {
        if (parent.name == name) return parent;
        foreach (Transform child in parent)
        {
            var r = FindDeep(child, name);
            if (r != null) return r;
        }
        return null;
    }
}
