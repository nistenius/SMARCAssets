using UnityEngine;
using UnityEditor;

using VehicleComponents.Sensors;
using VehicleComponents.Actuators;
using ROS.Publishers;
using ROS.Publishers.GroundTruth;
using ROS.Subscribers;
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
    // static readonly, NOT const: a const false makes the guarded block compile-time
    // unreachable and the compiler warns CS0162 on code that is deliberately switchable.
    static readonly bool PublishRawStereo = false;  // see MakeStereoCamera: 83 % of the vehicle's bandwidth
    const float CamCompressedPubFrequency = 5f;  // human/preview stream
    const float CamInfoPubFrequency = 5f;

    // ---- Mounting (base_link frame, Unity axes: x right, y up, z forward)
    // SAM nose is around z=0.70 (URDF x). Sonar on the centerline at the nose,
    // RealSense on top of the sonar housing (~60 mm tall). Both pitched 20 deg down.
    static readonly Vector3 SonarLinkPos = new Vector3(0f, 0.03f, 0.70f);
    static readonly Vector3 CamLeftLinkPos = new Vector3(-Baseline / 2f, 0.09f, 0.70f);
    static readonly Vector3 CamRightLinkPos = new Vector3(Baseline / 2f, 0.09f, 0.70f);
    const float MountPitchDeg = 20f;

    const string MissionHoopPrefabPath = PKG + "/Components/MissionWPHoop.prefab";

    // THIS IS A SCAFFOLD, NOT A REFRESH — and it refuses to run twice for a reason
    // (2026-08-17, after it silently destroyed three things in one press).
    // SaveAsPrefabAsset REPLACES its output wholesale, so every refinement made to a
    // generated prefab after the first build is discarded without a word. Measured
    // damage from a single press: Sonar3D15 lost `WaterLinkedSonar3DModes` entirely and
    // had its beam reset 150x17 -> 91x41; SAMSensorsV2's side-scan mount went 0/60 ->
    // 45/45; and sam2.2 was re-serialized end to end, which invalidated every fileID the
    // open scene's vehicle instance referred to — ForcePoints, sensors and publishers all
    // came up null and Play produced thousands of NREs and "No registered publisher"
    // exceptions. Two of those losses had previously been blamed on "Unity re-serializing
    // the prefab", which made a deterministic bug in OUR code look like an editor quirk.
    // To rebuild deliberately: flip the flag, build, and diff the result before committing.
    const bool OverwriteExistingPrefabs = false;

    [MenuItem("SMARC/Build SAM v2 Perception Prefabs")]
    public static void BuildAll()
    {
        if (!OverwriteExistingPrefabs)
        {
            foreach (var existingPath in new[] { SonarPrefabPath, RealSensePrefabPath,
                                                 SensorsV2PrefabPath, SamV2PrefabPath })
            {
                if (AssetDatabase.LoadAssetAtPath<GameObject>(existingPath) == null) continue;
                Debug.LogError("[SamV2PerceptionBuilder] REFUSED: " + existingPath + " already exists, " +
                               "and this builder replaces its outputs wholesale — it would discard every " +
                               "change made since the first build (components, beam counts, mount angles) " +
                               "and break the open scene's vehicle instance. Set OverwriteExistingPrefabs " +
                               "= true only if you mean to regenerate from scratch, and diff the result.");
                return;
            }
        }

        BuildSonarPrefab();
        BuildRealSensePrefab();
        BuildMissionHoopPrefab();
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

        // RAW STEREO IS OFF BY DEFAULT — measured 2026-08-17: two 848x480 rgb8 streams at
        // 2 Hz are 4.88 MB/s, which is ~83 % of everything this vehicle publishes and the
        // reason for the standing "Queue full! Messages are getting dropped!" warning. The
        // drop is indiscriminate, so a payload flood sheds core/* and the health checker
        // reads rate faults on healthy nodes (the 2026-08-09 finding, and the likely cause
        // of health flapping READY<->ERROR). Unchecking the component by hand does not
        // survive the next press of this menu item — this prefab is GENERATED — which is
        // the same trap that lost the side-scan mount, so the switch lives here.
        // Flip to true only for stereo development, and expect the bridge to saturate.
        if (PublishRawStereo)
        {
            var imgPub = go.AddComponent<CameraImage_Pub>();
            imgPub.topic = $"{topicBase}/image_raw";
            imgPub.frequency = CamRawPubFrequency;
        }

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

    static void BuildMissionHoopPrefab()
    {
        // Standalone, scene-draggable: shows the current WP of RobotName's mission as a
        // hula hoop (diameter = 2x goal tolerance), fed by the VM's mission/last_wp.
        var go = new GameObject("MissionWPHoop");
        var sub = go.AddComponent<MissionWPHoop_Sub>();
        sub.topic = "mission/last_wp";
        sub.RobotName = "sam_auv_v1";
        sub.NotARobot = true;
        PrefabUtility.SaveAsPrefabAsset(go, MissionHoopPrefabPath);
        Object.DestroyImmediate(go);
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

        // Session B ground-truth separation, completed: Unity must publish ONLY _gt
        // frames; the estimator owns the plain frames. The root TF_Pub already has
        // tf_suffix=_gt in sam_auv_v1.prefab, but the sensor suite's own tree
        // publishers were left at "" (field never serialized after the Session B
        // script change), so Unity ALSO published a plain truth tree — which is why
        // dr_vs_gt read 0.00 for a whole run (truth compared to itself).
        foreach (var name in new[] { "ROS_TF", "ROS_TF Rope" })
        {
            var t = FindDeep(root.transform, name);
            if (t != null && t.TryGetComponent<ROSTransformTreePublisher>(out var treePub))
                treePub.tf_suffix = "_gt";
        }

        // Estimator-session (2026-08-09 morning) Unity-side fixes that lived as SCENE
        // OVERRIDES on the old sam_auv_v1 instance and were therefore lost by building
        // from the prefab (found the hard way: 37 m DR divergence on the first
        // estimator-in-the-loop flight):
        //  - core/imu at 50 Hz = one message per physics step. At lower rates sim.yaml's
        //    assumed dt makes IMU preintegration accrue fake time (retraction #2 in
        //    2026-08-09-estimator-session-closure.md). Prefab had pub 20 Hz.
        var imuGO = FindDeep(root.transform, "IMU");
        if (imuGO != null && imuGO.TryGetComponent<IMU>(out var imuSensor)) imuSensor.frequency = 50f;
        SetPubFrequencyByTopic(root.transform, "core/imu", 50f);
        //  - SBG yaw random walk 0.5 deg/sqrt(min) (scene override; prefab had 0).
        var sbgGO = FindDeep(root.transform, "IMU SBG");
        if (sbgGO != null && sbgGO.TryGetComponent<IMU>(out var sbgSensor)) sbgSensor.yawDriftDegPerSqrtMin = 0.5f;

        // (The mission WP hoop is deliberately NOT part of the vehicle — it's a
        // standalone prefab, Components/MissionWPHoop.prefab, dragged into scenes
        // where wanted. See BuildMissionHoopPrefab.)

        // Deep Vision interferometric sidescan (DE340/DE680 family, as on the next
        // SAM): ONE transducer, switchable 340/680 kHz. The DeepVisionSSS component
        // owns the mode presets (range/bins/rays/ping rate) and forces interferometric
        // on, so SidescanMsg's angle bytes are populated for downstream point clouds.
        // Default HF680 (100 m/side, 5 cm bins) — right for the dry dock; flip the
        // Mode dropdown to LF340 for wide-area work.
        var sssT = FindDeep(root.transform, "SideScanSonar");
        if (sssT != null)
        {
            sssT.gameObject.name = "SideScanSonar DeepVision";
            var dv = sssT.gameObject.AddComponent<DeepVisionSSS>();
            dv.Mode = DeepVisionSSS.FrequencyMode.HF680;
            dv.Apply();
        }
        else Debug.LogWarning("[SamV2] SideScanSonar not found; DeepVision config skipped.");

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

        // 2) GT odom publisher: label its child frame as ground truth (Session B doc
        // intent, never actually serialized into the v1 prefab). The smarc/odom topic
        // name is left as-is deliberately — the duplicate-with-DR question is the
        // estimator session's open item.
        foreach (var gtOdom in root.GetComponentsInChildren<GT_Odom_Pub>(true))
            gtOdom.tf_suffix = "_gt";

        // 2b) VBS neutral trim 42% — also a scene override on the old instance
        // (prefab default 50 makes the vehicle heavy; the dive controller fights it).
        var vbsGO = FindDeep(root.transform, "VBS");
        if (vbsGO != null && vbsGO.TryGetComponent<VBS>(out var vbs))
        {
            vbs.percentage = 42f;
            vbs.resetValue = 42f;
        }

        // 3) Add the new nose links under base_link.
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

    /// <summary>
    /// On one GameObject: find the publisher whose 'topic' matches oldTopic and set its
    /// topic + frequency (SerializedObject, so internal publisher classes work).
    /// </summary>
    static void SetTopicAndFreqOn(GameObject go, string oldTopic, string newTopic, float freq)
    {
        foreach (var mb in go.GetComponents<MonoBehaviour>())
        {
            if (mb == null) continue;
            var so = new SerializedObject(mb);
            var topicProp = so.FindProperty("topic");
            if (topicProp == null || topicProp.propertyType != SerializedPropertyType.String) continue;
            if (topicProp.stringValue != oldTopic) continue;
            topicProp.stringValue = newTopic;
            var freqProp = so.FindProperty("frequency");
            if (freqProp != null) freqProp.floatValue = freq;
            so.ApplyModifiedPropertiesWithoutUndo();
        }
    }

    /// <summary>
    /// Set the 'frequency' field of any publisher whose 'topic' field matches, via
    /// SerializedObject so internal publisher classes don't need visibility changes.
    /// </summary>
    static void SetPubFrequencyByTopic(Transform root, string topic, float freq)
    {
        foreach (var mb in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (mb == null) continue;
            var so = new SerializedObject(mb);
            var topicProp = so.FindProperty("topic");
            if (topicProp == null || topicProp.propertyType != SerializedPropertyType.String) continue;
            if (topicProp.stringValue != topic) continue;
            var freqProp = so.FindProperty("frequency");
            if (freqProp == null) continue;
            freqProp.floatValue = freq;
            so.ApplyModifiedPropertiesWithoutUndo();
        }
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
