using System.Collections.Generic;

using UnityEngine;
using UnityEditor;

using VehicleComponents.Sensors;
using VehicleComponents.Comms;
using ROS.Publishers;

namespace SMARC.Editor
{
    /// <summary>
    /// Builds the Data Cube FIELD STATION prefab — ADR-007 steps 1 and 5.
    ///
    /// WHY A STATION IS A PREFAB AT ALL. The base station travels to the operational area
    /// carrying the comms modems and an RTK GNSS, so the moment it is powered on it knows where
    /// it is — which makes THE LAUNCH SITE MEASURED RATHER THAN CONFIGURED. What forced it:
    /// mission #35 was planned on Kristineberg while carrying a launch point at Beckholmen,
    /// 400 km away, and flew twice with nothing refusing, because the only field actually
    /// transmitted was the one field the site-mismatch guard did not check.
    ///
    /// Giving it a prefab rather than a bespoke path is the whole point of ADR-007: everything a
    /// station needs already exists for vehicles — identity, a GPS that reports, modems, a
    /// CommsManager — so it becomes an AGENT with `motion: none` instead of a special case. This
    /// project has paid repeatedly for parallel implementations (two launchers, two notions of
    /// neutral VBS, two definitions of divergence); one vocabulary is cheaper than two.
    ///
    /// HARDWARE, AND WHERE IT COMES FROM (Ivan, 2026-08-18): "put a RTK GPS (same as on SAM) and
    /// a Succorfish modem in there also". Both part names are carried across from CONFIRMED SAM
    /// entries rather than looked up independently — Sparkfun GPS-RTK2 from the SAM nosecone
    /// antenna (mrlwiki.se, 2026-07-01) and the Succorfish Delphis transceiver from SAM's
    /// underwater comms. See fleet.yaml's `datacube_station_01`, where they are tagged
    /// user_provided, not confirmed: no physical unit exists yet.
    ///
    /// WHAT THIS DELIBERATELY DOES NOT DO. It does not place the station in a scene, and it does
    /// not wire the vehicle's `CommsManager.baseStation` to it. Placement is a site question
    /// (the Kristineberg builder owns the quay), and the wiring is a scene edit an operator
    /// should make and see. A builder that reached into the open scene is exactly how three
    /// prefabs were destroyed on 2026-08-18.
    /// </summary>
    public static class StationBuilder
    {
        const string PKG = "Packages/com.smarc.assets/Runtime/Prefabs";
        const string MATS = "Packages/com.smarc.assets/Runtime/Materials";
        const string StationPrefabPath = PKG + "/datacube_station_01.prefab";
        const string CaseBodyMatPath = MATS + "/StationCaseBody.mat";
        const string CaseTrimMatPath = MATS + "/StationCaseTrim.mat";

        // ---- Peli iM2400 Storm Case, EXTERIOR, in metres --------------------------------
        // Source: peli.com's iM2400 product page, read by Ivan on 2026-08-18 and quoted to me
        // ("Exterior: 48.8 x 38.6 x 18.5 cm"; interior 45.7 x 33 x 17). NOT independently
        // verified: fetching that page returns Peli's homepage rather than the product data,
        // so this is a user_provided figure in the same sense fleet.yaml uses the term.
        //
        // WHY NOT THE REAL CAD. Peli DO publish CAD, at /professional/cad-downloads/, but it
        // sits behind a terms-and-conditions acceptance -- which is Ivan's agreement to make,
        // not mine -- and a STEP file would still need a pass through FreeCAD/Fusion before
        // Unity could read it. A dimensionally correct procedural case is honest, has no
        // licensing question, and needs no import pipeline. Swap it for the real mesh whenever
        // one exists: everything else in this prefab is independent of the visual.
        const float CaseLen = 0.488f;   // along local X
        const float CaseWid = 0.386f;   // along local Z
        const float CaseHgt = 0.185f;   // total, base + lid
        const float LidHgt  = 0.070f;   // the lid is the shallower half on a laptop case
        static readonly Color PeliYellow = new Color(0.94f, 0.62f, 0.06f);   // the yellow option
        static readonly Color CaseTrim   = new Color(0.10f, 0.10f, 0.11f);   // latches, handle, lip

        /// <summary>Same guard as SamV2PerceptionBuilder, for the same measured reason: that
        /// builder replaced its outputs wholesale, one menu press destroyed three prefabs and
        /// broke the open scene's vehicle instance with thousands of NREs, and everything had to
        /// be restored from HEAD. A builder that silently overwrites is a builder that will.</summary>
        const bool OverwriteExistingPrefab = false;

        // RTK figures are the GPS component's own defaults for a GPS-RTK2 — NOT re-derived here.
        // Two copies of a sigma is how a "measurement" becomes an invention (SETTLED, passim).
        // Measured out of Peli's own STEP files (2400-base_internal_tub.stp /
        // 2400-lid_internal_tub.stp, from L:/P-Dwgs/P111/iM Case Assemblies/2400/, supplied by
        // Ivan 2026-08-18) by taking the bounding box of the VERTEX_POINTs only. Both tubs share
        // an identical 469.90 x 346.07 mm footprint — that is the real moulded cavity — which
        // sits inside the published 488 x 386 exterior with ~9 mm walls on the long sides and
        // ~20 mm on the short. So the CAD CONFIRMS the exterior figures rather than contradicting
        // them, which is why the numbers above are used with more confidence than a website alone
        // would earn.
        //
        // NOTE the raw point cloud gives 503 mm, which is WRONG: B-spline control points lie
        // outside the surface they define. Vertices are the geometry; control points are the
        // recipe. Worth remembering the next time a STEP file is measured this way.
        const float TubLen = 0.46990f;
        const float TubWid = 0.34607f;

        // ---- THE AUTHORED TRANSDUCER STATE, IN ONE PLACE --------------------------------
        // Both the from-scratch builder and the surgical updater write EXACTLY these, and the
        // updater's planner compares against EXACTLY these. Two copies of an authored value is
        // how a builder and a checker start disagreeing about what "correct" means, and a
        // disagreement between a writer and its own checker is precisely the 2026-08-21 defect:
        // press the menu item, it reports a change; press it again, it reports the same change.
        //
        // AuthoredAcousticLocalY IS ZERO, AND THAT IS THE FIX. It used to be a fixed offset below
        // base_link — below the CASE — and with the case a metre up on the dock bridge that put the
        // modelled transducer in the air. Y is owned by `DeployedTransducer` and taken from the
        // WATER PLANE at runtime; zero here means "nothing authored", not "at the case".
        static readonly Vector3 AuthoredAcousticLocalPosition = Vector3.zero;
        const float AuthoredDipDepthM = 0.5f;        // Ivan: "at least 50cm down below the water level"
        const float AuthoredMinDipDepthM = 0.1f;
        const float AuthoredPracticeDipDepthM = 0.5f;
        const bool AuthoredAutoDeployInEditor = true;
        const bool AuthoredRequireWet = true;

        [MenuItem("SMARC/Build Data Cube Station Prefab")]
        public static void BuildStation()
        {
            if (!OverwriteExistingPrefab &&
                AssetDatabase.LoadAssetAtPath<GameObject>(StationPrefabPath) != null)
            {
                Debug.LogError("[StationBuilder] REFUSED: " + StationPrefabPath + " already exists. " +
                               "This builder replaces its output wholesale and would discard every " +
                               "change made since the first build. Set OverwriteExistingPrefab = true " +
                               "only if you mean to regenerate from scratch, and diff the result.");
                return;
            }

            // ---- root -------------------------------------------------------------------
            // Tagged "robot" because LinkAttachment.Attach() walks UP for a parent tagged
            // [robot] and then searches its children for the named link. A station that is not
            // tagged would have its GPS silently disable itself at Attach() — the sensor would
            // be present in the Inspector and dead, which is the worst of both.
            var root = new GameObject("datacube_station_01");
            TrySetRobotTag(root);

            var baseLink = new GameObject("base_link");
            baseLink.transform.SetParent(root.transform, false);

            BuildCaseVisual(baseLink.transform);

            // ---- links ------------------------------------------------------------------
            var gpsLink = new GameObject("gps_link");
            gpsLink.transform.SetParent(baseLink.transform, false);
            // Antenna a little above the box, so a future range/line-of-sight calculation has a
            // sensible origin rather than the middle of the case.
            gpsLink.transform.localPosition = new Vector3(0f, CaseHgt + 0.02f, 0f);

            var acousticLink = new GameObject("acoustic_link");
            acousticLink.transform.SetParent(baseLink.transform, false);
            // NO AUTHORED Y OFFSET, AND THAT IS THE FIX (2026-08-21). This used to be
            // (0, -0.10, 0): ten centimetres below the CASE. Ivan then placed the station on a
            // dock bridge with the case "some meter above the water line", which put the modelled
            // transducer roughly 0.9 m in the air — and a fixed offset from the case is wrong in
            // principle anyway, because moving the case (quay edge, bridge, Milou's deck) silently
            // changes the deployed depth with nothing reporting it. `DeployedTransducer` below owns
            // Y and takes it from the WATER PLANE, so the link stays at the configured depth
            // wherever the station is put. Zero here means "nothing authored"; if the component
            // ever refuses (no WaterSurface), the transducer sits inside the case, is dry, and the
            // modem says so by name — which is the correct failure, not a plausible one.
            acousticLink.transform.localPosition = AuthoredAcousticLocalPosition;
            var deployment = acousticLink.AddComponent<DeployedTransducer>();
            deployment.DipDepthM = AuthoredDipDepthM;
            deployment.MinDipDepthM = AuthoredMinDipDepthM;
            deployment.PracticeDipDepthM = AuthoredPracticeDipDepthM;
            deployment.AutoDeployInEditor = AuthoredAutoDeployInEditor;
            deployment.CaseAnchor = baseLink.transform;

            // ---- RTK GNSS ---------------------------------------------------------------
            var gpsGO = new GameObject("GPS");
            gpsGO.transform.SetParent(gpsLink.transform, false);
            var gps = gpsGO.AddComponent<GPS>();
            gps.linkName = "gps_link";
            gps.retryUntilSuccess = true;
            gps.rtkEnabled = true;
            // A station's antenna is bolted down and dry: it does NOT dunk in the waves the way
            // SAM's does (gps_link 0.071 m above base_link is why the vehicle's fix flickers and
            // why nav_ready had to latch evidence rather than read the live fix). So the station
            // has no reacquisition modelling — its fix is steady, which is exactly the property
            // that makes it worth trusting as a launch-point reference.
            gps.modelReacquisition = false;
            gps.frequency = 1f;      // a stationary receiver has nothing to say at 10 Hz

            // THE TOPIC MUST BE SET HERE (2026-08-19). `ROSBehaviour.topic` defaults to "" and
            // `OnEnable` disables the component on an empty topic — so a GPS_Pub added without one
            // is present in the Inspector, unticks its own checkbox, and publishes nothing. That is
            // exactly how the station sat silent through the ADR-007 demo while the Mockup's
            // hardcoded constant stood in for a measurement 49.4 m away (SETTLED §3s2). Not global
            // (no leading '/'), so it is namespaced to /datacube_station_01/core/gps — the same
            // `core/gps` relative name SAM's GPS_Pub uses, because the station is a fleet agent and
            // not a special case.
            var gpsPub = gpsGO.AddComponent<GPS_Pub>();
            gpsPub.topic = "core/gps";

            // ---- comms ------------------------------------------------------------------
            var acousticGO = new GameObject("AcousticModem_SuccorfishDelphis");
            acousticGO.transform.SetParent(acousticLink.transform, false);
            var acoustic = acousticGO.AddComponent<AcousticModem>();
            // requireWet STAYS ON (2026-08-21). It used to be forced false here, with the comment
            // that the case is topside — true of the case, false of the transducer, and it bought a
            // station whose acoustic link could not report the one problem it definitely had: the
            // modelled transducer was in the air. Now that `DeployedTransducer` puts the fish
            // 0.5 m under the water plane the test is real, and a station that has NOT been
            // deployed reports "transducer above the water line — not deployed" instead of
            // pretending. Every modem naming its refusal reason is the rule this component exists
            // for (ADR-007 §5); silencing the test to make a link look up is the opposite of it.
            // Written out rather than left to the field default, because the previous value of this
            // exact line is the defect.
            acoustic.requireWet = AuthoredRequireWet;

            var wifiGO = new GameObject("WiFiModem");
            wifiGO.transform.SetParent(baseLink.transform, false);
            wifiGO.AddComponent<WiFiModem>();

            var cellGO = new GameObject("CellularModem");
            cellGO.transform.SetParent(baseLink.transform, false);
            cellGO.AddComponent<CellularModem>();

            // CommsManager finds every CommsModem in its children and, per ADR-007 §5, link
            // ranges become the distance between two agents that both exist. `baseStation` (the
            // far end) is left UNASSIGNED on purpose: from the station's point of view the peer
            // is the VEHICLE, which does not exist in this prefab. Assign it in the scene, and
            // assign this station into the VEHICLE's CommsManager.baseStation — that second one
            // is what turns configured link modes into measured geometry.
            root.AddComponent<CommsManager>();

            // ---- save --------------------------------------------------------------------
            // No directory creation: the Prefabs folder already exists (every other prefab lives
            // there), and CreateDirectory on an asset path would resolve against the process's
            // working directory and quietly make a stray folder somewhere nobody looks.
            PrefabUtility.SaveAsPrefabAsset(root, StationPrefabPath);
            Object.DestroyImmediate(root);
            AssetDatabase.Refresh();

            Debug.Log("[StationBuilder] Built " + StationPrefabPath + "\n" +
                      "NEXT, and none of it is done for you:\n" +
                      "  1. Drag the prefab into the scene and place it where the case really sits — " +
                      "on the dock bridge, not floating. The transducer looks after itself: " +
                      "acoustic_link carries a DeployedTransducer that puts it 0.50 m below the " +
                      "WATER PLANE and keeps the case's X/Z, so re-placing the station cannot " +
                      "silently change the deployed depth.\n" +
                      "  2. Check the Scene view says 'transducer 0.50 m below water line'. If it says " +
                      "DRY, or 'no WaterSurface', the acoustic link is DOWN by design and will name " +
                      "that reason — it is not a display bug.\n" +
                      "  3. Set the VEHICLE's CommsManager.baseStation to this station's transform — " +
                      "until then link ranges stay configured rather than measured (ADR-007 §5). " +
                      "Ranges are measured FROM the transducer, which is why its position is a " +
                      "measurement and not decoration.\n" +
                      "  4. The station reports its own GPS; ADR-007 step 2 (status + position to " +
                      "the backend) is what makes the Arm gate real. Until that lands the gate is " +
                      "inert by design and arming is unchanged.\n" +
                      "  5. If the root is not tagged [robot], add that tag in Project Settings and " +
                      "re-run — LinkAttachment disables the GPS without it.\n" +
                      "  6. DO NOT re-run this menu item on an existing station. Use " +
                      "'SMARC/Station/Update transducer deployment (surgical)' instead — it edits the " +
                      "prefab in place and your scene placement survives.");
        }

        /// <summary>
        /// Give the EXISTING station prefab a water-plane-referenced transducer, surgically —
        /// without regenerating anything, because Ivan has `datacube_station_01` placed in
        /// Beckholmen at (1.3, 1.14567, 48.7) on the dock bridge and `BuildStation()` replaces its
        /// output wholesale. That is the `SMARC/Build SAM v2 Perception Prefabs` failure mode
        /// exactly (SETTLED §5: one press, three prefabs destroyed, every fileID in `sam2.2`
        /// churned, the open scene's vehicle instance broken with thousands of NREs).
        ///
        /// WHAT IT TOUCHES, AND NOTHING ELSE:
        ///   * `base_link/acoustic_link` — adds `DeployedTransducer` if absent, sets its depths,
        ///     and zeroes the authored localPosition (a case-relative offset, which put the
        ///     transducer roughly a metre in the air once the case went on the dock bridge).
        ///   * the `AcousticModem` under it — `requireWet` back ON, because the transducer is now
        ///     genuinely in the water and the test can be honest.
        /// The GPS, the WiFi/cellular modems, the CommsManager, the case visual, the root tag and
        /// every scene placement are untouched, and the log below says which of those it actually
        /// changed rather than claiming all of them.
        ///
        /// AND IT IS IDEMPOTENT, WHICH IS NOW TESTED RATHER THAN CLAIMED (2026-08-21, second
        /// defect). The first version planned against the PREFAB CONTENTS; `DeployedTransducer` is
        /// `[ExecuteAlways]` and ran inside the preview scene `LoadPrefabContents` opens, deployed
        /// against the OPEN SITE SCENE's water, and so every press found a "defect" to correct and
        /// rewrote the prefab. Six presses, six identical log lines. The three rules that came out
        /// of it, and that this method is built on:
        ///   1. PLAN AGAINST THE ASSET (`LoadAssetAtPath`) — persistent, sceneless, runs no
        ///      Awake/OnEnable, so nothing can have edited it before the planner looks.
        ///   2. NEVER TREAT A COMPONENT'S OWN OUTPUT AS A DEFECT. This authors settings; it does
        ///      not police `DeployedTransducer`'s live measurements.
        ///   3. CHECK THE CLAIM. After writing, it re-plans against the file it just wrote — the
        ///      second press, performed by the first — and reports which answer it got.
        /// Note for anyone reading the 2026-08-21 console: the first press reported the old value
        /// as (0, -0.30, 0) while this builder's source had authored -0.10. The prefab has never
        /// been committed, so there is no history to consult and NO EXPLANATION IS OFFERED HERE. The
        /// only corroborating fact on disk is that `gps_link` sits at +0.30 where the builder writes
        /// CaseHgt + 0.02 = 0.205, so the file had been edited by hand after it was built. Who and
        /// when is unknown; a plausible mechanism invented now would only make a false observation
        /// feel confirmed (SETTLED §1).
        /// </summary>
        [MenuItem("SMARC/Station/Update transducer deployment (surgical)")]
        public static void UpdateTransducerDeployment()
        {
            // ---- 1. PLAN AGAINST THE ASSET, WHICH NOTHING CAN HAVE TOUCHED -------------------
            // LoadAssetAtPath gives the PREFAB ASSET itself: persistent objects, no scene, no
            // Awake/OnEnable, so no [ExecuteAlways] component can have run and moved anything
            // before we look. That independence is the whole point — the previous version planned
            // against the CONTENTS, which `DeployedTransducer` had already edited on the way in.
            var asset = AssetDatabase.LoadAssetAtPath<GameObject>(StationPrefabPath);
            if (asset == null)
            {
                Debug.LogError("[StationBuilder] No station prefab at " + StationPrefabPath +
                               " — nothing to update. (Build it with 'SMARC/Build Data Cube Station " +
                               "Prefab' only if you have no placed instance to lose.)");
                return;
            }

            if (!TryFindTransducerLinks(asset.transform, out var assetBaseLink, out var assetAcousticLink,
                                        out string refusal))
            {
                Debug.LogError("[StationBuilder] " + refusal);
                return;
            }

            var authoredPos = assetAcousticLink.localPosition;
            var plan = PlanTransducerChanges(assetBaseLink, assetAcousticLink);

            // ---- 2. NOTHING TO DO? THEN DO NOTHING — AND STILL RUN THE TRAP ------------------
            if (plan.Count == 0)
            {
                // The prefab is opened anyway, and NOTHING IS SAVED on this path. The point is the
                // check below: it is the only place the two-writers defect can be caught
                // automatically, and it is free. SETTLED §3s3 — a structural check is only trusted
                // to fail once it has been run against the state it must PASS.
                string moved = ProbeForSecondWriter(authoredPos);
                if (moved != null)
                {
                    Debug.LogError("[StationBuilder] " + moved);
                    return;
                }

                Debug.Log("[StationBuilder] Station transducer already water-plane referenced — " +
                          "NOTHING TO CHANGE, and nothing was written. " + StationPrefabPath +
                          "\nChecked against the prefab ASSET (which no [ExecuteAlways] component " +
                          "can have touched), and the prefab was then opened once to confirm that " +
                          "nothing moves acoustic_link while it is open. Both clean.");
                return;
            }

            // ---- 3. APPLY, ON THE CONTENTS, EXACTLY THE AUTHORED STATE -----------------------
            var root = PrefabUtility.LoadPrefabContents(StationPrefabPath);
            try
            {
                if (!TryFindTransducerLinks(root.transform, out var baseLink, out var acousticLink,
                                            out refusal))
                {
                    Debug.LogError("[StationBuilder] " + refusal + " (in the opened contents, which " +
                                   "the asset itself passed — that mismatch is itself a defect.)");
                    return;
                }

                // A component that edited the contents on the way in would show up HERE, as a
                // difference between the asset's authored value and what the contents came up
                // holding. Report it and STOP: writing on top of it would bake another writer's
                // number into the asset while calling it authoring.
                if ((acousticLink.localPosition - authoredPos).sqrMagnitude > 1e-8f)
                {
                    Debug.LogError("[StationBuilder] " +
                                   SecondWriterMessage(authoredPos, acousticLink.localPosition));
                    return;
                }

                acousticLink.localPosition = AuthoredAcousticLocalPosition;

                var dep = acousticLink.GetComponent<DeployedTransducer>()
                          ?? acousticLink.gameObject.AddComponent<DeployedTransducer>();
                dep.DipDepthM = AuthoredDipDepthM;
                dep.MinDipDepthM = AuthoredMinDipDepthM;
                dep.PracticeDipDepthM = AuthoredPracticeDipDepthM;
                dep.AutoDeployInEditor = AuthoredAutoDeployInEditor;
                dep.CaseAnchor = baseLink;

                var modem = acousticLink.GetComponentInChildren<AcousticModem>(true);
                if (modem != null) modem.requireWet = AuthoredRequireWet;

                PrefabUtility.SaveAsPrefabAsset(root, StationPrefabPath);
            }
            finally { PrefabUtility.UnloadPrefabContents(root); }

            AssetDatabase.SaveAssets();
            AssetDatabase.ImportAsset(StationPrefabPath, ImportAssetOptions.ForceSynchronousImport);

            // ---- 4. SELF-CHECK: RUN THE PLANNER AGAINST THE STATE IT MUST NOW PASS ------------
            // The previous version CLAIMED "a second press must say nothing to change" and nobody
            // tested it; that claim is what failed, six presses running. So the menu item now
            // performs the second press itself, on the file it just wrote, and says which it got.
            string selfCheck = SecondPressWouldSayNothing(out var residue)
                ? "\nSELF-CHECK PASSED: re-planned against the freshly written asset and there is " +
                  "nothing left to change, so a second press will say 'nothing to change'. " +
                  "(Press it again anyway — pressing a builder twice is the cheapest idempotence " +
                  "test there is.)"
                : "\nSELF-CHECK FAILED — THIS MENU ITEM IS NOT IDEMPOTENT. After writing, the " +
                  "planner still wants:" + Bullets(residue) +
                  "\nDo NOT press it again to 'fix' that: something is writing this value besides " +
                  "this builder, and each press will rewrite the prefab forever (SETTLED §3s8).";

            Debug.Log("[StationBuilder] Updated " + StationPrefabPath + " SURGICALLY. Changed:" +
                      Bullets(plan) +
                      "\nUNTOUCHED: GPS + GPS_Pub, WiFi/cellular modems, CommsManager, the case " +
                      "visual, the [robot] tag, and every placement of this prefab in every scene — " +
                      "your Beckholmen instance keeps its position and inherits the fix." +
                      selfCheck +
                      "\nVERIFY WITHOUT PLAY: select the station in the Hierarchy and look at the " +
                      "Scene view. You want a cyan drop line from the case through the water line " +
                      "to a dot, labelled 'transducer 0.50 m below water line'. Red and 'DRY' means " +
                      "the acoustic link is down and will say why.");
        }

        /// <summary>Both links, or a refusal naming which one is missing. Never creates them: other
        /// code looks these up BY STRING, so inventing one here would produce a station that reports
        /// itself complete and answers to a name nothing else uses.</summary>
        static bool TryFindTransducerLinks(Transform root, out Transform baseLink,
                                           out Transform acousticLink, out string refusal)
        {
            acousticLink = null;
            refusal = "";
            baseLink = root.Find("base_link");
            if (baseLink == null)
            {
                refusal = "The prefab has no base_link — refusing to guess where the transducer " +
                          "hangs from. Nothing written.";
                return false;
            }
            acousticLink = baseLink.Find("acoustic_link");
            if (acousticLink == null)
            {
                refusal = "The prefab has no base_link/acoustic_link — refusing to create a link " +
                          "whose name other code looks up by string. Nothing written.";
                return false;
            }
            return true;
        }

        /// <summary>
        /// What this updater would change, and nothing else. ONE authored state (the Authored*
        /// constants) is compared against what the prefab holds; a field that already matches is
        /// not "corrected", not logged, and not counted as work.
        ///
        /// It is deliberately blind to `DeployedTransducer`'s read-only outputs (Deployed,
        /// ActualDepthM, RefusalReason). Those are the component's own live measurements, they are
        /// `[System.NonSerialized]` precisely so they cannot be baked into an asset, and a builder
        /// that treated a component's own legitimate output as a defect to correct would be the
        /// 2026-08-21 defect again with a different field in the fight.
        /// </summary>
        static List<string> PlanTransducerChanges(Transform baseLink, Transform acousticLink)
        {
            var plan = new List<string>();

            var pos = acousticLink.localPosition;
            if ((pos - AuthoredAcousticLocalPosition).sqrMagnitude > 1e-8f)
                plan.Add($"acoustic_link localPosition {pos} -> {AuthoredAcousticLocalPosition}. That Y " +
                         "was a FIXED OFFSET FROM THE CASE, and the case is a metre above the water on " +
                         "the dock bridge — so the transducer was modelled in the air. Y is now owned " +
                         "by DeployedTransducer and taken from the water plane.");

            var dep = acousticLink.GetComponent<DeployedTransducer>();
            if (dep == null)
                plan.Add($"add DeployedTransducer to acoustic_link: dip {AuthoredDipDepthM:F2} m below " +
                         "the STILL-WATER PLANE (Ivan, 2026-08-21: \"we always drop the transducer at " +
                         $"least 50cm down below the water level\"), minimum {AuthoredMinDipDepthM:F2} m, " +
                         "case anchor = base_link.");
            else if (dep.DipDepthM != AuthoredDipDepthM ||
                     dep.MinDipDepthM != AuthoredMinDipDepthM ||
                     dep.PracticeDipDepthM != AuthoredPracticeDipDepthM ||
                     dep.AutoDeployInEditor != AuthoredAutoDeployInEditor ||
                     dep.CaseAnchor != baseLink)
                plan.Add($"DeployedTransducer settings -> dip {AuthoredDipDepthM:F2} m below the " +
                         $"still-water plane, minimum {AuthoredMinDipDepthM:F2} m, practice " +
                         $"{AuthoredPracticeDipDepthM:F2} m, auto-deploy {AuthoredAutoDeployInEditor}, " +
                         "case anchor = base_link.");

            var modem = acousticLink.GetComponentInChildren<AcousticModem>(true);
            if (modem == null)
                Debug.LogWarning("[StationBuilder] No AcousticModem under acoustic_link — the " +
                                 "deployment rig is in place but nothing is using it. Check the prefab.");
            else if (modem.requireWet != AuthoredRequireWet)
                plan.Add($"{modem.name}.requireWet {modem.requireWet} -> {AuthoredRequireWet}. The old " +
                         "false was there because the case is topside; the transducer is not the case. " +
                         "With the fish actually at depth the wetness test is real, and an undeployed " +
                         "station reports 'transducer above the water line — not deployed' instead of a " +
                         "link that cannot say no.");

            return plan;
        }

        /// <summary>Re-plan against the asset as it now stands on disk. This IS the second press,
        /// performed by the first one.</summary>
        static bool SecondPressWouldSayNothing(out List<string> residue)
        {
            residue = new List<string>();
            var asset = AssetDatabase.LoadAssetAtPath<GameObject>(StationPrefabPath);
            if (asset == null)
            {
                residue.Add("the prefab could not be re-loaded after saving — self-check inconclusive.");
                return false;
            }
            if (!TryFindTransducerLinks(asset.transform, out var b, out var a, out string refusal))
            {
                residue.Add(refusal);
                return false;
            }
            residue = PlanTransducerChanges(b, a);
            return residue.Count == 0;
        }

        /// <summary>
        /// Open the prefab and see whether anything MOVES acoustic_link while it is open. Nothing
        /// is saved; the contents are unloaded either way.
        ///
        /// This is the trap for the defect of 2026-08-21: `DeployedTransducer` is `[ExecuteAlways]`,
        /// so it also runs inside the preview scene `LoadPrefabContents` opens, and the first
        /// version searched every loaded scene for water — finding the OPEN SITE SCENE's, and
        /// writing waterPlaneY − dip into the prefab contents. Returns null when clean.
        /// </summary>
        static string ProbeForSecondWriter(Vector3 authoredPos)
        {
            var root = PrefabUtility.LoadPrefabContents(StationPrefabPath);
            try
            {
                if (!TryFindTransducerLinks(root.transform, out _, out var acousticLink, out string refusal))
                    return refusal;
                var seen = acousticLink.localPosition;
                if ((seen - authoredPos).sqrMagnitude <= 1e-8f) return null;
                return SecondWriterMessage(authoredPos, seen);
            }
            finally { PrefabUtility.UnloadPrefabContents(root); }
        }

        static string SecondWriterMessage(Vector3 authored, Vector3 seen)
            => $"TWO WRITERS ON acoustic_link.localPosition. The prefab ASSET holds {authored}, but " +
               $"opening it produced {seen} — something moved it while the prefab was open, which " +
               "means this builder and that something are fighting over one value and every press " +
               "would rewrite the prefab. NOTHING WAS WRITTEN. The known cause is an [ExecuteAlways] " +
               "component deploying against ANOTHER open scene's water from inside the prefab preview " +
               "scene; DeployedTransducer now refuses that by name (SETTLED §3s8). Check what else " +
               "runs on acoustic_link.";

        static string Bullets(List<string> lines)
        {
            var sb = new System.Text.StringBuilder();
            foreach (var l in lines) sb.Append("\n  * ").Append(l);
            return sb.ToString();
        }

        /// <summary>
        /// Re-skin the case WITHOUT touching anything else — ADR-007's prefab is already placed
        /// in a scene (Ivan put it on Milou's fore deck), and rebuilding the whole prefab would
        /// replace the instance he positioned. This swaps only the `Visual` child, so the GPS,
        /// the modems, the links and the placement all survive.
        ///
        /// The general rule this follows: a builder that can only regenerate everything is a
        /// builder nobody dares run twice.
        /// </summary>
        [MenuItem("SMARC/Restyle Data Cube Station Case")]
        public static void RestyleCase()
        {
            var asset = AssetDatabase.LoadAssetAtPath<GameObject>(StationPrefabPath);
            if (asset == null)
            {
                Debug.LogError("[StationBuilder] No station prefab at " + StationPrefabPath +
                               " — run 'Build Data Cube Station Prefab' first.");
                return;
            }
            var root = PrefabUtility.LoadPrefabContents(StationPrefabPath);
            try
            {
                var baseLink = root.transform.Find("base_link");
                if (baseLink == null)
                {
                    Debug.LogError("[StationBuilder] The prefab has no base_link — refusing to " +
                                   "guess where the case should hang.");
                    return;
                }
                var old = baseLink.Find("Visual");
                if (old != null) Object.DestroyImmediate(old.gameObject);
                BuildCaseVisual(baseLink);
                PrefabUtility.SaveAsPrefabAsset(root, StationPrefabPath);
                Debug.Log("[StationBuilder] Case restyled — Peli iM2400 proportions, exterior " +
                          $"{CaseLen * 100f:F1} x {CaseWid * 100f:F1} x {CaseHgt * 100f:F1} cm " +
                          $"around a measured {TubLen * 1000f:F0} x {TubWid * 1000f:F0} mm cavity " +
                          "(from Peli's own STEP vertices). Everything else in the prefab — GPS, " +
                          "modems, links, and its placement in your scene — is untouched.");
            }
            finally { PrefabUtility.UnloadPrefabContents(root); }
            AssetDatabase.Refresh();
        }

        /// <summary>
        /// A Peli iM2400 in proportion: base, shallower lid, the rim they meet on, a side handle
        /// and two Press &amp; Pull latches. Built from primitives rather than a mesh because the
        /// real CAD is a NURBS B-rep (Peli's own STEP files, 116 spline curves) that Unity cannot
        /// import without a FreeCAD/Fusion pass — see the dimension block for what those files
        /// did contribute, which is the numbers.
        ///
        /// NO COLLIDERS ON ANY OF IT. The station sits on a boat deck and must never become
        /// something the vehicle's sonar or obstacle stack reacts to — the same rule the waypoint
        /// hoops learned this morning (SETTLED 3o) and that 3g recorded for trigger volumes.
        /// </summary>
        static void BuildCaseVisual(Transform parent)
        {
            // Shiny injection-moulded plastic (Ivan, 2026-08-18: "nice shiny plastic yellow").
            // High smoothness on the body, lower on the trim — the latches and handle are a
            // different, less glossy polymer on the real case and a uniformly shiny box reads
            // as one moulding rather than an assembly.
            var body = GetOrCreateMat(CaseBodyMatPath, PeliYellow, 0.80f);
            var trim = GetOrCreateMat(CaseTrimMatPath, CaseTrim, 0.45f);

            var visual = new GameObject("Visual");
            visual.transform.SetParent(parent, false);

            float baseH = CaseHgt - LidHgt;
            Part(visual.transform, "Base", body,
                 new Vector3(0f, baseH * 0.5f, 0f), new Vector3(CaseLen, baseH, CaseWid));
            Part(visual.transform, "Lid", body,
                 new Vector3(0f, baseH + LidHgt * 0.5f, 0f), new Vector3(CaseLen, LidHgt, CaseWid));
            // The rim stands slightly proud of both halves, which is what actually reads as
            // "Peli case" at a glance rather than "grey box".
            Part(visual.transform, "Rim", trim,
                 new Vector3(0f, baseH, 0f), new Vector3(CaseLen * 1.015f, 0.012f, CaseWid * 1.015f));

            // Handle on one long side, at the seam height where it really sits.
            Part(visual.transform, "Handle", trim,
                 new Vector3(0f, baseH - 0.01f, CaseWid * 0.5f + 0.018f),
                 new Vector3(CaseLen * 0.30f, 0.030f, 0.036f));

            // Two Press & Pull latches, on the same face as the handle.
            foreach (var sx in new[] { -1f, 1f })
                Part(visual.transform, sx < 0 ? "Latch_L" : "Latch_R", trim,
                     new Vector3(sx * CaseLen * 0.30f, baseH, CaseWid * 0.5f + 0.010f),
                     new Vector3(0.055f, 0.045f, 0.020f));
        }

        static void Part(Transform parent, string name, Material mat, Vector3 pos, Vector3 scale)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = pos;
            go.transform.localScale = scale;
            Object.DestroyImmediate(go.GetComponent<Collider>());
            go.GetComponent<MeshRenderer>().sharedMaterial = mat;
        }

        /// <summary>Materials must be ASSETS to survive in a prefab: a `new Material()` created in
        /// a builder is a scene-only object and the saved prefab would come back with the pink
        /// missing-material shader. Reused if already present, so restyling twice does not litter
        /// the project with duplicates.</summary>
        static Material GetOrCreateMat(string path, Color color, float smoothness)
        {
            var existing = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (existing != null) return existing;

            // Named in fallback order rather than assuming a pipeline: this project is HDRP, but
            // a material that silently renders black is worse than one that renders plain.
            var shader = Shader.Find("HDRP/Lit")
                      ?? Shader.Find("Universal Render Pipeline/Lit")
                      ?? Shader.Find("Standard");
            var mat = new Material(shader);
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", color);
            if (mat.HasProperty("_Color")) mat.SetColor("_Color", color);
            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", smoothness);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", smoothness);
            AssetDatabase.CreateAsset(mat, path);
            return mat;
        }

        /// <summary>Tags are project settings, not asset content: a tag that does not exist
        /// cannot be set from script. Say so loudly rather than leaving a station whose GPS
        /// disables itself on Attach() for reasons nobody can see.</summary>
        static void TrySetRobotTag(GameObject go)
        {
            try { go.tag = "robot"; }
            catch (UnityException)
            {
                Debug.LogWarning("[StationBuilder] The tag [robot] does not exist in this project, " +
                                 "so the station root is untagged. LinkAttachment walks up for a " +
                                 "parent tagged [robot] and will DISABLE the GPS without it. Add the " +
                                 "tag in Project Settings > Tags and Layers, then re-run this builder.");
            }
        }

    }
}
