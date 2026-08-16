using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

using GeoRef;

/// <summary>
/// In-editor acceptance checks for the Kristineberg site — the half that cannot be
/// checked outside Unity. The data-side checks live in
/// data-cube/scripts/kristineberg-site/verify_site.py and must pass too; this one
/// asks whether UNITY agrees with them.
///
/// Specifically it re-measures through Unity's own machinery (CoordinateSharp inside
/// GlobalReferencePoint, and the TerrainCollider), because a pipeline can be perfectly
/// correct and still be wired into the scene wrong.
///
/// Run from menu: SMARC -> Verify Kristineberg Site
/// </summary>
public static class KristinebergSiteVerifier
{
    const string DataDir = "Packages/com.smarc.assets/Runtime/Terrain/Kristineberg";

    [MenuItem("SMARC/Verify Kristineberg Site")]
    public static void Verify()
    {
        int fails = 0;
        void Check(string name, bool ok, string measured, string tol)
        {
            if (ok) Debug.Log($"[Kristineberg PASS] {name}: {measured}   (tolerance: {tol})");
            else { Debug.LogError($"[Kristineberg FAIL] {name}: {measured}   (tolerance: {tol})"); fails++; }
        }

        var manifestPath = KristinebergSiteBuilder.AssetPathToFullPublic(DataDir + "/kristineberg_unity.json");
        if (!File.Exists(manifestPath))
        {
            Debug.LogError("[Kristineberg] no manifest at " + manifestPath + " — run the pipeline and the builder first.");
            return;
        }
        var m = JsonUtility.FromJson<KristinebergSiteBuilder.Manifest>(File.ReadAllText(manifestPath));

        var grefs = UnityEngine.Object.FindObjectsByType<GlobalReferencePoint>(FindObjectsSortMode.None);
        Check("exactly one GlobalReferencePoint in the scene", grefs.Length == 1,
              $"{grefs.Length} found", "exactly 1");
        if (grefs.Length == 0) { Debug.LogError($"[Kristineberg] {fails} failure(s)."); return; }
        var gref = grefs[0];

        // 1. Unity's own UTM conversion must agree with the pipeline's pyproj one.
        Check("UTM zone/band", gref.UTMZone == m.utmZone && gref.UTMBand == m.utmBand,
              $"Unity {gref.UTMZone}{gref.UTMBand} vs pipeline {m.utmZone}{m.utmBand}", "identical");
        double de = gref.UTMEasting - m.anchorUtmEasting;
        double dn = gref.UTMNorthing - m.anchorUtmNorthing;
        Check("CoordinateSharp vs pyproj at the anchor", Math.Abs(de) < 1.0 && Math.Abs(dn) < 1.0,
              $"({de:F3}, {dn:F3}) m", "< 1 m in each axis");

        // 2. Each surveyed buoy marker must report back its surveyed lat/lon.
        //    This exercises transform position -> UTM -> WGS84 through Unity, which is
        //    what every mission plan and every logged track will use.
        double worst = 0;
        foreach (var b in m.buoys)
        {
            var go = GameObject.Find("Buoy_" + b.name);
            if (go == null) { Check($"buoy marker {b.name} present", false, "not found", "in scene"); continue; }
            var p = go.transform.position;
            var (e, n, lat, lon) = gref.GetUTMLatLonOfObject(go);
            // expected UTM = anchor + the marker's own scene offset
            double ee = m.anchorUtmEasting + b.x, en = m.anchorUtmNorthing + b.z;
            double d = Math.Sqrt((e - ee) * (e - ee) + (n - en) * (n - en));
            worst = Math.Max(worst, d);
            Debug.Log($"[Kristineberg]   {b.name}: scene ({p.x:F2}, {p.z:F2}) -> UTM ({e:F1}, {n:F1}) " +
                      $"-> {lat:F6}, {lon:F6}   [{d:F2} m from surveyed]");
        }
        Check("surveyed buoys round-trip through GlobalReferencePoint", worst < 2.0,
              $"worst {worst:F2} m", "< 2 m");

        // 3. The terrain must actually be under the buoys, at the depth the data says.
        var terrain = UnityEngine.Object.FindFirstObjectByType<Terrain>();
        Check("terrain present", terrain != null, terrain != null ? terrain.name : "none", "one Terrain");
        if (terrain != null)
        {
            Check("terrain is unscaled", terrain.transform.lossyScale == Vector3.one,
                  terrain.transform.lossyScale.ToString(), "(1,1,1) — never scale a georeferenced terrain");
            Check("terrain size", Mathf.Abs(terrain.terrainData.size.x - m.sizeX) < 0.01f &&
                                  Mathf.Abs(terrain.terrainData.size.z - m.sizeZ) < 0.01f,
                  terrain.terrainData.size.ToString(), $"{m.sizeX} x {m.sizeZ} m");

            float minD = float.MaxValue, maxD = float.MinValue;
            foreach (var b in m.buoys)
            {
                float y = terrain.SampleHeight(new Vector3(b.x, 0, b.z)) + terrain.transform.position.y;
                minD = Mathf.Min(minD, y); maxD = Mathf.Max(maxD, y);
            }
            Check("seabed under the buoys", maxD < -3f,
                  $"{minD:F2} .. {maxD:F2} m", "all below -3 m (matches the -8..-10 m in the source contours)");

            // Splat orientation: at the buoys the terrain is 8-10 m under water, so the
            // seabed layer must dominate there. A flipped or transposed control map passes
            // every visual glance on a roughly symmetric site and fails this.
            var td2 = terrain.terrainData;
            if (td2.terrainLayers != null && td2.terrainLayers.Length >= 1)
            {
                // Resolve layers BY NAME, not by index. The scheme grew from 4 classes to 6
                // when sand and mud were split out, and an index-based check silently starts
                // asking about the wrong layer the moment the list changes.
                int Idx(string nm)
                {
                    for (int i = 0; i < td2.terrainLayers.Length; i++)
                        if (td2.terrainLayers[i] != null &&
                            td2.terrainLayers[i].name.IndexOf(nm, StringComparison.OrdinalIgnoreCase) >= 0)
                            return i;
                    return -1;
                }
                int iSand = Idx("sand"), iMud = Idx("mud"), iGrass = Idx("grass");
                int an = td2.alphamapResolution;
                var alpha = td2.GetAlphamaps(0, 0, an, an);
                float worstSoft = 1f, worstGrass = 0f;
                foreach (var b in m.buoys)
                {
                    int ax = Mathf.Clamp(Mathf.RoundToInt((b.x - m.posX) / m.sizeX * (an - 1)), 0, an - 1);
                    int ay = Mathf.Clamp(Mathf.RoundToInt((b.z - m.posZ) / m.sizeZ * (an - 1)), 0, an - 1);
                    float soft = (iSand >= 0 ? alpha[ay, ax, iSand] : 0f)
                               + (iMud >= 0 ? alpha[ay, ax, iMud] : 0f);
                    worstSoft = Mathf.Min(worstSoft, soft);
                    if (iGrass >= 0) worstGrass = Mathf.Max(worstGrass, alpha[ay, ax, iGrass]);
                }
                Check("soft bottom (sand+mud) dominates under the buoys", worstSoft > 0.6f,
                      $"min sand+mud weight {worstSoft:F2}", "> 0.6 at 8-10 m depth");
                Check("no land grass under the buoys", worstGrass < 0.05f,
                      $"max grass weight {worstGrass:F2}", "< 0.05; grass at -9 m means a flipped control map");
            }

            // A raycast is the thing the sonar and the bottom profiler actually do.
            var probe = new Vector3(m.buoys[0].x, 50f, m.buoys[0].z);
            bool hit = Physics.Raycast(probe, Vector3.down, out var hitInfo, 200f);
            Check("TerrainCollider returns raycast hits", hit,
                  hit ? $"hit {hitInfo.collider.name} at y={hitInfo.point.y:F2}" : "NO HIT",
                  "a collider that returns nothing reads as 'obstacle clear'");
        }

        // 3b. The vehicle must be present, floating, and in genuinely navigable water —
        //     the whole point of the harbour carve.
        if (m.launch != null && !string.IsNullOrEmpty(m.launch.vehicle_name))
        {
            var v = GameObject.Find(m.launch.vehicle_name);
            Check("vehicle present in the scene", v != null,
                  v != null ? v.name : "not found", m.launch.vehicle_name);
            if (v != null && terrain != null)
            {
                var p = v.transform.position;
                float sea = terrain.SampleHeight(p) + terrain.transform.position.y;
                Check("vehicle is over water, not land", sea < -2.0f,
                      $"seabed {sea:F2} m under the hull", "< -2 m");
                Check("vehicle is at the surface", Mathf.Abs(p.y) < 1.0f,
                      $"y = {p.y:F2}", "|y| < 1 m of the water plane");
                // clearance: nothing solid within a hull length in any horizontal direction.
                // NOTE the name: `worst` is already a local further up (the buoy round-trip),
                // and C# forbids shadowing it here (CS0136) — which is what broke the whole
                // editor assembly and made every new menu item silently vanish.
                float worstClearance = 999f;
                for (int a = 0; a < 8; a++)
                {
                    float th = a * Mathf.PI / 4f;
                    var q = p + new Vector3(Mathf.Cos(th), 0, Mathf.Sin(th)) * 8f;
                    worstClearance = Mathf.Min(worstClearance,
                                               -(terrain.SampleHeight(q) + terrain.transform.position.y));
                }
                Check("8 m of clearance around the launch point", worstClearance > 1.0f,
                      $"shallowest neighbour {-worstClearance:F2} m", "> 1 m of water all round");
            }
            Check("GUI present", GameObject.Find("GUI") != null,
                  GameObject.Find("GUI") != null ? "ok" : "not found", "GUI prefab in the scene");
        }

        // 3c. The algae farm — position, and SONAR VISIBILITY, which is the point of it.
        var farm = GameObject.Find("KristinebergAlgaeFarm");
        if (farm != null)
        {
            // Every buoy where the survey says it is.
            var surveyed = new Dictionary<string, Vector2> {
                {"C2_corner_SW", new Vector2(238.17f, 108.30f)},
                {"C0_south_mid", new Vector2(248.17f, 110.30f)},
                {"C1_corner_SE", new Vector2(264.17f, 105.30f)},
                {"M2_east_mid",  new Vector2(263.17f, 121.30f)},
                {"M1_west_mid",  new Vector2(242.67f, 123.30f)},
                {"C4_corner_NW", new Vector2(247.17f, 138.30f)},
                {"C3_corner_NE", new Vector2(262.17f, 137.30f)},
            };
            float worstB = 0f; int found = 0;
            foreach (var kv in surveyed)
            {
                var t = farm.transform.Find("Buoys/" + kv.Key);
                if (t == null) continue;
                found++;
                worstB = Mathf.Max(worstB, Vector2.Distance(new Vector2(t.position.x, t.position.z), kv.Value));
            }
            Check("all 7 farm buoys present", found == 7, $"{found}/7", "7");
            Check("farm buoys at their surveyed positions", worstB < 1.0f,
                  $"worst {worstB:F2} m", "< 1 m");

            // THE test: does a physics ray see the farm the way the sonar will? Sonar.cs reads
            // Hit.collider.material.name, so this asks the same question the sonar asks.
            var labels = new Dictionary<string, int> { {"Rock",1},{"Mud",1},{"Algae",2},{"Buoy",3},{"Rope",4} };
            var seen = new Dictionary<string, int>();
            int unclassified = 0;
            foreach (var col in farm.GetComponentsInChildren<Collider>())
            {
                var pm = col.sharedMaterial;
                var nm = pm == null ? null : pm.name.Split('(')[0].Trim();
                if (nm != null && labels.ContainsKey(nm)) seen[nm] = seen.TryGetValue(nm, out var c) ? c + 1 : 1;
                else unclassified++;
            }
            Debug.Log("[Kristineberg] farm collider materials: " +
                      string.Join(", ", seen.Select(kv => $"{kv.Key}={kv.Value}")) +
                      $", unclassified={unclassified}");
            Check("farm colliders carry sonar-recognised materials", unclassified == 0,
                  $"{unclassified} colliders with no recognised physics material",
                  "0 — an unclassified collider returns reflectivity 0.5 and label 0");
            foreach (var need in new[] { "Buoy", "Rope", "Algae" })
                Check($"farm returns a '{need}' signature", seen.ContainsKey(need),
                      seen.TryGetValue(need, out var n) ? $"{n} colliders" : "none",
                      "at least one");

            // And an actual raycast, because a material assignment that does not survive
            // prefab serialisation would still pass the check above.
            var c0 = farm.transform.Find("Buoys/C0_south_mid");
            if (c0 != null)
            {
                // IGNORE TRIGGERS, exactly as the sonar now does. Without this the ray hit the
                // current field's trigger volume instead of the buoy — which is how that
                // whole class of defect was found.
                bool hit = Physics.Raycast(c0.position + Vector3.up * 20f, Vector3.down, out var hi,
                                           40f, ~0, QueryTriggerInteraction.Ignore);
                var hn = hit && hi.collider.sharedMaterial != null
                       ? hi.collider.sharedMaterial.name.Split('(')[0].Trim() : "(none)";
                Check("a ray onto a buoy returns the Buoy material", hit && hn == "Buoy",
                      hit ? $"hit {hi.collider.name}, material {hn}" : "no hit", "Buoy");
            }
        }
        else Debug.Log("[Kristineberg] no algae farm in the scene (build it with SMARC -> Build Kristineberg Algae Farm)");

        // 3c-bis. THE SIDE SCAN — the sensor the farm-inspection mission is built on
        //         (2026-08-16). Everything here is about one question: can this hull,
        //         in this scene, see a rope at the prior's depth from a lane it can fly?
        //         `sam_auv_v1.prefab` has no sonar at all, so the manifest points at
        //         sam2.2 while keeping the instance NAME `sam_auv_v1` (ROS namespace,
        //         and `.` is illegal in ROS names). A farm mission on a hull with no SSS
        //         would run, scan nothing, and report an empty farm.
        if (m.sss != null && m.launch != null && !string.IsNullOrEmpty(m.launch.vehicle_name))
        {
            var veh = GameObject.Find(m.launch.vehicle_name);
            if (veh == null)
            {
                Check("side scan: vehicle present", false, "no vehicle in the scene",
                      m.launch.vehicle_name);
            }
            else
            {
                var sonars = veh.GetComponentsInChildren<VehicleComponents.Sensors.Sonar>(true);
                var sss = sonars.FirstOrDefault(s => s.Type == VehicleComponents.Sensors.SonarType.SSS);
                Check("vehicle carries a side scan sonar", sss != null,
                      sss != null ? $"{sss.transform.name} (of {sonars.Length} sonar(s))"
                                  : $"{sonars.Length} sonar(s), none of Type SSS",
                      "one Sonar with Type = SSS — sam_auv_v1.prefab has none, use sam2.2");

                if (sss != null)
                {
                    Check("side scan is interferometric", sss.isInterferometric == m.sss.requireInterferometric,
                          sss.isInterferometric ? "on" : "off",
                          "on — the per-bin angles are what turn a ping into a 3D detection");

                    // The beam, in the SAME convention Sonar.cs casts it in:
                    //   theta from nadir spans [90 - tilt - breadth, 90 - tilt].
                    // Recomputed here from the LIVE component, so a prefab that was never
                    // re-saved is caught rather than assumed.
                    float thetaMin = Mathf.Max(0f, 90f - sss.TiltAngleDeg - sss.BeamBreadthDeg);
                    float thetaMax = Mathf.Min(90f, 90f - sss.TiltAngleDeg);
                    Debug.Log($"[Kristineberg] side scan beam: tilt {sss.TiltAngleDeg:F1} deg, breadth " +
                              $"{sss.BeamBreadthDeg:F1} deg -> off-nadir {thetaMin:F1}..{thetaMax:F1} deg, " +
                              $"nadir gap {2f * thetaMin:F1} deg, range {sss.MaxRange:F0} m, " +
                              $"{sss.NumBucketsPerBeam} bins ({sss.MaxRange / sss.NumBucketsPerBeam * 100f:F1} cm)");

                    // THE acceptance check, and it is a property, not a constant match.
                    Check("side scan reaches far enough off nadir to see a rope from a lane",
                          thetaMax >= m.sss.maxOffNadirMinDeg,
                          $"beam reaches {thetaMax:F1} deg off nadir; a {m.sss.laneStandoff:F1} m lane " +
                          $"standoff at a rope depth of {m.sss.ropeDepth:F1} m needs {m.sss.maxOffNadirMinDeg:F1} deg",
                          $">= {m.sss.maxOffNadirMinDeg:F1} deg, i.e. Sonar.TiltAngleDeg <= " +
                          $"{90f - m.sss.maxOffNadirMinDeg:F1}. A side scan cannot see anything at or " +
                          "above its own depth; see farm_prior.yaml sonar.lanes and " +
                          "sam_farm_inspection/sss_geometry.py");
                    Check("side scan nadir gap is narrow enough for the same lane",
                          thetaMin <= m.sss.minOffNadirMaxDeg,
                          $"nadir gap edge at {thetaMin:F1} deg off nadir; the lane needs <= {m.sss.minOffNadirMaxDeg:F1}",
                          $"<= {m.sss.minOffNadirMaxDeg:F1} deg — a wide nadir gap puts the rope in the blind cone");

                    // The publisher, because a sonar nobody publishes is a sonar the
                    // detector cannot read. Topic compared against the manifest so the
                    // node's subscription and the prefab cannot drift apart.
                    string sssTopic = null;
                    foreach (var mb in sss.GetComponents<MonoBehaviour>())
                    {
                        if (mb == null) continue;
                        var f = mb.GetType().GetField("topic");
                        if (f == null || f.FieldType != typeof(string)) continue;
                        var t = (string)f.GetValue(mb);
                        if (!string.IsNullOrEmpty(t) && t.Contains("sidescan")) sssTopic = t;
                    }
                    Check("side scan is published on the topic the detector subscribes to",
                          sssTopic == m.sss.topic,
                          sssTopic ?? "no publisher with a 'sidescan' topic on the sonar object",
                          m.sss.topic);
                }
            }

            if (!m.sss.asShippedFlyable)
                Debug.LogWarning("[Kristineberg] the manifest recorded the shipped beam as UNFLYABLE for " +
                                 "this lane when it was generated: " + m.sss.asShippedNote);
        }

        // 3d. Seabed dressing + the current field.
        var seabed = GameObject.Find("KristinebergSeabed");
        if (seabed != null)
        {
            var byMat = new Dictionary<string, int>();
            int unclassified = 0, below = 0, total = 0;
            foreach (var col in seabed.GetComponentsInChildren<Collider>())
            {
                if (col is BoxCollider bc && bc.isTrigger) continue;      // the current volume
                total++;
                var pm = col.sharedMaterial;
                var nm = pm == null ? null : pm.name.Split('(')[0].Trim();
                if (nm == "Algae" || nm == "Rock") byMat[nm] = byMat.TryGetValue(nm, out var c) ? c + 1 : 1;
                else unclassified++;
                if (col.transform.position.y < 0f) below++;
            }
            Debug.Log($"[Kristineberg] seabed dressing: {total} colliders, " +
                      string.Join(", ", byMat.Select(kv => $"{kv.Key}={kv.Value}")));
            Check("dressing colliders are sonar-classified", unclassified == 0,
                  $"{unclassified} unclassified of {total}", "0");
            Check("dressing is underwater", total == 0 || below == total,
                  $"{below}/{total} below the water plane", "all of them");

            // The seabed itself must have a signature too, else every ping off the bottom
            // returns the 0.5 default and label 0.
            if (terrain != null)
            {
                var tc = terrain.GetComponent<TerrainCollider>();
                var tn = tc != null && tc.sharedMaterial != null
                       ? tc.sharedMaterial.name.Split('(')[0].Trim() : "(none)";
                Check("terrain carries a sonar material", tn == "Mud" || tn == "Rock",
                      tn, "Mud (0.20) or Rock (0.80), not unclassified");
            }

            // THE SAFETY PROPERTY: the current must be off unless someone turned it on.
            var cf = seabed.GetComponentInChildren<Smarc.Environment.CurrentField>(true);
            Check("current field present", cf != null, cf != null ? "yes" : "missing", "one");
            if (cf != null)
            {
                Check("CURRENT IS OFF BY DEFAULT", !cf.CurrentEnabled,
                      cf.CurrentEnabled ? $"ON at {cf.SpeedMS} m/s bearing {cf.HeadingDeg}" : "off",
                      "off — it changes vehicle dynamics and invalidates controller comparisons");
                Check("current flow vector is zero while disabled", cf.FlowVector.magnitude < 1e-6f,
                      $"{cf.FlowVector.magnitude:F3} m/s", "0 when disabled");
                var ff = cf.GetComponent<Force.ForceFieldStatic>();
                Check("force field magnitude is zero while disabled",
                      ff != null && Mathf.Abs(ff.ForceMagnitude) < 1e-4f,
                      ff == null ? "no ForceFieldStatic" : $"{ff.ForceMagnitude:F3}", "0");
                // "Off" must mean OUT OF THE PHYSICS SCENE. A live trigger volume fired
                // OnTriggerStay into ForcePoint.ApplyForce (NullReferenceException storm) and,
                // worse, returned sonar echoes across the entire farm.
                var ffc = cf.GetComponent<Collider>();
                Check("current volume is out of physics while disabled",
                      (ff == null || !ff.enabled) && (ffc == null || !ffc.enabled),
                      $"component {(ff != null && ff.enabled ? "enabled" : "disabled")}, " +
                      $"collider {(ffc != null && ffc.enabled ? "enabled" : "disabled")}",
                      "both disabled — otherwise it pushes ForcePoints and echoes on sonar");
            }
        }
        else Debug.Log("[Kristineberg] no seabed dressing (run SMARC -> Build Kristineberg Seabed)");

        // 4. The dock is the geometry a heightmap cannot hold — it must be there.
        var dock = GameObject.Find("Kristineberg_dock");
        Check("dock present with a collider", dock != null && dock.GetComponent<MeshCollider>() != null,
              dock == null ? "not found" : "ok", "MeshCollider on the quay");
        if (dock != null)
        {
            var bb = dock.GetComponent<MeshFilter>().sharedMesh.bounds;
            Check("dock straddles the water plane", bb.min.y < 0f && bb.max.y > 0f,
                  $"y {bb.min.y:F2}..{bb.max.y:F2}", "crosses y = 0");
        }

        Debug.Log(fails == 0
            ? "[Kristineberg] ALL IN-EDITOR CHECKS PASSED"
            : $"[Kristineberg] {fails} FAILURE(S) — see the errors above.");
    }
}
