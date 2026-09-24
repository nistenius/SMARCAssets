// SamTightColliderBuilder — replace sam21's single fitted capsule with convex colliders that follow the
// real hull parts (2026-09-23, docking session, Ivan: "a tighter collider that more closely follows the
// real hull").
//
// WHY. The only solid collider on sam21.strips is `Collisions` — ONE CapsuleCollider, r 0.0696 m,
// height 1.318 m. It was bounds-fitted to the visual mesh, so its radius is set by the side-scan pods,
// and it is round where the vehicle is not. Measured from the meshes this tool uses
// (SMARCAssets/Runtime/URDF/old/sam/mesh/, 2026-09-23, radii about the hull axis):
//     bare hull cylinder (mesh-a31749b0)        r 0.0631   (D 0.126 — the 125 mm hull)
//     side-scan pods, both flanks (acdc/a0da)   r 0.0768   |y| <= 0.0701, z -0.046..-0.018 (below axis)
//     TOP RAIL (mesh-ec7ff5ff), full length      r 0.0953   z 0.054..0.095, |y| <= 0.0198
//     antenna dome marker (a19ec034)            r 0.0931
//     nozzle duct (SAM_NOZZLE.dae)              r 0.0689   aft of the hull, x -0.089..-0.013
// So the capsule is 7 mm too FAT on the bare hull and 26 mm too THIN over the top rail: it lets the
// rail pass through a dock wall and blocks contacts the real hull would not make.
//
// WHAT IT DOES. On the SELECTED vehicle instance in the open scene (a scene override — the prefab is
// untouched; apply to prefab only after Ivan has seen the gate numbers):
//   1. finds every MeshFilter under it whose mesh comes from SAM_HULL.dae or SAM_NOZZLE.dae;
//   2. for each, adds a CONVEX MeshCollider on a new child of the `Collisions` object's parent, at the
//      mesh's exact world pose — so every collider belongs to the SAME ArticulationBody as the capsule
//      (base_link). The nozzle deflects <= 7 deg; a base_link-fixed duct is off by <= ~5 mm at the rim,
//      and it avoids link-vs-link self-collision inside the articulation;
//   3. disables (does not delete) the capsule; copies its PhysicMaterial;
//   4. logs each collider's bounds and the envelope radius about the capsule axis.
// Inertia is NOT affected: every ArticulationBody on sam21.strips has m_ImplicitTensor 0 (explicit
// tensors, checked 2026-09-23). Hydrodynamics are ForcePoints/SAMHydrodynamicsV2 — not colliders.
// Convex cooking caps each hull at 255 polygons; on a 63 mm cylinder that facets by < 0.1 mm.
//
// GATE (Play): Console line "TIGHT COLLIDER" lists N colliders; then Ivan runs
// SMARC ▸ Docking ▸ Probe hull collider (raycasts from 0.5 m out at 12 angles x 5 stations and prints
// the hit radius per ray: expect ~0.063 on the bare hull, ~0.077 at the pods, ~0.095 over the rail;
// a ray with NO hit on a station that should hit = a dead cook (the MeshCollider trap), fail.)
//
// UNDO: Edit ▸ Undo, or re-enable `Collisions` capsule and delete `TightColliders`.
// Persists in: the SCENE (KTHTankDock.unity) as an override on the sam21 instance.

using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace SmarcEditor
{
    public static class SamTightColliderBuilder
    {
        static readonly string[] Sources = { "SAM_HULL.dae", "SAM_NOZZLE.dae" };
        const string GroupName = "TightColliders";

        [MenuItem("SMARC/Docking/Fit tight hull colliders (selected sam21)")]
        public static void Build()
        {
            var root = Selection.activeGameObject;
            if (root == null) { Debug.LogError("TIGHT COLLIDER: select the sam21 vehicle root in the Hierarchy first."); return; }

            var capsule = root.GetComponentsInChildren<CapsuleCollider>(true).FirstOrDefault(c => !c.isTrigger && c.gameObject.name == "Collisions");
            if (capsule == null) { Debug.LogError($"TIGHT COLLIDER: no non-trigger CapsuleCollider on a GameObject named 'Collisions' under {root.name}."); return; }
            var owner = capsule.transform.parent != null ? capsule.transform.parent : capsule.transform;

            var old = owner.Find(GroupName);
            if (old != null) Undo.DestroyObjectImmediate(old.gameObject);
            var group = new GameObject(GroupName);
            Undo.RegisterCreatedObjectUndo(group, "Tight colliders");
            group.transform.SetParent(owner, false);

            var filters = root.GetComponentsInChildren<MeshFilter>(true)
                .Where(f => f.sharedMesh != null && Sources.Any(s => AssetDatabase.GetAssetPath(f.sharedMesh).EndsWith(s)))
                .ToList();
            if (filters.Count == 0) { Debug.LogError("TIGHT COLLIDER: no MeshFilter from SAM_HULL.dae / SAM_NOZZLE.dae under the selection — wrong object?"); return; }

            var lines = new List<string>();
            foreach (var f in filters)
            {
                var go = new GameObject("tc_" + f.sharedMesh.name);
                Undo.RegisterCreatedObjectUndo(go, "Tight colliders");
                go.transform.SetParent(f.transform, false);          // exact pose + scale of the visual mesh
                go.transform.SetParent(group.transform, true);        // re-home onto base_link's hierarchy
                var mc = Undo.AddComponent<MeshCollider>(go);
                mc.sharedMesh = f.sharedMesh;
                mc.convex = true;
                mc.sharedMaterial = capsule.sharedMaterial;
                var b = mc.bounds;
                lines.Add($"  {go.name,-34} from {AssetDatabase.GetAssetPath(f.sharedMesh).Split('/').Last(),-15} bounds size {b.size.x:F3} {b.size.y:F3} {b.size.z:F3}");
            }

            Undo.RecordObject(capsule, "Tight colliders");
            capsule.enabled = false;

            // envelope about the capsule axis (capsule local direction), sampled from collider vertices
            var axisLocal = capsule.direction == 0 ? Vector3.right : capsule.direction == 1 ? Vector3.up : Vector3.forward;
            var axisW = capsule.transform.TransformDirection(axisLocal).normalized;
            var c0 = capsule.transform.TransformPoint(capsule.center);
            float rmax = 0f;
            foreach (var mc in group.GetComponentsInChildren<MeshCollider>())
                foreach (var v in mc.sharedMesh.vertices)
                {
                    var p = mc.transform.TransformPoint(v) - c0;
                    rmax = Mathf.Max(rmax, (p - Vector3.Dot(p, axisW) * axisW).magnitude);
                }

            EditorUtility.SetDirty(root);
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(root.scene);
            Debug.Log($"TIGHT COLLIDER: {filters.Count} convex colliders under {owner.name}/{GroupName}; capsule r {capsule.radius:F4} DISABLED.\n" +
                      $"  envelope radius about hull axis = {rmax:F4} m (expect ~0.095, set by the top rail; bare hull 0.063)\n" +
                      string.Join("\n", lines) + "\n  Save the scene (Cmd+S) to keep it.");
        }

        [MenuItem("SMARC/Docking/Probe hull collider (selected sam21)")]
        public static void Probe()
        {
            var root = Selection.activeGameObject;
            if (root == null) { Debug.LogError("PROBE: select the sam21 vehicle root first."); return; }
            var capsule = root.GetComponentsInChildren<CapsuleCollider>(true).FirstOrDefault(c => c.gameObject.name == "Collisions");
            if (capsule == null) { Debug.LogError("PROBE: no 'Collisions' capsule found (needed for the axis)."); return; }
            var axisLocal = capsule.direction == 0 ? Vector3.right : capsule.direction == 1 ? Vector3.up : Vector3.forward;
            var ax = capsule.transform.TransformDirection(axisLocal).normalized;
            var c0 = capsule.transform.TransformPoint(capsule.center);
            var up = Vector3.ProjectOnPlane(Vector3.up, ax).normalized;
            var side = Vector3.Cross(ax, up);
            Physics.SyncTransforms();
            var mine = new HashSet<Collider>(root.GetComponentsInChildren<Collider>().Where(c => c.enabled && !c.isTrigger));
            var sb = new System.Text.StringBuilder("PROBE hit radius [m] per station (rows) x angle 0..330 deg, 0 = up, 90 = starboard/right:\n");
            int dead = 0;
            foreach (var s in new[] { -0.55f, -0.25f, 0f, 0.25f, 0.55f })
            {
                sb.Append($"  s={s,5:F2} ");
                for (int k = 0; k < 12; k++)
                {
                    float a = k * 30f * Mathf.Deg2Rad;
                    var dir = Mathf.Cos(a) * up + Mathf.Sin(a) * side;
                    var from = c0 + s * ax + 0.5f * dir;
                    var hits = Physics.RaycastAll(from, -dir, 0.5f, ~0, QueryTriggerInteraction.Ignore)
                                      .Where(h => mine.Contains(h.collider)).OrderBy(h => h.distance).ToList();
                    if (hits.Count == 0) { sb.Append("  ---- "); dead++; }
                    else sb.Append($" {0.5f - hits[0].distance:F3}");
                }
                sb.Append('\n');
            }
            sb.Append(dead == 0 ? "  every ray hit." : $"  {dead} rays hit NOTHING — if that is on the hull body (not beyond the ends), a collider is dead: re-run Fit, re-save, re-probe.");
            Debug.Log(sb.ToString());
        }
    }
}
