// HDRPWaterQueryModel — the CPU water-height query the buoyancy reads.
//
// WHAT WAS WRONG (diagnosed 2026-08-18, SETTLED §3s; MEASURED AGAIN 2026-09-13 and this time
// it stopped the wave campaign dead). The original body was three lines long and all three
// mattered:
//
//     parameters.startPositionWS = result.candidateLocationWS;  // the PREVIOUS caller's answer
//     parameters.maxIterations   = 6;
//     water.ProjectPointOnWaterSurface(parameters, out result); // result.error never read
//
// `result` is a single field shared by every caller, so each query was seeded from wherever
// the last one happened to land — a different point, on a different object, possibly metres
// away — and was then given six iterations to recover. Whatever it returned was accepted; the
// search reports its own horizontal `error` and `numIterations` and neither was ever looked at.
//
// WHY IT GOT WORSE, NOT BETTER, WITH THE STRIP CLOUD. The legacy buoyancy cloud made 10 chained
// queries per FixedUpdate. The 27-strip cloud makes 27, plus the probes — and every extra link
// in the chain is another seed inherited from somewhere else. On 2026-09-13 the surfaced
// vehicle reached 5.7 m/s and 20° of pitch in a 0.5 m sea with NO external force applied to it:
// neighbouring strips disagreed about where the water was, and a disagreement about the water
// between two points 1.3 m apart is not a height error, it is a moment.
//
// THE FIX (ADR-011 option A, "a bug fix worth doing under any of these"):
//   * seed every search from its OWN target position, so the result cannot depend on call order;
//   * give it a real iteration budget and a real tolerance;
//   * read `error` and `numIterations` back and COUNT the searches that did not converge, so a
//     bad query surfaces as a number instead of as a vehicle leaving the scene;
//   * READ THE RETURN VALUE. `ProjectPointOnWaterSurface` returns a bool, and the original code
//     dropped it. When it is false the CPU simulation data was not available and the struct is
//     left at its invalidation values — error = FLT_MAX and projectedPositionWS = (0,0,0) — so
//     the old code cheerfully reported "the water is at y = 0". Measured on 2026-09-13: 79 of
//     65,804 queries in a still-water case and 192 of 64,404 in a wave case came back false.
//     One strip told "the surface is at 0" while its neighbours 5 cm away see +1.5 m is a
//     whole-hull buoyancy force applied to one section, and that is what threw the surfaced
//     vehicle to 8.65 m/s in a sea whose orbital velocity is 1.3 m/s. Measured rate: one failure
//     every ~20 physics steps, i.e. two or three per second, every second.
//
//     THE FALLBACK HAS TO BE THE NEIGHBOUR, NOT THE PLANE. Falling back to the still-water plane
//     looks principled and is worthless exactly where it matters: this scene's Ocean sits at
//     y = 0 (pinned there because of the defect above), so "the still plane" and "the zero the
//     broken code returned" are the same number. The useful fallback is the last SUCCESSFUL
//     query — the strips are 50 mm apart and are queried in sequence, so the previous good
//     answer is a far better estimate of this strip's water level than any fixed plane. The
//     still plane remains the fallback of last resort, for the first query of a frame.
//
// The old behaviour is kept, deliberately, behind `SeedFromPreviousResult`. It is how the
// before/after in the wave campaign was measured, and it is the only honest way to show what
// this defect was doing to every result taken before today.

using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace DefaultNamespace.Water
{
    public class HDRPWaterQueryModel : WaterQueryModel
    {
        public WaterSurface water;

        [Header("Search")]
        [Tooltip("Iteration budget per query. HDRP's own sample uses 8; the legacy value here was 6.")]
        public int MaxIterations = 24;

        [Tooltip("Horizontal tolerance in metres. The legacy value was 0.01 (1 cm).")]
        public float SearchError = 0.001f;

        [Tooltip("Include deformation (wakes, foam generators) in the queried height.")]
        public bool IncludeDeformation = true;

        [Header("Legacy defect (leave OFF)")]
        [Tooltip("ON restores the pre-2026-09-13 behaviour: seed every search from the PREVIOUS " +
                 "caller's result through one shared state. Kept only to reproduce the defect " +
                 "for an A/B — it makes buoyancy depend on the order the force points are visited.")]
        public bool SeedFromPreviousResult = false;

        [Header("Telemetry (read-only, reset by ResetStats)")]
        public int Queries;
        public int Failed;
        public int NonConverged;
        public float WorstErrorM;
        public int WorstIterations;

        WaterSearchResult _previous;
        float _stillLevel;
        float _lastGoodLevel;
        bool _haveGood;

        public void Awake()
        {
            if (water == null)
            {
                var all = FindObjectsByType<WaterSurface>(FindObjectsSortMode.None);
                if (all.Length == 0) { Debug.LogError("[HDRPWaterQueryModel] no WaterSurface in the scene."); return; }
                if (all.Length > 1)
                    Debug.LogWarning($"[HDRPWaterQueryModel] {all.Length} WaterSurfaces in the scene; " +
                                     $"taking '{all[0].name}'. Which one you get is not defined — fix the scene.");
                water = all[0];
            }
            _stillLevel = water.transform.position.y;
            if (!water.scriptInteractions)
                Debug.LogWarning("[HDRPWaterQueryModel] WaterSurface.scriptInteractions is OFF — " +
                                 "the CPU surface the physics reads does not exist. Buoyancy will be wrong.");
        }

        public void ResetStats() { Queries = 0; Failed = 0; NonConverged = 0; WorstErrorM = 0f; WorstIterations = 0; }

        public override float GetWaterLevelAt(Vector3 position)
        {
            if (water == null) return 0f;

            WaterSearchParameters parameters = new WaterSearchParameters
            {
                targetPositionWS = position,
                // Own position, not the last caller's: the answer must not depend on call order.
                startPositionWS = SeedFromPreviousResult ? _previous.candidateLocationWS : (Unity.Mathematics.float3)position,
                maxIterations = Mathf.Max(1, MaxIterations),
                error = Mathf.Max(1e-5f, SearchError),
                includeDeformation = IncludeDeformation,
                excludeSimulation = false,
                outputNormal = false,
            };

            bool ok = water.ProjectPointOnWaterSurface(parameters, out var result);
            Queries++;

            if (!ok)
            {
                // No simulation data. The struct holds (0,0,0), NOT the water level. Returning it
                // would put this one point's water at world zero while its neighbours see the real
                // surface — a differential buoyancy error, i.e. a moment on the hull.
                Failed++;
                return _haveGood ? _lastGoodLevel : _stillLevel;
            }

            _previous = result;
            _lastGoodLevel = result.projectedPositionWS.y;
            _haveGood = true;
            if (result.error > parameters.error) NonConverged++;
            if (result.error > WorstErrorM) WorstErrorM = result.error;
            if (result.numIterations > WorstIterations) WorstIterations = result.numIterations;

            return result.projectedPositionWS.y;
        }
    }
}
