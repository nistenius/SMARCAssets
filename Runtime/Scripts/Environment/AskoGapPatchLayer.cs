using System.IO;
using UnityEngine;

namespace Smarc.Environment
{
    /// <summary>
    /// Switches the Askö terrain's SYNTHETIC gap patch on and off.
    ///
    /// The Askö seabed is stitched from four real surveys; where none of them reached, the
    /// heightmap carries a patch extrapolated from the sea chart. That patch is useful (a
    /// continuous seabed to fly over) and dangerous (it is not a measurement), so it is
    /// switchable rather than baked in:
    ///
    ///   showSyntheticPatches = true   the patch is present, and the terrain's second splat
    ///                                 layer tints it so it is visible for what it is.
    ///   showSyntheticPatches = false  every synthetic cell becomes a Unity TERRAIN HOLE.
    ///                                 It stops being rendered AND stops being collided
    ///                                 with, so a raycast, a sonar ping or a bottom-follower
    ///                                 gets NO RETURN there instead of a confident wrong one.
    ///
    /// That second behaviour is the point. An invented seabed that answers a range query is
    /// worse than no seabed, because nothing downstream can tell the difference — the same
    /// failure as a health probe reading a foreign publisher, one layer down in the physics.
    ///
    /// The mask is a plain uint8 file written by data-cube/scripts/asko-site/build_heightmap.py
    /// (1 = synthetic), one byte per terrain hole cell, row 0 = SOUTH. It is read from the
    /// package folder rather than baked into the scene so the scene file stays small and the
    /// mask cannot drift from the heightmap it was generated beside.
    /// </summary>
    [ExecuteAlways]
    [RequireComponent(typeof(Terrain))]
    public class AskoGapPatchLayer : MonoBehaviour
    {
        [Tooltip("ON: synthetic gap patch present (tinted). OFF: those cells become terrain " +
                 "holes — no render, no collision, no sonar return.")]
        public bool showSyntheticPatches = true;

        [Tooltip("Path to the uint8 synthetic mask, relative to the project (Packages/... or Assets/...).")]
        public string maskAssetPath = "";

        [Tooltip("Mask side length in cells; must equal heightmapResolution - 1.")]
        public int maskResolution = 4096;

        [Header("Unconditional holes (hi-res patch cutout, added 2026-08-30)")]
        [Tooltip("Optional second uint8 mask, 1 = ALWAYS a hole regardless of the toggle " +
                 "above. Empty means no such cells. Used two ways by the DV hi-res ingest: " +
                 "on the PARENT terrain it is the cutout under the 0.125 m patch, and on the " +
                 "PATCH terrain it is the cells the survey never reached.")]
        public string alwaysHoleMaskAssetPath = "";

        [Header("Measured at build time — read only")]
        public float syntheticFraction;
        public float alwaysHoleFraction;
        [TextArea(2, 5)]
        public string provenance = "";

        bool[,] holes;
        bool[,] alwaysHoles;
        // The PATH the cached mask came from, not a "have we tried" flag. The builder adds
        // the component and only then assigns the path, so a boolean would latch on the
        // first, empty attempt and the cutout would silently never load.
        string alwaysLoadedFrom;
        bool applied;
        bool lastState;

        void OnEnable() { Apply(true); }

        void OnValidate()
        {
            if (!isActiveAndEnabled) return;
            // OnValidate fires on every inspector keystroke; only act on a real change.
            if (applied && showSyntheticPatches == lastState) return;
#if UNITY_EDITOR
            // TerrainData.SetHoles raises OnTerrainChanged, and Unity refuses to send that
            // from inside OnValidate: "SendMessage cannot be called during Awake,
            // CheckConsistency, or OnValidate". The holes were still applied, but the
            // warning is indistinguishable from a real problem the next time someone reads
            // this Console — so the work is deferred one editor tick instead of being
            // suppressed. Guarded against the object being deleted in between.
            UnityEditor.EditorApplication.delayCall += () =>
            {
                if (this == null) return;
                Apply(false);
            };
#else
            Apply(false);
#endif
        }

        [ContextMenu("Apply Gap Patch Setting")]
        public void ApplyNow() { Apply(true); }

        void Apply(bool force)
        {
            var terrain = GetComponent<Terrain>();
            if (terrain == null || terrain.terrainData == null) return;
            if (!force && applied && showSyntheticPatches == lastState) return;

            var td = terrain.terrainData;
            int need = td.heightmapResolution - 1;

            // ONE WRITER. Unity's SetHoles replaces the whole hole state, so a second
            // component punching the hi-res cutout would silently undo whichever of the two
            // ran last (invariant 12, one writer per actuator, in the terrain layer). Both
            // hole sources are therefore combined here and written once.
            LoadAlwaysHoles(need);

            bool[,] state;
            if (showSyntheticPatches)
            {
                state = new bool[need, need];
                for (int y = 0; y < need; y++)
                    for (int x = 0; x < need; x++) state[y, x] = true;   // true = NOT a hole
            }
            else
            {
                if (holes == null || holes.GetLength(0) != need)
                {
                    if (!LoadMask(need)) return;
                }
                state = (bool[,])holes.Clone();
            }
            if (alwaysHoles != null && alwaysHoles.GetLength(0) == need)
            {
                for (int y = 0; y < need; y++)
                    for (int x = 0; x < need; x++)
                        if (alwaysHoles[y, x]) state[y, x] = false;      // false = hole
            }
            td.SetHoles(0, 0, state);
            applied = true;
            lastState = showSyntheticPatches;
        }

        /// <summary>Cells that are holes whatever the synthetic toggle says. Loaded once;
        /// a missing or wrong-sized file is reported and then IGNORED rather than guessed
        /// at, because guessing here would either hide measured seabed or double it.</summary>
        void LoadAlwaysHoles(int need)
        {
            if (alwaysLoadedFrom == alwaysHoleMaskAssetPath &&
                (alwaysHoles == null || alwaysHoles.GetLength(0) == need)) return;
            alwaysLoadedFrom = alwaysHoleMaskAssetPath;
            alwaysHoles = null;
            alwaysHoleFraction = 0f;
            if (string.IsNullOrEmpty(alwaysHoleMaskAssetPath)) return;
            string full = ResolveAssetPath(alwaysHoleMaskAssetPath);
            if (!File.Exists(full))
            {
                Debug.LogWarning($"[Asko] {name}: hi-res cutout mask not found at {full} — " +
                                 "the patch and the base will BOTH render over the same " +
                                 "seabed. Re-run build_hires_patch.py.");
                return;
            }
            var bytes = File.ReadAllBytes(full);
            long expect = (long)need * need;
            if (bytes.LongLength != expect)
            {
                Debug.LogError($"[Asko] {name}: cutout mask is {bytes.LongLength} bytes, " +
                               $"expected {expect} for {need}x{need}. It does not belong to " +
                               "this heightmap — refusing to apply it.");
                return;
            }
            alwaysHoles = new bool[need, need];
            int k = 0, n = 0;
            for (int y = 0; y < need; y++)
                for (int x = 0; x < need; x++, k++)
                    if (bytes[k] != 0) { alwaysHoles[y, x] = true; n++; }
            alwaysHoleFraction = (float)n / expect;
        }

        bool LoadMask(int need)
        {
            if (string.IsNullOrEmpty(maskAssetPath))
            {
                Debug.LogWarning($"[Asko] {name}: no maskAssetPath — cannot switch the gap " +
                                 "patch off. Rebuild via SMARC -> Build Asko Curated Site.");
                return false;
            }
            string full = ResolveAssetPath(maskAssetPath);
            if (!File.Exists(full))
            {
                Debug.LogWarning($"[Asko] {name}: synthetic mask not found at {full} — the " +
                                 "gap patch cannot be switched off, so it stays ON rather " +
                                 "than silently pretending the seabed is measured.");
                showSyntheticPatches = true;
                return false;
            }
            var bytes = File.ReadAllBytes(full);
            long expect = (long)need * need;
            if (bytes.LongLength != expect)
            {
                Debug.LogError($"[Asko] {name}: mask is {bytes.LongLength} bytes, expected " +
                               $"{expect} for {need}x{need}. It does not belong to this " +
                               "heightmap — refusing to apply it.");
                showSyntheticPatches = true;
                return false;
            }
            holes = new bool[need, need];
            int k = 0, n = 0;
            for (int y = 0; y < need; y++)
                for (int x = 0; x < need; x++, k++)
                {
                    bool synth = bytes[k] != 0;
                    holes[y, x] = !synth;      // Unity: false = hole
                    if (synth) n++;
                }
            syntheticFraction = (float)n / expect;
            return true;
        }

        static string ResolveAssetPath(string assetPath)
        {
#if UNITY_EDITOR
            var pi = UnityEditor.PackageManager.PackageInfo.FindForAssetPath(assetPath);
            if (pi != null)
                return Path.Combine(pi.resolvedPath,
                    assetPath.Substring(("Packages/" + pi.name + "/").Length));
            if (assetPath.StartsWith("Assets/"))
                return Path.Combine(Application.dataPath, assetPath.Substring("Assets/".Length));
#endif
            return assetPath;
        }
    }
}
