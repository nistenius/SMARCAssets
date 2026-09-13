using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Smarc.Environment
{
    /// <summary>
    /// Per-cell bottom type for a Terrain: what the seabed is MADE OF, so a sonar return can
    /// depend on it.
    ///
    /// WHY THIS EXISTS. Unity gives a TerrainCollider exactly ONE PhysicsMaterial, and
    /// `Sonar.SonarHit` reads its name to pick a reflectivity. So without this, an entire
    /// 4 km terrain answers every ping with a single hardness — bedrock and soft clay
    /// indistinguishable. Splat layers cannot help: they are a renderer concept and the
    /// physics raycast never sees them. A per-cell raster is the only way to make the
    /// acoustic answer depend on the place.
    ///
    /// The map is generated beside the heightmap by
    /// data-cube/scripts/asko-site/build_splatmap.py: one byte per heightmap cell, low bits
    /// = class index into `classes`, high bit (0x80) = "the SHAPE of this cell is
    /// synthetic". The synthetic flag is carried through to every query rather than being
    /// dropped, because a hardness inferred from an invented shape is inferred twice and a
    /// consumer is entitled to know that.
    ///
    /// **The classes are INFERRED FROM MORPHOLOGY, not surveyed** (slope, roughness, depth —
    /// see the generator). Treat the numbers as a plausible seabed, not as ground truth.
    /// </summary>
    [RequireComponent(typeof(Terrain))]
    public class SeabedAcousticMap : MonoBehaviour
    {
        [System.Serializable]
        public class ClassEntry
        {
            public string name = "";
            [Tooltip("Sonar backscatter, same scale as Sonar.SonarHit.simpleMaterialReflectivity")]
            [Range(0f, 1f)] public float reflectivity = 0.5f;
            public int label = 1;
            [Tooltip("Acoustic micro-texture: fractional speckle modulation of the " +
                     "reflectivity, deterministic in world position. What separates bottom " +
                     "types in a real side-scan record is TEXTURE, not mean level — bedrock " +
                     "is only 4 dB over sand on average, far less than the ping-to-ping " +
                     "spread — so a per-class constant renders them indistinguishable. " +
                     "Bedrock/boulders scatter rough and bright with micro-shadows " +
                     "(high amp), rippled sand is moderate, soft clay is smooth (low amp).")]
            [Range(0f, 1f)] public float textureAmp = 0f;
        }

        // Per-class texture defaults, applied when a serialized map predates the field (all
        // amps zero). Keyed by class NAME so an explicit 0 on an unknown class stays 0.
        // RETUNED 2026-08-28 AGAINST THE REAL RECORD. The first values (bedrock 0.45 / sand
        // 0.18 / clay 0.06) were chosen to make bottom type VISIBLE, not to match anything
        // measured, and they are far too strong. Decomposing the real 2024-12-10 Askö record
        // (10,427-ping run; along-track autocorrelation separates the per-ping speckle from
        // the part of the grain that REPEATS on a second pass) gives:
        //     total residual CV 0.280 = speckle CV 0.269  +  PERSISTENT TEXTURE CV 0.076
        // The persistent part is what `textureAmp` models -- 0.076 over mostly-sand ground,
        // against the 0.18 shipped here: 2.4x too strong for sand, ~6x for bedrock. That
        // excess is a large part of why the synthetic waterfall reads as blotchy where a
        // DeepVision record reads as smooth ground with a bright target on it.
        // Relative ordering is kept (rough rock > sand > soft clay); the scale is now measured.
        // Numbers + method: docs/asko/sss_real_targets_2024-12-10.json, SETTLED §3f0h.
        static readonly Dictionary<string, float> DefaultTextureAmp = new()
        {
            { "bedrock", 0.19f }, { "sand", 0.076f }, { "clay", 0.025f }, { "land", 0.19f },
        };

        // The 2026-08-27 values. Present ONLY so a scene serialized with them can be
        // recognised as carrying an auto-applied default rather than a considered choice.
        static readonly Dictionary<string, float> LegacyTextureAmp = new()
        {
            { "bedrock", 0.45f }, { "sand", 0.18f }, { "clay", 0.06f }, { "land", 0.45f },
        };

        // REFLECTIVITY, same story one layer deeper (2026-08-29). These come from
        // asko_splat.json via AskoSiteBuilder and are then SERIALIZED INTO THE SCENE, so
        // retuning the generator changes nothing that is already built -- the values were
        // baked when the builder last ran. Rather than require a full site rebuild to carry a
        // four-decibel correction, a map still holding the exact superseded triple is
        // recognised and upgraded, loudly. Anything else is a considered choice and is left
        // alone. Measurement and reasoning: SETTLED 3f0h / 3f0m.
        static readonly Dictionary<string, float> LegacyReflectivity = new()
        {
            { "bedrock", 0.80f }, { "sand", 0.50f }, { "clay", 0.18f }, { "land", 0.80f },
        };
        static readonly Dictionary<string, float> MeasuredReflectivity = new()
        {
            { "bedrock", 0.53f }, { "sand", 0.50f }, { "clay", 0.45f }, { "land", 0.53f },
        };

        [Tooltip("uint8 class map, one byte per heightmap cell, row 0 = SOUTH. " +
                 "Path relative to the project (Packages/... or Assets/...).")]
        public string classMapAssetPath = "";

        [Tooltip("Side length of the class map in cells; must equal heightmapResolution.")]
        public int resolution = 4097;

        [Tooltip("Index = class id in the map's low bits.")]
        public ClassEntry[] classes = new ClassEntry[0];

        [Header("Measured at build time — read only")]
        [TextArea(2, 6)] public string provenance = "";

        byte[] data;
        bool tried;
        Terrain terrain;

        // One map per collider, resolved once. Sonar hits arrive in the thousands per
        // update, so this must not be a GetComponent per ray.
        static readonly Dictionary<int, SeabedAcousticMap> byCollider = new();

        /// Returns the map covering this collider, or null. Safe to call per raycast hit.
        public static SeabedAcousticMap For(Collider c)
        {
            if (c == null) return null;
            int id = c.GetInstanceID();
            if (byCollider.TryGetValue(id, out var m)) return m;
            m = c.GetComponent<SeabedAcousticMap>();
            byCollider[id] = m;          // cache the miss too
            return m;
        }

        void OnEnable()
        {
            terrain = GetComponent<Terrain>();
            byCollider.Clear();          // instance ids are not stable across reloads
        }

        void OnDisable() { byCollider.Clear(); }

        bool Ready()
        {
            if (data != null) return true;
            if (tried) return false;
            tried = true;
            if (string.IsNullOrEmpty(classMapAssetPath)) return false;
            string full = ResolveAssetPath(classMapAssetPath);
            if (!File.Exists(full))
            {
                Debug.LogWarning($"[SeabedAcousticMap] {name}: class map not found at {full} — " +
                                 "sonar falls back to the collider's single physics material, " +
                                 "so the whole terrain will answer with one hardness.");
                return false;
            }
            var bytes = File.ReadAllBytes(full);
            long expect = (long)resolution * resolution;
            if (bytes.LongLength != expect)
            {
                Debug.LogError($"[SeabedAcousticMap] {name}: class map is {bytes.LongLength} " +
                               $"bytes, expected {expect} for {resolution}^2 — it does not " +
                               "belong to this terrain. Refusing to use it.");
                return false;
            }
            data = bytes;
            if (terrain == null) terrain = GetComponent<Terrain>();

            // Scenes serialized before textureAmp existed carry 0 for every class, which
            // would silently disable the micro-texture. All-zero means "legacy data", so
            // apply the per-name defaults; a hand-set nonzero anywhere disables this.
            if (classes != null && classes.Length > 0)
            {
                // Reflectivity upgrade — see LegacyReflectivity. Runs before the texture
                // block because the two are independent: a scene can carry old levels with
                // new amps or the reverse.
                bool allLegacyR = true;
                foreach (var ce in classes)
                {
                    if (ce == null) continue;
                    if (!LegacyReflectivity.TryGetValue(ce.name.ToLowerInvariant(), out var old)
                        || Mathf.Abs(ce.reflectivity - old) > 1e-4f) { allLegacyR = false; break; }
                }
                if (allLegacyR)
                {
                    foreach (var ce in classes)
                        if (ce != null && MeasuredReflectivity.TryGetValue(
                                ce.name.ToLowerInvariant(), out var r))
                            ce.reflectivity = r;
                    Debug.Log($"[SeabedAcousticMap] {name}: UPGRADED the superseded per-class " +
                              "reflectivities (bedrock 0.80 / sand 0.50 / clay 0.18 = +4.08 dB, " +
                              "literature-typical) to the values measured against the real " +
                              "2024-12-10 record (0.53 / 0.50 / 0.45 = +0.5 dB). Bottom type " +
                              "travels in TEXTURE, not level. Hand-set values are never touched.");
                }

                bool anySet = false;
                foreach (var ce in classes) if (ce != null && ce.textureAmp > 0f) { anySet = true; break; }

                // A SCENE ALREADY SAVED WITH THE SUPERSEDED VALUES WOULD KEEP THEM FOREVER.
                // The `anySet` guard above exists so a hand-tuned map is never overwritten --
                // correct, and it would also have made the 2026-08-28 retune INERT on the one
                // scene we are testing, because AskoCurated was saved with the old auto-applied
                // 0.45/0.18/0.06 baked in. Those exact values are a fingerprint of the old
                // defaults, not of anyone's judgement, so they are upgraded and SAID OUT LOUD.
                // Anything else the user set is left alone. (Same family as the prefab
                // "regressions" that turned out to be our own builder: when a value refuses to
                // change, find who wrote it last.)
                if (anySet)
                {
                    bool allLegacy = true;
                    foreach (var ce in classes)
                    {
                        if (ce == null) continue;
                        if (!LegacyTextureAmp.TryGetValue(ce.name.ToLowerInvariant(), out var old)
                            || Mathf.Abs(ce.textureAmp - old) > 1e-4f) { allLegacy = false; break; }
                    }
                    if (allLegacy)
                    {
                        foreach (var ce in classes)
                            if (ce != null && DefaultTextureAmp.TryGetValue(ce.name.ToLowerInvariant(), out var amp))
                                ce.textureAmp = amp;
                        Debug.Log($"[SeabedAcousticMap] {name}: UPGRADED the superseded texture amps " +
                                  "(bedrock 0.45/sand 0.18/clay 0.06 — chosen to be visible) to the " +
                                  "values measured from the real 2026-08-28 record " +
                                  "(0.19/0.076/0.025). Hand-set values are never touched.");
                        anySet = false;   // handled; skip the legacy-zero path below
                    }
                }

                if (!anySet)
                {
                    foreach (var ce in classes)
                        if (ce != null && DefaultTextureAmp.TryGetValue(ce.name.ToLowerInvariant(), out var amp))
                            ce.textureAmp = amp;
                    Debug.Log($"[SeabedAcousticMap] {name}: applied default acoustic texture amps " +
                              "(bedrock 0.19 / sand 0.076 / clay 0.025 — measured from the real " +
                              "2024-12-10 record) — serialized data predates the field.");
                }
            }
            return true;
        }

        /// <summary>Bottom type at a world position. False if outside or unavailable.</summary>
        public bool TryGet(Vector3 world, out float reflectivity, out int label,
                           out int classIndex, out bool syntheticShape)
        {
            reflectivity = 0.5f; label = 0; classIndex = -1; syntheticShape = false;
            if (!Ready() || terrain == null || terrain.terrainData == null) return false;

            var p = terrain.transform.position;
            var size = terrain.terrainData.size;
            float u = (world.x - p.x) / size.x;
            float v = (world.z - p.z) / size.z;
            if (u < 0f || u > 1f || v < 0f || v > 1f) return false;

            int col = Mathf.Clamp(Mathf.RoundToInt(u * (resolution - 1)), 0, resolution - 1);
            int row = Mathf.Clamp(Mathf.RoundToInt(v * (resolution - 1)), 0, resolution - 1);
            byte b = data[row * resolution + col];
            syntheticShape = (b & 0x80) != 0;
            classIndex = b & 0x7F;
            if (classes == null || classIndex >= classes.Length) return false;
            var e = classes[classIndex];
            reflectivity = e.reflectivity;
            label = e.label;

            // Acoustic micro-texture: modulate the class reflectivity by a hash of WORLD
            // position at ~0.5 m grain. Deterministic on purpose — the bottom's texture must
            // be a property of the place, identical every time it is ensonified, or a repeat
            // pass would image different ground. (Ping-to-ping variability is reverberation
            // noise and belongs in the Sonar, not here.)
            float amp = e.textureAmp;
            if (amp > 0f)
            {
                float n = Hash01(Mathf.RoundToInt(world.x * 2f), Mathf.RoundToInt(world.z * 2f));
                reflectivity = Mathf.Clamp(reflectivity * (1f - amp + 2f * amp * n), 0.02f, 1f);
            }
            return true;
        }

        static float Hash01(int x, int z)
        {
            unchecked
            {
                uint h = (uint)(x * 374761393) + (uint)(z * 668265263);
                h = (h ^ (h >> 13)) * 1274126177u;
                return ((h ^ (h >> 16)) & 0xFFFFFF) / 16777215f;
            }
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
