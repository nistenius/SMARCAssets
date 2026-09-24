using System.Collections.Generic;
using UnityEngine;

namespace Smarc.Environment
{
    /// <summary>
    /// Region tiles on a need-to-be basis (OCEANVERSE regions, ADR-013 addendum 2026-09-19).
    /// Every curated tile of a region is a Terrain under the world prefab; this keeps only the
    /// ones within <see cref="activeRadius"/> of the tracked transform active. An inactive tile
    /// costs no rendering and no physics (no sonar returns either — which is right, the vehicle
    /// is more than a tile away). All TerrainData assets stay in the project; this is
    /// activation, not asset streaming. Runs once per <see cref="interval"/> seconds.
    /// </summary>
    public class TerrainStreamer : MonoBehaviour
    {
        [Tooltip("What to keep tiles loaded around — the vehicle. Found by name at Start if empty.")]
        public Transform tracked;
        [Tooltip("Name of the object to track when `tracked` is empty (the ROS robot name).")]
        public string trackedName = "sam21";
        [Tooltip("A tile is active while any point of it is within this distance (m) of the tracked object.")]
        public float activeRadius = 6144f;
        [Tooltip("Seconds between checks.")]
        public float interval = 1.0f;
        [Tooltip("Off = every tile stays active (the pre-streaming behaviour).")]
        public bool enabledStreaming = true;

        readonly List<Terrain> tiles = new List<Terrain>();
        readonly List<Bounds> boundsXZ = new List<Bounds>();
        float next;

        void Start()
        {
            tiles.Clear(); boundsXZ.Clear();
            foreach (var t in GetComponentsInChildren<Terrain>(true))
            {
                tiles.Add(t);
                var p = t.transform.position; var s = t.terrainData.size;
                boundsXZ.Add(new Bounds(new Vector3(p.x + s.x / 2f, 0f, p.z + s.z / 2f), new Vector3(s.x, 1f, s.z)));
            }
            if (tracked == null && !string.IsNullOrEmpty(trackedName))
            {
                var go = GameObject.Find(trackedName);
                if (go != null) tracked = go.transform;
            }
            Debug.Log($"[TerrainStreamer] {tiles.Count} tile(s); tracking {(tracked != null ? tracked.name : "NOTHING — all tiles stay active")}; radius {activeRadius} m");
            Apply(true);
        }

        void Update()
        {
            if (Time.time < next) return;
            next = Time.time + interval;
            Apply(false);
        }

        void Apply(bool force)
        {
            if (!enabledStreaming || tracked == null)
            {
                foreach (var t in tiles) if (!t.gameObject.activeSelf) t.gameObject.SetActive(true);
                return;
            }
            var v = tracked.position; v.y = 0f;
            int on = 0;
            for (int i = 0; i < tiles.Count; i++)
            {
                var d = Mathf.Sqrt(boundsXZ[i].SqrDistance(v));       // 0 when inside the tile
                bool want = d <= activeRadius;
                if (want) on++;
                if (tiles[i].gameObject.activeSelf != want) tiles[i].gameObject.SetActive(want);
            }
            if (force) Debug.Log($"[TerrainStreamer] {on}/{tiles.Count} tiles active around {tracked.name}");
        }
    }
}
