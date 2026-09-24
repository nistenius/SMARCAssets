// SurfaceParityMarkers — makes "the physics floats on the surface you see" a thing you can LOOK at
// (2026-09-23, gate 1 of the wave-surface work order).
//
// There is no API that reads the rendered height back for a check, so the check is visual: a row of
// small unlit spheres whose centres are placed, every rendered frame, at the elevation the scene's
// WaterQueryModel returns — the exact call ForcePoint makes. If physics = pixels, every sphere sits
// half-sunk ON the rendered water at every crest and trough. A sphere floating above a trough or
// buried under a crest is a physics/visual mismatch, and its size is the error.
//
// Readback latency shows up here too: in GPUReadback mode the query reads the rendered displacement
// a few frames late, so on a fast, steep sea the row lags the surface slightly. That is the honest
// picture of what the hull feels; LagProbe logs how many frames the queried level stays unchanged.

using DefaultNamespace.Water;
using UnityEngine;

namespace Diagnostics
{
    [AddComponentMenu("Smarc/Diagnostics/Surface Parity Markers")]
    public class SurfaceParityMarkers : MonoBehaviour
    {
        [Min(1)] public int Count = 21;
        [Tooltip("Row length, m, centred on this object, along its local +Z")]
        public float Span_m = 20f;
        [Tooltip("Sphere diameter, m")]
        public float Diameter_m = 0.15f;
        public Color MarkerColor = new Color(1f, 0.1f, 0.8f, 1f);

        [Header("Latency (read-only)")]
        [Tooltip("Mean number of rendered frames the queried level at the centre marker stays bit-identical " +
                 "(1 = updated every frame).")]
        public float FramesPerSurfaceUpdate;

        WaterQueryModel _water;
        Transform[] _m;
        float _last = float.NaN; int _same, _runs, _frames;

        void Start()
        {
            _water = WaterQueryModel.GetWaterQueryModel();
            var shader = Shader.Find("HDRP/Unlit");
            var mat = shader != null ? new Material(shader) : null;
            if (mat != null) { mat.SetColor("_UnlitColor", MarkerColor); mat.color = MarkerColor; }
            _m = new Transform[Count];
            for (int i = 0; i < Count; ++i)
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = $"parity_{i:00}";
                var col = go.GetComponent<Collider>(); if (col != null) Destroy(col);   // never a physics object
                var r = go.GetComponent<Renderer>();
                if (r != null) { if (mat != null) r.sharedMaterial = mat; r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off; }
                go.transform.SetParent(transform, false);
                go.transform.localScale = Vector3.one * Diameter_m;
                _m[i] = go.transform;
            }
        }

        // Update, not FixedUpdate: the markers must move with the rendered frame they are judged against.
        void Update()
        {
            if (_water == null || _m == null) return;
            for (int i = 0; i < _m.Length; ++i)
            {
                float s = _m.Length == 1 ? 0f : (i / (float)(_m.Length - 1) - 0.5f) * Span_m;
                Vector3 p = transform.position + transform.forward * s;
                float y = _water.GetWaterLevelAt(p);
                _m[i].position = new Vector3(p.x, y, p.z);
                if (i == _m.Length / 2)
                {
                    _frames++;
                    if (y == _last) _same++; else { _runs++; _last = y; }
                    if (_runs > 0) FramesPerSurfaceUpdate = _frames / (float)_runs;
                }
            }
        }
    }
}
