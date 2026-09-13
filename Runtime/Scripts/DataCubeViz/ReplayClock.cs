using System;
using UnityEngine;

namespace DataCubeViz
{
    /// <summary>
    /// The ONE bag-time clock every replayed thing reads.
    ///
    /// WHY IT IS ITS OWN OBJECT AND WHY IT NEVER TOUCHES /clock
    /// -------------------------------------------------------
    /// Mission Control's Debrief already established that map markers and graph cursors must be
    /// driven from one bag-time clock, not each from its own cursor; the same applies the moment
    /// a ghost vehicle, a track line and a channel strip are on screen together. Two clocks is
    /// how you get a ghost at t=41 s beside a depth reading from t=38 s and no way to see it.
    ///
    /// It is explicitly NOT sim time. Nothing here publishes to `/clock`, and nothing here reads
    /// it: a replay running beside a live sim must not be able to move the sim, and a paused
    /// replay must not stop it. `Time.unscaledDeltaTime` is used for exactly that reason — a
    /// `Time.timeScale` of 0 during a sim pause leaves the replay scrubbing.
    /// </summary>
    [Serializable]
    public class ReplayClock
    {
        [SerializeField] double _t;
        [SerializeField] double _duration;
        [SerializeField] bool _playing;
        [SerializeField] float _speed = 1f;
        [SerializeField] bool _loop = true;

        public event Action<double> TimeChanged;

        /// <summary>Current bag time, seconds from the start of the recording.</summary>
        public double Time
        {
            get => _t;
            set
            {
                var clamped = Math.Max(0.0, Math.Min(_duration, value));
                if (Math.Abs(clamped - _t) < 1e-9) return;
                _t = clamped;
                TimeChanged?.Invoke(_t);
            }
        }

        public double Duration => _duration;
        public bool Playing => _playing;
        public bool Loop { get => _loop; set => _loop = value; }

        /// <summary>Playback rate. Clamped to a sane band; 0 is what Pause is for.</summary>
        public float Speed
        {
            get => _speed;
            set => _speed = Mathf.Clamp(value, 0.05f, 50f);
        }

        public float Fraction => _duration <= 0 ? 0f : (float)(_t / _duration);

        public void SetDuration(double seconds)
        {
            _duration = Math.Max(0.0, seconds);
            if (_t > _duration) Time = _duration;
        }

        public void Play() => _playing = _duration > 0;
        public void Pause() => _playing = false;
        public void TogglePlay() { if (_playing) Pause(); else Play(); }

        public void Rewind()
        {
            Time = 0;
        }

        /// <summary>Step by a fixed amount, for frame-accurate inspection. Pauses first — a
        /// step that keeps running is a scrub, and the operator asked for a step.</summary>
        public void Step(double seconds)
        {
            _playing = false;
            Time = _t + seconds;
        }

        /// <summary>Call once per frame from a MonoBehaviour's Update.</summary>
        public void Tick(float unscaledDeltaTime)
        {
            if (!_playing || _duration <= 0) return;
            var next = _t + unscaledDeltaTime * _speed;
            if (next >= _duration)
            {
                if (_loop) { Time = 0; }
                else { Time = _duration; _playing = false; }
                return;
            }
            Time = next;
        }

        public string TimeLabel =>
            $"{TimeSpan.FromSeconds(_t):mm\\:ss\\.f} / {TimeSpan.FromSeconds(_duration):mm\\:ss\\.f}";
    }
}
