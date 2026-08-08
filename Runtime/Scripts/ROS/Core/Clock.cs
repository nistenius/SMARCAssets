using System;
using UnityEngine;

namespace Unity.Robotics.Core
{
    public static class Clock
    {
        // Since UnityScaled is the default Unity Time mode, we'll use that for this project
        // None of the other time modes are fully validated and guaranteed to be without issues
        public enum ClockMode
        {
            // Time delta scaled by simulation speed to the start of the current frame, since the beginning of the game
            UnityScaled,
            // Real time delta to the exact moment this function is called, since beginning of the game 
            // UnityUnscaled,
            // Real time delta since the Unix Epoch (1/1/1970 @ midnight)
            // UnixEpoch,
            // Examples of other potentially useful clock modes...
            // UnixEpochUtc,
            // DateTimeNow,
            // DateTimeNowUtc,
            // ExternalClock
        }

        public const double k_NanoSecondsInSeconds = 1e9;

        static readonly DateTime k_UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, 0);
        // Time the application started, relative to Unix Epoch
        static readonly double k_StartTimeEpochSeconds = SecondsSinceUnixEpoch - Time.realtimeSinceStartupAsDouble;
        
        static double SecondsSinceUnixEpoch => (DateTime.Now - k_UnixEpoch).TotalSeconds;
        static double UnityUnscaledTimeSinceFrameStart => 
            Time.realtimeSinceStartupAsDouble - Time.unscaledTimeAsDouble;

        public static double TimeSinceFrameStart => Now - FrameStartTimeInSeconds;

        /// <summary>
        /// Added to every time this class hands out. Zero means "Unity's own play-relative clock",
        /// which is what this class always used to return.
        ///
        /// WHY THIS EXISTS (2026-08-07). Time.timeAsDouble restarts at 0 on every Play. With
        /// use_sim_time, that means the whole ROS stack sees its clock jump BACKWARDS by however
        /// long the previous session ran. Nothing downstream survives that: tf2 buffers hold
        /// transforms stamped in what is now the future and reject every new one with
        /// "TF_OLD_DATA ignoring data from the past", so pid_wp_following can never look up
        /// base_link->utm and silently stops commanding thrust -- a vehicle that accepts a
        /// mission, reports RUNNING, and just sits there. The only known cure was to restart the
        /// entire ROS stack after every Play, which cost us most of a working day.
        ///
        /// Anchoring the clock to wall time removes the failure mode at the source: each Play
        /// starts where real time is now, so restarting Unity moves the clock FORWARD by the
        /// length of the pause. A forward jump is something ROS handles routinely (stale
        /// transforms simply expire); a backward jump is not.
        /// </summary>
        public static double EpochOffsetSeconds { get; private set; } = 0.0;

        /// <summary>
        /// Anchor the clock to wall time, so published sim time is monotonic across Play sessions.
        /// </summary>
        /// <param name="notBefore">
        /// Sim time this session must not start below -- pass the last value published by the
        /// previous session. Guards the one case wall time alone would not: a session that ran
        /// faster than real time, or a machine clock that moved backwards, would otherwise still
        /// hand the stack a backward jump.
        /// </param>
        public static void AnchorToWallClock(double notBefore = 0.0)
        {
            var baseline = Math.Max(SecondsSinceUnixEpoch, notBefore);
            EpochOffsetSeconds = baseline - Time.timeAsDouble;
        }

        /// <summary>Return to Unity's play-relative clock (offset 0). Mainly for tests.</summary>
        public static void ClearWallClockAnchor() => EpochOffsetSeconds = 0.0;

        public static double FrameStartTimeInSeconds
        {
            get
            {
                return Mode switch
                {
                    // This might be an approximation... needs testing.
                    ClockMode.UnityScaled => Time.timeAsDouble + EpochOffsetSeconds,
                    // ClockMode.UnityUnscaled => Time.unscaledTimeAsDouble,
                    // ClockMode.UnixEpoch => k_StartTimeEpochSeconds + UnityUnscaledTimeSinceFrameStart,
                    _ => throw new NotImplementedException()
                };
            }
        }

        public static double NowTimeInSeconds
        {
            get
            {
                return Mode switch
                {
                    ClockMode.UnityScaled => Time.timeAsDouble + UnityUnscaledTimeSinceFrameStart * Time.timeScale + EpochOffsetSeconds,
                    // ClockMode.UnityUnscaled => Time.realtimeSinceStartupAsDouble,
                    // ClockMode.UnixEpoch => SecondsSinceUnixEpoch,
                    _ => throw new NotImplementedException()
                };
            }
        }
        
        // NOTE: Precision loss vs. other time measurements due to no deltaTimeAsDouble interface
        public static float DeltaTimeInSeconds
        {
            get
            {
                return Mode switch
                {
                    ClockMode.UnityScaled => Time.deltaTime,
                    _ => Time.unscaledDeltaTime,
                };
            }
        }

        public static ClockMode Mode = ClockMode.UnityScaled;

        // Simple interfaces for supporting commonly used vocabulary
        public static double Now => NowTimeInSeconds;
        public static double time => FrameStartTimeInSeconds;
        public static float deltaTime => DeltaTimeInSeconds;

        // WARNING: These functions could potentially mess up threaded access to this clock class.
        //          Would need to include some mutex locking to keep these calls thread-safe
        public static double GetFrameTime(ClockMode temporaryMode)
        {
            var originalMode = Mode;
            Mode = temporaryMode;
            var t = FrameStartTimeInSeconds;
            Mode = originalMode;
            return t;
        }

        public static double GetNowTime(ClockMode temporaryMode)
        {
            var originalMode = Mode;
            Mode = temporaryMode;
            var t = NowTimeInSeconds;
            Mode = originalMode;
            return t;
        }
    }
}