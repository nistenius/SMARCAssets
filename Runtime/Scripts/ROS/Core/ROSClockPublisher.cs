using System;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;
using ROS.Core;

namespace Unity.Robotics.Core
{
    public class ROSClockPublisher : ROSBehaviour
    {
        [SerializeField]
        Clock.ClockMode m_ClockMode;

        [SerializeField, HideInInspector]
        Clock.ClockMode m_LastSetClockMode;
        
        [SerializeField]
        double m_PublishRateHz = 100f;

        [SerializeField]
        [Tooltip("Anchor sim time to wall time so it never runs backwards when you restart Play. " +
                 "Off = Unity's play-relative clock, which restarts at 0 and forces a full ROS " +
                 "stack restart after every Play. See Clock.EpochOffsetSeconds.")]
        bool m_AnchorClockToWallTime = true;

        // Survives Play sessions (and editor restarts) so a session that ran faster than real time
        // cannot hand the next one a backward jump. Key is per-project, value is seconds.
        const string k_LastPublishedPrefKey = "SMARC.Clock.LastPublishedSeconds";

        double m_LastPublishTimeSeconds;
        double m_LastPrefWriteSeconds;

        TimeMsg clockMsg;

        double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

        bool ShouldPublishMessage => Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > m_LastPublishTimeSeconds;

        bool registered = false;

        void OnValidate()
        {
            // var clocks = FindObjectsOfType<ROSClockPublisher>();
            var clocks = FindObjectsByType<ROSClockPublisher>(FindObjectsSortMode.None);
            if (clocks.Length > 1)
            {
                Debug.LogWarning("Found too many clock publishers in the scene, there should only be one!");
            }

            if (Application.isPlaying && m_LastSetClockMode != m_ClockMode)
            {
                Debug.LogWarning("Can't change ClockMode during simulation! Setting it back...");
                m_ClockMode = m_LastSetClockMode;
            }
            
            SetClockMode(m_ClockMode);
        }

        void SetClockMode(Clock.ClockMode mode)
        {
            Clock.Mode = mode;
            m_LastSetClockMode = mode;
        }


        protected override void StartROS()
        {
            SetClockMode(m_ClockMode);

            // Anchor BEFORE anything stamps a message: every publisher reads Clock.time, so the
            // offset has to be in place before the first sensor tick or /clock and the message
            // headers would disagree with each other.
            if (m_AnchorClockToWallTime)
            {
                var lastPublished = double.TryParse(
                    PlayerPrefs.GetString(k_LastPublishedPrefKey, "0"),
                    System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture,
                    out var stored) ? stored : 0.0;

                Clock.AnchorToWallClock(notBefore: lastPublished);
                Debug.Log($"[ROSClockPublisher] Sim time anchored to wall clock at {Clock.time:F3} " +
                          $"(offset {Clock.EpochOffsetSeconds:F3} s, previous session ended at {lastPublished:F3}). " +
                          "Restarting Play moves sim time forward, so the ROS stack does not need restarting.");
            }
            else
            {
                Clock.ClearWallClockAnchor();
            }

            if (!registered)
            {
                rosCon.RegisterPublisher<ClockMsg>("/clock");
                registered = true;
            }
            clockMsg = new TimeMsg();
        }


        void Update()
        {
            if (!ShouldPublishMessage) return;

            var publishTime = Clock.time;
            clockMsg.sec = (int)publishTime;
            clockMsg.nanosec = (uint)((publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds);
            m_LastPublishTimeSeconds = publishTime;
            rosCon.Publish("/clock", clockMsg);

            // Remember where this session got to, once a second. Cheap, and it is what lets the
            // NEXT session guarantee monotonicity without assuming the wall clock behaved.
            if (m_AnchorClockToWallTime && publishTime - m_LastPrefWriteSeconds >= 1.0)
            {
                m_LastPrefWriteSeconds = publishTime;
                PlayerPrefs.SetString(k_LastPublishedPrefKey,
                    publishTime.ToString("R", System.Globalization.CultureInfo.InvariantCulture));
            }
        }
    }
}