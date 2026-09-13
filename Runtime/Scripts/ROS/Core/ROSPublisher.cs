using UnityEngine;
using ROSMessage = Unity.Robotics.ROSTCPConnector.MessageGeneration.Message;
using Unity.Robotics.Core;


namespace ROS.Core
{
    public abstract class ROSPublisher<RosMsgType> : ROSBehaviour
        where RosMsgType: ROSMessage, new()
    {
        [Header("ROS Publisher")]
        public float frequency = 10f;
        
        protected RosMsgType ROSMsg;
        protected string robot_name = "";

        bool registered = false;
        FrequencyTimer timer;

        bool firstPub = true;

        /// <summary>
        /// Publish slots this component chose to leave EMPTY rather than fill with a repeat of
        /// the previous message -- i.e. the size of the gap it has left in the record so far.
        /// A property, not a field, so Unity does not serialize it onto every prefab.
        /// Non-zero means the scene could not sustain <see cref="frequency"/>; see FixedUpdate.
        /// </summary>
        public int SkippedPublishes { get; private set; }

        /// <summary>A single FixedUpdate owing more than this many publishes is a stall, not
        /// jitter, and is reported as an error the way the old burst was. Kept at the old
        /// threshold on purpose: the same event still produces a console line, it just no
        /// longer produces 82 duplicate messages on the wire as well.</summary>
        const int StallTickCount = 50;

        /// <summary>Ordinary shortfall is reported at most this often. At 18.75 Hz a scene that
        /// runs 18 % slow drops a tick several times a second; a line per drop would be log
        /// spam, and log spam is how the "82 messages" error went unread for a whole run.</summary>
        const double SkipReportPeriodS = 10.0;
        double nextSkipReportAt = 0.0;
        int skippedSinceReport = 0;

        protected override void StartROS()
        {
            timer = new FrequencyTimer(frequency);
            ROSMsg = new RosMsgType();
            if (!registered)
            {
                rosCon.RegisterPublisher<RosMsgType>(topic);
                registered = true;
            }
            if (GetRobotGO(out var robotGO))
            {
                robot_name = robotGO.name;
            }
            InitPublisher();
        }

        /// <summary>
        /// Override this method to update the ROS message with the sensor data.
        /// This method is called in Update, so that the message can be published at a fixed frequency.
        /// </summary>
        protected abstract void UpdateMessage();

        /// <summary>
        /// Override this method to initialize the ROS message.
        /// This method is called in StartROS which is called in Start, so that the message can be published at a fixed frequency.
        /// </summary>
        protected virtual void InitPublisher(){}

        /// <summary>
        /// Publish the message to ROS.
        /// We do this in FixedUpdate, so that things can be disabled and enabled at runtime.
        /// And not in Update, because usually FixedUpdate is called at a consistent rate and faster than frames.
        ///
        /// ONE SAMPLE, ONE MESSAGE -- A GAP IS HONEST, A COPY IS INVENTED DATA (2026-08-30).
        /// What was here:
        ///
        ///     if (timer.NeedsTick(Clock.Now)) UpdateMessage();          // at most ONCE
        ///     while (timer.NeedsTick(Clock.Now)) { rosCon.Publish(topic, ROSMsg); timer.Tick(); }
        ///
        /// -- sample once, then publish that one message as many times as the timer said it
        /// owed, "to match the expected frequency". Every copy carries the same payload AND the
        /// same header.stamp: UpdateMessage() ran once, and the stamp publishers set is
        /// Clock.time, which is the FRAME START time and does not move inside a FixedUpdate. So
        /// the extra messages are not late samples, they are the same look re-sent with nothing
        /// new about them, and anything that counts messages counts looks that never happened.
        /// `Sensor.FixedUpdate` had already settled the opposite rule for the sampling side --
        /// "we dont actually want to do more sensor updates per fixedupdate ... since the sensor
        /// data would be exactly the same" -- and this loop was the one place that disagreed.
        ///
        /// MEASURED, run 12 at Askoe (bag `sam_mk2_02_57_1788090909`,
        /// `docs/2026-08-30-asko-run12-pipeline-sss.md` section 8): 22,499 side-scan pings
        /// recorded, 18,472 distinct; 4,014 of them (17.9 %) bit-identical to their predecessor
        /// and carrying its stamp. The publisher held 18.63 Hz while the SONAR only managed
        /// 15.30 -- the duplicate count is exactly the sonar's shortfall, dressed as data.
        /// Twice, a ~4.4 s application freeze made Clock.Now jump forward INSIDE one FixedUpdate
        /// (Clock.Now = frame time + REAL time since the frame started), the timer said it owed
        /// ~82 ticks, and one sonar buffer went out 82 times -- which is the console error
        /// verbatim: "Published 82 messages ... in one FixedUpdate". The number was never a
        /// count of looks, it was a count of debt.
        ///
        /// Now: at most ONE publish per FixedUpdate, and the backlog is DROPPED, not filled.
        /// A freeze therefore leaves a hole in the record the size of the freeze, which is what
        /// actually happened, and a consumer can see it. The old error is kept for the same
        /// event (StallTickCount) and an ordinary shortfall is reported rate-limited, because a
        /// publisher that cannot keep up must say so rather than look busy.
        ///
        /// Consequence to know about: a publisher whose `frequency` exceeds the physics rate now
        /// publishes at the physics rate instead of duplicating up to the nominal one. That was
        /// never extra information; `Sensor.OnValidate` already clamps sensors for this reason.
        /// </summary>
        void FixedUpdate()
        {
            // One reading of the clock for the whole call. Clock.Now includes real time since
            // frame start, so it MOVES between two calls inside the same FixedUpdate -- which is
            // precisely what turned a 4.4 s hitch into an 82-message burst. Sample it once and
            // this FixedUpdate has a single, consistent idea of "now".
            double now = Clock.Now;

            if (firstPub)
            {
                timer.ExhaustTicks(now);
                firstPub = false;
            }

            if (!timer.NeedsTick(now)) return;

            UpdateMessage();
            rosCon.Publish(topic, ROSMsg);
            timer.Tick();

            // Everything the timer still owes is time nobody sampled. Drop it and say so.
            int skipped = timer.DropOwedTicks(now);
            if (skipped <= 0) return;
            SkippedPublishes += skipped;
            skippedSinceReport += skipped;

            if (skipped >= StallTickCount)
            {
                Debug.LogError(
                    $"[ROSPublisher<{typeof(RosMsgType)}>] {topic} ({robot_name}) at {now}: the " +
                    $"application stalled for about {skipped / Mathf.Max(frequency, 1e-6f):F2} s " +
                    $"and {skipped} publish slots were SKIPPED. They are left EMPTY on purpose -- " +
                    "the sensor produced one sample, so republishing it would put copies of one " +
                    "look on the wire under one stamp. Expect a gap of that length in the record.");
                nextSkipReportAt = now + SkipReportPeriodS;
                skippedSinceReport = 0;
                return;
            }

            if (now >= nextSkipReportAt)
            {
                Debug.LogWarning(
                    $"[ROSPublisher<{typeof(RosMsgType)}>] {topic} ({robot_name}): cannot sustain " +
                    $"{frequency} Hz -- {skippedSinceReport} publish slots skipped in the last " +
                    $"{SkipReportPeriodS:F0} s ({SkippedPublishes} this session). The record is " +
                    "correspondingly sparser; it is not padded.");
                nextSkipReportAt = now + SkipReportPeriodS;
                skippedSinceReport = 0;
            }
        }

    }

}
