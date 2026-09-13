namespace ROS.Core
{
    public class FrequencyTimer
    {
        double lastUpdate = 0f;
        float frequency = 10f;
        float period => 1.0f / frequency;

        /// <summary>
        /// A simple class that simply keeps track of number of periods passed and
        /// provides a way to check if it's time to update.
        /// </summary>
        /// <param name="frequency"></param>
        public FrequencyTimer(float frequency)
        {
            this.frequency = frequency;
        }

        public bool NeedsTick(double now)
        {
            if (frequency <= 0) return true;
            if (lastUpdate == 0f)
            {
                lastUpdate = now;
                return true;
            }
            return now - lastUpdate >= period;
        }

        public void Tick()
        {
            if (frequency <= 0) return;
            lastUpdate += period;
        }

        public bool ExhaustTicks(double now)
        {
            if (frequency <= 0) return true;
            if (lastUpdate == 0f)
            {
                lastUpdate = now;
                return true;
            }
            return DropOwedTicks(now) > 0;
        }

        /// <summary>
        /// Skip forward over every whole period that has already elapsed, and say HOW MANY were
        /// skipped. This is <see cref="ExhaustTicks"/>'s body with the count returned instead of
        /// thrown away; `ExhaustTicks` now calls it, so there is one implementation and its
        /// behaviour is unchanged.
        ///
        /// It exists because a caller that drops a backlog needs to be able to REPORT the drop.
        /// A skipped tick is a sample that was never taken, i.e. a gap in the record; the only
        /// alternative to dropping it is to fill it with a repeat of the last sample, which is
        /// invented data (see ROSPublisher.FixedUpdate's note, 2026-08-30). A gap that says how
        /// big it is can be reasoned about; a gap silently papered over with copies cannot.
        ///
        /// NOT the same as `ExhaustTicks` in the two edge cases it deliberately leaves alone:
        /// an unstarted timer (`lastUpdate == 0`) and a free-running one (`frequency &lt;= 0`)
        /// have no backlog, so both return 0 here while `ExhaustTicks` returns true for them.
        /// </summary>
        public int DropOwedTicks(double now)
        {
            if (frequency <= 0) return 0;
            if (lastUpdate == 0f)
            {
                lastUpdate = now;
                return 0;
            }
            var timeToTick = now - lastUpdate;
            var numTicks = (int)(timeToTick / period);
            if (numTicks > 0) lastUpdate += numTicks * period;
            return numTicks;
        }
    }
}