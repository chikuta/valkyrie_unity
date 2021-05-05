using System;
using StdMsgs = RosMessageTypes.Std;

namespace RosMessageTypes.Utils
{
    public class Timer
    {
        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);

        public virtual StdMsgs.MTime Now()
        {
            Now(out uint secs, out uint nsecs);
            return new StdMsgs.MTime(secs, nsecs);
        }

        public virtual void Now(StdMsgs.MTime stamp)
        {
            uint secs;
            uint nsecs;
            Now(out secs, out nsecs);
            stamp.secs = secs;
            stamp.nsecs = nsecs;
        }

        private static void Now(out uint secs, out uint nsecs)
        {
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double msecs = timeSpan.TotalMilliseconds;
            secs = (uint)(msecs / 1000);
            nsecs = (uint)((msecs / 1000 - secs) * 1e+9);
        }
    }
}