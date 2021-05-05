using StdMsgs = RosMessageTypes.Std;

namespace RosMessageTypes
{
    public static class HeaderExtensions
    {
        private static Utils.Timer timer = new Utils.Timer();
        public static void Update(this StdMsgs.MHeader header)
        {
            header.seq++;
            timer.Now(header.stamp);
        }
    }
}