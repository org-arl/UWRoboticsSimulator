using System.Text;

using SimpleJSON;

namespace ROSBridgeLib
{
    namespace sensor_msgs
    {
        /// <summary>
        /// Reports the state of a joysticks axes and buttons.
        /// This class matches the definition of the joy.msg message type on ROS.
        /// <see cref="http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html"/>
        /// </summary>
        public class JoyMsg : ROSBridgeMsg
        {
            /// <value> Timestamp in the header is the time the data is received from the joystick.</value>
            std_msgs.HeaderMsg _header;
            /// <value> The axes measurements from a joystick.</value>
            float[] _axes;
            /// <value> The buttons measurements from a joystick.</value>
            int[] _buttons;

            public JoyMsg(JSONNode msg)
            {
                _header = new std_msgs.HeaderMsg(msg["header"]);

                JSONArray temp = msg["axes"].AsArray;
                _axes = new float[temp.Count];
                for (int i = 0; i < _axes.Length; i++)
                {
                    _axes[i] = temp[i].AsFloat;
                }

                temp = msg["buttons"].AsArray;
                _buttons = new int[temp.Count];
                for (int i = 0; i < _buttons.Length; i++)
                {
                    _buttons[i] = temp[i].AsInt;
                }
            }

            public JoyMsg(std_msgs.HeaderMsg header, float[] axes, int[] buttons)
            {
                _header = header;
                _axes = axes;
                _buttons = buttons;
            }

            public static string getMessageType()
            {
                return "sensor_msgs/Joy ";
            }
            
            public std_msgs.HeaderMsg GetHeader()
            {
                return _header;
            }

            public float[] GetAxes()
            {
                return (float[])_axes.Clone();
            }

            public int[] GetButtons()
            {
                return (int[])_buttons.Clone();
            }

            public override string ToString()
            {
                return "Axes: [" + string.Join(", ", _axes) + "]\nButtons: [" + string.Join(", ", _buttons) + "]";
            }

            public override string ToYAMLString()
            {
                StringBuilder sb = new StringBuilder("{\"header\": ");
                sb.Append(_header.ToYAMLString());
                sb.Append(", \"axes\": [");
                sb.Append(string.Join(", ", _axes));
                sb.Append("], \"buttons\": [");
                sb.Append(string.Join(", ", _buttons));
                sb.Append("]}");
                return sb.ToString();
            }
        }
    }
}
