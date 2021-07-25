using SimpleJSON;

namespace ROSBridgeLib
{
    namespace custom_utils
    {
        /// <summary>
        /// A class for putting utility methods used across various components/classes in the ROSBridgeLib namespace.
        /// </summary>
        public class ROSBridgeUtils
        {
            /// <summary>
            /// Convert a JSONNode containing only raw data in string form to a byte array.
            /// </summary>
            /// <param name="node">The node directly encapsulating the data.</param>
            /// <param name="is_bigendian">Whether the data is big endian.</param>
            /// <returns>The raw bytes as a byte array.</returns>
            /// <remarks>Assumes the string is in base64.</remarks>
            public static byte[] JSONDataToBytes(JSONNode node, bool is_bigendian = true)
            {
                byte[] data = System.Convert.FromBase64String(node.Value);
                if (!is_bigendian)
                {
                    System.Array.Reverse(data);
                }
                return data;
            }

            /// <summary>
            /// Convert a JSONNode containing raw data as a JSON array to a byte array.
            /// </summary>
            /// <param name="node">The JSON array directly holding the data.</param>
            /// <param name="is_bigendian">Whether the data is big endian.</param>
            /// <returns>The raw bytes as a byte array.</returns>
            public static byte[] JSONArrayToBytes(JSONArray array, bool is_bigendian = true)
            {
                byte[] data = new byte[array.Count];
                if (is_bigendian)
                {
                    for (int i = 0; i < data.Length; i++)
                    {
                        data[i] = (byte)array[i].AsInt;
                    }
                }
                else
                {
                    for (int i = 0; i < data.Length; i++)
                    {
                        data[data.Length - i - 1] = (byte)array[i].AsInt;
                    }
                }
                
                return data;
            }

            /// <summary>
            /// A convenience function to convert a JSONNode object containing raw data to a byte array.
            /// Auto-detects whether it is in string format or JSON array format.
            /// </summary>
            /// <param name="node">The node directly contaiing the data.</param>
            /// <param name="is_bigendian">Whether the data is big endian.</param>
            /// <returns>The raw bytes as a byte array.</returns>
            public static byte[] ParseJSONRawData(JSONNode node, bool is_bigendian = true)
            {
                if (node.GetType() == typeof(JSONArray)) {
                    return JSONArrayToBytes(node.AsArray, is_bigendian);
                } else
                {
                    return JSONDataToBytes(node, is_bigendian);
                }
            }
        }
    }
}
