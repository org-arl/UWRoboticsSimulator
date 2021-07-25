using System.Collections.Generic;

using UnityEngine;

namespace ROSBridgeLib
{
    /// <summary>
    /// A singleton class that ensures there is only one ROSBridgeWebSocketConnection per address/port pair.
    /// It handles opening, giving thread time to, and closing each ROSBridgeWebSocketConnection.
    /// <remarks>
    /// This allows multiple data client objects to share a ROSBridgeWebSocketConnection to the same rosbridge server.
    /// It also reduces unecessary calls to ROSBridgeWebSocketConnection.Render(). Before each user of the ROSBridgeWebSocketConnection
    /// object would call its Render() function to guarantee it had at least one object giving it render thread runtime to execute its
    /// necessary functions. Having it be managed by one script unifies this so that it is given one call per render cycle.
    /// </remarks>
    /// </summary>
    public class ROSBridgeConnectionsManager : MonoBehaviour
    {
        /// <value> The reference to the singleton instance of this class.</value>
        private static ROSBridgeConnectionsManager _instance;
        /// <value> A lock to ensure that only one object can check and set the _instance variable at a time.</value>
        private static object instance_lock = new object();
        /// <value> The public accessor variable to the singleton instance.</value>
        public static ROSBridgeConnectionsManager Instance
        {
            get
            {
                return _instance;
            }
        }

        /// <value> The mapping from rosbridge address "ws://address:port" to the ROSBridgeWebSocketConnection object associated with that address.</value>
        private static Dictionary<string, ROSBridgeWebSocketConnection> connections = new Dictionary<string, ROSBridgeWebSocketConnection>();
        /// <value> The lock for the connections dictionary.</value>
        private object connection_lock = new object();

        private void Awake()
        {
            // Fill the singleton instance variable with the current object if it is the first instance.
            lock (instance_lock)
            {
                if (_instance == null)
                {
                    _instance = this;
                }
            }
        }

        /// <summary>
        /// Obtain the unique ROSBridgeWebSocketConnection object for a given address and port.
        /// </summary>
        /// <param name="address">The network or web address of the rosbridge server.</param>
        /// <param name="port">The port that the rosbridge server uses on the remote machine.</param>
        /// <returns></returns>
        public ROSBridgeWebSocketConnection GetConnection(string address, int port)
        {
            string full_address = string.Format("ws://{0}:{1}", address, port);
            ROSBridgeWebSocketConnection conn;
            lock (connection_lock)
            {
                // Check whether a connection has already been established for this address/port pair.
                if (!connections.TryGetValue(full_address, out conn))
                {
                    Debug.Log("Starting submap connection to " + full_address);
                    conn = new ROSBridgeWebSocketConnection("ws://" + address, port);
                    conn.Connect();
                    connections[full_address] = conn;
                }
            }
            return conn;
        }

        // Update is called once per frame
        void Update()
        {
            // Give each ROSBridgeWebSocketConnection render thread time.
            foreach (ROSBridgeWebSocketConnection conn in connections.Values)
            {
                conn.Render();
            }
        }

        void OnApplicationQuit()
        {
            // Disconnect connections to all remote machines.
            Debug.Log("Disconnecting from ROS");
            foreach (ROSBridgeWebSocketConnection conn in connections.Values)
            {
                conn.Disconnect();
            }
        }
    }
}