using System;
using System.Reflection;
using System.Collections.Generic;

using UnityEngine;

using WebSocketSharp;
using SimpleJSON;

/**
 * This class handles the connection with the external ROS world, deserializing
 * json messages into appropriate instances of packets and messages.
 * 
 * This class also provides a mechanism for having the callback's exectued on the rendering thread.
 * (Remember, Unity has a single rendering thread, so we want to do all of the communications stuff away
 * from that. 
 * 
 * The one other clever thing that is done here is that we only keep 1 (the most recent!) copy of each message type
 * that comes along.
 * 
 * Version History
 * 3.1 - changed methods to start with an upper case letter to be more consistent with c#
 * style.
 * 3.0 - modification from hand crafted version 2.0
 * 
 * @author Michael Jenkin, Robert Codd-Downey and Andrew Speers
 * @version 3.1
 */

 namespace ROSBridgeLib {

    /// <summary>
    /// The interface for classes to subscribe to a topic.
    /// </summary>
    public interface ROSTopicSubscriber
    {
        /// <summary>
        /// The method called when a new message is received on any topic the instance is subscribed to.
        /// </summary>
        /// <param name="topic">The name of the topic the message is from.</param>
        /// <param name="raw_msg">The raw contents of the message as a JSONNode object.</param>
        /// <param name="parsed">A parsed version of the message as a subclass of ROSBridgeMsg. This may be null if this is the first callback for this message.</param>
        /// <returns>A parsed version of the message as a subclass of ROSBridgeMsg. This helps reduce redundant parsing.</returns>
        ROSBridgeMsg OnReceiveMessage(string topic, JSONNode raw_msg, ROSBridgeMsg parsed = null);
        /// <summary>
        /// A method for determining the type of each topic that class wishes to subscribe to.
        /// </summary>
        /// <param name="topic">The topic whose type is requested.</param>
        /// <returns>The string representing the message type of the topic.</returns>
        string GetMessageType(string topic);
    }

    /// <summary>
    /// Manages all communications with a single ROSBridge server.
    /// </summary>
    /// <remarks>
    /// Currently, the class is in an awkward, but fully functional state.
    /// It keeps support for the old system paradigm of using static methods and static variables for everything.
    /// It is not recommended to use the old paradigm, which will be referred to as the legacy system.
    /// It worked like this:
    /// <list type="bullet">
    /// <item>Publishers, subscribers, and service callers have static methods, parameter-less methods for obtaining its topic/service name and message type.</item>
    /// <item>Receivers had static methods for parsing a received message and another static method for handling it. This was likely to reduce redudant parsing on the assumption that all subscribers parsed the message the same way.</item>
    /// <item>Service responses are not parsed, but handed to the callback as is.</item>
    /// <item>When a message arrived, the list of subscribers was traversed, and a callback was made to every subscriber whose topic name matched the message's topic name.</item>
    /// <item>Similarily, when a message arrived, the list of subscribers was traversed, and a callback is made to the object whose service name matched the reponse's service name.</item>
    /// </list>
    /// The legacy system had several issues/drawbacks that inspired the new system:
    /// <list type="bullet">
    /// <item>It assumes that a subscriber class should only need to subscribe to one topic. This creates the need to write a new class for each topic.</item>
    /// <item>
    /// There cannot be multiple ROSBridgeWebSocketConnection instances with the same subscriber class as a registered subscriber.
    /// This is because the callback methods are static, so whatever is being done in the callback method will be done exactly the same
    /// for calls from both instances of ROSBridgeWebSocketConnection. For instance, if the callback method updates a specific texture instance, 
    /// callbacks for that topic from both ROSBridgeWebSocketConnection instances (which are connected to different ROSBridge servers) will end up updating
    /// the SAME texture object. This is not good if, for example, the two ROSBridgeWebSocketConnection instances are communicating with two
    /// different machines/robots. The callbacks will get muddled.
    /// </item>
    /// <item>
    /// There can only be one service call in-flight/pending at a time.
    /// This is due to the fact that there was no mechanism to keep track of what service calls were pending.
    /// Furthermore, the "id" field of the json service call was unused, so the ROSBridge will not attach the id to the response; therefore,
    /// there was no way to distinguish which responses belonged to which caller other than to only have one service call pending at a time.
    /// </item>
    /// </list>
    /// Meanwhile an improved, non-static method for managing messages/subscribers/service-calls has been impelemented.
    /// <list type="bullet">
    /// <item>To publish, simply call AddPublisher with the topic name and string representing the message type. This is only needed to advertise to the ROSBridge server. No reference to publisher is kept.</item>
    /// <item>For a class to receive messages from any number of topics, it only needs to implement the ROSTopicSubscriber interface and make a call to AddSubscriber for each topic it subscribes to.</item>
    /// <item>
    /// To make a service call, pass a JSONServiceResponseHandler callback method, service name, an id string to identify this call, and the argument string for the call.
    /// The id is included in the final service call and is used to identify the correct callback method to invoke upon receiving the response, which also includes the id.
    /// The raw service response (id included) is passed to the callback method.
    /// </item>
    /// </list>
    /// </remarks>
 	public class ROSBridgeWebSocketConnection {

        // <Changed>
        /// <summary>
        /// The signature for callback methods for a service request.
        /// </summary>
        /// <param name="node">The raw response as a JSON object.</param>
        public delegate void JSONServiceResponseHandler(JSONNode node);
        // </Changed>

        // <Changed>
        /// <summary>
        /// A data class for a message queue item.
        /// </summary>
        private class RenderTask {
            //private Type _subscriber
            /// <value> The name of the topic this message is from. </value>
 			public string topic;
            /// <value> The contents of the message as a JSON object. </value>
 			public JSONNode msg;

            /// <summary>
            /// Constructor for a RenderTask object.
            /// </summary>
            /// <param name="topic">The name of the topic this message is from.</param>
            /// <param name="msg">The contents of the message as a JSON object.</param>
 			public RenderTask(string topic, JSONNode msg) {
 				this.topic = topic;
 				this.msg = msg;
 			}
 		};
        // </Changed>

        /// <value> The address of the host running the ROSBridge server.</value>
 		private string _host;
        /// <value> The port on which the ROSBridge server communicates.</value>
 		private int _port;
        /// <value> The Websocket object responseible for managing the socket and sending/receiving the raw data over the network.</value>
 		private WebSocket _ws;


        private bool _connected = false;
 		private System.Threading.Thread _myThread;

        // <Changed>
        /// <value> A mapping from topic name to a list of legacy subscribers to that topic.</value>
        private Dictionary<string, List<Type>> static_subscribers;
        /// <value> A mapping from topic name to a list of legacy publishers to that topic.</value>
        private Dictionary<string, List<Type>> static_publishers;
        /// <value> A mapping from topic name to a list of subscribers to that topic.</value>
        private Dictionary<string, List<ROSTopicSubscriber>> subscribers;
        /// <value> A mapping from topic name to topic type for topics that to be published to.</value>
        private Dictionary<string, string> publishTopic_to_messageType;
        //private Dictionary<string, List<Type>> publishers;
        // </Changed>

        // Legacy service response fields.
        private Type _serviceResponse; // to deal with service responses
		private string _serviceName = null;
        private JSONNode _rawServiceValues = null;
		private string _serviceValues = null;
        // <Changed>
        // Giving legacy system support for multiple service calls (as long as they are different types).
        private List<Type> _jsonResponseListeners;
        // </Changed>

        /// <value> The waiting queue for received messages from topics. They are received but have not been broadcasted to subscribers yet.</value>
		private Queue<RenderTask> _taskQ = new Queue<RenderTask>();

        // <Changed>
        /// <value> A mapping from id's of pending service calls to the JSONServiceResponseHandler callback to invoke upon its resonse.</value>
        private Dictionary<string, JSONServiceResponseHandler> _pendingServiceResponses;
        /// <value> A waiting queue for received service responses that haven't been passed to their respective callback methods yet.</value>
        private Queue<JSONNode> _responsesQ;
        // </Changed>

        private object _queueLock = new object ();

		/**
		 * Make a connection to a host/port. 
		 * This does not actually start the connection, use Connect to do that.
		 */
        public ROSBridgeWebSocketConnection(string host, int port) {
		 	_host = host;
		 	_port = port;
		 	_myThread = null;

            // <Changed>
            //_subscribers = new List<Type> ();
            //_publishers = new List<Type> ();
            static_subscribers = new Dictionary<string, List<Type>>();
            static_publishers = new Dictionary<string, List<Type>>();
            subscribers = new Dictionary<string, List<ROSTopicSubscriber>>();
            publishTopic_to_messageType = new Dictionary<string, string>();
            //publishers = new Dictionary<string, List<Type>>();
            // </Changed>

            // <Changed>
            _jsonResponseListeners = new List<Type> ();
            _pendingServiceResponses = new Dictionary<string, JSONServiceResponseHandler>();
            _responsesQ = new Queue<JSONNode>();
            // </Changed>
        }

        
        // <Changed>
        /// <summary>
        /// Register a ROSTopicSubscriber to receive messages from a topic.
        /// </summary>
        /// <param name="topicName">The topic to receive messages from.</param>
        /// <param name="subscriber">The ROSTopicSubscriber object to receive the messages from that topic.</param>
        public void AddSubscriber(string topicName, ROSTopicSubscriber subscriber)
        {
            List<ROSTopicSubscriber> topic_subs;
            if (subscribers.TryGetValue(topicName, out topic_subs))
            {
                if (!topic_subs.Contains(subscriber))
                {
                    topic_subs.Add(subscriber);
                }
            } else
            {
                topic_subs = new List<ROSTopicSubscriber>(2);
                topic_subs.Add(subscriber);
                subscribers.Add(topicName, topic_subs);
            }
            if (_connected)
            {
                _ws.Send(ROSBridgeMsg.Subscribe(topicName, subscriber.GetMessageType(topicName)));
                Debug.Log("Sending " + ROSBridgeMsg.Subscribe(topicName, subscriber.GetMessageType(topicName)));
            }
        }

        /// <summary>
        /// Unsubscribe from a specific topic
        /// TODO: Test this function.
        /// </summary>
        /// <param name="topicName"></param>
        public void RemoveSubscriber(string topicName)
        {
            if (_connected)
            {
                if (subscribers.ContainsKey(topicName))
                {
                    _ws.Send(ROSBridgeMsg.UnSubscribe(topicName));
                    Debug.Log("Sending " + ROSBridgeMsg.UnSubscribe(topicName));
                    subscribers.Remove(topicName);
                }
                else
                {
                    Debug.Log("Invalid subscriber topic name key: " + topicName);
                }
            }
            else
            {
                Debug.Log("ROS Connection is inactive");
            }
        }

        /// <summary>
        /// Advertise to the ROSBridge server that a topic will be published to.
        /// </summary>
        /// <param name="topicName">The name of the topic to be published to.</param>
        /// <param name="message_type">The message type of that topic.</param>
        public void AddPublisher(string topicName, string message_type)
        {
            if (publishTopic_to_messageType.ContainsKey(topicName))
            {
                return;
            }
            publishTopic_to_messageType[topicName] = message_type;

            if (_connected)
            {
                _ws.Send(ROSBridgeMsg.Advertise(topicName, message_type));
                Debug.Log("Sending " + ROSBridgeMsg.Advertise(topicName, message_type));
            }
        }

        /// <summary>
        /// Publish a message to a topic.
        /// </summary>
        /// <param name="topic">The name of the topic to publish to.</param>
        /// <param name="msg">The message to publish to the topic.</param>
        /// <remarks>AddPublisher() must be called with this topic's name at least once before publishing to it.</remarks>
        public void Publish(String topic, ROSBridgeMsg msg)
        {
            if (_ws != null)
            {
                string s = ROSBridgeMsg.Publish(topic, msg.ToYAMLString());
                //Debug.Log ("Sending " + s);
                _ws.Send(s);
            }
        }

        /// <summary>
        /// Perform a service call with a callback for the response.
        /// </summary>
        /// <param name="handler">The callback function to invoke when the response of the service call arrives.</param>
        /// <param name="service_name">The name of the service to be called.</param>
        /// <param name="id">The ID by which this call is distinguished both to the server and by this class.</param>
        /// <param name="args">The formatted argument string, as specified by the ROSBridge protocol.</param>
        /// <see cref="https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md"/>
        public void CallService(JSONServiceResponseHandler handler, string service_name, string id, string args = "[]")
        {
            if (_ws != null)
            {
                _pendingServiceResponses[id] = handler;
                string s = ROSBridgeMsg.CallService(service_name, id, args);
                Debug.Log("Sending " + s);
                _ws.Send(s);
            }
        }
        // </Changed>

        /// <summary>
        /// Connect to the remote ros environment.
        /// </summary>
        public void Connect() {
		 	_myThread = new System.Threading.Thread (Run);
		 	_myThread.Start ();
		}

        /// <summary>
        /// Disconnect from the remote ros environment.
        /// </summary>
        public void Disconnect() {
            _connected = false;
		 	_myThread.Abort ();
		 	foreach(string topicName in static_subscribers.Keys) {
		 		_ws.Send(ROSBridgeMsg.UnSubscribe(topicName));
		 		Debug.Log ("Sending " + ROSBridgeMsg.UnSubscribe(topicName));
		 	}
            foreach(string topicName in subscribers.Keys)
            {
                if (!static_subscribers.ContainsKey(topicName))
                {
                    _ws.Send(ROSBridgeMsg.UnSubscribe(topicName));
                    Debug.Log("Sending " + ROSBridgeMsg.UnSubscribe(topicName));
                }
            }
		 	foreach(string topicName in static_publishers.Keys) {
		 		_ws.Send(ROSBridgeMsg.UnAdvertise (topicName));
		 		Debug.Log ("Sending " + ROSBridgeMsg.UnAdvertise(topicName));
		 	}
		 	_ws.Close ();
            static_subscribers.Clear();
            static_publishers.Clear();
            subscribers.Clear();
		}

        /// <summary>
        /// The function run by the _myThread thread. Establishes WebSocket connection with the remote ROSBridge server.
        /// Performs all the topic subscriptions and advertisements calls made to this class prior to connection establishment.
        /// </summary>
		private void Run() {
		 	_ws = new WebSocket(_host + ":" + _port);
		 	_ws.OnMessage += (sender, e) => this.OnMessage(e.Data);
		 	_ws.Connect();
            _connected = true;

		 	foreach(KeyValuePair<string, List<Type>> topic in static_subscribers) {
		 		_ws.Send(ROSBridgeMsg.Subscribe (topic.Key, GetMessageType (topic.Value[0])));
		 		Debug.Log ("Sending " + ROSBridgeMsg.Subscribe(topic.Key, GetMessageType(topic.Value[0])));
		 	}
            foreach(KeyValuePair<string, List<ROSTopicSubscriber>> topic in subscribers)
            {
                if (!static_subscribers.ContainsKey(topic.Key))
                {
                    _ws.Send(ROSBridgeMsg.Subscribe(topic.Key, topic.Value[0].GetMessageType(topic.Key)));
                    Debug.Log("Sending " + ROSBridgeMsg.Subscribe(topic.Key, topic.Value[0].GetMessageType(topic.Key)));
                }
            }

		 	foreach(KeyValuePair<string, List<Type>> topic in static_publishers) {
		 		_ws.Send(ROSBridgeMsg.Advertise (topic.Key, GetMessageType(topic.Value[0])));
		 		Debug.Log ("Sending " + ROSBridgeMsg.Advertise(topic.Key, GetMessageType(topic.Value[0])));
		 	}
            foreach(KeyValuePair<string, string> topic in publishTopic_to_messageType)
            {
                if (!static_publishers.ContainsKey(topic.Key))
                {
                    _ws.Send(ROSBridgeMsg.Advertise(topic.Key, topic.Value));
                    Debug.Log("Sending " + ROSBridgeMsg.Advertise(topic.Key, topic.Value));
                }
            }
		 	//while(true) {
		 	//	Thread.Sleep (1000);
		 	//}
		}

        /// <summary>
        /// The callback method called when the WebSocket receives new data. Parses it and queuees it on the correct queue (topic message or service response) for processing.
        /// </summary>
        /// <param name="s"></param>
		private void OnMessage(string s) {
		 	//Debug.Log ("Got a message " + s);
		 	if((s != null) && !s.Equals ("")) {
		 		JSONNode node = JSONNode.Parse(s);
                //Debug.Log ("Parsed it");
		 		string op = node["op"];
                //Debug.Log ("Operation is " + op);
		 		if ("publish".Equals (op)) {
                    //Debug.Log("Socket: " + _ws.IsAlive + ", " + _ws.ReadyState);
                    string topic = node["topic"];
                    //Debug.Log ("Got a message on " + topic);
                    // <Changed>
                    lock (_queueLock)
                    {
                        _taskQ.Enqueue(new RenderTask(topic, node["msg"]));
                    }
                    //foreach(Type p in _subscribers) {
                    //	if(topic.Equals (GetMessageTopic (p))) {
                    //      //Debug.Log ("And will parse it " + GetMessageTopic (p));
                    //		ROSBridgeMsg msg = ParseMessage(p, node["msg"]);
                    //		RenderTask newTask = new RenderTask(p, topic, msg);
                    //		lock(_queueLock) {
                    //			bool found = false;
                    //			for(int i=0;i<_taskQ.Count;i++) {
                    //				if(_taskQ[i].getTopic().Equals (topic)) {
                    //					_taskQ.RemoveAt (i);
                    //					_taskQ.Insert (i, newTask);
                    //					found = true;
                    //					break;
                    //				}
                    //			}
                    //			if(!found)
                    //			_taskQ.Add (newTask);
                    //		}

                    //	}
                    //}
                    // </Changed>
                }
                else if("service_response".Equals (op)) {
                    // <Changed>
                    //Debug.Log ("Got service response " + node.ToString ());
                    // </Changed>
                    string service_name = node["service"];
                    // <Changed>
                    //Debug.Log(service_name);
                    Type targetListener = null;
                    if (!(node["id"] is JSONLazyCreator))
                    {
                        _responsesQ.Enqueue(node);
                    }
                    else
                    {
                        foreach (Type p in _jsonResponseListeners)
                        {
                            if (GetJSONServiceName(p).Equals(service_name))
                            {
                                targetListener = p;
                                break;
                            }
                        }
                        if (targetListener != null)
                        {
                            //Debug.Log("Socket: " + _ws.IsAlive + ", " + _ws.ReadyState);
                            //JSONServiceResponse(targetListener, node["values"]);
                            _serviceName = service_name;
                            _rawServiceValues = node["values"];
                            _serviceResponse = targetListener;

                        }
                        else
                        {
                            _serviceName = service_name;
                            _serviceValues = (node["values"] == null) ? "" : node["values"].ToString();
                        }
                        // _serviceValues = (node["values"] == null) ? "" : node["values"].ToString ();
                    }
                    // </Changed>
                }
                else
 					Debug.Log ("Must write code here for other messages");
			} else
				Debug.Log ("Got an empty message from the web socket");
		}

        /// <summary>
        /// Performs broadcasting of all queued messages to all subscribers to their respective topics. Serves service call reponses to callers/callback methods.
        /// </summary>
        /// <remarks>This method needs to be called on the render thread (inside the Update() function) by at least one active script (Monobehavior) in the scene for it to work.</remarks>
		public void Render() {
			RenderTask newTask = null;
            lock (_queueLock)
            {
                while (_taskQ.Count > 0)
                {
                    newTask = _taskQ.Dequeue();

                    // <Changed>
                    if (newTask != null)
                    {
                        List<Type> static_subs;
                        if (static_subscribers.TryGetValue(newTask.topic, out static_subs) && static_subs.Count > 0)
                        {
                            ROSBridgeMsg parsed_msg = ParseMessage(static_subs[0], newTask.msg);
                            foreach (Type curr_sub in static_subs)
                            {
                                Update(curr_sub, parsed_msg);
                            }
                        }
                        List<ROSTopicSubscriber> subs;
                        if (subscribers.TryGetValue(newTask.topic, out subs) && subs.Count > 0)
                        {
                            ROSBridgeMsg parsed_msg = null;
                            foreach (ROSTopicSubscriber curr_sub in subs)
                            {
                                if (parsed_msg == null)
                                {
                                    parsed_msg = curr_sub.OnReceiveMessage(newTask.topic, newTask.msg);
                                }
                                else
                                {
                                    curr_sub.OnReceiveMessage(newTask.topic, newTask.msg, parsed: parsed_msg);
                                }
                            }
                        }
                    }
                    // </Changed>

                    // <Changed>
                    while (_responsesQ.Count > 0)
                    {
                        JSONNode curr_response = _responsesQ.Dequeue();
                        JSONServiceResponseHandler handler;
                        if (!_pendingServiceResponses.TryGetValue(curr_response["id"].Value, out handler))
                        {
                            continue;
                        }
                        handler(curr_response);
                    }
                    // </Changed>

                    // <Changed>
                    if (_serviceName != null && _serviceResponse != null)
                    {
                        if (_serviceValues != null)
                        {
                            ServiceResponse(_serviceResponse, _serviceName, _serviceValues);
                        }
                        else if (_rawServiceValues != null)
                        {
                            JSONServiceResponse(_serviceResponse, _rawServiceValues);
                        }
                        _serviceValues = null;
                        _rawServiceValues = null;

                        _serviceName = null;
                        _serviceResponse = null;
                    }
                    // </Changed>
                }
            }
        }



        // Legacy (static) system methods


        private static string GetMessageType(Type t)
        {
            return (string)t.GetMethod("GetMessageType", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, null);
        }

        private static string GetMessageTopic(Type t)
        {
            return (string)t.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, null);
        }

        private static ROSBridgeMsg ParseMessage(Type t, JSONNode node)
        {
            return (ROSBridgeMsg)t.GetMethod("ParseMessage", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { node });
        }

        private static void Update(Type t, ROSBridgeMsg msg)
        {
            t.GetMethod("CallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { msg });
        }

        private static void ServiceResponse(Type t, string service, string yaml)
        {
            t.GetMethod("ServiceCallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { service, yaml });
        }

        private static void IsValidServiceResponse(Type t)
        {
            if (t.GetMethod("ServiceCallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("invalid service response handler");
        }

        // <Changed>
        private static string GetJSONServiceName(Type t)
        {
            return (string)t.GetMethod("GetServiceName", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, null);
        }

        private static void JSONServiceResponse(Type t, JSONNode node)
        {
            t.GetMethod("JSONServiceCallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { node });
        }

        private static void IsValidJSONServiceResponse(Type t)
        {
            if (t.GetMethod("JSONServiceCallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("invalid JSON service response handler");
            if (t.GetMethod("GetServiceName", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetServiceName method");
        }
        // </Changed>

        private static void IsValidStaticSubscriber(Type t)
        {
            if (t.GetMethod("CallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing Callback method");
            if (t.GetMethod("GetMessageType", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageType method");
            if (t.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageTopic method");
            if (t.GetMethod("ParseMessage", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing ParseMessage method");
        }

        private static void IsValidStaticPublisher(Type t)
        {
            if (t.GetMethod("GetMessageType", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageType method");
            if (t.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageTopic method");
        }

        /**
		 * Add a service response callback to this connection.
		 */
        public void AddServiceResponse(Type serviceResponse)
        {
            IsValidServiceResponse(serviceResponse);
            _serviceResponse = serviceResponse;
        }

        // <Changed>
        /**
		 * Add a subscriber callback to this connection. There can be many subscribers.
		 */
        public void AddSubscriber(Type subscriber)
        {
            IsValidStaticSubscriber(subscriber);
            string topicName = (string)subscriber.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { });
            List<Type> topic_subs;
            if (static_subscribers.TryGetValue(topicName, out topic_subs))
            {
                if (!topic_subs.Contains(subscriber))
                {
                    topic_subs.Add(subscriber);
                }
            }
            else
            {
                topic_subs = new List<Type>(2);
                topic_subs.Add(subscriber);
                static_subscribers.Add(topicName, topic_subs);
            }
            if (_connected)
            {
                _ws.Send(ROSBridgeMsg.Subscribe(topicName, GetMessageType(subscriber)));
                Debug.Log("Sending " + ROSBridgeMsg.Subscribe(topicName, GetMessageType(subscriber)));
            }
        }

        /**
		 * Add a publisher to this connection. There can be many publishers.
		 */
        public void AddPublisher(Type publisher)
        {
            IsValidStaticPublisher(publisher);
            string topicName = (string)publisher.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { });
            List<Type> topic_pubs;
            if (static_publishers.TryGetValue(topicName, out topic_pubs))
            {
                if (!topic_pubs.Contains(publisher))
                {
                    topic_pubs.Add(publisher);
                }
            }
            else
            {
                topic_pubs = new List<Type>(2);
                topic_pubs.Add(publisher);
                static_publishers.Add(topicName, topic_pubs);
            }

            if (_connected)
            {
                _ws.Send(ROSBridgeMsg.Advertise(topicName, GetMessageType(publisher)));
                Debug.Log("Sending " + ROSBridgeMsg.Advertise(topicName, GetMessageType(publisher)));
            }
        }
        // </Changed>

        // <Changed>
        public void AddJSONServiceResponse(Type jsonServiceResponse)
        {
            IsValidJSONServiceResponse(jsonServiceResponse);
            _jsonResponseListeners.Add(jsonServiceResponse);
        }
        // </Changed>

        public void CallService(string service, string args)
        {
            if (_ws != null)
            {
                string s = ROSBridgeMsg.CallService(service, args);
                Debug.Log("Sending " + s);
                _ws.Send(s);
            }
        }
    }
}
