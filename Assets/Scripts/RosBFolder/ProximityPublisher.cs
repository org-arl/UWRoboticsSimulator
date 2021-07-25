using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using SimpleJSON;

public class ProximityPublisher : ROSBridgePublisher
{

	public static string GetMessageTopic() // To get the topic name
	{
		return "proximitySensor"; // Define the topic's name
	}

	public static string GetMessageType() //To get the topic type
	{
		return "std_msgs/String"; // Defining the topic type

	}

	public static string ToYAMLString(StringMsg msg)
	{
		return msg.ToYAMLString();
	}

	public new static ROSBridgeMsg ParseMessage(JSONNode msg)
	{
		return new StringMsg(msg);
	}
}