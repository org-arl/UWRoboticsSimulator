//A base publisher for String topic type, used to transfer video feed as string message type.
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib; // Calling the Rosbridge library
using ROSBridgeLib.std_msgs; // Calling RosBridge message types that come under std_ msgs (So that we can use String message type)
using SimpleJSON;

public class StringImagePublisher : ROSBridgePublisher
{

	public static string GetMessageTopic() // To get the topic name
	{
		return "/images"; // Define the topic's name
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