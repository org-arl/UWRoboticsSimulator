//A base Raw Image Publisher.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib; // Calling the Rosbridge library
using ROSBridgeLib.sensor_msgs; // Calling RosBridge message types that come under sensor_ msgs (So that we can use Raw Image message type)
using SimpleJSON;

public class RawImagePublisher : ROSBridgePublisher  {

	public static string GetMessageTopic() // To get the topic name
	{
		return "/images"; // Define the topic's name
	}
	public static string GetMessageType() //To get the topic type
	{
		return "sensor_msgs/Image"; // Defining the topic type
	}
	public static string ToYAMLString(ImageMsg msg) //Publish the Image
	{
		return msg.ToYAMLString();
	}
}
