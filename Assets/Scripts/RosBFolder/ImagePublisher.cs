//Standard compressed image publisher which is used to transfer station camera feed to ROS.
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib; // Calling the Rosbridge library
using ROSBridgeLib.sensor_msgs; // Calling RosBridge message types that come under sensor_ msgs (So that we can use Compressed Image message type)
using SimpleJSON;

public class ImagePublisher : ROSBridgePublisher  {

	public static string GetMessageTopic()// To get the topic name
	{
		return "image/compressed"; // Define the topic's name
	}
	public static string GetMessageType() //To get the topic type
	{
		return "sensor_msgs/CompressedImage"; // Defining the topic type
	}
	public static string ToYAMLString(CompressedImageMsg msg) //Publish the Image
	{
		return msg.ToYAMLString();
	}
}