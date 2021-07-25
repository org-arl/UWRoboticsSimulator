using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;
using ROSBridgeLib.geometry_msgs;
using SimpleJSON;

public class PosPublisher : ROSBridgePublisher
{
	public static string GetMessageTopic()
	{
		return "/propPos";
	}

	public static string GetMessageType()
	{
		return "geometry_msgs/Pose";
	}

	public static string ToYAMLString(PosPublisher msg)
	{
		return msg.ToString();
	}

	public new static ROSBridgeMsg ParseMessage(JSONNode msg)
	{
		return new PoseMsg(msg);
	}
}