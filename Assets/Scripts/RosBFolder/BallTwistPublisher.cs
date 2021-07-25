// This script serves a publisher for Pose type message and can be edited according to the user's need. It is called by other scripts
//like RosInitializer to get the data that needs to be transfered. 
//Called by RosInitializer, therefor doesn't need to be attached to an object.
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROSBridgeLib;// Calling the Rosbridge library
using SimpleJSON;
using ROSBridgeLib.geometry_msgs; // Calling RosBridge message types that come under geometry_ msgs (So that we can use Pose message type)

public class BallTwistPublisher : ROSBridgePublisher
{
    // The following three functions are important
    public static string GetMessageTopic() // To get the topic name
    {
        // Define the topic's name
        return "/twist_info";
    }

    public static string GetMessageType() //To get the topic type
    {
        return "geometry_msgs/Twist"; // Defining the topic type
    }

    public static string ToYAMLString(TwistMsg msg)
    {
        return msg.ToYAMLString();
    }

    public new static ROSBridgeMsg ParseMessage(JSONNode msg)
    {
        return new TwistMsg(msg);
    }
}