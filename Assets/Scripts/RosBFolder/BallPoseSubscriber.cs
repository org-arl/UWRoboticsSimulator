// This script serves a subscriber for Pose type message and can be edited according to the user's need. Here, it receives 
// control commands and button functions which are then called by their respective scripts. 
//Called by RosInitializer, therefor doesn't need to be attached to an object.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROSBridgeLib; // Calling the Rosbridge library
using SimpleJSON;
using ROSBridgeLib.geometry_msgs; // Calling RosBridge message types that come under geometry_ msgs (So that we can use Pose message type)


// Ball subscriber:
public class BallPoseSubscriber : ROSBridgeSubscriber
{
    public static Vector3 position; // A vector3 that will store translation vectors
    public static Quaternion rotation; // A Quaternion which will store rotation vectors for roll, pitch and yaw in the first three
                                        // values and button functions in the last value
   
    public new static string GetMessageTopic() // To get the topic name
    {
        return "/key"; // Define the topic's name
    }

    public new static string GetMessageType() //To get the topic type
    {
        return "geometry_msgs/Pose"; // Defining the topic type
    }

    // This function converts JSon to Pose Message
    public new static ROSBridgeMsg ParseMessage(JSONNode msg)
    {
        return new PoseMsg(msg);
    }

    // This function should fire on each received ROS message
    public new static void CallBack(ROSBridgeMsg msg) //msg is the recieved message
    {

        Debug.Log("Recieved Message : " + msg.ToYAMLString()); // Prints the recieved message
        // Update ball position, or whatever
        PoseMsg PoseData = (PoseMsg)msg; // This converts the message to Pose type so that it can be used.
        //ball = GameObject.Find("ball");
        //Vector3 ballPos = ball.transform.position;
        position.x = PoseData.GetPosition().GetX(); // Vector3 position and Quaternion rotation are assigned values from the recieved message.
        position.y = PoseData.GetPosition().GetY();
        position.z = PoseData.GetPosition().GetZ();
        rotation.x = PoseData.GetOrientation().GetX();
        rotation.y = PoseData.GetOrientation().GetY();
        rotation.z = PoseData.GetOrientation().GetZ();
        rotation.w = PoseData.GetOrientation().GetW();
        //ballPos.y = PoseData.GetPosition().GetY();
        //ballPos.z = PoseData.GetPosition().GetZ();
        //Changing ball's position to the updated position vector
        //ball.transform.position = ballPos;
    }
}