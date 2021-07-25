//This script generates the data that will be published by Unity, and then calls the publisher script to publish that particular data. 
//Needs to be attached to a game object. 

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROSBridgeLib; // Calling the Rosbridge library
using SimpleJSON;
using ROSBridgeLib.geometry_msgs; // Calling RosBridge message types that come under geometry_ msgs (So that we can use Pose message type)

public class DataManager : MonoBehaviour
{
    Rigidbody rb;
    public GameObject rosObj;
    //Required for TwistMsg
    Vector3Msg linearVel;
    Vector3Msg angularVel;
    TwistMsg msg;

    // Start is called before the first frame update
    void Start()
    {
        //Since we attached ROSInitiazer to Main Camera:
        rosObj = GameObject.Find("Main Camera");
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        //dependant on the message defintion:
        linearVel = new Vector3Msg(
            rb.velocity.x,
            rb.velocity.y,
            rb.velocity.z
        );
        angularVel = new Vector3Msg(
            rb.angularVelocity.x,
            rb.angularVelocity.y,
            rb.angularVelocity.z
        );
        msg = new TwistMsg(linearVel, angularVel); // Defining the message that needs to be published 
        rosObj.GetComponent<ROSInitializer>().ros.Publish( BallTwistPublisher.GetMessageTopic(), msg);// Calling the publisher script and publishing the message
    }
}