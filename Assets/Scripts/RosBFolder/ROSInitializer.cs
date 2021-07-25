// This script is attached to a game object and serves as the foundaton of the connection. In our project, thi script 
// initializes the rosbridge connection as well as encodes the video feeds and publish them. 
using UnityEngine.Rendering;
using System.Collections;
using System.Net;
using System.Net.Sockets;
using System;
using System.Text;
using System.IO;
using UnityEngine;
using System.Collections.Generic;
using ROSBridgeLib;
using ROSBridgeLib.auv_msgs;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.sensor_msgs;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
public class ROSInitializer : MonoBehaviour
{
    public ROSBridgeWebSocketConnection ros = null; // This variable is used to setup the connection and it takes the IP address and port number as its input
    int count;
    DateTime lastFrame;
    DateTime camStart;
    public Camera frontCam;
    public Camera downCam;


    void Start()
    {
        // Takes the value of IP address and port number from the user input in the Main Menu
        ros = new ROSBridgeWebSocketConnection(StartScreen.IPAddress, int.Parse(StartScreen.portNumber));

        // Add subscribers and publishers (if any)
        ros.AddSubscriber(typeof(BallPoseSubscriber));
        ros.AddPublisher(typeof(ImagePublisher));
        //ros.AddPublisher(typeof(downcamPublisher));
        ros.AddPublisher(typeof(frontcamPublisher));
        ros.AddPublisher(typeof(StringDepthPublisher));
        ros.AddPublisher(typeof(PosPublisher));
        ros.AddPublisher(typeof(FloatPublisher));
        ros.AddPublisher(typeof(StatePublisher));
        ros.AddPublisher(typeof(ModelTriggerPublish));
        ros.AddPublisher(typeof(ProximityPublisher));
        ros.AddPublisher(typeof(ScannerPosPublisher));
        // Fire up the subscriber(s) and publisher(s)
        ros.Connect();
        count = 0;
        lastFrame = DateTime.Now;
        camStart = DateTime.Now;



    }

    // Extremely important to disconnect from ROS. Otherwise packets continue to flow
    void OnApplicationQuit()
    {
        if (ros != null)
        {
            ros.Disconnect();
        }
    }
    // Update is called once per frame in Unity
    void Update()
    {

        StartCoroutine(SendImage()); // Separate functions for every video feed.
        //StartCoroutine(SendImagefront());
        //StartCoroutine(SendImagedown());
        ros.Render();

    }



    IEnumerator SendImage() // To send station Camera Video feed. All the other video feed function are similar to this one. 
    {
        yield return new WaitForEndOfFrame();
        int width = 640; // Width and height of the output image
        int height = 360;
        RenderTexture tex2 = GetComponent<Camera>().targetTexture; //using a render texture to get Video feed projection. 
        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false); // Defining a texture which will read the Render texture
        RenderTexture.active = tex2;
        tex.ReadPixels(new Rect(0, 0, width, height), 0, 0); // reading the render texture into 'tex'
        tex.Apply();
        byte[] data = tex.EncodeToJPG(); // Encode the data in 'tex' i.e. the video feed into jpg format


        var now = DateTime.Now;
        var timeSpan = now - lastFrame;
        var timeSinceStart = now - camStart;
        var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);
        var headerMessage = new HeaderMsg(count, timeMessage, "camera");

        Debug.Log("data length: " + data.Length);
        string format = "jpeg";
        var compressedImageMsg = new CompressedImageMsg(headerMessage, format, data); // Defining a compressed image message from the encoded 'tex' above
        Debug.Log(compressedImageMsg);
        ros.Publish(ImagePublisher.GetMessageTopic(), compressedImageMsg); // Publishing the compressed image data
        Destroy(tex);
        ros.Render();
    }

    IEnumerator SendImagefront() // For the forward-facing camera
    {
        yield return new WaitForEndOfFrame();
        int width = 640;
        int height = 360;


        RenderTexture texf = frontCam.GetComponent<Camera>().targetTexture;
        frontCam.Render();
        Texture2D texftransfer = new Texture2D(width, height, TextureFormat.RGB24, false);
        RenderTexture.active = texf;
        texftransfer.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texftransfer.Apply();
        byte[] data2 = texftransfer.EncodeToJPG();

        var now = DateTime.Now;
        var timeSpan = now - lastFrame;
        var timeSinceStart = now - camStart;
        var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);
        var headerMessage = new HeaderMsg(count, timeMessage, "camera");

        Debug.Log("data length: " + data2.Length);
        string format = "jpeg";
        var compressedFront = new CompressedImageMsg(headerMessage, format, data2);
        Debug.Log(compressedFront);

        ros.Publish(frontcamPublisher.GetMessageTopic(), compressedFront);
 
        Destroy(texftransfer);
        ros.Render();
    }

    IEnumerator SendImagedown() // For the downward-facing camera
    {
        yield return new WaitForEndOfFrame();
        int width = 640;
        int height = 360;
       
        RenderTexture texd = downCam.GetComponent<Camera>().targetTexture;
        Texture2D texdtransfer = new Texture2D(width, height, TextureFormat.RGB24, false);
        RenderTexture.active = texd;
        texdtransfer.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texdtransfer.Apply();
        byte[] data3 = texdtransfer.EncodeToJPG();


        var now = DateTime.Now;
        var timeSpan = now - lastFrame;
        var timeSinceStart = now - camStart;
        var timeMessage = new TimeMsg(timeSinceStart.Seconds, timeSinceStart.Milliseconds);
        var headerMessage = new HeaderMsg(count, timeMessage, "camera");


        string format = "jpeg";

        var compressedDown = new CompressedImageMsg(headerMessage, format, data3);

        ros.Publish(downcamPublisher.GetMessageTopic(), compressedDown);

        Destroy(texdtransfer);
        ros.Render();
    }

}