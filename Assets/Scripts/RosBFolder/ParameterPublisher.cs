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
using ROSBridgeLib.geometry_msgs;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
public class ParameterPublisher : MonoBehaviour
{

    public PointMsg position;
    public QuaternionMsg quatern;
    public PoseMsg poseMsg;
    public PoseMsg poseMsgscan;
    public GameObject auv;
    public GameObject station;
    public Camera sonarCamera;
    Vector3 iterVar;
    Vector3 iterVar2;
    Vector3 iterVar3;
    public float[] depthLog;
    public float[] depthsend;
    public int rayNumber;
    public float angle;
    public float scannerAngle = 0;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        SendPose();
        SendFOV();
        DepthRayCast();
        SendVehState();
        ModelTrigger();
        ProximitySensor();
        ScannerPose();
    }


    public void SendFOV()
    {

        float vFOVrad = sonarCamera.fieldOfView * Mathf.Deg2Rad;
        float cameraHeightAt1 = Mathf.Tan(vFOVrad * 0.5f);
        float hFOVrad = Mathf.Atan(cameraHeightAt1 * sonarCamera.aspect) * 2;
        float hFOVtemp = hFOVrad * Mathf.Rad2Deg;
        var hFOV = new StringMsg(hFOVtemp.ToString());
        station.GetComponent<ROSInitializer>().ros.Publish(FloatPublisher.GetMessageTopic(), hFOV);

    }

    public void SendPose()
    {
        
            Vector3 pose = auv.transform.position;
            Quaternion rot = auv.transform.rotation;
            position = new PointMsg(-pose.x, -pose.z, pose.y);
            quatern = new QuaternionMsg(rot.x, rot.z, -rot.y, rot.w);
            poseMsg = new PoseMsg(position, quatern);
            station.GetComponent<ROSInitializer>().ros.Publish(PosPublisher.GetMessageTopic(), poseMsg);
            poseMsg = null;

    }

    public void SendVehState()
    {
        String paraString = "";
        // Encoding for the parameters is as follows
        // Speed a Angular Speed a Temperature a Pressure a Depth a Distance Station a Distance Seabed a Mission Time a Battery Remaining
        paraString = paraString + SeabedScript.speed.ToString() + "a" + SeabedScript.aSpeed.ToString() + "a" + SeabedScript.temperature.ToString() + "a" + SeabedScript.pressure.ToString() + "a" + SeabedScript.depth.ToString() + "a" + SeabedScript.relativeDistance.ToString() + "a" + SeabedScript.distanceSeabed.ToString() + "a" + UIScript.theSeconds.ToString() + "a" + UIScript.battery.ToString() + "a";
        var stringtobesent = new StringMsg(paraString);
        station.GetComponent<ROSInitializer>().ros.Publish(StatePublisher.GetMessageTopic(), stringtobesent);
    }


    public void DepthRayCast()
    {
        
        rayNumber = StartScreen.raynumber + 1;
        float range = StartScreen.range;
        
        if (rayNumber == 513)
        {
            angle = 85;
        }
        if (rayNumber == 1025)
        {
            angle = 170;
        }
        float[] depthLog = new float[rayNumber];
        float[] depthsend = new float[rayNumber];

        if (BallPoseSubscriber.rotation.w == 6 && scannerAngle <=90)
        {
            scannerAngle += 0.1f;
        }
        if (BallPoseSubscriber.rotation.w == 8 && scannerAngle >= -90)
        {
            scannerAngle -= 0.1f;
        }

        Quaternion pitch_quat = Quaternion.AngleAxis(scannerAngle,auv.transform.forward);
        Debug.DrawRay(auv.transform.position, pitch_quat * auv.transform.up * 30);
        
        Quaternion q = Quaternion.AngleAxis((angle/StartScreen.raynumber), pitch_quat*auv.transform.up);
        Quaternion q2 = Quaternion.AngleAxis(-(rayNumber-1)*0.5f* (angle / StartScreen.raynumber), pitch_quat*auv.transform.up);


        

        RaycastHit[] castArray = new RaycastHit[rayNumber];

        Vector3 iter_temp = pitch_quat * auv.transform.right;
        iterVar = q2*iter_temp;

        for (int i = 0; i < rayNumber; i++)
        {
            
            Debug.DrawRay(auv.transform.position, iterVar * -range);
            if (Physics.Raycast(auv.transform.position, iterVar * -1, out castArray[i], range))
            {
                depthLog[i] = castArray[i].distance;
            }
            else
            {
                depthLog[i] = range;
            }
            iterVar = q * iterVar;
        }

        for (int j=0; j<rayNumber; j++)
        {
            depthsend[j] = depthLog[rayNumber - 1 - j];
        }

        String msgtobeSent = "";
        for (int p = 0; p < rayNumber; p++)
        {
            msgtobeSent = msgtobeSent + depthsend[p].ToString() + "a";
        }

        var intarraymessage = new StringMsg(msgtobeSent);
        station.GetComponent<ROSInitializer>().ros.Publish(StringDepthPublisher.GetMessageTopic(), intarraymessage);
    }

    public void ModelTrigger()
    {
        var modelTrigger = new StringMsg(StartScreen.modeltrigger.ToString());
        station.GetComponent<ROSInitializer>().ros.Publish(ModelTriggerPublish.GetMessageTopic(), modelTrigger);

    }

    public void ProximitySensor()
    {
        RaycastHit hitLeft = new RaycastHit();
        float proximityLeft;
        Debug.DrawRay(auv.transform.position, auv.transform.forward* -10);

        if(Physics.Raycast(auv.transform.position, auv.transform.forward*-1, out hitLeft, 10))
        {
            proximityLeft = hitLeft.distance;
        }
        else
        {
            proximityLeft = 10;
        }
        var proxString = new StringMsg(proximityLeft.ToString());
        station.GetComponent<ROSInitializer>().ros.Publish(ProximityPublisher.GetMessageTopic(), proxString);
    }

    public void ScannerPose()
    {
        Vector3 poseAUV = auv.transform.position;
        Vector3 rotAUV = auv.transform.eulerAngles;
        Quaternion rotQuater = Quaternion.Euler(0, 0, (scannerAngle));
        position = new PointMsg(0, 0, 0);
        quatern = new QuaternionMsg(rotQuater.x, rotQuater.z, -rotQuater.y, rotQuater.w);
        poseMsgscan = new PoseMsg(position, quatern);
        station.GetComponent<ROSInitializer>().ros.Publish(ScannerPosPublisher.GetMessageTopic(), poseMsgscan);
        poseMsgscan = null;
    }

}
