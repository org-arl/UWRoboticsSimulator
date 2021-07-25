// Encode the video feed into a string message which can then be used to call
//the StringImagePublisher to publish the video feed as a string topic type.

#define img
#define self

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
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class ImageTransfer : MonoBehaviour
{
    int ImageWidth = 648; // Output Resolution
    int ImageHeight = 488;
    GameObject downCam; // Initializaion for camera and texture variables.
    GameObject frontCam;
    RenderTexture downImage;
    RenderTexture frontImage;
    Texture2D imageToSendF;
    Texture2D imageToSendD;
    //bool firstSend = true;
    public bool Lock = false;
    GameObject obj;
    StringMsg imgMsg;
    string sceneName;

    // Use this for initialization
    void Start()
    {
        Time.fixedDeltaTime = 0.04f;
        obj = GameObject.Find("Main Camera");

        #region Texture Initializations
#if img
        downImage = new RenderTexture(ImageWidth, ImageHeight, 16, RenderTextureFormat.ARGB32);
        downImage.Create();

        downCam = GameObject.Find("downCam");
        downCam.GetComponent<Camera>().targetTexture = downImage;
        downCam.GetComponent<Camera>().Render();

        frontImage = new RenderTexture(ImageWidth, ImageHeight, 16, RenderTextureFormat.ARGB32);
        frontImage.Create();

        frontCam = GameObject.Find("frontCam");
        frontCam.GetComponent<Camera>().targetTexture = frontImage;
        frontCam.GetComponent<Camera>().Render();

        imageToSendD = new Texture2D(downImage.width, downImage.height, TextureFormat.RGB24, false);
        imageToSendF = new Texture2D(frontImage.width, frontImage.height, TextureFormat.RGB24, false);
#endif
        #endregion
    }

    // Update is called once per frame
    void Update()
    {
#if img
        //encoding part:
        StringBuilder imgToSend = new StringBuilder("", 500000);
        //bottom cam encoding
        RenderTexture.active = downImage; // Using Render Texture to projects camera's feed onto a texture
        imageToSendD.ReadPixels(new Rect(0, 0, downImage.width, downImage.height), 0, 0); // Reading pixels from the Render texture which can then be used further.
        imageToSendD.Apply();
        Byte[] bottom_cam_image_jpg = ImageConversion.EncodeToJPG(imageToSendD, 100); // Conversion of images
        string bottom_cam_image_base64 = Convert.ToBase64String(bottom_cam_image_jpg);
        imgToSend.Append(bottom_cam_image_base64).Append("!");

        //front cam encoding
        RenderTexture.active = frontImage;
        imageToSendF.ReadPixels(new Rect(0, 0, frontImage.width, frontImage.height), 0, 0);
        imageToSendF.Apply();

        Byte[] front_cam_image_jpg = ImageConversion.EncodeToJPG(imageToSendF, 100);
        string front_cam_image_base64 = Convert.ToBase64String(front_cam_image_jpg);
        imgToSend.Append(front_cam_image_base64).Append("!");

        //sending the image data

        try
        {
#if self
            imgMsg = new StringMsg(imgToSend.ToString());
            obj.GetComponent<ROSInitializer>().ros.Publish(StringImagePublisher.GetMessageTopic(), imgMsg);

#endif
        }
        catch (Exception e)
        {
            Debug.Log("Socket error" + e);
        }
#endif
    }
}

