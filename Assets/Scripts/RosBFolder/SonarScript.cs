#define self
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
using UnityEngine.UI;
using UnityEngine.SceneManagement;
public class SonarScript : MonoBehaviour
{
    public Camera cameraObject;
    public Shader uberReplacementShader;
    public float[] depthLog;
    public float[,] depthTransmission;
    public int[] depthArray;
    public GameObject stationCamera;
    // Start is called before the first frame update
    void Start()
    {
        cameraObject.farClipPlane = 40f; // Maximum range of the Sonar
        cameraObject.nearClipPlane = 0.3f; // Minumum distance that teh Sonar can detect

        SetupCameraWithReplacementShader(cameraObject, uberReplacementShader, ReplacelementModes.DepthCompressed, Color.white);

    }

    // Update is called once per frame
    void Update()
    {
        StartCoroutine(DepthEncoding());

    }


    IEnumerator DepthEncoding()
    {
        float[] depthLog = new float[640];
        int[] depthArray = new int[640];
        yield return new WaitForEndOfFrame();
        int width = 640; // Width and height of the output image
        int height = 360;
        RenderTexture tex2 = cameraObject.GetComponent<Camera>().targetTexture; //using a render texture to get Video feed projection. 
        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false); // Defining a texture which will read the Render texture
        cameraObject.Render();
        RenderTexture.active = tex2;
        tex.ReadPixels(new Rect(0, 0, width, height), 0, 0); // reading the render texture into 'tex'
        tex.Apply();

        for (int i=0; i < 640; i++)
        {
            depthLog[i] = tex.GetPixel(i,180).grayscale;
        }

        float sum=0;

        for (int j = 0; j < depthLog.Length; j++)
        {
            sum += depthLog[j];
        }

        float average = sum / width ;
        

        int pixelMap = (int)(20*average);
        float[,] depthTransmission = new float[pixelMap, 640];

        if (pixelMap > 0)
        {

            for (int k = 0; k < pixelMap; k++)
            {
                for (int l = 0; l < 640; l++)
                {
                    depthTransmission[k, l] = tex.GetPixel(l, 180 + pixelMap).grayscale;
                }
            }
        }

        float rowMinima; ;
        for(int m = 0; m < 640; m++)
        {
            rowMinima = 10000;
            for (int n = 0; n < pixelMap; n++)
            {
                rowMinima = Mathf.Min(rowMinima, depthTransmission[n, m]);
            }
            depthArray[m] = (int)(rowMinima * 1000);
        }


        //byte[] data = tex.EncodeToJPG(); // Encode the data in 'tex' i.e. the video feed into jpg format
        string msgtobeSent = "";
        for (int p = 0; p < 640; p++)
        {
            msgtobeSent = msgtobeSent + depthArray[p].ToString() + "a";
        }

        Debug.Log(msgtobeSent);


        var intarraymessage = new StringMsg(msgtobeSent);
        stationCamera.GetComponent<ROSInitializer>().ros.Publish(StringDepthPublisher.GetMessageTopic(), intarraymessage);

    }


    enum ReplacelementModes
    {
        ObjectId = 0,
        CatergoryId = 1,
        DepthCompressed = 2,
        DepthMultichannel = 3,
        Normals = 4
    };

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode)
    {
        SetupCameraWithReplacementShader(cam, shader, mode, Color.black);
    }

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode, Color clearColor)
    {
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
        cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
    }


    

}
