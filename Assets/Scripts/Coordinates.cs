/* This scripts displays the vehicle's coordinates in the UI and,
 * Freezes them when 'M' is pressed to mark the location of the Leak.
 */


using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;       // To allow us to use the UI elements. 
using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;

public class Coordinates : MonoBehaviour
{

    public bool leakFound = false;  // Serves as an indicator to freeze the coordinates when the leak is found.
    public static float xcoord;     // Variables for the Corrdinates
    public static float ycoord;
    public static float zcoord;
    public GameObject auvBody;      // GameObject for the AUV
    public GameObject CoordinatesX; // GameObject for the UI elements to display the coordinates. 
    public GameObject CoordinatesY;
    public GameObject CoordinatesZ;
    public float depth;
    public RenderTexture outputCamRenderTexture;
    WaitForEndOfFrame frameEnd = new WaitForEndOfFrame();
    // Update is called once per frame
    void Update()
    {
        if (leakFound == false)                // If 'M' is not pressed, the coordinates will be updated every frame. 
        {
            StartCoroutine(PrintCoord());   // Calls for PrintCoord()
        }

        // If 'M' is pressed, the coordinates are freezed by changing leakFound to true,
        // thus the PrintCoord() funtion will not be called further, freezing the coordinates. 
        if (BallPoseSubscriber.rotation.w == 6)        //Checks for w = 6 i.e. when user presses 'm' on the ROS side.   
        {
            leakFound = true;         //Changes leakFound to true if 'M' is pressed.
        }
        
        //StartCoroutine(printDepth());
    }
    //IEnumerator printDepth()
    //{
    //    yield return frameEnd;
    //    ImageSynthesis.trial.targetTexture = outputCamRenderTexture;
    //    RenderTexture.active = outputCamRenderTexture;
    //    ImageSynthesis.trial.Render();
    //    Texture2D tex = new Texture2D(640, 480, TextureFormat.RGB24, false);
    //    tex.ReadPixels(new Rect(0, 0, Screen.width, Screen.height), 0, 0);
    //    tex.Apply();
    //    depth = tex.GetPixel(255, 255).grayscale;
    //    RenderTexture.active = null;
    //    ImageSynthesis.trial.targetTexture = null;
    //    Debug.Log(depth);
    //    Destroy(tex);
    //}
    IEnumerator PrintCoord()        // Continuisly update and prints the vehicle's coordinates. 
    {
        ycoord = auvBody.transform.position.y;      //using the transform to get the x and y coordinate of the vehiucle.
        xcoord = auvBody.transform.position.x;
        zcoord = SeabedScript.depth;                // Referencing the Seabed Script to get the vehicle's dpeth so that the z coordinate can be obtained.
        // Prints the coordinates on their respective UI by using GetComponent to access the text component of the UI object.
        CoordinatesX.GetComponent<Text>().text = "X: " + xcoord;
        CoordinatesY.GetComponent<Text>().text = "Y: " + ycoord;
        CoordinatesZ.GetComponent<Text>().text = "Z: " + zcoord;
        yield return new WaitForSeconds(1);         //  Wait for one second to allow IEnumerator to funtion. 



    }
}
