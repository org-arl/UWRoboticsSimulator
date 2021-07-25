/*This script maps some of the button functions used in the simulation
 * The script allows for the release of the AUV from the control station when 'R' is pressed. 
 * This is done by switching on the gravity and giving non zero speed to movement speed values.
 * This script also controls the forward and downward illumination using the UI buttons.
 * It also contains optional fuinctions to switch camera views when the user dowsnot want to have two cameras in the smae view. 
 */


using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI; // To allow us to use UI elements. 
using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;

public class ButtonFunctions : MonoBehaviour
{
    public Camera frontCamera;              // Gameobject for the front camera.
    public Camera downCamera;               // GameObject for the Downward Camera
    public Rigidbody auvBody;               // RigidBody of the AUV. Can also be extracted by using GetComponent on the AUV GameObject. 
    public GameObject waterLevel;           // GameObject associated with the water surface.
    public GameObject AUVLevel;             // Gameobject of the AUV. 
    public GameObject frontLight;           // GameObject for the Front Light.
    public GameObject frontLight2;
    public GameObject downLight;            // GameObject for the Down Light. 
    public GameObject downLight2;
    public bool fL = false;                 // Use to monitor whether the lights are switched on or off so that they can be toggled. 
    public bool dL = false;

    public static float speed = 0f;           // To be used in AUVControl speed as Vehicle's Translation and Rotation Speed. 
    public static float speedf = 0f;        //To be used in ForceControls as Vehicle's translatation force multiplier. 
    public static float speedr = 0f;        //To be used in ForceControls as Vehicle's Torque multiplier.

    public GameObject rosConnector;

    void Start()
    {
        //RosConnector.RosBridgeServerUrl = StartScreen.IPAddress;
        //rosConnector.SetActive(true);
        // Debug.Log(StartScreen.IPAddress);
    }


    void Update()
    {
        //       if (Input.GetKeyDown("p"))       // Switches the view to the front camera when using only one camera view. 
        //          {
        //              FrontCamera();
        //          }
        //       if (Input.GetKeyDown("t"))       // Switches the view to the down camera when using only one camera view. 
        //          {
        //              DownCamera();
        //      }
        if (BallPoseSubscriber.rotation.w == 1)                 // Releases the AUV when w is 1 i.e R is pressed on the ROS side 
        {
            if (waterLevel.transform.position.y < AUVLevel.transform.position.y) // Checks if the vehicle is above water. 
            {
                releaseAUV();   // Calls for the releaseAUV function. 

            }
            if (waterLevel.transform.position.y > AUVLevel.transform.position.y)
            {
                auvBody.GetComponent<Rigidbody>().useGravity = false;
            }
        }


        if (BallPoseSubscriber.rotation.w == 2) // w = 2 when user presse '[' on ROS side 
        {
            frontLight.SetActive(false); // Switches the front lights off
            frontLight2.SetActive(false);
        }
        if (BallPoseSubscriber.rotation.w == 3) // w = 3 when user presse ']' on ROS side
        {
            frontLight.SetActive(true); // Switches the front light on
            frontLight2.SetActive(true);
        }

        if (BallPoseSubscriber.rotation.w == 4)  // w = 4 when user presse ';' on ROS side
        {
            downLight.SetActive(false); //Switches the down light off.
            downLight2.SetActive(false);
        }
        if (BallPoseSubscriber.rotation.w == 5)  // w = 5 when user presse <'> on ROS side
        {
            downLight.SetActive(true); //Switches the down light on.
            downLight2.SetActive(true);
        }
    }


    //This function is to release the AUV from the control Station.
    public void releaseAUV()
    {
        auvBody.GetComponent<Rigidbody>().useGravity = true; // Switches on gravity.
        speed = 3f;                                          // Sets speed as 3 for AUVControl script.
        speedf = 100000f;                                     // Sets translation force multiplier as 10,000 for ForceControls script.
        speedr = 3000f;                                       // Sets Torque multiplier as 300 for ForceControls script. 
    }
    // This Function is to enable the front camera and disable tha down camera.
    public void Downcamera()
    {
        frontCamera.enabled = false;
        downCamera.enabled = true;
    }

    // This Function is to enable the down camera and disable tha front camera.
    public void FrontCamera()
    {
        frontCamera.enabled = true;
        downCamera.enabled = false;
    }

    // Toggles the Front light when the UI button is pressed. 
    public void fronLight()
    {

        if (fL == false)
        {
            frontLight.SetActive(true);
            frontLight2.SetActive(true);
            fL = true;
        }
        else if (fL == true)
        {
            frontLight.SetActive(false);
            frontLight2.SetActive(false);
            fL = false;
        }
    }

    // Toggles the Down light when the UI button is pressed. 
    public void doLight()
    {
        if (dL == false)
        {
            downLight.SetActive(true);
            downLight2.SetActive(true);
            dL = true;
        }
        else if (dL == true)
        {
            downLight.SetActive(false);
            downLight2.SetActive(false);
            dL = false;
        }
    }
}
