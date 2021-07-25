/* This scipt allows the user to control the AUV by applying forces on the vehicle according to the user input.
 * This is the preferred method as it mimics the real world closely.
 */


using ROSBridgeLib;
using ROSBridgeLib.sensor_msgs;
using System.Collections;
using SimpleJSON;
using UnityEngine;

public class ForceControls : MonoBehaviour
{


	public GameObject vehicle;      // Vehicle's GameObject
	public Rigidbody rb;


	void Update()
	{
		// The following lines of code are used to import control values coming from ROS,  subscribed by the Subscriber script
		float sway = BallPoseSubscriber.position.y;           
		float heave = BallPoseSubscriber.position.z;
		float surge = BallPoseSubscriber.position.x;
		float yaw = BallPoseSubscriber.rotation.z;
		float roll = BallPoseSubscriber.rotation.y;
		float pitch = BallPoseSubscriber.rotation.x;

		Vector3 movement = new Vector3(-surge, heave, sway);        // Make a vector for translation using the user input. 
		Vector3 rotation = new Vector3(roll, yaw, -1 * pitch);       // Make a vector for rotation using the user input. 
																	 // Add relative force, i.e. force in the local coordinate system, using the Vector3 created by user Input. 
		rb.AddRelativeForce(movement * ButtonFunctions.speedf);

		// Add torque on the vehicle in the global coordinate system.
		// Separate commands are used for each direction so that rotation speed in each direction can be controlled individually. 
		rb.AddTorque(transform.up * ButtonFunctions.speedr * yaw * 2);
		rb.AddTorque(transform.right * (ButtonFunctions.speedr * (roll)));
		rb.AddTorque(transform.forward * ButtonFunctions.speedr * (pitch) * 2);
		
		// If the suer wishes to use a vector3 to rotate, then this command will be used instead of the individual rotations. 
		//rb.AddTorque(rotation * ButtonFunctions.speedr);	
	}


}
