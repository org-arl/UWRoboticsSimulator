/*This script is used to calculate the parameters and values used by various other scripts.
 * This script serve as the sensor array of teh AUV.
 * It calculates the Speed, angular speed, acceleration, carious distances, temperature, pressure, depth and warning LED funtions. 
 */


using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;       // Allows us to use UI Elements and Functions.

public class SeabedScript : MonoBehaviour
{
    public static float distanceSeabed;     // Variable for distance from the seabed value.
    public GameObject AUVehicle;            // AUV GameObject
    public GameObject seaBed;               // Seabed GameObject
    public GameObject colgreen;             // Green Collision Warning UI Element
    public GameObject colred;               // Red Collision Warning UI Element
    public GameObject proxgreen;            // Green Proximity Warning UI Element
    public GameObject proxred;              // Red Proximity Warning UI Element
    public GameObject rangegreen;           // Green Range Warning UI element
    public GameObject rangered;             // Red Range Warning UI element
    public static float speed;              // Speed Float Variable
    public static Vector3 vel;              // Velocity Vector3
    public static float aSpeed;             // Angular Speed Float Variable
    public static Vector3 aVel;             // Andular Velocity Vector3
    public static Vector3 acceleration;     // Acceleration Vector3
    public static Vector3 lastVelocity=Vector3.zero;    // Last velocity Vector for Acceleration Calculation
    public static float accScalar;                      // Acceleration Float Variable
    public static float temperature;        // Temperature Float Variable
    public static float depth;              // Depth Float Variable
    public static float pressure;           // Pressure Float Variable
    public GameObject controlStation;       // GameObejct for Control Station
    public static float relativeDistance;   // Float Variable for Distance between the Control station and the AUV.



    // Update is called once per frame
    void Update()
    {
        
       vel = AUVehicle.GetComponent<Rigidbody>().velocity;                  // To get a Vector3 representation of the velocity
        speed = vel.magnitude;                                              // To get magnitude i.e Speed
        aVel = AUVehicle.GetComponent<Rigidbody>().angularVelocity;         // To  get a Vector3 representation of the angular velocity
        aSpeed = aVel.magnitude;                                            // To get angular speed scalar
        acceleration = (AUVehicle.GetComponent<Rigidbody>().velocity - lastVelocity) / Time.fixedDeltaTime;     // Caculate Acceleration Vector
        lastVelocity = AUVehicle.GetComponent<Rigidbody>().velocity;        // Update last velocity value
        accScalar = acceleration.magnitude;                                 // Calculate Acceleration scalar

        // Calculate the AUV's distance from the seabed
        distanceSeabed = AUVehicle.transform.position.y - seaBed.transform.position.y;  
            

        // Calculate AUV's depth below the water surface, its surrounding temperature and pressure. 
        depth = 41.5f - distanceSeabed;
        pressure = Mathf.Max(101.325f + (depth * 997f * 9.8f) / 1000f, 101.325f);
        temperature = 0.144f * distanceSeabed + 18f;

        // Caculate AUV's distance from the Control Station
        relativeDistance = Vector3.Distance(AUVehicle.transform.position, controlStation.transform.position);


        // Determine if the AUV is reching its Range limit and Swap the green UI image 
        //with the red one to mimic an LED turning from green to red as a warning LED.
        if (relativeDistance < 110)         // If the Distance between the AUV and the Control station is less than 110, then the AUV is in range
        {
            rangegreen.SetActive(true); // Set the green LED as true, indicating within range
            rangered.SetActive(false);  // Keeping the RED image as inactive
        }
        // Otherwise, glow the RED LED for Range
        if (relativeDistance > 110)
        {
            rangegreen.SetActive(false);    // Making the green image inactive
            rangered.SetActive(true);       // Making the Red LED active, indicating a warning sign that the AUV is near its range limit. 
        }

        // Checking if the AUV is very close to the Seabed
        if (distanceSeabed < 10)
        {
            proxred.SetActive(true);
            proxgreen.SetActive(false);
        }
        if (distanceSeabed > 10)
        {
            proxred.SetActive(false);
            proxgreen.SetActive(true);
        }


    }

    // Checking if the AUV has collided with the Seabed.
    public void OnTriggerEnter(Collider other)
    {
        //Giving the AUV a "Player" tag allows us to check whether any body with Player tag has collided with the Seabed mesh. 
        //If yes, then the collision Red LED for the AUV is activated. 
        if (other.tag == "Player")       
        {
            colred.SetActive(true);
            colgreen.SetActive(false);
        }
    }
        // The collison warning LED is switchd off when the vehicles exits the seabed collider. 
    public void OnTriggerExit(Collider other)
    {
        if (other.tag == "Player")
        {

            colred.SetActive(false);
            colgreen.SetActive(true);
        }
    }
}
