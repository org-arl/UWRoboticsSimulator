/*This script is used to update the UI elements as and when required. These include
 * Mission Time, Battery Percentage, Speed, Angular speed, Accelaration, Temperature,
 * Pressure, Depth, Distance from the Seadbed and Distance from the Control station.
 */



using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIScript : MonoBehaviour
{
    public GameObject timeDisplay01;    // Mission Timer UI GameObject
    public bool isTakingTime = false;   // Act as logic for passing seconds
    public static int theSeconds = 0;          // Stating Mission Time
    public GameObject distanceSeabed;   // Distance from the Seabed UI GameObject
    public GameObject speedText;        // Speed UI GameObject
    public GameObject angspeedText;     // Angular Speed UI GameObject
    public GameObject accText;          // Accelaration UI GameObject
    public GameObject temperatureText;  // Temperature UI GameObject
    public GameObject depthText;        // Depth UI GameObject
    public GameObject pressureText;     // Pressure UI GameObject
    public GameObject batteryText;      // Battery UI GameObject
    public float startingBattery=100;   // Starting Battery Percentage. Can be edited from the Inspector Panel. 
    public static float battery;               // Remaining Battery percentage value
    public GameObject stationText;      // Distance from the control station UI GameObject





    // Update is called once per frame
    void Update()
    {

        // Update Speed, Angular Speed and Accelaration in the UI.  
        speedText.GetComponent<Text>().text= "Speed = " + SeabedScript.speed.ToString("F2") + " m / s";
        angspeedText.GetComponent<Text>().text = "Angular Speed = " + SeabedScript.aSpeed.ToString("F2") + " rad / s";
        accText.GetComponent<Text>().text = "Acceleration = " + SeabedScript.accScalar.ToString("F2") + " m / sqsec";

        // Update Temperature, Pressure and depth values in the UI. The values are referenced from the SeabedScript.
        temperatureText.GetComponent<Text>().text = "Temperature = " + SeabedScript.temperature.ToString("F1") + " °C";
        depthText.GetComponent<Text>().text = "Depth = " + SeabedScript.depth.ToString("F2") + " m";
        pressureText.GetComponent<Text>().text = "Pressure = " + SeabedScript.pressure.ToString("F2") + " kPa";

        // Update Distance from the seabed and from the control station in the UI. The values are referenced from the SeabedScript.
        distanceSeabed.GetComponent<Text>().text = "Distance from Seabed = " + SeabedScript.distanceSeabed.ToString("F2") + " m";
        stationText.GetComponent<Text>().text = "Distance from the Station = " + SeabedScript.relativeDistance.ToString("F2") + " m";

        // Calculate the remaing battery by mapping the time elapsed to 0-100%.
        battery = startingBattery - StartScreen.batteryTime * theSeconds;
        batteryText.GetComponent<Text>().text = "Battery Remaining = " + battery.ToString("F0") + " %";

        //If Battery reaches 0%, all the thrusters become idle and the force multipliers are assigned zero value. 
        if(battery<0)
        {
            ButtonFunctions.speed = 0f;
            ButtonFunctions.speedf = 0f;
            ButtonFunctions.speedr = 0f;
            isTakingTime = true;
        }

        if (isTakingTime == false) // Updates Mission time
        {
            StartCoroutine(AddSecond());
        }
    }

    IEnumerator AddSecond()     // To calculate elaped seconds
    {
        isTakingTime = true;
        theSeconds += 1;        // Add one second 
        timeDisplay01.GetComponent<Text>().text = "Mission Time= " + theSeconds+ " sec";    // Update the Mission Time in the UI
        yield return new WaitForSeconds(1);
        isTakingTime = false;
    }
}
