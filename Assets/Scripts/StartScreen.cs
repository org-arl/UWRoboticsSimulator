// Script controlling the elements of the main menu including user input for IP address, 
// port number, and mission timing. Takes these inputs and uses them in the simulator and 
// redirects to the simulator screen when 'Start' is pressed.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class StartScreen : MonoBehaviour
{

    public AudioSource buttonPress; // Audio source for the button press sound
    public static string IPAddress; // To store the user input IP address
    public GameObject userInput; // The input field for IP address
    public static float batteryTime; // To store the Mission maximum time
    public GameObject batteryInput; // The input field for battery time
    public GameObject port; // The input field for port number
    public static string portNumber; // To store the user input port number
    public static int raynumber = 512;
    public static int range = 55; //Default 
    public static int modeltrigger = 2;
    // Start is called before the first frame update
    void Start()
    {
        Cursor.visible = true;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void PlayGame()
    {
        
        buttonPress.Play();
        if (userInput.GetComponent<InputField>().text == "") // If user input is blank, IP address takes the local host address as default value
        {
            IPAddress = "ws://localhost";
        }
        else
        {

            IPAddress = userInput.GetComponent<InputField>().text; // Otherwise, the user input value is taken as the IP address
            
        }

        if (port.GetComponent<InputField>().text == "") //If user input is blank, port takes 9090 as default value
        {
            portNumber = "9090";
        }
        else
        {

            portNumber = port.GetComponent<InputField>().text; // Otherwise, the user input value is taken as the port number

        }


        if (batteryInput.GetComponent<InputField>().text == "") //If user input is blank, battery takes 0.05  as default value
        {
            batteryTime=0.05f;
        }
        else
        {
            batteryTime = 100f / float.Parse(batteryInput.GetComponent<InputField>().text); // Otherwise, the user input value is taken as the battery parameter.
        }
       
        Cursor.visible = false;
        SceneManager.LoadScene(2);
    }

    public void QuitGame() // To quit the simulator when 'Quit' is pressed
    {
        buttonPress.Play();
        Application.Quit();
    }

    public void SingleSwath600()
    {
        raynumber = 512;
        range = 95;
        modeltrigger = 1;
    }
    public void SingleSwath700()
    {
        raynumber = 512;
        range = 55;
        modeltrigger = 2;
    }

    public void DualSwath600()
    {
        raynumber = 1024;
        range = 95;
        modeltrigger = 3;

    }
    public void DualSwath700()
    {
        raynumber = 1024;
        range = 55;
        modeltrigger = 4;
    }

}
