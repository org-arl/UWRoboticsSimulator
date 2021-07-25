/* This scipt uses simple chnages in the transform of the AUV rather than applying actual force on the body. 
 * Therefore this script cannot be used regularly as this will not give updates about the body's velocity and accelaration.
 * This is also impractical as in the actual vehicle, the movement will be done by the thrusters applying forces at various points.
 * Therefore this script serves only as a stand-in when quick testing is needed. 
 */


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AUVController : MonoBehaviour
{
    public GameObject waterLevel;                   // GameObject assosicated with the water surface.  
    public GameObject AUVLevel;                     // The AUV's GameObject.

    void Update()
    {
            PlayerMovement();                       //The function PlayerMovemenr() is called for every frame.
    }

    void PlayerMovement()                           // This functions takes input from Unity's built-in axis information and converts them into the vehicles movement. 
    {
        float hor = Input.GetAxis("Horizontal");    // The following lines of code are used to take user input. 
        float ver = Input.GetAxis("Upward");
        float upw = Input.GetAxis("Vertical");
        float horR = Input.GetAxis("xax");          
        float verR = Input.GetAxis("yax");
        float upwR = Input.GetAxis("zax");


        Vector3 playerMovement = new Vector3(-hor, upw, ver) * ButtonFunctions.speed* Time.deltaTime;       // Makes a Vector3 from the user input for translation.
        Vector3 playerRotation = new Vector3(horR, -upwR, verR) * ButtonFunctions.speed * Time.deltaTime;   // Makes a Vector3 from the user input for rotation.

        transform.Translate(playerMovement, Space.Self); //Translates the vehicle in the direction of the Vector3 obtained above.
        transform.Rotate(playerRotation, Space.Self);   //Rotates the vehicle along the Vector3 obtained above.
    }
}