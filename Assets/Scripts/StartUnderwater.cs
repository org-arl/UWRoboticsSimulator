// Starts the underwater distortion effects when a particular 
//camera goes below the water level. Also plays the background sounds when the vehicle goes underwater.
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StartUnderwater : MonoBehaviour
{
    public GameObject waterTrigger;
    public GameObject frontCamera;
    public GameObject downCamera;
    public GameObject splash;
    public GameObject uwater;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update() // When a camera goes underwater, the UnderwaterEffect script is enabled for the camera, thus introducing 
        //underwater distortion. It also plays a splash sound when the vehicle goes underwater, and a background sound on loop for underwater sound. 
    {
       if (frontCamera.transform.position.y> waterTrigger.transform.position.y)
        {

            frontCamera.GetComponent<UnderWaterEffects>().enabled = false;
            uwater.SetActive(false);

        }
        if (frontCamera.transform.position.y < waterTrigger.transform.position.y)
        {

            frontCamera.GetComponent<UnderWaterEffects>().enabled = true;
            uwater.SetActive(true);

        }


        if (downCamera.transform.position.y > waterTrigger.transform.position.y)
        {

            downCamera.GetComponent<UnderWaterEffects>().enabled = false;
            splash.SetActive(false);
        }
        if (downCamera.transform.position.y < waterTrigger.transform.position.y)
        {

            downCamera.GetComponent<UnderWaterEffects>().enabled = true;
            splash.SetActive(true);
        }
    }
}
