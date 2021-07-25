// Switches on Unity's Fog when the vehicle's position is below the water level. The color of the fog is preset in Unity settings.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FogStart : MonoBehaviour
{
    public bool AllowFog;

    private bool FogOn;

    public GameObject waterLevel;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
       if  (transform.position.y> waterLevel.transform.position.y) // When vehicle is above water, fog is turned off.
        {
            AllowFog = false;
        }
        if (transform.position.y < waterLevel.transform.position.y) // And its turned on when the vehicle goes under water.. 
        {
            AllowFog = true;
        }
    }


    private void OnPreRender()
    {
        FogOn = RenderSettings.fog;
        RenderSettings.fog = AllowFog; // If the variable Allowfog is true, it switches on the fog and similarly switches it off when the variable is false

    }

    private void OnPostRender()
    {
        RenderSettings.fog = FogOn;
    }
}
