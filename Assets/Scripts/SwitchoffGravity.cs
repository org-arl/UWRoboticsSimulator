/* This is an optional script when the user doesn't want to deal with actual Buoyancy force. 
 * This is especially useful when there is no ROS integration as buoyancy force without ROS's 
 * stabalization algorithms will case the user problems controlling the vehicles as there will 
 * be a continuous force in the upwards direction, which might also impart some moment. 
 * This script switches off the gravity when the vehicle enter water. 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SwitchoffGravity : MonoBehaviour
{
    public Rigidbody rigidb;
    public GameObject vehicleMesh;

    void Update()
    {

    }

    void OnTriggerEnter(Collider other)
    {
        if (other.tag == "Player")
        {
            rigidb.GetComponent<Rigidbody>().useGravity = false;

        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.tag == "Player")
        {

            vehicleMesh.transform.Translate(Vector3.down, Space.Self);

        }
    }


}

