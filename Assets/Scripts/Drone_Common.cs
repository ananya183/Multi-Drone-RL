using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Drone_Common : MonoBehaviour
{
    private Rigidbody Rigidbody;
    
    void Start()
    {
        Rigidbody = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        Vector3 UpForce = - Rigidbody.mass * Physics.gravity;
        Rigidbody.AddForce(UpForce);
    }
}
