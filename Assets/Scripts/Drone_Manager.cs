using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Drone_Manager : MonoBehaviour
{
    public GameObject FollowerPrefab;
    public List<GameObject> drones;

    private void Start()
    {
        for (int i = 0; i < Drone_Values.NumberDrones; i++)
        {
            GameObject followerDrone = Instantiate(FollowerPrefab, this.transform.parent);
            drones.Add(followerDrone);
            followerDrone = ResetDrone(followerDrone);
            followerDrone.GetComponent<Drone_Agent>().drone_Manager = this;
        }
    }

    Vector3 GetRandomPosition()
    {
        // Return a random position within the simulation environment
        return new Vector3(Random.Range(-50f, 50f), 1.0f, Random.Range(-50.0f, 50.0f));
    }

    public GameObject ResetDrone(GameObject drone)
    {
        drone.transform.localPosition = GetRandomPosition();
        drone.transform.localRotation = Quaternion.identity;
        
        var rBody = drone.GetComponent<Rigidbody>();
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;

        return drone;
    }
}
