using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Drone_Manager : MonoBehaviour
{
    public GameObject FollowerPrefab;
    public List<GameObject> drones;

    private void Start()
    {
        List<Vector3> spawnPos = new List<Vector3>();

        Vector3 center = GetRandomPosition();
        for (int i = 0; i < Drone_Values.NumberDrones; i++)
        {
            spawnPos.Add(PlaceOnCircle(center, Drone_Values.R_sense, spawnPos));
        }

        for (int i = 0; i < Drone_Values.NumberDrones; i++)
        {
            GameObject followerDrone = Instantiate(FollowerPrefab, this.transform.parent);
            drones.Add(followerDrone);
            followerDrone.GetComponent<Drone_Agent>().drone_Manager = this;
            followerDrone = ResetDrone(followerDrone);
            followerDrone.transform.localPosition = spawnPos[i];
        }
    }

    Vector3 GetRandomPosition()
    {
        // Return a random position within the simulation environment
        return new Vector3(Random.Range(-40f, 40f), 1.0f, Random.Range(-40.0f, 40.0f));
    }

    Vector3 PlaceOnCircle(Vector3 center, float radius, List<Vector3> already, float threshold = 5f)
    {
        var r = Random.Range(0, radius);
        var angle = Random.Range(0, 2 * Mathf.PI);

        Vector3 pos = center;
        pos.x = center.x + Mathf.Cos(angle) * r;
        pos.z = center.z + Mathf.Sin(angle) * r;

        foreach (var no_spawn in already)
        {
            if (Vector3.Distance(no_spawn, pos) < threshold)
            {
                pos.x = center.x + Mathf.Cos(angle) * r;
                pos.z = center.z + Mathf.Sin(angle) * r;
            }
        }
        return pos;
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
