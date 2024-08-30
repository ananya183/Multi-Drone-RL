using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Policies;

public class Drone_Manager : MonoBehaviour
{
    public GameObject FollowerPrefab;
    public GameObject LeaderPrefab;
    public List<GameObject> drones;
    public GameObject LeaderDrone;
    public bool EpisodeEnded;

    public int currentAction = 0;

    private void Start()
    {
        InitializeDrones();
    }

    private void InstantiateDrones()
    {
        LeaderDrone = Instantiate(LeaderPrefab, this.transform.parent);
        drones.Add( LeaderDrone);
        for (int i = 0; i < Drone_Values.NumberDrones - 1; i++)
        {
            GameObject followerDrone = Instantiate(FollowerPrefab, this.transform.parent);
            followerDrone.GetComponent<Drone_Agent>().drone_Manager = this;
            drones.Add(followerDrone);
        }
    }

    public void startEpisodeForAllDrones()
    {
        foreach (var drone in drones)
        {
            drone.SetActive(true);
            var agent = drone.GetComponent<Drone_Agent>();
            if (agent != null)
            {
                agent.OnEpisodeBegin();
            }
        }
    }

    void OnEpisodeBegins()
    {
        EpisodeEnded = false;
        ReInitializeDrones();
    }

    public void EndEpisodeForAllDrones()
    {
        foreach (var each in drones)
        {
            var agent = each.GetComponent<Agent>();
            if (agent != null)
            {
                agent.EndEpisode();
            }
        }
        currentAction= 0;
        Drone_Values.CurrentEpisode++;

        OnEpisodeBegins();
    }

    private void SpawnDrones()
    {
        List<Vector3> spawnPos = new List<Vector3>();
        Vector3 center = GetRandomPosition(Drone_Values.TrainingAreaSize);
        int i = 0;
        while (i < drones.Count) 
        {
            ResetDrone(drones[i]);
            spawnPos.Add(PlaceOnCircle(center, Drone_Values.R_sense, spawnPos));
            drones[i].transform.localPosition = spawnPos[i];
            i++;
        }
    }

    private void InitializeDrones()
    {
        InstantiateDrones();
        SpawnDrones();
    }

    private void ReInitializeDrones()
    {
        SpawnDrones();
        startEpisodeForAllDrones();
    }

    private void FixedUpdate()
    {
        if (currentAction > Drone_Values.NumActionsInEpisode)
            EpisodeEnded = true;
        if (EpisodeEnded)
        {
            EndEpisodeForAllDrones();
        }
        currentAction++;
    }

    Vector3 GetRandomPosition(float size)
    {
        // Return a random position within the simulation environment
        return new Vector3(Random.Range(-size, size), 1.0f, Random.Range(-size, size));
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
        drone.transform.localPosition = GetRandomPosition(Drone_Values.TrainingAreaSize);
        drone.transform.localRotation = Quaternion.identity;
        
        var rBody = drone.GetComponent<Rigidbody>();
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;

        return drone;
    }
}
