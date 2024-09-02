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
        LeaderDrone.GetComponent<Leader_Drone>().drone_manager = this;
        drones.Add(LeaderDrone);
        for (int i = 0; i < Drone_Values.NumberDrones - 1; i++)
        {
            GameObject followerDrone = Instantiate(FollowerPrefab, this.transform.parent);
            followerDrone.GetComponent<Drone_Agent>().drone_Manager = this;
            drones.Add(followerDrone);
        }

        foreach (GameObject drone in drones)
        {
            drone.SetActive(false);
        }
    }

    public void StartEpisodeForAllDrones()
    {
        foreach (var drone in drones)
        {
            drone.SetActive(true);
            var agent = drone.GetComponent<Drone_Agent>();
            if (agent != null)
            {
                agent.OnEpisodeBegin();
            }
            var leader_agent = drone.GetComponent<Leader_Drone>();
            if (leader_agent != null)
            {
                leader_agent.OnEpisodeBegins();
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
            each.SetActive(false);
        }
        currentAction= 0;
        Drone_Values.CurrentEpisode++;

        OnEpisodeBegins();
    }

    private void SpawnDrones()
    {
        List<Vector3> spawnPos = new List<Vector3>();
        Vector3 center = GetRandomPosition(Drone_Values.TrainingAreaSize - Drone_Values.R_spawn - 5f);
        int i = 0;
        while (i < drones.Count) 
        {
            ResetDrone(drones[i]);
            spawnPos.Add(PlaceOnCircle(center, Drone_Values.R_spawn, spawnPos));
            drones[i].transform.localPosition = spawnPos[i];
            float angle = Random.Range(0, 360);
            drones[i].transform.localRotation = Quaternion.Euler(0, angle ,0);
            i++;
        }
    }

    private void InitializeDrones()
    {
        InstantiateDrones();
        OnEpisodeBegins();
    }

    private void ReInitializeDrones()
    {
        SpawnDrones();
        StartEpisodeForAllDrones();
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
        return new Vector3(Random.Range(-size, size), Drone_Values.DroneHeight, Random.Range(-size, size));
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
        drone.transform.localPosition = Vector3.zero;
        drone.transform.localRotation = Quaternion.identity;
        
        var rBody = drone.GetComponent<Rigidbody>();
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;

        return drone;
    }
}
