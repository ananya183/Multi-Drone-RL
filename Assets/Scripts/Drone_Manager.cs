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
        SpawnDrones();
        ActivateDrones();

    }

    void OnEpisodeBegins()
    {
        EpisodeEnded = false;
        SpawnDrones();
        ActivateDrones();
        StartEpisodeForAllDrones(); // Ensure episodes are started after reinitialization
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
        currentAction = 0;
        Drone_Values.CurrentEpisode++;

        OnEpisodeBegins();
    }

    //private void InstantiateDrones()
    //{
    //    LeaderDrone = Instantiate(LeaderPrefab, this.transform.parent);
    //    LeaderDrone.GetComponent<Leader_Drone>().drone_manager = this;
    //    drones.Add(LeaderDrone);
    //    for (int i = 0; i < Drone_Values.NumberDrones - 1; i++)
    //    {
    //        GameObject followerDrone = Instantiate(FollowerPrefab, this.transform.parent);
    //        followerDrone.GetComponent<Drone_Agent>().drone_Manager = this;
    //        drones.Add(followerDrone);
    //    }

    //    foreach (GameObject drone in drones)
    //    {
    //        drone.SetActive(false);
    //    }
    //}

    public void StartEpisodeForAllDrones()
    {
        foreach (var drone in drones)
        {
            var agent = drone.GetComponent<Drone_Agent>();
            if (agent != null)
            {
                agent.OnEpisodeBegin();
            }
            var leader_agent = drone.GetComponent<Leader_Drone>();
            if (leader_agent != null)
            {
                leader_agent.OnEpisodeBegin();
            }

        }

    }





    //private void SpawnDrones()
    //{
    //    List<Vector3> spawnPos = new List<Vector3>();
    //    Vector3 center = GetRandomPosition(Drone_Values.TrainingAreaSize - Drone_Values.R_spawn - 5f);
    //    int i = 0;
    //    for (i = 0; i < drones.Count; i++)
    //    {
    //        ResetDrone(drones[i]);
    //        spawnPos.Add(PlaceOnCircle(center, Drone_Values.R_spawn, spawnPos));
    //        drones[i].transform.localPosition = spawnPos[i];
    //        float angle = Random.Range(0, 360);
    //        drones[i].transform.localRotation = Quaternion.Euler(0, angle, 0);
    //    }
    //}

    private void SpawnDrones()
    {
        List<Vector3> spawnPos = new List<Vector3>();
        Vector3 center = GetRandomPosition(Drone_Values.TrainingAreaSize - Drone_Values.R_spawn - 5f);
        int i = 0;
        for (i = 0; i < drones.Count; i++)
        {
            Vector3 newSpawnPos;
            bool validSpawn = false;

            // Try multiple positions until we find one that is not too close to obstacles
            int maxAttempts = 50000000; // Limit the number of attempts to find a valid spawn position
            int attempts = 0;

            while (!validSpawn && attempts < maxAttempts)
            {
                newSpawnPos = PlaceOnCircle(center, Drone_Values.R_spawn, spawnPos);

                // Check if there are any obstacles within 3 units of the spawn position
                if (!IsNearObstacle(newSpawnPos, 3f))
                {
                    validSpawn = true; // Valid spawn position found
                    spawnPos.Add(newSpawnPos);
                    drones[i].transform.localPosition = newSpawnPos;
                    float angle = Random.Range(0, 360);
                    drones[i].transform.localRotation = Quaternion.Euler(0, angle, 0);
                }
                attempts++;
            }

            // Reset the drone once a valid spawn position is found
            ResetDrone(drones[i]);

            // If max attempts were reached without finding a valid spot, handle accordingly
            if (attempts >= maxAttempts)
            {
                Debug.LogWarning("Could not find a valid spawn position for drone " + i);
                drones[i].SetActive(false); // Optionally disable the drone if no valid spawn is found
            }
        }
    }

    private void  InitializeDrones()
    {
        // Instantiate and add the leader drone
        LeaderDrone = Instantiate(LeaderPrefab, this.transform.parent);
        LeaderDrone.GetComponent<Drone_Common>().drone_Manager = this;
        drones.Add(LeaderDrone);


        // Randomising the number of drones being spawned in every new episode.
        NumberDrones = Random.Range(2, maxNumberDrones+1); //Randomise from 1 to maxNumDrones
        Debug.Log($"Number of Drones Randomised to: {numberDrones}");

        // Instantiate and add follower drones
        for (int i = 0; i < Drone_Values.NumberDrones - 1; i++)
        {
            GameObject followerDrone = Instantiate(FollowerPrefab, this.transform.parent);
            followerDrone.GetComponent<Drone_Common>().drone_Manager = this;
            drones.Add(followerDrone);
        }
    }

    private void ActivateDrones()
    {
        // Activate each drone after placing them
        foreach (GameObject drone in drones)
        {
            drone.SetActive(true);
        }
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
        Vector3 pos = center;

        foreach (var no_spawn in already)
        {

            do
            {
                var r = Random.Range(0, radius);
                var angle = Random.Range(0, 2 * Mathf.PI);
                pos.x = center.x + Mathf.Cos(angle) * r;
                pos.z = center.z + Mathf.Sin(angle) * r;
            } while ((Vector3.Distance(no_spawn, pos) < threshold) &&
                    Mathf.Abs(pos.x) < Drone_Values.TrainingAreaSize &&
                    Mathf.Abs(pos.z) < Drone_Values.TrainingAreaSize);
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

    private bool IsNearObstacle(Vector3 position, float radius)
    {
        // Check for any colliders with the tag "Obstacle" within the specified radius
        Collider[] colliders = Physics.OverlapSphere(position, radius);
        foreach (var collider in colliders)
        {
            if (collider.CompareTag("Obstacle"))
            {
                // If an obstacle is found within the radius, return true
                return true;
            }
        }
        // If no obstacles are found within the radius, return false
        return false;
    }

}
