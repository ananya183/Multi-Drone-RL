using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;

public class Leader_Drone : Drone_Agent
{
    //private Drone_Common drone_common;
    //private Rigidbody rBody;
    private List<Vector3> Waypoints;
    private int currentWaypoint = 0;
    private float threshold = 7f;
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
        drone_common = GetComponent<Drone_Common>();
        //OnEpisodeBegin();
        GetComponent<BehaviorParameters>().BehaviorType = BehaviorType.HeuristicOnly;
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        Waypoints = CreateWaypoints(Drone_Values.NumWaypoints, this.transform.parent);
    }

    private void FixedUpdate()
    {
        // Force
        Vector3 direction = (Waypoints[currentWaypoint] - transform.position).normalized;
        rBody.AddForce(direction * Drone_Values.MaxForce);

        rBody.velocity = Vector3.ClampMagnitude(rBody.velocity, Drone_Values.MaxSpeed) * Drone_Values.LeaderDroneMultiplier;

        //// Increase Waypoint count
        if (Vector3.Distance(transform.position, Waypoints[currentWaypoint]) < threshold)
        {
            currentWaypoint++;
        }


        // End Episode
        if (currentWaypoint >= Drone_Values.NumWaypoints)
        {
            currentWaypoint = 0;
            drone_common.drone_Manager.EpisodeEnded = true;
        }


        // Draw rays
        // 1. From the current position to the next waypoint
        if (currentWaypoint < Waypoints.Count)
        {
            Debug.DrawRay(transform.position, Waypoints[currentWaypoint] - transform.position, Color.green);

            // 2. Between all remaining waypoints
            for (int i = currentWaypoint; i < Waypoints.Count - 1; i++)
            {
                Debug.DrawRay(Waypoints[i], Waypoints[i + 1] - Waypoints[i], Color.white);
            }
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {

    }

    //List<Vector3> CreateWaypoints(int numWayPoints, Transform parent)
    //{
    //    List<Vector3> waypoints = new List<Vector3>();
    //    for (int i = 0; i < numWayPoints; i++)
    //    {
    //        float minMaxValue = Drone_Values.TrainingAreaSize - Drone_Values.WaypointAreaThreshold;
    //        Vector3 waypoint = new Vector3(parent.position.x + Random.Range(-minMaxValue, minMaxValue), Drone_Values.DroneHeight, parent.position.z + Random.Range(-minMaxValue, minMaxValue));
    //        waypoints.Add(waypoint);
    //    }
    //    return waypoints;
    //}

    List<Vector3> CreateWaypoints(int numWayPoints, Transform parent)
    {
        List<Vector3> waypoints = new List<Vector3>();
        float minMaxValue = Drone_Values.TrainingAreaSize - Drone_Values.WaypointAreaThreshold;
        int maxAttempts = 50000000; // Max attempts to find valid waypoints
        float obstacleAvoidanceRadius = 3f; // Radius to avoid obstacles

        for (int i = 0; i < numWayPoints; i++)
        {
            Vector3 waypoint = Vector3.zero;
            bool validWaypoint = false;
            int attempts = 0;

            // Try to find a valid waypoint that avoids obstacles
            while (!validWaypoint && attempts < maxAttempts)
            {
                // Generate a random waypoint position
                waypoint = new Vector3(
                    parent.position.x + Random.Range(-minMaxValue, minMaxValue),
                    Drone_Values.DroneHeight,
                    parent.position.z + Random.Range(-minMaxValue, minMaxValue)
                );

                // Check if the waypoint is near an obstacle
                if (!IsNearObstacle(waypoint, obstacleAvoidanceRadius))
                {
                    validWaypoint = true;
                }

                attempts++;
            }

            if (validWaypoint)
            {
                waypoints.Add(waypoint);
            }
            else
            {
                Debug.LogWarning("Could not find a valid waypoint for index: " + i);
            }
        }

        return waypoints;
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
