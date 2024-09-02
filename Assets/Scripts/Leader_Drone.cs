using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Leader_Drone : MonoBehaviour
{
    public Drone_Manager drone_manager;

    private Rigidbody rb;
    private List<Vector3> Waypoints;
    private int currentWaypoint = 0;
    private float threshold = 5f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        OnEpisodeBegins();
    }

    public void OnEpisodeBegins()
    {
        Waypoints = CreateWaypoints(Drone_Values.NumWaypoints, this.transform.parent);
    }

    private void FixedUpdate()
    {

        // Force
        Vector3 direction = (Waypoints[currentWaypoint] - transform.position).normalized;
        rb.AddForce(direction * Drone_Values.MaxForce * Drone_Values.LeaderDroneMultiplier);

        rb.velocity = Vector3.ClampMagnitude(rb.velocity, Drone_Values.MaxSpeed);

        // Increase Waypoint count
        if (Vector3.Distance(transform.position, Waypoints[currentWaypoint]) < threshold)
        {
            currentWaypoint++;
        }


        // End Episode
        if (currentWaypoint >= Drone_Values.NumWaypoints)
        {
            currentWaypoint = 0;
            drone_manager.EpisodeEnded = true;
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

    List<Vector3> CreateWaypoints(int numWayPoints, Transform parent)
    {
        List<Vector3> waypoints = new List<Vector3>();
        for (int i = 0; i < numWayPoints; i++)
        {
            float minMaxValue = Drone_Values.TrainingAreaSize - Drone_Values.R_spawn;
            Vector3 waypoint = new Vector3(parent.position.x + Random.Range(-minMaxValue, minMaxValue), Drone_Values.DroneHeight, parent.position.z + Random.Range(-minMaxValue, minMaxValue));
            waypoints.Add(waypoint);
        }
        return waypoints;
    }
}
