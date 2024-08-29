using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class Drone_Agent : Agent
{
    protected Rigidbody rBody;
    LayerMask dronesLayer; // For sensing drones
    public Drone_Manager drone_Manager;

    public HashSet<GameObject> sensedDrones = new HashSet<GameObject>();
    public Dictionary<float, float> sensedObjectsDistanceAngle = new Dictionary<float, float>();

    private void Awake()
    {
        dronesLayer = LayerMask.NameToLayer("Drone");
        dronesLayer = 1 << dronesLayer;
    }

    public override void OnEpisodeBegin()
    {
        sensedDrones.Clear();
        sensedObjectsDistanceAngle.Clear();
        rBody = GetComponent<Rigidbody>();
        
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensedDrones = SenseDrones();
        sensedObjectsDistanceAngle = SenseObjects();

        //sensor.AddObservation(transform.localPosition.x);
        //sensor.AddObservation(transform.localPosition.z);
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);

        foreach (var drone in sensedDrones)
        {
            sensor.AddObservation(Vector3.Distance(transform.localPosition, drone.transform.localPosition) / Drone_Values.R_sense);
            sensor.AddObservation(Vector3.SignedAngle(transform.localPosition, drone.transform.localPosition, transform.forward));
        }

        for (int i = 0; i < Drone_Values.NumberDrones - sensedDrones.Count; i++)
        {
            sensor.AddObservation(1);
            sensor.AddObservation(0);
        }

        for (int i = 0; i < sensedObjectsDistanceAngle.Count; i++)
        {
            sensor.AddObservation(sensedObjectsDistanceAngle.Keys.ToArray()[i]);
            sensor.AddObservation(sensedObjectsDistanceAngle.Values.ToArray()[i]);
        }

        for (int i = 0; i < Drone_Values.NumberRays - sensedObjectsDistanceAngle.Count; i++)
        {
            sensor.AddObservation(1);
            sensor.AddObservation(0);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var action = actions.ContinuousActions;

        Vector3 controlSignal = Vector3.zero;

        controlSignal.x = action[0] * Drone_Values.MaxForce;
        controlSignal.z = action[1] * Drone_Values.MaxForce;
        
        Vector3 forceToApply = Vector3.ClampMagnitude(controlSignal, Drone_Values.MaxForce);
        
        Debug.DrawRay(transform.position, forceToApply, Color.green);
        rBody.AddForce(forceToApply);

        //Debug.Log($"Cumulative Reward before Physicomimetics: {GetCumulativeReward()}");
        AddNetReward();
        //Debug.Log($"Cumulative Reward After Physicomimetics: {GetCumulativeReward()}");
    }

    private void OnCollisionEnter(Collision collision)
    {
        SetReward(0);
        drone_Manager.EndEpisodeForAllDrones();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Implement heuristic controls for manual testing
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }

    //private void EndEpisodeForAllDrones()
    //{
    //    for (int i = 0; i < drone_Manager.drones.Count; i++)
    //    {
    //        var agent = drone_Manager.drones[i].GetComponent<Agent>();
    //        agent.EndEpisode();
    //        drone_Manager.drones[i] = drone_Manager.ResetDrone(drone_Manager.drones[i]);
    //    }

    //}

    private void AddNetReward()
    {
        float netReward = 0;
        netReward += AddPhysicomimeticReward();
        netReward += AddObstacleReward();

        if (sensedDrones.Count + sensedObjectsDistanceAngle.Count > 0)
        {
            netReward = netReward/ (sensedDrones.Count + sensedObjectsDistanceAngle.Count);
        }
        SetReward(netReward);
    }

    private float AddPhysicomimeticReward()
    {
        float netReward = 0;
        foreach (var drone in sensedDrones)
        {
            float distance = Vector3.Distance(transform.localPosition, drone.transform.localPosition) / Drone_Values.R_sense;
            var currentReward = 0f;
            if (distance < Drone_Values.R_in / Drone_Values.R_sense)
            {
                currentReward = -1 + distance;
            }
            else
            {
                currentReward = 1 - distance;
            }
            netReward += currentReward;
        }
        return netReward;
    }

    private float AddObstacleReward()
    {
        float netReward = 0;
        foreach (var distanceAngle in sensedObjectsDistanceAngle)
        {
            if (distanceAngle.Key < Drone_Values.R_in / Drone_Values.R_sense)
            {
                netReward += -1 + distanceAngle.Key;
            }
        }
        return netReward;
    }

    private HashSet<GameObject> SenseDrones()
    {
        HashSet<GameObject> sensed = new HashSet<GameObject>();

        Collider[] drones = Physics.OverlapSphere(transform.position, Drone_Values.R_sense, dronesLayer);

        foreach (Collider col in drones)
        {
            if (col.gameObject != this.gameObject)
            {
                sensed.Add(col.gameObject);
                Debug.DrawRay(this.transform.position, col.transform.position - this.transform.position, Color.blue);
            }
        }

        return sensed;
    }

    private Dictionary<float, float> SenseObjects()
    {
        Dictionary<float, float> sensedObjectsPositionAngle = new Dictionary<float, float>();

        Vector3 origin = transform.position;

        float angleIncrement = 360.0f / Drone_Values.NumberRays;

        for (int i = 0; i < Drone_Values.NumberRays; i++)
        {
            // Calculate the direction of the ray
            float angle = i * angleIncrement;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            // Perform the raycast
            if (Physics.Raycast(origin, direction, out RaycastHit hit, Drone_Values.R_sense))
            {
                float distance = Vector3.Distance(origin, hit.point) / Drone_Values.R_sense;

                // Add the local hit point to the list if the ray hits an object
                if (hit.collider.gameObject.tag != "Drone")
                {
                    sensedObjectsPositionAngle[distance] = Vector3.SignedAngle(transform.position, hit.point, transform.forward);
                    Debug.DrawRay(origin, direction * (hit.point - origin).magnitude, Color.red);
                }
                else
                {
                    sensedObjectsPositionAngle[1] = 0;
                }
            }
            else
            {
                sensedObjectsPositionAngle[1] = 0;
            }
        }
        return sensedObjectsPositionAngle;
    }
}
