using System;
using System.Collections.Generic;
using Unity.Mathematics;
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
    public List<Vector2> sensedObjectsPos;

    [SerializeField] private Vector3 force;
    private System.Random random = new System.Random();
    private void Awake()
    {
        dronesLayer = LayerMask.NameToLayer("Drone");
        dronesLayer = 1 << dronesLayer;
    }

    private void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        sensedDrones.Clear();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensedDrones = SenseDrones();
        sensedObjectsPos = SenseObjects();

        sensor.AddObservation(Vector3to2(transform.localPosition));
        sensor.AddObservation(Vector3to2(rBody.velocity));

        //sensor.AddObservation(sensedDrones.Count);
        // Drone Location Input

        AddDroneObservationsRandomly(sensor);

        // Drone Angle Input
        //foreach (var drone in sensedDrones)
        //{
        //    float signedAngle = Vector3.SignedAngle(transform.localPosition, drone.transform.localPosition, transform.forward);
        //    int angle = (int)signedAngle;
        //    if (signedAngle < 0)
        //    {
        //        angle = (int)signedAngle + 360;
        //    }
        //    sensor.AddObservation(angle);
        //}

        //for (int i = 0; i < Drone_Values.NumberDrones - sensedDrones.Count; i++)
        //{
        //    sensor.AddObservation(1);
        //}

        //sensor.AddObservation(sensedObjectsDistance.Count);
        
        // Obstacle Input
        for (int i = 0; i < Drone_Values.NumberRays; i++)
        {
            foreach (var objectPos in sensedObjectsPos)
            {
                sensor.AddObservation(objectPos);
            }
        }

        //// Obstacle Angle Input
        //for (int i = 0; i < Drone_Values.NumberRays; i++)
        //{
        //    sensor.AddOneHotObservation(sensedObjectsAngle[i], Drone_Values.NumberRays);
        //}
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var action = actions.ContinuousActions;

        Vector3 controlSignal = Vector3.zero;

        controlSignal.x = action[0] * Drone_Values.MaxForce;
        controlSignal.z = action[1] * Drone_Values.MaxForce;
        
        Vector3 forceToApply = Vector3.ClampMagnitude(controlSignal, Drone_Values.MaxForce);
        force = forceToApply;
        Debug.DrawRay(transform.position, forceToApply, Color.green);
        rBody.AddForce(forceToApply);

        rBody.velocity = Vector3.ClampMagnitude(rBody.velocity, Drone_Values.MaxSpeed);

        //Debug.Log($"Cumulative Reward before Physicomimetics: {GetCumulativeReward()}");
        AddNetReward();
        //Debug.Log($"Cumulative Reward After Physicomimetics: {GetCumulativeReward()}");
    }

    private void OnCollisionEnter(Collision collision)
    {
        AddReward(-1 * Drone_Values.NumberDrones);
        EndEpisode();
        this.gameObject.SetActive(false);
    }

    

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Implement heuristic controls for manual testing
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }

    private void AddDroneObservationsRandomly(VectorSensor sensor)
    {
        int totalSlots = Drone_Values.NumberDrones - 1;
        Dictionary<int, Vector2> observationSlots = new Dictionary<int, Vector2>();

        // Fill the dictionary with Vector2.positiveInfinity for all slots
        for (int i = 0; i < totalSlots; i++)
        {
            observationSlots[i] = Vector2.positiveInfinity;
        }

        // Randomly assign sensed drone observations
        foreach (var drone in sensedDrones)
        {
            int index;
            do
            {
                index = random.Next(totalSlots);
            } while (observationSlots[index] != Vector2.positiveInfinity);

            Vector3 droneRel = drone.transform.localPosition - this.transform.localPosition;
            observationSlots[index] = Vector3to2(droneRel);
        }

        // Add observations to the sensor in order
        for (int i = 0; i < totalSlots; i++)
        {
            sensor.AddObservation(observationSlots[i]);
        }
    }

    private void AddNetReward()
    {
        float netReward = 0;
        float droneProximityReward = AddDroneProximityReward();        
        netReward +=  droneProximityReward;
        float obstacleProximityReward = AddObstacleProximityReward();
        netReward += obstacleProximityReward;
        //if (sensedDrones.Count + sensedObjectsDistance.Count > 0)
        //{
        //    netReward = netReward/ (sensedDrones.Count + sensedObjectsDistance.Count);
        //}
        //float lostReward = AddLostReward();
        //netReward += lostReward;
        Debug.Log($"drone:{droneProximityReward} obstacle:{obstacleProximityReward} Net:{netReward}");
        AddReward(netReward);
    }

    // Survival Reward

    private float AddDroneProximityReward()
    {
        float netReward = 0;
        foreach (var drone in sensedDrones)
        {
            float distance = Vector3.Distance(transform.localPosition, drone.transform.localPosition) / Drone_Values.R_sense;
            var currentReward = 0f;

            var ratio = Drone_Values.R_out / Drone_Values.R_sense;

            if (distance > ratio)
            {
                currentReward = 1 - distance;
            }
            else
            {
                currentReward = (1f/ratio) * (-ratio + distance);
            }
            netReward += currentReward;
        }
        return netReward;
    }

    private float AddObstacleProximityReward()
    {
        float netReward = 0;
        foreach (var objectPos in sensedObjectsPos)
        {
            var distance = Vector3.Distance(transform.localPosition, objectPos) / Drone_Values.R_sense;
            var ratio = Drone_Values.R_out / Drone_Values.R_sense;
            if (distance < ratio)
            {
                netReward += (1f / ratio) * (-ratio + distance);
                //float e = math.E;
                //float e2 = e * e;
                //netReward += Mathf.Pow(e, -e2 * distance);
            }
        }
        return netReward;
    }

    private float AddLostReward()
    {
        float netReward = 0;
        if (sensedDrones.Count == 0)
        {
            netReward = -1;
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

    private List<Vector2> SenseObjects()
    {
        List<Vector2> hitPos = new List<Vector2>();
        Vector3 origin = transform.position;
        int angleIncrement = 360 / Drone_Values.NumberRays;
        for (int i = 0; i < Drone_Values.NumberRays; i++)
        {
            int angle = angleIncrement * i;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            
            // Perform the raycast
            if (Physics.Raycast(origin, direction, out RaycastHit hit, Drone_Values.R_sense))
            {
                float distance = Vector3.Distance(origin, hit.point) / Drone_Values.R_sense;

                // Add the local hit point to the list if the ray hits an object
                if (hit.collider.gameObject.tag != "Drone")
                {
                    hitPos.Add(Vector3to2(hit.point));
                    Debug.DrawRay(origin, direction * (hit.point - origin).magnitude, Color.red);
                }
                else
                {
                    hitPos.Add(Vector3to2(direction.normalized * Drone_Values.R_sense));
                }
            }
            else
            {
                hitPos.Add(Vector3to2(direction.normalized * Drone_Values.R_sense));
            }
        }

        return hitPos;
    }

    //private void SenseObjects(out List<float> obstacleDistance)
    //{
    //    obstacleDistance = new();

    //    Vector3 origin = transform.position;

    //    foreach (var angle in sensedObjectsAngle)
    //    {
    //        Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

    //        // Perform the raycast
    //        if (Physics.Raycast(origin, direction, out RaycastHit hit, Drone_Values.R_sense))
    //        {
    //            float distance = Vector3.Distance(origin, hit.point) / Drone_Values.R_sense;

    //            // Add the local hit point to the list if the ray hits an object
    //            if (hit.collider.gameObject.tag != "Drone")
    //            {
    //                obstacleDistance.Add(distance);
    //                Debug.DrawRay(origin, direction * (hit.point - origin).magnitude, Color.red);
    //            }
    //            else
    //            {
    //                obstacleDistance.Add(1);
    //            }
    //        }
    //        else
    //        {
    //            obstacleDistance.Add(1);
    //        }
    //    }
    //}

    private Vector2 Vector3to2 (Vector3 v)
    {
        Vector2 v2 = new Vector2 (v.x, v.z);
        return v2;
    }
}
