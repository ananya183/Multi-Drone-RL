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
    protected Drone_Common drone_common;

    // Reward Valuations in High to Low Order
    private readonly float swarmationReward = 200.0f;
    private readonly float insideGoodRegionReward = 40.0f;
    private readonly float insideSensingZoneReward = 10.0f;
    private readonly float intraSwarmCollisionPenalty = -80.0f;
    private readonly float obstacleCollisionPenalty = -60.0f;
    private readonly float boundaryCollisionPenalty = -60.0f;
    private readonly float tooClosePenalty = -30.0f;
    private readonly float insideBadRegionPenalty = -10.0f;

    private readonly float rr_factor = 0.5f;

    private bool collidedWithObstacle = false;
    private bool collidedWithDrone = false;
    private bool collidedWithBoundary = false;


    public List<Vector2> sensedObjectsPos;
    //private Drone_Common droneCommon;

    [SerializeField] float TotalReward;


    // [SerializeField] private Vector3 force;
    private System.Random random = new System.Random();
    private void Awake()
    {
        dronesLayer = LayerMask.NameToLayer("Drone");
        dronesLayer = 1 << dronesLayer;
    }

    private void Start()
    {
        rBody = GetComponent<Rigidbody>();
        drone_common = GetComponent<Drone_Common>();
    }

    public override void OnEpisodeBegin()
    {
        TotalReward = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //sensor.AddOneHotObservation(sensedDrones.Count, Drone_Values.NumberDrones - 1);
        sensedObjectsPos = SenseObjects();



        var localVel = transform.InverseTransformDirection(rBody.velocity);
        sensor.AddObservation(Vector3to2(localVel) / Drone_Values.MaxSpeed); // normalised velocity input

        // Drone Location Input

        AddDroneObservationsRandomly(sensor);

        // Obstacle Input
        for (int i = 0; i < Drone_Values.NumberRays; i++)
        {
            sensor.AddObservation(sensedObjectsPos[i] / Drone_Values.R_sense);
        }

        // Adding Input of prevSensed, current sensed and total sensed to NN
        sensor.AddObservation(drone_common.prevSensed);
        sensor.AddObservation(drone_common.currentSensed);
        sensor.AddObservation(drone_common.totalSensed);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var action = actions.ContinuousActions;

        Vector3 controlSignal = Vector3.zero;

        controlSignal.x = action[0] * Drone_Values.MaxForce;
        controlSignal.z = action[1] * Drone_Values.MaxForce;

        Vector3 forceToApply = Vector3.ClampMagnitude(controlSignal * Drone_Values.MaxForce, Drone_Values.MaxForce);

        var localForce = transform.InverseTransformDirection(forceToApply);
        //Debug.DrawLine(transform.position, transform.position + localForce, Color.green);
        rBody.AddForce(localForce);

        rBody.velocity = Vector3.ClampMagnitude(rBody.velocity, Drone_Values.MaxSpeed);

        //Debug.Log($"Cumulative Reward before Physicomimetics: {GetCumulativeReward()}");
        AddNetReward();
        //Debug.Log($"Cumulative Reward After Physicomimetics: {GetCumulativeReward()}");

        if (drone_common.HasJustFormedSwarm())
        {
            var reboundReward = swarmationReward * rr_factor * (drone_common.swarmDrones.Count - 1);
            AddReward(swarmationReward);
            AddReward(reboundReward);
        }

        if (drone_common.IsInBadZone())
        {
            AddReward(insideBadRegionPenalty);
        }

        if (drone_common.IsInGoodZone())
        {
            AddReward(insideGoodRegionReward);
        }

        if (drone_common.IsTooCloseToOtherDrone())
        {
            AddReward(tooClosePenalty);
        }

        if (drone_common.DroneInSense())
        {
            AddReward(insideSensingZoneReward);
        }

        if (CollidedWithObstacle())
        {
            AddReward(obstacleCollisionPenalty);
            this.gameObject.SetActive(false);
            EndEpisode();
        }

        if (CollidedWithDrone())
        {
            AddReward(intraSwarmCollisionPenalty);
            this.gameObject.SetActive(false);
            EndEpisode();
        }

        if (CollidedWithBoundary())
        {
            AddReward(boundaryCollisionPenalty);
            this.gameObject.SetActive(false);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        foreach (var drone in drone_common.drone_Manager.drones)
        {
            var agent = drone.GetComponent<Agent>();
            if (agent != null)
                //agent.SetReward(-1);
                if (collision.gameObject.tag == "Drone")
                {
                    collidedWithDrone = true;
                }
                else if (collision.gameObject.tag == "Obstacle")
                {
                    collidedWithObstacle = true;
                }
                else if (collision.gameObject.tag == "Boundary")
                {
                    collidedWithBoundary = true;
                }
        }
        drone_common.drone_Manager.EpisodeEnded = true;
        //EndEpisode();
        //this.gameObject.SetActive(false);
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
        int totalSlots = Drone_Values.MaxNumberDrones - 1;
        Dictionary<int, Vector2> observationSlots = new Dictionary<int, Vector2>();

        // Fill the dictionary with Vector2.positiveInfinity for all slots
        for (int i = 0; i < totalSlots; i++)
        {
            observationSlots[i] = Vector2.zero;
        }

        // Randomly assign sensed drone observations
        foreach (var drone in drone_common.sensedDrones)
        {
            int index;
            do
            {
                index = random.Next(totalSlots);
            } while (observationSlots[index] != Vector2.zero);

            Vector3 droneRel = transform.InverseTransformPoint(drone.transform.position);
            observationSlots[index] = Vector3to2(droneRel / Drone_Values.R_sense); // Normalising
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
        netReward += droneProximityReward;
        float survivalReward = AddSurvivalReward();
        netReward += survivalReward;

        // Sensed Number Changed Reward

        if (drone_common.currentSensed < drone_common.totalSensed)
            netReward += (drone_common.totalSensed - drone_common.currentSensed) * (-0.1f);
        if (drone_common.currentSensed > drone_common.prevSensed)
            netReward += 20f;
        if (drone_common.swarmDrones.Count < drone_common.totalSensed)
            netReward += -5f;

        float obstacleProximityReward = AddObstacleProximityReward();
        netReward += obstacleProximityReward;
        if (drone_common.sensedDrones.Count + sensedObjectsPos.Count > 0)
        {
            netReward = netReward / (drone_common.sensedDrones.Count + sensedObjectsPos.Count);
        }

        float lostReward = AddLostReward();
        netReward += lostReward;
        AddReward(netReward);
        TotalReward += netReward;
    }
    private float AddDroneProximityReward()
    {
        float netReward = 0;
        foreach (var drone in drone_common.sensedDrones)
        {
            if (drone != this.gameObject)               // Don't check against itself
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if (distance < Drone_Values.R_out)
                {
                    netReward += 1;
                    Debug.DrawLine(transform.position, drone.transform.position, Color.blue);
                }
            }
        }
        return netReward;
    }


    bool CollidedWithObstacle()
    {
        bool collided = collidedWithObstacle;
        collidedWithObstacle = false;        // Reset the flag to only detect the collision once
        return collided;
    }

    bool CollidedWithDrone()
    {
        bool collided = collidedWithDrone;
        collidedWithDrone = false;            // Reset the flag to only detect the collision once
        return collided;
    }

    bool CollidedWithBoundary()
    {
        bool collided = collidedWithBoundary;
        collidedWithBoundary = false;         // Reset the flag to only detect the collision once
        return collided;
    }



    private float AddSurvivalReward()
    {
        return 0.1f;
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
        if (drone_common.sensedDrones.Count == 0)
        {
            netReward = -1;
        }
        return netReward;
    }


    //private HashSet<GameObject> SenseDrones()
    //{
    //    HashSet<GameObject> sensed = new HashSet<GameObject>();

    //    Collider[] drones = Physics.OverlapSphere(transform.position, Drone_Values.R_sense, dronesLayer);

    //    foreach (Collider col in drones)
    //    {
    //        if (col.gameObject != this.gameObject)
    //        {
    //            sensed.Add(col.gameObject);
    //            Debug.DrawRay(this.transform.position, col.transform.position - this.transform.position, Color.blue);
    //        }
    //    }

    //    return sensed;
    //}

    // Case 1: infinite drone sensing


    //private List<Vector2> SenseObjects()
    //{
    //    List<Vector2> hitPos = new List<Vector2>();
    //    Vector3 origin = transform.position;
    //    int angleIncrement = 360 / Drone_Values.NumberRays;
    //    for (int i = 0; i < Drone_Values.NumberRays; i++)
    //    {
    //        int angle = angleIncrement * i;
    //        Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;


    //        // Perform the raycast
    //        if (Physics.Raycast(origin, direction, out RaycastHit hit, Drone_Values.R_sense))
    //        {
    //            float distance = Vector3.Distance(origin, hit.point) / Drone_Values.R_sense;

    //            // Add the local hit point to the list if the ray hits an object
    //            if (hit.collider.gameObject.tag != "Drone")
    //            {
    //                hitPos.Add(Vector3to2(hit.point));
    //                Debug.DrawRay(origin, direction * (hit.point - origin).magnitude, Color.red);
    //            }
    //            else
    //            {
    //                hitPos.Add(Vector3to2(direction.normalized * Drone_Values.R_sense));
    //            }
    //        }
    //        else
    //        {
    //            hitPos.Add(Vector3to2(direction.normalized * Drone_Values.R_sense));
    //        }
    //    }

    //    return hitPos;
    //}

    //private List<Vector2> SenseObjects()
    //{
    //    List<Vector2> hitPos = new List<Vector2>();
    //    Vector3 origin = transform.position;
    //    float angleIncrement = 360f / Drone_Values.NumberRays;
    //    for (int i = 0; i < Drone_Values.NumberRays; i++)
    //    {
    //        float angle = i * angleIncrement;
    //        Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

    //        // Perform the raycast
    //        if (Physics.Raycast(origin, direction, out RaycastHit hit))
    //        {
    //            var localHitPos = transform.InverseTransformPoint(hit.point);
    //            hitPos.Add(localHitPos);
    //            //Debug.DrawLine(origin, hit.point, Color.red);
    //        }
    //    }

    //    return hitPos;
    //}

    private List<Vector2> SenseObjects()
    {
        List<Vector2> hitPos = new List<Vector2>();
        Vector3 origin = transform.position;
        float angleIncrement = 360f / Drone_Values.NumberRays;

        for (int i = 0; i < Drone_Values.NumberRays; i++)
        {
            float angle = i * angleIncrement;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            // Perform the raycast
            if (Physics.Raycast(origin, direction, out RaycastHit hit, Drone_Values.R_sense))
            {
                // Check if the hit object is an obstacle or drone
                if (hit.collider.gameObject.tag == "Obstacle")
                {
                    // If we hit an obstacle, we do not sense anything behind it
                    hitPos.Add(Vector3to2(direction.normalized * Drone_Values.R_sense));
                }
                else if (hit.collider.gameObject.tag == "Drone")
                {
                    // Check if there is an obstacle between the current drone and the sensed drone
                    RaycastHit obstacleHit;
                    if (Physics.Raycast(origin, hit.point - origin, out obstacleHit, Vector3.Distance(origin, hit.point)))
                    {
                        // If the obstacle is closer than the drone, ignore this drone
                        if (obstacleHit.collider.gameObject.tag == "Obstacle")
                        {
                            hitPos.Add(Vector3to2(direction.normalized * Drone_Values.R_sense)); // Empty sensing for obstacle
                        }
                        else
                        {
                            var localHitPos = transform.InverseTransformPoint(hit.point);
                            hitPos.Add(localHitPos);  // Drone is visible (no obstacle in between)
                        }
                    }
                    else
                    {
                        // If no obstacle is hit, the drone is sensed
                        var localHitPos = transform.InverseTransformPoint(hit.point);
                        hitPos.Add(localHitPos);
                    }
                }
                else
                {
                    // If it's neither obstacle nor drone, it's some other object
                    hitPos.Add(Vector3to2(hit.point));
                }
            }
            else
            {
                // If nothing was hit, assume max sensing range
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

    private Vector2 Vector3to2(Vector3 v)
    {
        Vector2 v2 = new Vector2(v.x, v.z);
        return v2;
    }
}
