using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class Drone_Agent : Agent
{
    protected Rigidbody rBody;
    LayerMask dronesLayer; // For sensing drones
    public Drone_Manager drone_Manager;

    // Reward Valuations in High to Low Order
    private readonly float swarmationReward = 200.0f;
    private readonly float insideGoodRegionReward = 40.0f;
    private readonly float insideSensingZoneReward = 10.0f;
    private readonly float intraSwarmCollisionPenalty = -50.0f;
    private readonly float obstacleCollisionPenalty = -30.0f;
    private readonly float boundaryCollisionPenalty = -30.0f;
    private readonly float tooClosePenalty = -10.0f;
    private readonly float insideBadRegionPenalty = -5.0f;
    private readonly float rr_factor = 0.25f;

    private bool collidedWithObstacle = false;
    private bool collidedWithDrone = false;
    private bool collidedWithBoundary = false;

    public HashSet<GameObject> swarmDrones = new HashSet<GameObject>();
    public HashSet<GameObject> sensedDrones = new HashSet<GameObject>();

    private int current = 0; // For current Action count
    private void Awake()
    {
        dronesLayer = LayerMask.NameToLayer("Drone");
        dronesLayer = 1 << dronesLayer;
    }

    public override void OnEpisodeBegin()
    {
        sensedDrones.Clear();
        swarmDrones.Clear();
        swarmDrones.Add(this.gameObject);
        sensedDrones.Add(this.gameObject);

        rBody = GetComponent<Rigidbody>();
        
        current = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensedDrones = SenseDrones();

        sensor.AddObservation(transform.localPosition);             // Adds 3 observations for the local position
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);

        foreach (GameObject fellow in this.swarmDrones)
        {
            sensor.AddObservation(fellow.transform.localPosition);  // 3 spaces per detected drone
        }

        for (int i = 0; i < Drone_Values.MaxNumberDrones - (swarmDrones.Count - 2); i++)
        {
            sensor.AddObservation(Vector3.zero);                    // 3 spaces per missing drone
        }
        current++;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var action = actions.ContinuousActions;

        Vector3 controlSignal = Vector3.zero;

        controlSignal.x = action[0];
        controlSignal.z = action[1];

        Vector3 forceToApply = controlSignal * 10;
        Debug.DrawRay(transform.position, forceToApply, Color.green);
        rBody.AddForce(forceToApply);


        if (HasJustFormedSwarm())
        {
            var reboundReward = swarmationReward * rr_factor * (swarmDrones.Count - 1);
            AddReward(swarmationReward);
            AddReward(reboundReward);
        }

        if (IsInBadZone())
        {
            AddReward(insideBadRegionPenalty);
        }

        if (IsInGoodZone())
        {
            AddReward(insideGoodRegionReward);
        }

        if (CollidedWithObstacle())
        {
            AddReward(obstacleCollisionPenalty);
            //EndEpisode();
        }

        if (CollidedWithDrone())
        {
            AddReward(intraSwarmCollisionPenalty);
            //EndEpisode();
        }

        if (CollidedWithBoundary())
        {
            AddReward(boundaryCollisionPenalty);
        }

        if (current > Drone_Values.NumActionsInEpisode)
        {
            EndEpisodeForAllDrones();
            Drone_Values.CurrentEpisode++;
        }

        current++;
    }

    private void OnCollisionEnter(Collision collision)
    {
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

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Implement heuristic controls for manual testing
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal") * 10;
        continuousActions[1] = Input.GetAxis("Vertical") * 10;
    }

    private void EndEpisodeForAllDrones()
    {
        for (int i = 0; i < drone_Manager.drones.Count; i++)
        {
            var agent = drone_Manager.drones[i].GetComponent<Agent>();
            agent.EndEpisode();
            drone_Manager.drones[i] = drone_Manager.ResetDrone(drone_Manager.drones[i]);
        }

    }

    void ApplySwarmationReward()
    {
        if (HasJustFormedSwarm())
        {
            float reboundReward = swarmationReward * rr_factor * (this.swarmDrones.Count - 1);
            AddReward(swarmationReward);
            AddReward(reboundReward);
        }
    }

    bool HasJustFormedSwarm()
    {
        foreach (GameObject drone in sensedDrones)
        {
            var drone_script = drone.GetComponent<Drone_Agent>();
            if (drone != this.gameObject)
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if (distance <= Drone_Values.R_sense)
                {
                    // Check if Incomer Drone is a Fellow (part of its own swarm) or Not:
                    // Proceed only if the Incomer is a Foreign Drone:
                    if (!this.swarmDrones.Contains(drone))
                    {
                        // Case 1: this and drone are both single ;P
                        if (this.swarmDrones.Count == 1 && drone_script.swarmDrones.Count == 1)
                        {
                            swarmDrones.Add(drone);
                            return true;
                        }

                        // Case 2: this is single, drone is swarmed :(
                        else if (this.swarmDrones.Count == 1 && drone_script.swarmDrones.Count > 1)
                        {
                            foreach (GameObject droneMember in drone_script.swarmDrones)
                            {
                                if (!this.swarmDrones.Contains(droneMember))
                                {
                                    swarmDrones.Add(droneMember);
                                    return true;
                                }
                            }
                        }

                        // Case 3: this is swarmed, drone is single :O
                        else if (this.swarmDrones.Count > 1 && drone_script.swarmDrones.Count == 1)
                        {
                            swarmDrones.Add(drone);
                            return true;
                        }

                        // Case 4: Both are swarmed in their own different Swarms ~( -_-)~
                        else if (this.swarmDrones.Count > 1 && drone_script.swarmDrones.Count > 1)
                        {
                            foreach (GameObject droneMember in drone_script.swarmDrones)
                            {
                                if (!this.swarmDrones.Contains(droneMember))
                                {
                                    swarmDrones.Add(droneMember);
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    bool IsTooCloseToOtherDrone()
    {
        foreach (GameObject drone in swarmDrones)
            if (drone != this.gameObject)                // Don't check against itself
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if (distance < Drone_Values.R_tooclose)
                {
                    return true;
                }
            }
        return false;
    }

    bool IsInBadZone()
    {
        foreach (GameObject drone in swarmDrones)
        {
            if (drone != this.gameObject)               // Don't check against itself
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if ((distance >= Drone_Values.R_tooclose) && (distance <= Drone_Values.R_in))
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool IsInGoodZone()
    {
        foreach (GameObject drone in swarmDrones)
        {
            if (drone != this.gameObject)               // Don't check against itself
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if ((distance > Drone_Values.R_in) && (distance <= Drone_Values.R_out))
                {
                    return true;
                }
            }
        }
        return false;
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


    protected void ApplyRegionRewardsAndPenalties()
    {
        foreach (var drone in sensedDrones)
        {
            float distance = Vector3.Distance(this.transform.position, drone.transform.position);
            if (distance < Drone_Values.R_tooclose)
            {
                AddReward(tooClosePenalty);
            }
            else if ((distance >= Drone_Values.R_tooclose) && (distance <= Drone_Values.R_in))
            {
                AddReward(insideBadRegionPenalty);
            }
            else if ((distance > Drone_Values.R_in) && (distance <= Drone_Values.R_out))
            {
                AddReward(insideGoodRegionReward);
            }
            else if ((distance <= Drone_Values.R_sense))
            {
                AddReward(insideSensingZoneReward);
            }
        }
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
}
