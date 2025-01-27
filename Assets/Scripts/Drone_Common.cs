using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Drone_Common : MonoBehaviour
{
    private Rigidbody Rigidbody;
    public Drone_Manager drone_Manager;

    public HashSet<GameObject> swarmDrones = new HashSet<GameObject>();
    public HashSet<GameObject> sensedDrones = new HashSet<GameObject>();

    public int totalSensed;
    public int prevSensed;
    public int currentSensed;
    private void Awake()
    {
        swarmDrones = new HashSet<GameObject>();
    }
    void Start()
    {
        Rigidbody = GetComponent<Rigidbody>();
    }

    public void OnEpisodeBegins()
    {
        totalSensed = 0;
        prevSensed = 0;
        currentSensed = 0;
        sensedDrones.Clear();
        swarmDrones.Clear();
        swarmDrones.Add(this.gameObject);
    }

    private void FixedUpdate()
    {
        prevSensed = sensedDrones.Count;
        sensedDrones = SenseDrones();
        currentSensed = sensedDrones.Count;

        if (currentSensed > totalSensed)
            totalSensed = currentSensed;
    }

    public bool HasJustFormedSwarm()
    {
        foreach (GameObject drone in sensedDrones)
        {
            var drone_script = drone.GetComponent<Drone_Common>();
            if (drone != this.gameObject)
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if (distance <= Drone_Values.R_sense)
                {
                    // Check if Incomer Drone is a Fellow (part of its own swarm) or Not:
                    // Proceed only if the Incomer is a Foreign Drone:
                    if (!swarmDrones.Contains(drone))
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

    public bool IsTooCloseToOtherDrone()
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

    public bool IsInBadZone()
    {
        foreach (GameObject drone in swarmDrones)
        {
            if (drone != this.gameObject)               // Don't check against itself
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if ((distance >= Drone_Values.R_tooclose) && (distance <= Drone_Values.R_in))
                //if ((distance >= Drone_Values.R_tooclose) && (distance <= Drone_Values.R_opt))
                {
                    return true;
                }
            }
        }
        return false;
    }

    public bool IsInGoodZone()
    {
        foreach (GameObject drone in swarmDrones)
        {
            if (drone != this.gameObject)               // Don't check against itself
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if ((distance > Drone_Values.R_in) && (distance <= Drone_Values.R_out))
                //if ((distance > Drone_Values.R_opt) && (distance <= Drone_Values.R_sense))
                {
                    return true;
                }
            }
        }
        return false;
    }

    public bool DroneInSense()
    {
        foreach (GameObject drone in swarmDrones)
        {
            if (drone != this.gameObject)               // Don't check against itself
            {
                float distance = Vector3.Distance(this.transform.position, drone.transform.position);
                if ((distance > Drone_Values.R_out) && (distance <= Drone_Values.R_sense))
                {
                    return true;
                }
            }
        }
        return false;
    }



    private HashSet<GameObject> SenseDrones()
    {
        HashSet<GameObject> sensed = new HashSet<GameObject>();

        foreach (var drone in drone_Manager.drones)
        {
            if (drone != this.gameObject)
                sensed.Add(drone);
        }
        return sensed;
    }
}
