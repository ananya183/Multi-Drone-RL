using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Drone_Values : MonoBehaviour
{

    [Header("Drone Values")]
    [SerializeField] float maxForce;
    [SerializeField] float maxSpeed;
    [SerializeField] float maxAngularSpeed;
    [SerializeField] int numberDrones;
    [SerializeField] int maxNumberDrones;
    [SerializeField] float droneHeight;


    [Header("Epsiode Values")]
    [SerializeField] int numActionsInEpisode;
    [SerializeField] float trainingAreaSize;
    [SerializeField] int trainingAreaNumber;
    [SerializeField] int numberRays;
    [SerializeField] int currentEpisode;


    [Header("Leader Drone Values")]
    [SerializeField] float leaderDroneMultiplier;
    [SerializeField] int numWaypoints;
    [SerializeField] float waypointAreaThreshold;
    [SerializeField] bool userControlledOrAutomated;

    [Header("Follower Drone Values")]
    [SerializeField] float r_sense;     // sensingZoneRadius
    [SerializeField] float r_spawn;
    [SerializeField] float r_out;       // goodRegionOuterRadius
    [SerializeField] float r_in;        // goodRegionInnerRadius, also, Bad Region Outer Radius
    [SerializeField] float r_tooclose;


    public static float MaxForce;
    public static float MaxSpeed;
    public static float MaxAngularSpeed;
    public static int NumberDrones;
    public static int MaxNumberDrones;
    public static float DroneHeight;

    public static int NumActionsInEpisode;
    public static float TrainingAreaSize;
    public static int TrainingAreaNumber;
    public static int NumberRays;
    public static int CurrentEpisode;

    public static float LeaderDroneMultiplier;
    public static int NumWaypoints;
    public static float WaypointAreaThreshold;
    public static bool UserControlledOrAutomated;

    public static float R_sense;
    public static float R_spawn;
    public static float R_out;
    public static float R_in;
    public static float R_tooclose;
    public static float R_opt;


    private void Awake()
    {
        MaxForce = maxForce;
        MaxSpeed = maxSpeed;
        MaxAngularSpeed = maxAngularSpeed;
        NumberDrones = numberDrones;
        MaxNumberDrones = maxNumberDrones;
        DroneHeight = droneHeight;
        WaypointAreaThreshold = waypointAreaThreshold;

        NumActionsInEpisode = numActionsInEpisode;
        TrainingAreaSize = trainingAreaSize;
        TrainingAreaNumber = trainingAreaNumber;
        NumberRays = numberRays;

        UserControlledOrAutomated = userControlledOrAutomated;
        LeaderDroneMultiplier = leaderDroneMultiplier;
        NumWaypoints = numWaypoints;

        R_sense = r_sense;
        R_spawn = r_spawn;
        R_out = r_out;
        R_in = r_in;
        R_opt = (R_out + R_in) / 2;
        R_tooclose = r_tooclose;
        CurrentEpisode = currentEpisode;
    }

    private void FixedUpdate()
    {
        currentEpisode = CurrentEpisode;
    }
}
