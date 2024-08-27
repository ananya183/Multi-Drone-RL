using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Drone_Values : MonoBehaviour
{

    [Header("Drone Maximum Values")]
    [SerializeField] float maxForce;
    [SerializeField] float maxSpeed;
    [SerializeField] float maxAngularSpeed;
    [SerializeField] int numberDrones;
    [SerializeField] int maxNumberDrones;

    [Header("Epsiode Values")]
    [SerializeField] int numActionsInEpisode;
    [SerializeField] int currentEpisode;

    [Header("Leader Drone Values")]
    [SerializeField] float leaderDroneMultiplier;
    [SerializeField] int numWaypoints;
    [SerializeField] bool userControlledOrAutomated;

    [Header("Follower Drone Values")]
    [SerializeField] float r_sense;     // sensingZoneRadius
    [SerializeField] float r_out;       // goodRegionOuterRadius
    [SerializeField] float r_in;        // goodRegionInnerRadius, also, Bad Region Outer Radius
    [SerializeField] float r_tooclose;


    public static float MaxForce;
    public static float MaxSpeed;
    public static float MaxAngularSpeed;
    public static int NumberDrones;
    public static int MaxNumberDrones;

    public static int NumActionsInEpisode;
    public static int CurrentEpisode;

    public static float LeaderDroneMultiplier;
    public static int NumWaypoints;
    public static bool UserControlledOrAutomated;

    public static float R_sense;
    public static float R_out;
    public static float R_in;
    public static float R_tooclose;


    private void Awake()
    {
        MaxForce = maxForce;
        MaxSpeed = maxSpeed;
        MaxAngularSpeed = maxAngularSpeed;
        NumberDrones = numberDrones;
        MaxNumberDrones = maxNumberDrones;

        NumActionsInEpisode = numActionsInEpisode;

        UserControlledOrAutomated = userControlledOrAutomated;
        LeaderDroneMultiplier = leaderDroneMultiplier;
        NumWaypoints = numWaypoints;

        R_sense = r_sense;
        R_out = r_out;
        R_in = r_in;
        R_tooclose = r_tooclose;
        CurrentEpisode = currentEpisode;
    }

    private void FixedUpdate()
    {
        currentEpisode = CurrentEpisode;
    }
}
