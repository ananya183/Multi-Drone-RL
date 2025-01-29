using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingObstacle : MonoBehaviour
{
    public float maxSpeed = 0.05f;
    public float maxAngularSpeed = 0.03f;
    public float maxForce = 0.25f;

    protected Rigidbody rBody;
    LayerMask dronesLayer; // For sensing drones
    protected Vector3 direction;
    protected float angularVelocity;

    private bool collidedWithObstacle = false;
    private bool collidedWithBoundary = false;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
        direction = Random.insideUnitSphere.normalized;
        angularVelocity = Random.Range(-maxAngularSpeed, maxAngularSpeed);
    }

    void FixedUpdate()
    {
        // Apply force to move the obstacle
        Vector3 force = direction * maxForce;
        rBody.AddForce(force);

        // Apply angular velocity to rotate the obstacle
        rBody.angularVelocity = new Vector3(0, angularVelocity, 0);

        // Clamp the velocity to maxSpeed
        rBody.velocity = Vector3.ClampMagnitude(rBody.velocity, maxSpeed);

        // Check for collisions with boundaries or other obstacles
        if (CollidedWithBoundary() || CollidedWithObstacle())
        {
            // Reverse direction on collision
            direction = -direction;
            angularVelocity = -angularVelocity;
        }
    }

        void OnCollisionEnter(Collision collision)
    {
        if (CollidedWithBoundary(collision) || CollidedWithObstacle(collision))
        {
            // Reverse direction on collision
            direction = -direction;
            angularVelocity = -angularVelocity;
        }
    }


    private bool CollidedWithBoundary(Collision collision)
    {
        return collision.gameObject.CompareTag("Boundary");
    }

    private bool CollidedWithObstacle(Collision collision)
    {
        return collision.gameObject.CompareTag("Obstacle");
    }
}