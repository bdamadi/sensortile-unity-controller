using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Player : MonoBehaviour
{
    public float speed = 200f;

    public Connection input;

    Rigidbody rb;
    Vector3 initialPosition;


    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        initialPosition = rb.position;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        var vertical = input.GetAxis("Vertical");
        var horizontal = input.GetAxis("Horizontal");

        Vector3 velocity = Vector3.zero;
        velocity += (transform.forward * vertical); //Move forward
        velocity += (transform.right * horizontal); //Strafe
        velocity *= speed * Time.fixedDeltaTime; //Framerate and speed adjustment
        velocity.y = rb.velocity.y;

        rb.velocity = velocity;
    }

    public void Reset()
    {
        rb.velocity = Vector3.zero;
        rb.position = initialPosition;
    }
}
