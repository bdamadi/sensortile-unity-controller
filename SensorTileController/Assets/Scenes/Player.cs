using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Player : MonoBehaviour
{
    public float speed = 200f;

    public float jumpForce = 100f;
    public float shootForce = 10f;

    public Connection input;

    public GameObject bulletPrefab;

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

        Jump(input.GetButton("Primary"));
        if (input.GetButton("Secondary") > 0)
        {
            Shoot();
        }
    }

    public void Reset()
    {
        rb.velocity = Vector3.zero;
        rb.position = initialPosition;
    }

    public void Jump(float strength)
    {
        rb.AddForce(Vector3.up * jumpForce * strength);
    }

    public void Shoot()
    {
        var instance = Instantiate(bulletPrefab, transform.position, Quaternion.identity, transform.parent);
        instance.SetActive(true);
        var bullet = instance.GetComponent<Rigidbody>();
        bullet.AddForce(Vector3.forward * shootForce, ForceMode.Impulse);

        Destroy(instance, 2f);
    }
}
