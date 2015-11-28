using UnityEngine;
using System.Collections;

[AddComponentMenu("Infinite Camera-Control/Mose Orbit with zoom")]
public class CameraFollow : MonoBehaviour {
    public Transform target;
    public float xSpeed = 12.0f;
    public float ySpeed = 12.0f;
    public float zSpeed = 12.0f;

    public float scrollSpeed = 10.0f;

    public float zoomMin = 1.0f;
    public float zoomMax = 20.0f;

    public float distance = 2;
    public float Ypos;
    public Vector3 position;
    public bool isActivated;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    // Use this for initialization
    void Start () {
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;
        z = angles.z;

        position = -(transform.forward * distance) + target.position;
        transform.position = position;
        Ypos = transform.position.y;
    }

    void LateUpdate () {
        if (Input.GetMouseButtonDown(1))
        {
            isActivated = true;
        }
        if(Input.GetMouseButtonUp(1))
        {
            isActivated = false;
        }

        if(target && isActivated)
        {
            x += Input.GetAxis("Mouse X") * xSpeed;
            y -= Input.GetAxis("Mouse Y") * ySpeed;
            
            transform.RotateAround(target.position, transform.up, x);
            transform.RotateAround(target.position, transform.right, y);

            transform.rotation = Quaternion.Euler(transform.rotation.x, transform.rotation.y, 0);
            transform.rotation = Quaternion.LookRotation(target.position - transform.position);

            //Ypos = transform.position.y;

            x = 0;
            y = 0;

        }else
        {
            if(Input.GetAxis("Mouse ScrollWheel") != 0)
            {
                distance = Vector3.Distance(transform.position, target.position);
                distance =  ZoomLimit(distance - Input.GetAxis("Mouse ScrollWheel") * scrollSpeed, zoomMin, zoomMax);
                position = -(transform.forward*distance) + target.position;
                //Ypos = position.y;
                transform.position = position;
            }
        }

        float d2 = Vector3.Distance(transform.position, target.position);
        Ypos = Ypos - (-(target.position.y + distance*0.5f) + Ypos) * 0.02f;
        if (d2 != distance)
        {
            position = -(transform.forward * distance) + target.position;
            //Ypos = Ypos - (- position.y + Ypos)*0.05f;
            position.y = Ypos;
            transform.position = position;
        }
    }
	
    public static float ZoomLimit(float dist, float min, float max)
    {
        if(dist < min)
            dist = min;
        if (dist > max)
            dist = max;
        return dist;
    }
	// Update is called once per frame
	void Update () {
	
	}
}
