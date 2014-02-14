using UnityEngine;
using System.Collections;

public class SimpleCameraMovementScript : MonoBehaviour 
{
    public bool RunScript = false;
    public float Speed = 10;

    private Vector3 _direction = new Vector3(0, 0, 10);
    void Update()
    {
        if (RunScript)
        {
            if (transform.position.z > 1)
                _direction = new Vector3(0, 0, -Speed);
            else if (transform.position.z < -10)
                _direction = new Vector3(0, 0, Speed);

            transform.position += _direction * Time.deltaTime;
        }
    }
}
