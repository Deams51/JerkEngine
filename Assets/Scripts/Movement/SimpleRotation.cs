using UnityEngine;
using System.Collections;

public class SimpleRotation : MonoBehaviour
{
    public Vector3 EulerRotation;

    void Update()
    {
        transform.Rotate(EulerRotation * Time.deltaTime);
    }
}
