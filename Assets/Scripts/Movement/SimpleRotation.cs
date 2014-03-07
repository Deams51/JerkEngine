using UnityEngine;
using System.Collections;

/// Author: Anders Treptow
/// <summary>
/// A simple rotation script that rotates an object along a desired set of euler angles.
/// </summary>
public class SimpleRotation : MonoBehaviour
{
    public Vector3 EulerRotation;

    void Update()
    {
        transform.Rotate(EulerRotation * Time.deltaTime);
    }
}
