using UnityEngine;
using System.Collections;

/// Author: Anders Treptow
/// <summary>
/// This class handles camera movement. The class defines 3 possible camera positions with different 
/// behaviours depending on the type of tracking. Follow lerps between the camera and a fixed point, Fixed and FirstPerson are
/// just setting the camera at a set of points that the camera then becomes the child of.
/// </summary>
public class CameraTracker : MonoBehaviour 
{
    public enum CameraMode
    {
        Follow,
        Fixed,
        FirstPerson
    };

    public GameObject TrackingObject; //the object to follow
    public GameObject FirstPersonObject;
    public GameObject FollowPoint;
    public GameObject FixedPoint;
    public float TrackingAmount = 5.5f;
    public CameraMode Mode = CameraMode.Follow; 

    private Transform _trackingObject;
    private Vector3 _relCameraPosition;

    private float _mouseX;
    private float _mouseY;

    private float _verticalRotationMin = 0.0f;
    private float _verticalRotationMax = 65f;

    void Awake()
    {
        _trackingObject = TrackingObject.transform;
        _relCameraPosition = _trackingObject.position - this.transform.position;
    }

    void FixedUpdate()
    {
        switch (Mode)
        {
            case CameraMode.Follow:
                transform.position = Vector3.Lerp(transform.position, FollowPoint.transform.position, TrackingAmount * Time.deltaTime);
                transform.LookAt (_trackingObject);
                break;
        }
    }

    void Update()
    {
        switch (Mode)
        {
            case CameraMode.FirstPerson:
                transform.position = FirstPersonObject.transform.position;
                transform.rotation = FirstPersonObject.transform.rotation;
                transform.parent = FirstPersonObject.transform;
                break;
            case CameraMode.Fixed:
                transform.position = FixedPoint.transform.position;
                transform.rotation = FixedPoint.transform.rotation;
                transform.parent = FixedPoint.transform;
                break;
        }
    }
}
