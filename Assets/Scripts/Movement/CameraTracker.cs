using UnityEngine;
using System.Collections;

public class CameraTracker : MonoBehaviour 
{
    public enum CameraMode
    {
        Follow,
        Fixed,
        FirstPerson
    };

    public GameObject TrackingObject;
    public GameObject FirstPersonObject;
    public float TrackingAmount = 5.5f;
    public CameraMode Mode = CameraMode.Follow; 
	

    private Transform _trackingObject;
    private Vector3 _relCameraPosition;

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
                transform.position = Vector3.Lerp (transform.position, _trackingObject.position - _relCameraPosition, TrackingAmount * Time.deltaTime);
			transform.LookAt (_trackingObject);
                break;
            case CameraMode.Fixed:
                transform.position = _trackingObject.position;
            transform.rotation = _trackingObject.rotation;
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
        }
    }
}
