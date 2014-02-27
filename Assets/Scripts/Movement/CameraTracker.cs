using UnityEngine;
using System.Collections;

public class CameraTracker : MonoBehaviour 
{
    public GameObject TrackingObject;
    public float TrackingAmount = 5.5f;

    private Transform _trackingObject;
    private Vector3 _relCameraPosition;

    void Awake()
    {
        _trackingObject = TrackingObject.transform;
        _relCameraPosition = _trackingObject.position - this.transform.position;
    }

    void FixedUpdate()
    {
        transform.position = Vector3.Lerp(transform.position, _trackingObject.position - _relCameraPosition, TrackingAmount * Time.deltaTime);
        transform.LookAt(_trackingObject);
    }


}
