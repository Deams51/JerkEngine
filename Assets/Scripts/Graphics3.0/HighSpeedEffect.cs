using UnityEngine;
using System.Collections;

[RequireComponent(typeof(ParticleSystem))]
public class HighSpeedEffect : MonoBehaviour 
{
    public float Velocity = 0;
    public float MaxEmissionRate = 150f;
    public float MinEmissionRate = 50f;
    public GameObject VelocityTrackedObject;
    public bool ManualVelocity = false;

    private ParticleSystem _particles;
    private Camera _mainCamera;
	private CameraTracker _trackerScript;

    void Awake()
    {
        _particles = this.particleSystem;
        _particles.renderer.sortingLayerName = "Foreground";
        _mainCamera = GameObject.FindGameObjectWithTag("MainCamera").GetComponent<Camera>();
		_trackerScript = _mainCamera.GetComponent<CameraTracker> ();
    }

    void Update()
    {
        if (VelocityTrackedObject != null && !ManualVelocity)
            if (VelocityTrackedObject.rigidbody != null)
                Velocity = VelocityTrackedObject.rigidbody.velocity.magnitude * 3.6f; //km/h

        float emissionRate = Mathf.Clamp(Velocity, 0, MaxEmissionRate);
        float aCol = Mathf.Clamp(Velocity / 255f, 0, 100f / 255f);

        if (emissionRate < MinEmissionRate)
            emissionRate = 0;
        else
            emissionRate = Mathf.Lerp(emissionRate, MaxEmissionRate, Mathf.Clamp(Velocity / MaxEmissionRate, 0.0f, 1.0f) / 2);

        _particles.emissionRate = emissionRate;
        _particles.startColor = new Color(1, 1, 1, aCol);
		if (_trackerScript != null) 
		{
			switch(_trackerScript.Mode)
			{
				case CameraTracker.CameraMode.Follow:
                    transform.rotation = Quaternion.LookRotation(VelocityTrackedObject.rigidbody.velocity);
                    transform.localEulerAngles = new Vector3(_mainCamera.transform.eulerAngles.x, this.transform.localEulerAngles.y + 180, this.transform.localEulerAngles.z);
                    break;
				case CameraTracker.CameraMode.FirstPerson:
				    transform.localEulerAngles = new Vector3(0, 180, 0);
					break;
				case CameraTracker.CameraMode.Fixed: //assumes that the camera is to the left of the car
                    transform.rotation = Quaternion.LookRotation(VelocityTrackedObject.rigidbody.velocity);
                    transform.localEulerAngles = new Vector3(_mainCamera.transform.eulerAngles.x, this.transform.localEulerAngles.y + 180, this.transform.localEulerAngles.z);
					break;
			}
		}
    }
}