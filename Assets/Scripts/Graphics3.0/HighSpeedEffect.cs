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

    void Awake()
    {
        _particles = this.particleSystem;
        _particles.renderer.sortingLayerName = "Foreground";
        _mainCamera = GameObject.FindGameObjectWithTag("MainCamera").GetComponent<Camera>();
    }

    void Update()
    {
        if (VelocityTrackedObject != null && !ManualVelocity)
            if (VelocityTrackedObject.rigidbody != null)
                Velocity = VelocityTrackedObject.rigidbody.velocity.magnitude;

        float emissionRate = Mathf.Clamp(Velocity, 0, MaxEmissionRate);
        float aCol = Mathf.Clamp(Velocity / 255f, 0, 100f / 255f);

        if (emissionRate < MinEmissionRate)
            emissionRate = 0;
        else
            emissionRate = Mathf.Lerp(emissionRate, MaxEmissionRate, Mathf.Clamp(Velocity / MaxEmissionRate, 0.0f, 1.0f) / 2);

        _particles.emissionRate = emissionRate;
        _particles.startColor = new Color(1, 1, 1, aCol);
        transform.localEulerAngles = new Vector3(_mainCamera.transform.eulerAngles.x, 180 - (_mainCamera.transform.localEulerAngles.y - VelocityTrackedObject.transform.localEulerAngles.y), 0);
    }
}