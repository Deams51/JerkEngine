using UnityEngine;
using System.Collections;

[RequireComponent(typeof(ParticleSystem))]
public class HighSpeedEffect : MonoBehaviour 
{
    public float Velocity = 0;
    public float MaxEmissionRate = 150f;
    public float MinEmissionRate = 50f;

    private ParticleSystem _particles;

    void Awake()
    {
        _particles = this.particleSystem;
        _particles.renderer.sortingLayerName = "Foreground";
    }

    void Update()
    {
        float emissionRate = Mathf.Clamp(Velocity, 0, MaxEmissionRate);
        float aCol = Mathf.Clamp(Velocity / 255f, 0, 100f / 255f);

        if (emissionRate < MinEmissionRate)
            emissionRate = 0;
        else
            emissionRate = Mathf.Lerp(emissionRate, MaxEmissionRate, Mathf.Clamp(Velocity / MaxEmissionRate, 0.0f, 1.0f) / 2);

        _particles.emissionRate = emissionRate;
        _particles.startColor = new Color(1, 1, 1, aCol);
    }
}