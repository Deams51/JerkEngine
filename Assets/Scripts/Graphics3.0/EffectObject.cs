using UnityEngine;
using System.Collections;

public class EffectObject : MonoBehaviour 
{
    public static Shader VelocityBufferShader
    {
        get
        {
            if (_velocityBufferShader == null)
                _velocityBufferShader = Shader.Find("Custom/Velocity Shader");

            return _velocityBufferShader;
        }
    }
    private static Shader _velocityBufferShader;

    private Material _velocityMaterial;
    private Material _regularMaterial;

    private Matrix4x4 _prevModelMatrix;

    public static Material NewMaterial()
    {
        return new Material(VelocityBufferShader);
    }

    void Start()
    {
        _velocityMaterial = NewMaterial();
        _regularMaterial = renderer.material;
        _prevModelMatrix = transform.localToWorldMatrix;
    }

    void OnEnable()
    {
        CameraMotionBlurEffect.AddEffectObject(this);
    }

    void OnDisable()
    {
        CameraMotionBlurEffect.AddEffectObject(this);
    }

    void LateUpdate()
    {
        Vector4 currentPos = transform.position;
        currentPos.w = 1f;

        Matrix4x4 _mv = CameraMotionBlurEffect.ViewMatrix * transform.localToWorldMatrix;
        Matrix4x4 _mvPrev = CameraMotionBlurEffect.PreviousViewMatrix * _prevModelMatrix;

        _velocityMaterial.SetMatrix("_mv", _mv);
        _velocityMaterial.SetMatrix("_mvPrev", _mvPrev);
        _velocityMaterial.SetMatrix("_mvInvTrans", _mv.transpose.inverse);
        _velocityMaterial.SetMatrix("_mvpPrev", CameraMotionBlurEffect.PreviousViewProjMatrix * _prevModelMatrix);

        _prevModelMatrix = transform.localToWorldMatrix;
    }

    public void PreVelocityRender()
    {
        _regularMaterial = renderer.material; // in case the material has changed in-game during previous call
        renderer.material = _velocityMaterial;
    }

    public void PostVelocityRender()
    {
        renderer.material = _regularMaterial;
    }
}
