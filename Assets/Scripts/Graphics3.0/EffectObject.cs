using UnityEngine;
using System.Collections;

public class EffectObject : MonoBehaviour 
{
    public bool IsObjectVisible { get { return renderer.isVisible; } }
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
    private Material[] _velocityMaterials;
    private Material[] _regularMaterials;

    private Matrix4x4 _prevModelMatrix;

    public static Material NewMaterial()
    {
        return new Material(VelocityBufferShader);
    }

    void Start()
    {
        _velocityMaterial = NewMaterial();
        _velocityMaterials = new Material[renderer.materials.Length];

        for (int i = 0; i < renderer.materials.Length; i++)
            _velocityMaterials[i] = _velocityMaterial;

        _regularMaterials = renderer.materials;

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
        _regularMaterials = renderer.materials; // in case the material has changed in-game during previous call
        renderer.materials = _velocityMaterials;
    }

    public void PostVelocityRender()
    {
        renderer.materials = _regularMaterials; // in case the material has changed in-game during previous call
    }
}
