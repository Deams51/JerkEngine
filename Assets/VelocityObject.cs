using UnityEngine;
using System.Collections;

public class VelocityObject : MonoBehaviour 
{
    private Matrix4x4 _previousObject2World;
    private VelocityCamera _velCam;
    private Shader _velShader;
    private Shader _defaultShader = null;

    void Start()
    {
        _velCam = GameObject.FindGameObjectWithTag("VelocityCamera").GetComponent<VelocityCamera>();
        _defaultShader = renderer.material.shader;
        if(_velCam != null)
            _velCam.AddToRenderList(this);
    }

    public void SetShader(Shader shader)
    {
        renderer.material.shader = shader;
    }

    public void ResetShader()
    {
        renderer.material.shader = _defaultShader;
    }

    void OnRenderObject()
    {
        renderer.material.SetMatrix("_PrevObject2World", _previousObject2World);
    }

    public void OnPostRenderUpdate()
    {
        _previousObject2World = transform.renderer.localToWorldMatrix;
    }

    void OnDestroy()
    {
        if (_velCam != null)
            _velCam.RemoveFromRenderList(this);
    }
}
