using UnityEngine;
using System.Collections;

/// Author: Anders Treptow
/// <summary>
/// Deprecated code (not used), see Scripts/Graphics3.0/EffectObject for latest version
/// </summary>
public class ObjectRenderer : MonoBehaviour 
{
    private Camera _camera;
    private Matrix4x4 _previousObject2World;
    private PostEffectsCamera _postEffects;
    private Shader _velShader;
    private Shader _defaultShader = null;

	void Start () 
    {
        _camera = GameObject.FindGameObjectWithTag("MainCamera").GetComponent<Camera>();
        _postEffects = _camera.GetComponent<PostEffectsCamera>();
        _velShader = _postEffects.VelocityShader;
        _postEffects.AddToRenderList(this);
        _defaultShader = renderer.material.shader;
	}

    void OnRenderObject()
    {
        if (_postEffects.IsVelocityRenderStage)
        {
            renderer.material.shader = _defaultShader;
        }
        else
        {
            renderer.material.shader = _velShader;
        }
        renderer.material.SetMatrix("_PrevObject2World", _previousObject2World);
    }

    public void OnPostRenderUpdate()
    {
        _previousObject2World = transform.renderer.localToWorldMatrix;
    }

    void OnDestroy()
    {
        if (_postEffects != null)
            _postEffects.RemoveFromRenderList(this);
    }
}
