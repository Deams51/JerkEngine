using UnityEngine;
using System.Collections;

public class ObjectRenderScript : MonoBehaviour
{
    private Matrix4x4 _previousObject2World;
    private PostProcessScript _postProcessRenderer;

    void Start()
    {
        _postProcessRenderer = GameObject.FindGameObjectWithTag("MainCamera").GetComponent<PostProcessScript>();
        _postProcessRenderer.AddToRenderList(this);
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
        if(_postProcessRenderer != null)
            _postProcessRenderer.RemoveFromRenderList(this);
    }
}
