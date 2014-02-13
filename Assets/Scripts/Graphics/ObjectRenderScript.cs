using UnityEngine;
using System.Collections;

public class ObjectRenderScript : MonoBehaviour
{
    private Matrix4x4 _previousObject2World;

    void Start()
    {
        PostProcessScript camera = GameObject.FindGameObjectWithTag("MainCamera").GetComponent<PostProcessScript>();
        camera.AddToRenderList(this);
    }

    void OnRenderObject()
    {
        renderer.material.SetMatrix("_PrevObject2World", _previousObject2World);
    }

    public void OnPostRenderUpdate()
    {
        _previousObject2World = transform.renderer.localToWorldMatrix;
    }
}
