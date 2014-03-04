using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class VelocityCamera : MonoBehaviour 
{
    public Shader VelocityShader;
    public float BlurFactor = 15.0f;
    public float BlurIntensity = 40.0f;
    public Material MotionBlurMaterial;

    private Material _material = null;
    private Matrix4x4 _oldViewProjMat;
    private List<VelocityObject> _renderObjects;

    private GameObject _shaderCamera;
    private RenderTexture _renderTexture;

    private bool _velocityRenderStage = false;
    public bool IsVelocityRenderStage { get { return _velocityRenderStage; } }

    void OnEnable()
    {
        _material = new Material(VelocityShader);
        camera.depthTextureMode = DepthTextureMode.Depth;
    }

    void OnPreRender()
    {
        if (_renderObjects != null)
        {
            foreach (VelocityObject obj in _renderObjects)
            {
                obj.SetShader(VelocityShader);
            }
        }
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Shader.SetGlobalFloat("_BlurIntensity", BlurIntensity);
        Graphics.Blit(source, destination);
    }

    public void AddToRenderList(VelocityObject obj)
    {
        if (_renderObjects == null)
            _renderObjects = new List<VelocityObject>();

        _renderObjects.Add(obj);
    }

    public void RemoveFromRenderList(VelocityObject obj)
    {
        if (_renderObjects != null)
        {
            if (_renderObjects.Contains(obj))
                _renderObjects.Remove(obj);
        }
    }

    void StoreOldProjectionMatrix()
    {
        Matrix4x4 P = camera.projectionMatrix;
        if (SystemInfo.graphicsDeviceVersion.IndexOf("Direct3D") > -1) //if D3D
        {
            // Invert Y for rendering to a render texture
            for (int i = 0; i < 4; i++)
            {
                P[1, i] = -P[1, i];
            }
            // Scale and bias from OpenGL -> D3D depth range
            for (int i = 0; i < 4; i++)
            {
                P[2, i] = P[2, i] * 0.5f + P[3, i] * 0.5f;
            }
        }

        _oldViewProjMat = P * camera.worldToCameraMatrix;
        Shader.SetGlobalMatrix("_PrevVP", _oldViewProjMat);
    }

    void OnPostRender()
    {
        //RenderTexture.ReleaseTemporary(_renderTexture);
        StoreOldProjectionMatrix();
        if (_renderObjects != null)
        {
            foreach (VelocityObject obj in _renderObjects)
            {
                obj.OnPostRenderUpdate();
                obj.ResetShader();
            }
        }
    }
}
