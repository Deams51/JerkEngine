using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// Author: Anders Treptow
/// <summary>
/// Deprecated code (not used), see Scripts/Graphics3.0/CameraMotionBlurEffect for latest version
/// </summary>
[RequireComponent(typeof(Camera))]
public class PostEffectsCamera : MonoBehaviour 
{
    public Shader VelocityShader;
    public float BlurFactor = 15.0f;
    public float BlurIntensity = 40.0f;
    public Material MotionBlurMaterial;
    public bool DisplayVelocityBuffer = false;

    private Material _material = null;
    private Matrix4x4 _oldViewProjMat;
    private List<ObjectRenderer> _renderObjects;

    private GameObject _shaderCamera;
    private RenderTexture _renderTexture;

    private bool _velocityRenderStage = false;
    public bool IsVelocityRenderStage { get { return _velocityRenderStage; } }

    void OnEnable()
    {
        _material = new Material(VelocityShader);
    }

    void OnPreRender()
    {
        _velocityRenderStage = true;
        _renderTexture = RenderTexture.GetTemporary((int)camera.pixelWidth, (int)camera.pixelHeight, 24);
        if (_shaderCamera == null)
        {
            _shaderCamera = new GameObject("ShaderCamera", typeof(Camera));
            _shaderCamera.camera.enabled = false;
            _shaderCamera.hideFlags = HideFlags.HideAndDontSave;
        }

        Camera cam = _shaderCamera.camera;
        cam.CopyFrom(camera);
        cam.backgroundColor = new Color(0.5f, 0.5f, 0.0f, 1.0f);
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.targetTexture = _renderTexture;
        cam.cullingMask = 1 << 8; //NOTE: Requires the velocity objects to be on layer 8!!!

        Shader.SetGlobalFloat("_BlurFactor", BlurFactor);

        cam.Render();
        Shader.SetGlobalTexture("_VelTex", _renderTexture);
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        _velocityRenderStage = false;
        Shader.SetGlobalFloat("_BlurIntensity", BlurIntensity);

        //Graphics.Blit(source, destination);
        Graphics.Blit(source, destination, MotionBlurMaterial);
    }

    public void AddToRenderList(ObjectRenderer obj)
    {
        if (_renderObjects == null)
            _renderObjects = new List<ObjectRenderer>();

        _renderObjects.Add(obj);
    }

    public void RemoveFromRenderList(ObjectRenderer obj)
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
        StoreOldProjectionMatrix();
        if (_renderObjects != null)
        {
            foreach (ObjectRenderer obj in _renderObjects)
            {
                obj.OnPostRenderUpdate();
            }
        }
        RenderTexture.ReleaseTemporary(_renderTexture);
    }
}
