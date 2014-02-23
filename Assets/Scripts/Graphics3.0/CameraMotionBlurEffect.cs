using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[RequireComponent(typeof(Camera))]
public class CameraMotionBlurEffect : ImageEffectBase
{
    public static Matrix4x4 ViewMatrix { get; private set; }
    public static Matrix4x4 PreviousViewMatrix { get; private set; }

    public static Matrix4x4 ProjectionMatrix { get; private set; }
    public static Matrix4x4 PreviousProjectionMatrix { get; private set; }

    public static Matrix4x4 ViewProjMatrix { get; private set; }
    public static Matrix4x4 PreviousViewProjMatrix { get; private set; }

    protected static HashSet<EffectObject> EffectObjects
    {
        get
        {
            if (_effectObjects == null)
                _effectObjects = new HashSet<EffectObject>();

            return _effectObjects;
        }
    }
    protected static HashSet<EffectObject> _effectObjects;

    public static void AddEffectObject(EffectObject obj)
    {
        EffectObjects.Add(obj);
    }

    public static void RemoveEffectObject(EffectObject obj)
    {
        if (EffectObjects.Contains(obj))
            EffectObjects.Remove(obj);
    }

    protected Camera _velocityCamera;

    virtual protected void Awake()
    {
        camera.depthTextureMode = DepthTextureMode.Depth;

        GameObject velCamera = new GameObject("VelocityCamera", typeof(Camera));
        velCamera.transform.parent = transform;
        _velocityCamera = velCamera.camera;
        _velocityCamera.transform.localPosition = Vector3.zero;
        _velocityCamera.depthTextureMode = DepthTextureMode.Depth;
        velCamera.active = false;

        ViewMatrix = Matrix4x4.identity;
        PreviousViewMatrix = Matrix4x4.identity;
        ProjectionMatrix = Matrix4x4.identity;
        PreviousProjectionMatrix = Matrix4x4.identity;
        ViewProjMatrix = Matrix4x4.identity;
        PreviousViewProjMatrix = Matrix4x4.identity;

        UpdateViewProjectionMatrices();
    }

    void Update()
    {
        PreviousViewMatrix = ViewMatrix;
        PreviousProjectionMatrix = ProjectionMatrix;
        PreviousViewProjMatrix = ViewProjMatrix;

        UpdateViewProjectionMatrices();
    }

    void UpdateViewProjectionMatrices()
    {
        ViewMatrix = camera.worldToCameraMatrix;
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
        ProjectionMatrix = P;
        ViewProjMatrix = ProjectionMatrix * ViewMatrix;
    }

    virtual protected void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        //Set shaders for objects to render velocity buffer
        foreach (EffectObject obj in EffectObjects)
        {
            obj.PreVelocityRender();
        }

        RenderTexture velocityBuffer = RenderTexture.GetTemporary(source.width, source.height, 24);
        _velocityCamera.CopyFrom(camera);
        _velocityCamera.backgroundColor = new Color(0.4980392f, 0.5f, 0.4980392f, 0.5f); //EncodeFloatRG(0.5) from UnityCG.cginc, this is needed due to floating point precision issues
        _velocityCamera.targetTexture = velocityBuffer;
        _velocityCamera.RenderWithShader(EffectObject.VelocityBufferShader, "");
        _velocityCamera.targetTexture = null;

        //Render everything
        material.SetTexture("_VelocityBuffer", velocityBuffer);
        Graphics.Blit(source, destination, material);
        //Graphics.Blit(velocityBuffer, destination); //render velocity buffer

        //reset shaders for objects
        foreach (EffectObject obj in EffectObjects)
        {
            obj.PostVelocityRender();
        }

        RenderTexture.ReleaseTemporary(velocityBuffer);
    }
}
