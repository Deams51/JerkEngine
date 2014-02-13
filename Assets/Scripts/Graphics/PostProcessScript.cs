﻿using UnityEngine;
using System.Collections.Generic;

public class PostProcessScript : MonoBehaviour
{
    public bool RenderEffect = true;
    public Material PostProcessMaterial;
    public float BlurFactor = 40.0f;

    private Matrix4x4 _oldViewProjMat;
    private List<ObjectRenderScript> _renderObjects;

    void Start()
    {
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

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Shader.SetGlobalFloat("_BlurFactor", BlurFactor);

        if (RenderEffect)
            Shader.SetGlobalFloat("_DebugShader", 1);
        else
            Shader.SetGlobalFloat("_DebugShader", -1);

        Graphics.Blit(source, destination, PostProcessMaterial);
    }

    public void AddToRenderList(ObjectRenderScript obj)
    {
        if(_renderObjects == null)
            _renderObjects = new List<ObjectRenderScript>();

        _renderObjects.Add(obj);
    }

    void OnPostRender()
    {
        StoreOldProjectionMatrix();
        foreach (ObjectRenderScript obj in _renderObjects)
        {
            obj.OnPostRenderUpdate();
        }
    }
}