using UnityEngine;
using System.Collections;

public class PostProcessScript : MonoBehaviour
{
    private Matrix4x4 _prevVP;

    void OnRenderImage(RenderTexture source, RenderTexture destination)
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

        Shader.SetGlobalMatrix("_P", P);

        _prevVP = P * camera.worldToCameraMatrix;
        Shader.SetGlobalMatrix("_PrevVP", _prevVP);


        Shader.SetGlobalMatrix("_V", camera.worldToCameraMatrix);
        Graphics.Blit(source, destination);
    }
}
