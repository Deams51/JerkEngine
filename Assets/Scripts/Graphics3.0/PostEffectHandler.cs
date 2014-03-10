using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// Author: Anders Treptow
/// <summary>
/// This script that handles the rendering of the velocity buffer and the post-processing effect of motion blur.
/// The script will find every mesh renderer component in the scene and apply a EffectObjectHandler script to that object dynamically
/// in order to render the velocity buffer using the shader material attached to each object as an extra material.
/// This script will also handle the rendering of the final image by first rendering the velocity buffer to a separate texture and
/// then use this texture together with the attached post-process effect shader that will render the final image to the frame buffer.
/// </summary>
[RequireComponent(typeof(Camera))]
public class PostEffectHandler : ImageEffectBase
{
    public bool Active = true;
    public bool RenderVelocityBuffer = false;
	public float BlurFactor = 1.5f; //blur factor is an extra amount that blur will be multiplied with

    public static Matrix4x4 ViewMatrix { get; private set; }
    public static Matrix4x4 PreviousViewMatrix { get; private set; }

    public static Matrix4x4 ProjectionMatrix { get; private set; }
    public static Matrix4x4 PreviousProjectionMatrix { get; private set; }

    public static Matrix4x4 ViewProjMatrix { get; private set; }
    public static Matrix4x4 PreviousViewProjMatrix { get; private set; }

	protected static HashSet<ObjectEffectHandler> EffectObjects //A set of objects that will be rendered to the velocity buffer in the scene
    {
        get
        {
            if (_effectObjects == null)
				_effectObjects = new HashSet<ObjectEffectHandler>();

            return _effectObjects;
        }
    }
	protected static HashSet<ObjectEffectHandler> _effectObjects;
    
    /// <summary>
	/// Static function so that each ObjectEffectHandler can add itself into the list of objects to be rendered
    /// to the velocity buffer. (OnEnable)
    /// </summary>
	/// <param name="obj">The instance of the ObjectEffectHandler</param>
	public static void AddEffectObject(ObjectEffectHandler obj)
    {
        EffectObjects.Add(obj);
    }

    /// <summary>
	/// Static function so that each ObjectEffectHandler can remove itself from the list of objects to be rendered
    /// to the velocity buffer. (OnDisable)
    /// </summary>
	/// <param name="obj">The instance of the ObjectEffectHandler</param>
	public static void RemoveEffectObject(ObjectEffectHandler obj)
    {
        if (EffectObjects.Contains(obj))
            EffectObjects.Remove(obj);
    }

    protected Camera _velocityCamera;

    /// <summary>
    /// Instanciates the necessary datastructures for the rendering of the velocity buffer and the post-processing effect.
	/// Basically it finds every gameobject that contains a MeshRenderer component and adds the ObjectEffectHandler script to that object.
    /// </summary>
    override protected void Start()
    {
		//sets up the ObjectEffectHandler script for each object that is rendered by the mesh renderer
        Object[] sceneObjects = GameObject.FindObjectsOfType(typeof(MeshRenderer));

        foreach (Object obj in sceneObjects)
        {
            if (obj is MeshRenderer)
            {
                GameObject gmObj = ((MeshRenderer)obj).gameObject;

				ObjectEffectHandler component = gmObj.GetComponent<ObjectEffectHandler>();
                if (component == null)
					gmObj.AddComponent<ObjectEffectHandler>();
                else if (!component.enabled)
                    component.enabled = true;
            }
        }

        base.Start();
    }

    /// <summary>
    /// When Awake() is invoked the scripts sets up a separate camera to render the velocity buffer from. This camera
    /// renders the velocity buffer to a RenderTexture that can be accessed from the post-processing shader.
    /// It also sets up the necessary matrices (view, projection, previous view, previous projection) and gives them
    /// the starting values.
    /// </summary>
    virtual protected void Awake()
    {
        camera.depthTextureMode |= DepthTextureMode.Depth;

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

    /// <summary>
    /// Each Update() call sets the values of the current view, projection, and viewprojection matrices and then
    /// updates the current values from the scene.
    /// </summary>
    void LateUpdate()
    {
        PreviousViewMatrix = ViewMatrix;
        PreviousProjectionMatrix = ProjectionMatrix;
        PreviousViewProjMatrix = ViewProjMatrix;

        UpdateViewProjectionMatrices();
    }

    /// <summary>
    /// Sets the values of the current view and projection matrices from the main camera.
    /// </summary>
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

    /// <summary>
	/// On every render call each object that is visible from the camera containing the ObjectEffectHandler script will 
    /// replace its current shader with the velocity shader. It will then render this to a separate texture.
    /// This texture is then sent as an uniform to the post-processing shader (motion blur shader) and each
	/// ObjectEffectHandler will reset the shaders to render properly and everything is rendered from the post-processing shader
    /// in a second rendering pass from the main camera.
    /// </summary>
    /// <param name="source">The source texture to render from</param>
    /// <param name="destination">The destination texture to render to (framebuffer)</param>
    virtual protected void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
#if UNITY_EDITOR
        if (!UnityEditorInternal.InternalEditorUtility.HasPro() || !Active)
        {
            Graphics.Blit(source, destination);
            return;
        }
#else
        if(!Active)
        {
            Graphics.Blit(source, destination);
            return;
        }
#endif

		List<ObjectEffectHandler> objs = new List<ObjectEffectHandler>();

        //check which objects are visible by the camera, (this includes the editor camera)
		foreach (ObjectEffectHandler obj in EffectObjects)
        {
            if (obj.IsObjectVisible)
                objs.Add(obj);
        }

        //Set shaders for these objects to render velocity buffer
		foreach (ObjectEffectHandler obj in objs)
            obj.PreVelocityRender();

        //Renders only the velocity buffer to a texture
        RenderTexture velocityBuffer = RenderTexture.GetTemporary(source.width, source.height, 24);
        _velocityCamera.CopyFrom(camera);
        _velocityCamera.backgroundColor = new Color(0.4980392f, 0.5f, 0.4980392f, 0.5f); //EncodeFloatRG(0.5) from UnityCG.cginc, this is needed due to floating point precision issues
        _velocityCamera.targetTexture = velocityBuffer;
        _velocityCamera.renderingPath = RenderingPath.Forward;
        _velocityCamera.cullingMask = ~(1 << 8); //exclude layer 8 (fx_layer)
		_velocityCamera.RenderWithShader(ObjectEffectHandler.VelocityBufferShader, "");
        _velocityCamera.targetTexture = null;

        //render everything
        if (!RenderVelocityBuffer)
        {
            material.SetTexture("_VelocityBuffer", velocityBuffer);
            material.SetFloat("_CurrentFPS", 1.0f/Time.deltaTime);
			material.SetFloat("_BlurFactor", BlurFactor);
            Graphics.Blit(source, destination, material);
        }
        else
            Graphics.Blit(velocityBuffer, destination); //render only velocity buffer

        //reset shaders for visible objects
		foreach (ObjectEffectHandler obj in objs)
            obj.PostVelocityRender();

        RenderTexture.ReleaseTemporary(velocityBuffer);
    }
}
