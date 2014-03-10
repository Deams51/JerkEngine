using UnityEngine;
using System.Collections;

/// Author: Anders Treptow
/// <summary>
/// The ObjectEffectHandler script is applied to every gameobject that contains the Mesh Renderer Component.
/// The script keeps track of previous frame's transform data, keeps track of materials of the mesh
/// and replaces it with a material that has the velocity buffer attached to it when rendering the velocity buffer, 
/// and inputs the necessary uniforms for the velocity shader for each object draw call.
/// </summary>
public class ObjectEffectHandler : MonoBehaviour 
{
    public bool IsObjectVisible { get { return renderer.isVisible; } } //whether this object is visible from the camera view frustrum
    public static Shader VelocityBufferShader
    {
        get
        {
            if (_velocityBufferShader == null)
                _velocityBufferShader = Shader.Find("Custom/Velocity Shader");

            return _velocityBufferShader;
        }
    }
    private static Shader _velocityBufferShader;


    private Material _velocityMaterial;
    private Material[] _velocityMaterials;
    private Material[] _regularMaterials;

    private Matrix4x4 _prevModelMatrix;

    public static Material NewMaterial()
    {
        return new Material(VelocityBufferShader);
    }

    /// <summary>
    /// Start function copies the list of materials for the mesh renderer and creates a
    /// separate array that matches the current array but containing the velocity shader instead.
    /// It doesn't work to just iterate through the array and replace the materials but a separate
    /// list is needed and then swap the reference to which list to render.
    /// </summary>
    void Start()
    {
        _velocityMaterial = NewMaterial();
        _velocityMaterials = new Material[renderer.materials.Length];

        for (int i = 0; i < renderer.materials.Length; i++)
            _velocityMaterials[i] = _velocityMaterial;

        _regularMaterials = renderer.materials;

        _prevModelMatrix = transform.localToWorldMatrix;
    }

    void OnEnable()
    {
        PostEffectHandler.AddEffectObject(this);
    }

    void OnDisable()
    {
		PostEffectHandler.AddEffectObject(this);
    }

    /// <summary>
    /// During the lateupdate (meaning hopefully after the rendering call) the data from the rendering
    /// call is stored for the next rendering call in order to calculate delta values. The Current model, view, and
    /// projection matrix for the object is stored after the values have been set as uniforms for the
    /// velocity shader. "_mv" is current model view matrix, "_mvPrev" is previous model view matrix, "_mvInvTrans" is the
    /// inverse transpose of the current model view matrix, "_mvpPrev" is the previous model view projection matrix, and "_deltaTime" is
    /// the elapsed time in seconds since the last rendering loop call.
    /// </summary>
    void LateUpdate()
    {
        Vector4 currentPos = transform.position;
        //currentPos.w = 1f;

		Matrix4x4 _mv = PostEffectHandler.ViewMatrix * transform.localToWorldMatrix;
		Matrix4x4 _mvPrev = PostEffectHandler.PreviousViewMatrix * _prevModelMatrix;

        _velocityMaterial.SetMatrix("_mv", _mv);
        _velocityMaterial.SetMatrix("_mvPrev", _mvPrev);
        _velocityMaterial.SetMatrix("_mvInvTrans", _mv.transpose.inverse);
		_velocityMaterial.SetMatrix("_mvpPrev", PostEffectHandler.PreviousViewProjMatrix * _prevModelMatrix);
        _velocityMaterial.SetFloat("_deltaTime", Time.deltaTime);

        _prevModelMatrix = transform.localToWorldMatrix;
    }

    /// <summary>
    /// Before rendering the velocity buffer each object needs to replace it's current shader that normally would render
    /// to the frame buffer to the velocity shader for the velocity buffer.
    /// </summary>
    public void PreVelocityRender()
    {
        _regularMaterials = renderer.materials; // in case the material has changed in-game during previous call
        renderer.materials = _velocityMaterials;
    }

    /// <summary>
    /// After the velocity buffer has been rendered each object is reverted back to its regular materials to be rendered
    /// to the frame buffer.
    /// </summary>
    public void PostVelocityRender()
    {
        renderer.materials = _regularMaterials; // in case the material has changed in-game during previous call
    }
}
