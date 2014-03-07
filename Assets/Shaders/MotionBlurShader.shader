
/// Author: Anders Treptow
/// <summary>
/// The main shader used to render the post-processing effect of Motion Blur.
/// </summary>
Shader "Custom/MotionBlurShader" 
{
	Properties
	{
		_MainTex ("Main Texture", 2D) = "white" {}
		_VelocityBuffer ("Velocity Buffer", 2D) = "gray" {}
	}

	CGINCLUDE
	#include "UnityCG.cginc"

	#define NUM_SAMPLES 8 //The number of samples for each pixel when blurring
	#define SOFT_Z_DISTANCE 0.0005 //The distance to check for in the continuous classification filter
	#define TARGET_FPS 60 //The target frame rate to render for

	uniform sampler2D _MainTex;
	uniform sampler2D _VelocityBuffer;
	uniform sampler2D _CameraDepthTexture;

	uniform float _CurrentFPS;
	uniform float _BlurFactor; //Extra blurring amount factor

	struct v2f 
	{
		float4 pos : POSITION;
		float2 uv  : TEXCOORD0;
	};

	/// <summary>
	/// The vertex shader. Nothing special is happening here. Just an ordinare vertex shader.
	/// </summary>
	v2f vert(appdata_img v) 
	{
		v2f o;
		o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
		o.uv = v.texcoord.xy;
		return o;
	}

	/// <summary>
	/// A continuous classification filter for depth. A mathematical function that returns an
	/// amount between 0.0 and 1.0 depending on the depth distance between two values.
	/// </summary>
	float softDepthCompare(float za, float zb)
	{
		return clamp(1.0 - (za - zb) / SOFT_Z_DISTANCE, 0.0, 1.0);
	}

	/// <summary>
	/// A fragment shading solution for motion blur that takes depth into account when sampling
	/// each fragment. Each fragment is sampled along an vector depending on the velocity
	/// calculated from the velocity buffer at the same fragment coordinate. Each fragment
	/// takes num_sample steps in the direction of the velocity vector. The sample is then multiplied
	/// by the continuous classification filter based on the original sample and the current sample.
	/// These samples are then added together to create the end result of the post-processing effect.
	/// </summary>
	float4 crudeMotionBlur(v2f f) : COLOR
	{
		float4 result = float4(0);

		float2 velocityUV = f.uv;
		float2 x = f.uv;

		float4 velocitySample = tex2D(_VelocityBuffer, velocityUV);
		float2 velocity = float2(DecodeFloatRG(velocitySample.rg), DecodeFloatRG(velocitySample.ba));
		float4 compare = float4(EncodeFloatRG(0.5), EncodeFloatRG(0.5));
		float2 compare2 = float2(DecodeFloatRG(compare.rg), DecodeFloatRG(compare.ba));

		// Get the velocity for fragment
		velocity = (velocity - 0.5) * (_CurrentFPS / TARGET_FPS); //decode from color to velocity value

		float zx = UNITY_SAMPLE_DEPTH(tex2Dlod(_CameraDepthTexture, float4(x,0,0)));
		zx = -Linear01Depth(zx);

		for(int i = 1; i < NUM_SAMPLES; ++i)
		{
			float intensity = 1.0 / (NUM_SAMPLES + 1);
			float length = (0.5 / NUM_SAMPLES);
			
			//the offset is calculated so that the blurring is equal on both sides of the object to be blurred
			float2 offset = velocity * (float(i) / float(NUM_SAMPLES - 1) - 0.5) * _BlurFactor;

			float zy = UNITY_SAMPLE_DEPTH(tex2Dlod(_CameraDepthTexture, float4(x+offset,0,0)));
			zy = -Linear01Depth(zy);

			float d = softDepthCompare(zy, zx);

			float4 sample = tex2D(_MainTex, f.uv + offset * d);

			result += sample * intensity;
		}

		return result;
	}

	/// <summary>
	/// A fragment shader to debug the velocity buffer, it just renders the fragment at coordinate in the velocity buffer.
	/// </summary>
	float4 debugVelocity(v2f f) : COLOR
	{
		return tex2D(_VelocityBuffer, f.uv);
	}

	/// <summary>
	/// A depth shader to debug the depth buffer, it just renders the fragment at coordinate in the depth buffer.
	/// </summary>
	float4 debugDepth(v2f f) : COLOR
	{
		return tex2D(_CameraDepthTexture, f.uv);
	}
	
	ENDCG

	SubShader 
	{
		Pass 
		{
			CGPROGRAM
			#pragma target 3.0
			#pragma vertex vert
			#pragma fragment crudeMotionBlur
			#pragma fragmentoption ARB_precision_hint_fastest
			#pragma glsl

			ENDCG
		}
	}
}
