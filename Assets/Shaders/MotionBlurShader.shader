Shader "Custom/MotionBlurShader" 
{
	Properties
	{
		_MainTex ("Main Texture", 2D) = "white" {}
		_VelocityBuffer ("Velocity Buffer", 2D) = "gray" {}
	}

	CGINCLUDE
	#include "UnityCG.cginc"

	#define NUM_SAMPLES 12
	#define SOFT_Z_DISTANCE 0.0005

	uniform sampler2D _MainTex;
	uniform sampler2D _VelocityBuffer;
	uniform sampler2D _CameraDepthTexture;
	uniform sampler2D _GrabTexture;

	struct v2f 
	{
		float4 pos : POSITION;
		float2 uv  : TEXCOORD0;
	};

	v2f vert(appdata_img v) 
	{
		v2f o;
		o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
		o.uv = v.texcoord.xy;
		return o;
	}

	float softDepthCompare(float za, float zb)
	{
		return clamp(1.0 - (za - zb) / SOFT_Z_DISTANCE, 0.0, 1.0);
	}

	float4 crudeMotionBlur(v2f f) : COLOR
	{
		float4 result = float4(0);

		float2 velocityUV = f.uv;
		float2 x = f.uv;

		float4 velocitySample = tex2D(_VelocityBuffer, velocityUV);
		float2 velocity = float2(DecodeFloatRG(velocitySample.rg), DecodeFloatRG(velocitySample.ba));

		velocity = (velocity - 0.5) * 4; //decode from color to velocity value

		float zx = UNITY_SAMPLE_DEPTH(tex2Dlod(_CameraDepthTexture, float4(x,0,0)));
		zx = -Linear01Depth(zx);

		for(int i = 1; i < NUM_SAMPLES; ++i)
		{
			float intensity = 1.0 / (NUM_SAMPLES + 1);
			float length = (0.5 / NUM_SAMPLES);
			
			float2 offset = velocity * (float(i) / float(NUM_SAMPLES - 1) - 0.5);

			float zy = UNITY_SAMPLE_DEPTH(tex2Dlod(_CameraDepthTexture, float4(x+offset,0,0)));
			zy = -Linear01Depth(zy);

			float d = softDepthCompare(zy, zx);

			float4 sample = tex2D(_MainTex, f.uv + offset * d);

			result += sample * intensity;
		}

		return result;
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
