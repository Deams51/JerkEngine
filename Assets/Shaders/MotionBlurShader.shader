Shader "Custom/MotionBlurShader" 
{
	Properties
	{
		_MainTex ("Main Texture", 2D) = "white" {}
		_VelocityBuffer ("Velocity Buffer", 2D) = "gray" {}
	}

	SubShader 
	{
		ZTest Always
		Cull Off
		ZWrite Off
		Fog { Mode off }
		Pass 
		{
			Blend One Zero
			CGPROGRAM
			#pragma vertex vert_img
			#pragma fragment frag
			#pragma fragmentoption ARB_precision_hint_fastest 
			#include "UnityCG.cginc"

			#define NUM_SAMPLES 10

			uniform sampler2D _MainTex;
			uniform sampler2D _VelocityBuffer;
			//uniform sampler2D _CameraDepthTexture;

			float4 crudeMotionBlur(v2f_img f)
			{
				float4 result = float4(0);

				float2 velocityUV = f.uv;

				float4 velocitySample = tex2D(_VelocityBuffer, velocityUV);
				float2 velocity = float2(DecodeFloatRG(velocitySample.rg), DecodeFloatRG(velocitySample.ba));

				velocity = (velocity - 0.5) * 4; //decode from color to velocity value

				for(int i = 0; i <= NUM_SAMPLES; i++)
				{
					float intensity = 1.0 / (NUM_SAMPLES + 1);
					float length = (0.5 / NUM_SAMPLES);
					float4 sample = tex2D(_MainTex, f.uv + velocity * i * length);

					result += sample * intensity;
				}

				return result;
			}

			float4 frag(v2f_img f) : COLOR 
			{
				//float4 color = crudeMotionBlur(_MainTex, _GlobalVelocityTexture, i.uv, _BlurIntensity);
				//return tex2D(_GlobalVelocityTexture, i.uv);
				//return color;
				return crudeMotionBlur(f);
			}
			ENDCG
		}
	}
}
