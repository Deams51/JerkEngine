Shader "Custom/PostProcessShader" 
{
	Properties 
	{
		_MainTex ("Main Texture", 2D) = "white" {}
	}
	SubShader 
	{
		Tags { "RenderType"="Opaque" }
		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma fragmentoption ARB_precision_hint_fastest
        
			#include "UnityCG.cginc"

			uniform sampler2D _MainTex;
			uniform sampler2D _VelocityTexture : register(s1);
			uniform sampler2D _CameraDepthTexture;
			uniform float _BlurIntensity;
			uniform half _DebugShader = -1;

			struct fragmentInput
			{
				float4 pos : POSITION;
				half2 uv : TEXCOORD0;
				float4 scrPos : TEXCOORD1;
			};

			fragmentInput vert(appdata_img v)
			{
				fragmentInput o;
				o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
				o.uv = MultiplyUV(UNITY_MATRIX_TEXTURE0, v.texcoord.xy);
				o.scrPos=ComputeScreenPos(o.pos);
			   //for some reason, the y position of the depth texture comes out inverted
			   o.scrPos.y = 1 - o.scrPos.y;
				return o;
			}

			float4 crudeMotionBlur(sampler2D color, sampler2D motion, float2 uv, float intensity)
			{
				float2 texcoord = uv;
				fixed2 speed = (tex2D(motion, uv)) - 0.5;
				speed /= intensity;

				float4 fragment = tex2D(color, uv);
				texcoord += speed;
				float numSamples = 11.0;

				for(int i = 1; i < numSamples; ++i, texcoord += speed)
				{
					float4 currentFragment = tex2D(color, texcoord);
					fragment += currentFragment;
				}
				float4 result = fragment / numSamples;

				return result;
			}

			float random(float3 co)
			{
				return frac(sin( dot(co.xyz ,float3(12.9898,78.233,45.5432) )) * 43758.5453);
			}

			float4 motionReconstructionFilter(float2 uv, sampler2D colorTex, sampler2D velocityTex, sampler2D depthTex)
			{
				float4 resultColor = tex2D(colorTex, uv);				

				return resultColor;
			}

			float4 frag(fragmentInput f) : COLOR
			{
				float4 color = tex2D(_MainTex, f.uv);

				if(_DebugShader > 0)
				{
					color = tex2D(_VelocityTexture, f.uv);
				}
				else
				{
					//color = crudeMotionBlur(_MainTex, _VelocityTexture, f.uv, _BlurIntensity);
					//color = motionFilter(f.uv, _MainTex, _VelocityTexture, _CameraDepthTexture);
					float depthValue = Linear01Depth(tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(f.scrPos)).r);
					half4 depth;

					depth.r = depthValue;
					depth.g = depthValue;
					depth.b = depthValue;

					depth.a = 1;

					color = depth;
				}

				return color;
			}
			ENDCG
		}
	} 
	FallBack "Diffuse"
}
