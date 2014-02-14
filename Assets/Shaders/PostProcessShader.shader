Shader "Custom/PostProcessShader" 
{
	Properties 
	{
		_MainTex ("Main Texture", 2D) = "white" {}
	}
	SubShader 
	{
		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
        
			#include "UnityCG.cginc"

			uniform sampler2D _MainTex;
			uniform sampler2D _VelocityTexture : register(s1);
			uniform float _BlurIntensity;
			uniform half _DebugShader = -1;

			struct fragmentInput
			{
				float4 pos : POSITION;
				half2 uv : TEXCOORD0;
			};

			fragmentInput vert(appdata_img v)
			{
				fragmentInput o;
				o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
				o.uv = MultiplyUV(UNITY_MATRIX_TEXTURE0, v.texcoord.xy);
				return o;
			}

			float4 motionBlur(sampler2D color, sampler2D motion, float2 uv, float intensity)
			{
				float2 texcoord = uv;
				float2 speed = (tex2D(motion, uv) - 0.5) / intensity;

				float4 fragment = tex2D(color, uv);
				texcoord -= speed;

				for(int i = 1; i < 11; ++i, texcoord += speed)
				{
					float4 currentFragment = tex2D(color, texcoord);
					fragment += currentFragment;
				}
				float4 finalColor = fragment / 11.0;

				return finalColor;
				//return float4(speed.x, speed.y, 0, 1);
				//return tex2D(motion, float2(0.01,0.01));
				
				//return float4(speed.xy, 0, speed.w);
			}

			float4 frag(fragmentInput f) : COLOR
			{
				half4 color = tex2D(_MainTex, f.uv);

				if(_DebugShader > 0)
				{
					color = tex2D(_VelocityTexture, f.uv);
				}
				else
				{
					color = motionBlur(_MainTex, _VelocityTexture, f.uv, _BlurIntensity);
				}

				return color;
			}
			ENDCG
		}
	} 
	FallBack "Diffuse"
}
