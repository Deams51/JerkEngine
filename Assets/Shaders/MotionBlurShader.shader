Shader "Custom/MotionBlurShader" 
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
			Fog { Mode Off }
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#include "UnityCG.cginc"

			uniform sampler2D _MainTex;
			uniform sampler2D _GlobalVelocityTexture;

			struct v2f 
			{
				float4 pos : POSITION;
				float2 uv : TEXCOORD0;
			};

			v2f vert (appdata_base v) 
			{
				v2f o;
				o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
				o.uv = MultiplyUV(UNITY_MATRIX_TEXTURE0, v.texcoord.xy);
				return o;
			}

			half4 frag(v2f i) : COLOR 
			{
				return tex2D(_GlobalVelocityTexture, i.uv);
			}
			ENDCG
		}
	}
}
