Shader "Custom/PostProcessShader" 
{
	Properties 
	{
		_MainTex ("Base (RGB)", 2D) = "white" {}
	}
	SubShader 
	{
		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
        
			#include "UnityCG.cginc"

			uniform sampler2D _VelocityTexture : register(s1);

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

			float4 frag(fragmentInput f) : COLOR
			{
				return tex2D(_VelocityTexture, f.uv);
			}
			ENDCG
		}
	} 
	FallBack "Diffuse"
}
