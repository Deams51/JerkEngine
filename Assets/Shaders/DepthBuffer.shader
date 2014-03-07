
/// Author: Anders Treptow
/// <summary>
/// A shader to manually render the depth buffer, this shader is not used in the current version as this
/// is now retrieved from the _CameraDepthTexture instead. This shader was used in a previous version 
/// where it was not possible to use the _CameraDepthTexture.
/// </summary>
Shader "Custom/Depth Buffer" 
{
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

			struct v2f 
			{
				float4 pos : SV_POSITION;
				float2 depth : TEXCOORD0;
			};

			v2f vert (appdata_base v) 
			{
				v2f o;
				o.pos = mul (UNITY_MATRIX_MVP, v.vertex);
				UNITY_TRANSFER_DEPTH(o.depth);
				return o;
			}

			half4 frag(v2f i) : COLOR 
			{
				UNITY_OUTPUT_DEPTH(i.depth);
			}
			ENDCG
		}
	}
}