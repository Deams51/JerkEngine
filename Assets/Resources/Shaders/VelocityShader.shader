
/// Author: Anders Treptow
/// <summary>
/// A shader to render velocity by calculating delta values between vertices.
/// </summary>
Shader "Custom/Velocity Shader" 
{
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

			uniform float4x4 _mv;
			uniform float4x4 _mvPrev;
			uniform float4x4 _mvInvTrans; // not used in this implementation
			uniform float4x4 _mvpPrev;

			struct vertexInput
			{
				float4 vertex : POSITION;
				float3 normal : NORMAL;
			};
 
			struct fragmentInput 
			{
				float4 pos : POSITION;
				float4 newPos;
				float4 oldPos;
			};

			/// <summary>
			/// The vertex shader for the velocity shading. The vertex shader pass along the current
			/// and previous positions of each vertex to the fragment shader based on the current model view projection
			/// matrix and the previous model view projection matrix.
			/// </summary>
			fragmentInput vert(vertexInput v)
			{
				fragmentInput o;

				float4 curPoint = mul(UNITY_MATRIX_MVP, v.vertex);
				float4 prevPoint = mul(_mvpPrev, v.vertex);

				o.pos = curPoint;
				o.newPos = curPoint;
				o.oldPos = prevPoint;

				return o;
			}

			/// <summary>
			/// The fragment shader calculates delta value between the previous and current position of each 
			/// fragment in screen space. Note that this needs to be calculated in the fragment shader rather than the vertex shader
			/// in order to get correct values when clipping occurs. The value is then encoded into RGBA where RG is delta X and BA is delta Y.
			/// </summary>
			float4 frag(fragmentInput i) : COLOR
			{
				float2 curPosition = i.newPos.xy / i.newPos.w;
				float2 prevPosition = i.oldPos.xy / i.oldPos.w;
				float2 delta = curPosition.xy - prevPosition.xy;
					
				delta.xy = delta.xy * 0.25 + 0.5; //additional blur factor: *0.25, normalizing to color value: +0.5
					
				return float4(EncodeFloatRG(delta.x), EncodeFloatRG(delta.y));
			}
			ENDCG
		}
	}
}
