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
			uniform float4x4 _mvInvTrans;
			uniform float4x4 _mvpPrev;

			uniform float _BlurFactor;

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

			//vertex shader
			//////////////////////////////////////////////////////////
			fragmentInput vert(vertexInput v)
			{
				fragmentInput o;
				float4 curPoint = mul(_mv, v.vertex);
				float4 prevPoint = mul(_mvPrev, v.vertex);

				float3 N = (float3)mul(_mvInvTrans, float4(v.normal, 1));
				float3 eyeMotion = curPoint.xyz - prevPoint.xyz;

				curPoint = mul(UNITY_MATRIX_MVP, v.vertex);
				prevPoint = mul(_mvpPrev, v.vertex);

				float dotMN = dot(eyeMotion, N);
				float4 pointStrech = dotMN > 0 ? curPoint : prevPoint;

				o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
				o.newPos = curPoint;
				o.oldPos = prevPoint;

				return o;
			}

			//fragment shader
			//////////////////////////////////////////////////////////
			float4 frag(fragmentInput i) : COLOR
			{
				float3 curPosition = i.newPos.xyz / i.newPos.w;
				float3 prevPosition = i.oldPos.xyz / i.oldPos.w;
				float2 delta = curPosition.xy - prevPosition.xy;
					
				delta.xy = delta.xy * 0.25 + 0.5; //additional blur factor: *0.25, normalizing to color value: +0.5
					
				return float4(EncodeFloatRG(delta.x), EncodeFloatRG(delta.y));
				//return float4(delta.x, delta.y, 0, 1);
				//return float4(1,1,1,1);
			}
			ENDCG
		}
	}
}
