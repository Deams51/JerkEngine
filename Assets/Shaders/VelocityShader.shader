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
			uniform float _deltaTime;

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

				float4 curPoint = mul(UNITY_MATRIX_MVP, v.vertex);
				float4 prevPoint = mul(_mvpPrev, v.vertex);

				o.pos = curPoint;
				o.newPos = curPoint;
				o.oldPos = prevPoint;

				return o;
			}

			//fragment shader
			//////////////////////////////////////////////////////////
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
