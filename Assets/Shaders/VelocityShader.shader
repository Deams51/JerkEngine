Shader "Custom/VelocityShader" 
{
	Properties 
	{
	}
	SubShader 
	{
		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma fragmentoption ARB_precision_hint_fastest

			#include "UnityCG.cginc"

			//uniform sampler2D _MainTex;
			//uniform sampler2D _GrabTexture;
			uniform float4x4 _PrevObject2World;
			uniform float4x4 _PrevVP;
			uniform float _BlurFactor;

			struct vertexInput
			{
				float4 vertex : POSITION;
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

				o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
				o.newPos = mul(mul(UNITY_MATRIX_VP, _Object2World), v.vertex);
				o.oldPos = mul(mul(_PrevVP, _PrevObject2World), v.vertex);

				return o;
			}

			//fragment shader
			//////////////////////////////////////////////////////////
			float4 frag(fragmentInput i) : COLOR
			{
				float2 position1 = i.newPos.xy / i.newPos.w;
				float2 position2 = i.oldPos.xy / i.oldPos.w;
				float2 delta = (position2 - position1) * _BlurFactor + 0.5;

				return half4(delta.x, delta.y, 0, 1);
				//return float4(0,0,0,1);
			}
			ENDCG
		}
	}
}
