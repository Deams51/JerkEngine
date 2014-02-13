Shader "Custom/MotionBlur2" 
{
	Properties 
	{
		_MainTex ("Base (RGB)", 2D) = "white" {}
		_Top("Top", Range(0,2)) = 1
		_Bottom("Bottom", Range(0,2)) = 1
	}

	SubShader 
	{
		Tags { "RenderType"="Opaque" }
		LOD 200
		
			CGPROGRAM
			#pragma surface surf Lambert vertex:vert

			sampler2D _MainTex;

			struct Input 
			{
				float2 uv_MainTex;
			};

			void vert (inout appdata_full v) 
			{
			}

			void surf (Input IN, inout SurfaceOutput o) 
			{
				half4 c = tex2D (_MainTex, IN.uv_MainTex);
				o.Albedo = c.rgb;
				o.Alpha = c.a;
			}
			ENDCG

			GrabPass { }

			Pass
			{
				CGPROGRAM
				#pragma vertex vert
				#pragma fragment frag
				#pragma fragmentoption ARB_precision_hint_fastest

				#include "UnityCG.cginc"

				uniform sampler2D _MainTex;
				uniform sampler2D _GrabTexture;
				uniform float4x4 _PrevObject2World;
				uniform float4x4 _PrevVP;

				struct vertexInput
				{
					float4 vertex : POSITION;
				};
 
				struct fragmentInput 
				{
					float4 pos : POSITION;
					float4 screenPos;
					float4 oldScreenPos; 
				};

				//vertex shader
				//////////////////////////////////////////////////////////
				fragmentInput vert(vertexInput v)
				{
					fragmentInput o;

					o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
					o.screenPos = mul(mul(UNITY_MATRIX_VP, _Object2World), v.vertex);
					o.oldScreenPos = mul(mul(UNITY_MATRIX_VP, _PrevObject2World), v.vertex);

					return o;
				}

				//fragment shader
				//////////////////////////////////////////////////////////
				float4 frag(fragmentInput i) : COLOR
				{
					/*
					float2 screenPos = i.screenPos.xy / i.screenPos.w;
					float halved = (_Top + _Bottom) * 0.5;
					float diff = (_Bottom - _Top) * 0.5;
					screenPos.x = screenPos.x * (halved + diff * screenPos.y);
					screenPos.x = (screenPos.x + 1) * 0.5;
					screenPos.y = 1-(screenPos.y + 1) * 0.5 ;
					half4 sum = half4(0.0h,0.0h,0.0h,0.0h);  
					*/

					float blurVectorScale = 40.0;
					float2 position1 = i.screenPos.xy / i.screenPos.w;
					float2 position2 = i.oldScreenPos.xy / i.oldScreenPos.w;
					float2 delta = (position1 - position2) * blurVectorScale + 0.5;

					return float4(delta.x, delta.y, 0, 1);
					//return float4(r, g, 0, 1);
					//return tex2D( _GrabTexture, screenPos);
					//return float4(i.velocity.x, i.velocity.y, 0, 1);
					//return float4(0,0,1,1);
					//return tex2D(_GrabTexture, i.uv);
				}
				ENDCG
			}
	} 
	FallBack "Diffuse"
}
