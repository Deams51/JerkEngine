Shader "Custom/MotionBlur2" 
{
	Properties 
	{
		_MainTex ("Base (RGB)", 2D) = "white" {}
	}

	SubShader 
	{
		Tags { "Queue" = "Geometry-1" }

			Pass
			{
				CGPROGRAM
				#pragma vertex vert
				#pragma fragment frag
				#pragma fragmentoption ARB_precision_hint_fastest

				#include "UnityCG.cginc"

				uniform sampler2D _MainTex;
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
				half4 frag(fragmentInput i) : COLOR
				{
					float2 position1 = i.newPos.xy / i.newPos.w;
					float2 position2 = i.oldPos.xy / i.oldPos.w;
					float2 delta = (position2 - position1) * _BlurFactor + 0.5;

					return half4(delta.x, delta.y, 0, 1);
				}
				ENDCG
			}

			GrabPass { "_VelocityTexture" }

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

/*
			Pass
			{
				CGPROGRAM
				#pragma vertex vert
				#pragma fragment frag
        
				#include "UnityCG.cginc"

				uniform sampler2D _GrabTexture : register(s1);

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
					return tex2D(_GrabTexture, f.uv);
				}
				ENDCG
			}
*/
	} 
	FallBack "Diffuse"
}
