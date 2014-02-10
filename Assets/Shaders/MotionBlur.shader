Shader "Custom/MotionBlur" 
{
	Properties 
	{
		_MainTex ("Main Texture", 2D) = "white" {}
		_Color ("Diffuse Material COlor", Color) = (1.0, 1.0, 1.0, 1.0)
	}

	SubShader 
	{
		//ZTest Always Cull Off ZWrite On Fog { Mode Off }

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
        
			#include "UnityCG.cginc"

			#define NORMAL_TO_WORLD(normal) normalize(mul(float4(normal, 1.0), _World2Object).xyz)

			uniform sampler2D _MainTex;
			uniform fixed4 _Color;
			uniform fixed4 _LightColor0;
			uniform float4x4 _PrevVP;
			uniform float4x4 _P;
			uniform float4x4 _V;

			struct fragmentInput 
			{
				float4 pos : POSITION;
				half2 uv : TEXCOORD0;
				float4 velocity : COLOR;
				float4 color;
			};

			fragmentInput vert(appdata_base v)
			{
				//UNITY_MATRIX_MVP
				fragmentInput o;
				float4x4 m = _Object2World;
				float4x4 vp = _PrevVP;
				float4x4 mvp = mul(vp, m);

				//model view projection matrices
				o.pos = mul(UNITY_MATRIX_MVP, v.vertex); //testing to get mvp from script
				o.uv = MultiplyUV(UNITY_MATRIX_TEXTURE0, v.texcoord.xy);

				//velocity
				o.velocity = float4(0,1,0,1);

				//lighting
				float3 normalDirection = NORMAL_TO_WORLD(v.normal);
				float3 lightDirection = normalize(_WorldSpaceLightPos0.xyz);
				float3 diffuse = _LightColor0.xyz * _Color.rgb * max(0.0, dot(normalDirection, lightDirection));

				o.color = half4(diffuse, 1.0);

				return o;
			}

			fixed4 frag(fragmentInput f) : COLOR
			{
				return tex2D(_MainTex, f.uv) * f.color;
				//return float4(f.pos.x, f.pos.y, f.pos.z, 1);
				//return f.velocity;
				//return float4(1,1,1,1);
			}

			ENDCG
		}
	} 
	FallBack "Diffuse"
}
