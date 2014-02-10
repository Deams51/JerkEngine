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
				uniform float4x4 _PrevVP;
				uniform sampler2D _GrabTexture;
				uniform float _Top;
				uniform float _Bottom;
 
				struct fragmentInput 
				{
					float4 pos : POSITION;
					float4 velocity : COLOR;
					float4 color;
					float4 screenPos : TEXCOORD0;
				};
 
				fragmentInput vert(appdata_base v)
				{
					fragmentInput o;
					float4x4 m = _Object2World;
					float4x4 vp = _PrevVP;
					float4x4 mvp = mul(vp, m);

					o.velocity = float4(0,1,0,1);

					o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
					//o.uv = MultiplyUV(UNITY_MATRIX_TEXTURE0, v.texcoord.xy);
					o.screenPos = o.pos;

					return o;
				}
 
				float4 frag(fragmentInput i) : COLOR
				{
					float2 screenPos = i.screenPos.xy / i.screenPos.w;
					float halved = (_Top + _Bottom) * 0.5;
					float diff = (_Bottom - _Top) * 0.5;
					screenPos.x = screenPos.x * (halved + diff * screenPos.y);
					screenPos.x = (screenPos.x + 1) * 0.5;
					screenPos.y = 1-(screenPos.y + 1) * 0.5 ;
					half4 sum = half4(0.0h,0.0h,0.0h,0.0h);  
					//return tex2D( _GrabTexture, screenPos);
					return float4(0,0,1,1);
					//return tex2D(_GrabTexture, i.uv);
				}
				ENDCG
			}
	} 
	FallBack "Diffuse"
}
