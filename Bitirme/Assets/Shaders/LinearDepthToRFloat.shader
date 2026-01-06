Shader "Hidden/LinearDepthToRFloat"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Overlay" }
        Pass
        {
            ZTest Always Cull Off ZWrite Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            UNITY_DECLARE_DEPTH_TEXTURE(_CameraDepthTexture);
            float _MaxDepthMeters;

            struct v2f
            {
                float4 pos : SV_POSITION;
                float2 uv  : TEXCOORD0;
            };

            v2f vert(appdata_img v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv  = v.texcoord;
                return o;
            }

            float frag(v2f i) : SV_Target
            {
                float raw01 = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);
                float eye   = LinearEyeDepth(raw01); // meters (Unity units)
                return min(eye, _MaxDepthMeters);
            }
            ENDHLSL
        }
    }
}

