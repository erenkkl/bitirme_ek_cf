Shader "Hidden/DepthToMeters"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            ZTest Always Cull Off ZWrite Off

            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _CameraDepthTexture;

            float4 frag(v2f_img i) : SV_Target
            {
                // Non-linear depth (0..1)
                float d = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);

                // Linear eye depth (Unity units; çoğu projede 1 unit ≈ 1 metre)
                float eye = LinearEyeDepth(d);

                // RFloat RT: metre değerini R kanalına yaz
                return float4(eye, 0, 0, 1);
            }
            ENDCG
        }
    }
    Fallback Off
}

