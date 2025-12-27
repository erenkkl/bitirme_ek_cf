using UnityEngine;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;
using System.Collections;
using Unity.Collections;

[RequireComponent(typeof(Camera))]
public class DepthImagePublisher : MonoBehaviour
{
    [Header("ROS")]
    public string rosTopic = "/front_depth/image_raw";
    public string frameId  = "front_cam";

    [Header("Image")]
    public int width = 640;
    public int height = 480;

    [Tooltip("Publish rate (Hz). Game FPS'ten bağımsızdır.")]
    public int publishFps = 20;

    [Header("Performance")]
    public bool useAsyncGPUReadback = true;
    public bool dropIfBusy = true;

    [Header("Shader")]
    public Shader depthToMetersShader; // Hidden/DepthToMeters

    private ROSConnection ros;
    private Camera cam;

    private RenderTexture depthRT;
    private Material depthMat;
    private CommandBuffer cmd;

    // Sync fallback
    private Texture2D readTex;
    private Rect rect;

    private byte[] depthBytes;
    private bool requestInFlight = false;
    private float period;

    void Start()
    {
        cam = GetComponent<Camera>();

        width = Mathf.Max(16, width);
        height = Mathf.Max(16, height);
        publishFps = Mathf.Clamp(publishFps, 1, 120);
        period = 1f / publishFps;

        // Depth texture üretimi
        cam.depthTextureMode |= DepthTextureMode.Depth;

        // ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(rosTopic);

        // Shader bul
        if (depthToMetersShader == null)
            depthToMetersShader = Shader.Find("Hidden/DepthToMeters");

        if (depthToMetersShader == null)
        {
            Debug.LogError("Hidden/DepthToMeters shader bulunamadı. Shader dosyasını eklediğinizden emin olun.");
            enabled = false;
            return;
        }

        depthMat = new Material(depthToMetersShader);

        // Depth RT (RFloat = 32-bit float tek kanal)
        depthRT = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat);
        depthRT.Create();

        // Bu RT’yi her frame, kameranın depth texture’ı hazır olduktan sonra doldur
        cmd = new CommandBuffer { name = "DepthToMeters (RFloat)" };
        cmd.Blit(BuiltinRenderTextureType.None, new RenderTargetIdentifier(depthRT), depthMat);
        cam.AddCommandBuffer(CameraEvent.AfterDepthTexture, cmd);

        rect = new Rect(0, 0, width, height);

        if (!useAsyncGPUReadback)
        {
            // Not: Bazı Unity sürümlerinde RFloat ReadPixels kısıtlı olabilir.
            // Sorun yaşarsanız TextureFormat.RGBAFloat’a geçilebilir.
            readTex = new Texture2D(width, height, TextureFormat.RFloat, false, true);
        }

        depthBytes = new byte[width * height * 4];

        StartCoroutine(PublishLoop());
    }

    void OnDestroy()
    {
        if (cam != null && cmd != null)
        {
            cam.RemoveCommandBuffer(CameraEvent.AfterDepthTexture, cmd);
            cmd.Release();
            cmd = null;
        }

        if (depthRT != null)
        {
            depthRT.Release();
            Destroy(depthRT);
        }
        if (depthMat != null) Destroy(depthMat);
        if (readTex != null) Destroy(readTex);
    }

    private IEnumerator PublishLoop()
    {
        var wait = new WaitForSecondsRealtime(period);

        while (enabled && gameObject.activeInHierarchy)
        {
            yield return wait;
            yield return new WaitForEndOfFrame();

            if (useAsyncGPUReadback)
            {
                if (requestInFlight && dropIfBusy)
                    continue;

                requestInFlight = true;
                AsyncGPUReadback.Request(depthRT, 0, OnReadbackComplete);
            }
            else
            {
                CaptureSyncAndPublish();
            }
        }
    }

    private void OnReadbackComplete(AsyncGPUReadbackRequest req)
    {
        requestInFlight = false;

        if (req.hasError)
            return;

        // RFloat RT => raw byte data (width*height*4)
        NativeArray<byte> data = req.GetData<byte>();
        if (!data.IsCreated || data.Length < depthBytes.Length)
            return;

        data.CopyTo(depthBytes);

        Publish(depthBytes, "32FC1", 4);
    }

    private void CaptureSyncAndPublish()
    {
        if (readTex == null) return;

        var prev = RenderTexture.active;
        RenderTexture.active = depthRT;

        readTex.ReadPixels(rect, 0, 0, false);
        readTex.Apply(false, false);

        RenderTexture.active = prev;

        var raw = readTex.GetRawTextureData<byte>();
        if (raw.Length >= depthBytes.Length)
            raw.CopyTo(depthBytes);

        Publish(depthBytes, "32FC1", 4);
    }

    private void Publish(byte[] data, string encoding, int bytesPerPixel)
    {
        // CameraPublisher ile aynı zaman damgası yaklaşımı
        double now = Time.realtimeSinceStartupAsDouble;
        int sec = (int)Math.Floor(now);
        uint nsec = (uint)Math.Round((now - sec) * 1e9);

        var msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg(sec, nsec),
                frame_id = frameId
            },
            height = (uint)height,
            width = (uint)width,
            encoding = encoding,       // "32FC1"
            is_bigendian = 0,
            step = (uint)(width * bytesPerPixel),  // width * 4
            data = data
        };

        ros.Publish(rosTopic, msg);
    }
}

