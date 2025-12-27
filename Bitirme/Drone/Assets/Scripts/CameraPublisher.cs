using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Assertions;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;
using System.Collections;

[RequireComponent(typeof(Camera))]
public class CameraPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string rosTopic = "/front_rgb/image_raw";
    public string frameId  = "front_cam";

    [Header("Image")]
    public int width = 640;
    public int height = 480;

    [Tooltip("Publish rate (Hz). Game FPS'ten bağımsızdır.")]
    public int publishFps = 20;

    [Tooltip("True: mono8 publish eder (daha hafif).")]
    public bool grayscale = false;

    [Header("Performance")]
    [Tooltip("Önerilen: AsyncGPUReadback (daha az ana-thread blok).")]
    public bool useAsyncGPUReadback = true;

    [Tooltip("AsyncGPUReadback yetişemezse yeni frame'i at (kuyruk biriktirme).")]
    public bool dropIfBusy = true;

    private ROSConnection ros;
    private Camera cam;

    private RenderTexture rt;

    // Sync fallback için
    private Texture2D readTex;
    private Rect rect;

    // Reuse buffers (GC azaltmak için)
    private byte[] rgbBuffer;
    private byte[] grayBuffer;

    private bool requestInFlight = false;
    private float period;

    void Start()
    {
        cam = GetComponent<Camera>();

        // Basit doğrulamalar
        width = Mathf.Max(16, width);
        height = Mathf.Max(16, height);
        publishFps = Mathf.Clamp(publishFps, 1, 120);
        period = 1f / publishFps;

        // ROS
        ros = ROSConnection.GetOrCreateInstance();
        // Not: Burada IP/Port set ediyorsun; projende global ayarlıyorsan kaldırabilirsin.
        ros.RosIPAddress = "127.0.0.1";
        ros.RosPort = 10000;
        ros.ConnectOnStart = true;

        ros.RegisterPublisher<ImageMsg>(rosTopic);

        // Render target
        rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        rt.Create();
        cam.targetTexture = rt;

        // Sync fallback data
        rect = new Rect(0, 0, width, height);
        if (!useAsyncGPUReadback)
        {
            readTex = new Texture2D(width, height, TextureFormat.RGB24, false);
        }

        // Buffers
        rgbBuffer = new byte[width * height * 3];
        if (grayscale)
            grayBuffer = new byte[width * height];

        // Game FPS'ine DOKUNMUYORUZ:
        // Application.targetFrameRate = ...
        // Time.captureFramerate = ...

        StartCoroutine(PublishLoop());
    }

    void OnDestroy()
    {
        if (rt != null)
        {
            cam.targetTexture = null;
            rt.Release();
            Destroy(rt);
        }
        if (readTex != null)
        {
            Destroy(readTex);
        }
    }

    private IEnumerator PublishLoop()
    {
        var wait = new WaitForSecondsRealtime(period);

        while (enabled && gameObject.activeInHierarchy)
        {
            yield return wait;

            // Bu frame sonunda RT güncellenmiş olsun
            yield return new WaitForEndOfFrame();

            if (useAsyncGPUReadback)
            {
                if (requestInFlight && dropIfBusy)
                    continue;

                requestInFlight = true;

                // TextureFormat.RGB24 diyerek daha küçük payload hedefliyoruz
                AsyncGPUReadback.Request(rt, 0, TextureFormat.RGB24, OnReadbackComplete);
            }
            else
            {
                // Sync fallback (daha basit ama ana thread'i bloklayabilir)
                CaptureSyncAndPublish();
            }
        }
    }

    private void OnReadbackComplete(AsyncGPUReadbackRequest req)
    {
        requestInFlight = false;

        if (req.hasError)
            return;

        var data = req.GetData<byte>();
        if (!data.IsCreated || data.Length < rgbBuffer.Length)
            return;

        // NativeArray -> managed byte[] (ROS msg byte[] istiyor)
        data.CopyTo(rgbBuffer);

        if (grayscale)
        {
            EnsureGrayBuffer();
            ToGrayscale(rgbBuffer, grayBuffer, width, height);
            Publish(grayBuffer, "mono8", 1);
        }
        else
        {
            Publish(rgbBuffer, "rgb8", 3);
        }
    }

    private void CaptureSyncAndPublish()
    {
        if (readTex == null) return;

        var prev = RenderTexture.active;
        RenderTexture.active = rt;

        // Kamera zaten rt'ye render ediyor. Ekstra cam.Render() çoğu durumda gerekmez.
        // Eğer kameran "enabled=false" ise veya pipeline'da render almıyorsa cam.Render() açabilirsin.
        // cam.Render();

        readTex.ReadPixels(rect, 0, 0, false);
        RenderTexture.active = prev;

        // Raw data (NativeArray) -> managed buffer
        var raw = readTex.GetRawTextureData<byte>();
        if (raw.Length >= rgbBuffer.Length)
            raw.CopyTo(rgbBuffer);

        if (grayscale)
        {
            EnsureGrayBuffer();
            ToGrayscale(rgbBuffer, grayBuffer, width, height);
            Publish(grayBuffer, "mono8", 1);
        }
        else
        {
            Publish(rgbBuffer, "rgb8", 3);
        }
    }

    private void EnsureGrayBuffer()
    {
        if (grayBuffer == null || grayBuffer.Length != width * height)
            grayBuffer = new byte[width * height];
    }

    // Float yerine integer ağırlıklar: (77R + 150G + 29B) >> 8  ~ ITU-R BT.601
    private static void ToGrayscale(byte[] rgb, byte[] gray, int w, int h)
    {
        int n = w * h;
        int i = 0;
        for (int p = 0; p < n; p++)
        {
            int r = rgb[i];
            int g = rgb[i + 1];
            int b = rgb[i + 2];
            gray[p] = (byte)((77 * r + 150 * g + 29 * b) >> 8);
            i += 3;
        }
    }

    private void Publish(byte[] data, string encoding, int channels)
    {
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
            encoding = encoding,
            is_bigendian = 0,
            step = (uint)(width * channels),
            data = data
        };

        ros.Publish(rosTopic, msg);
    }
}

