using UnityEngine;
using UnityEngine.Rendering;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using System;
using System.Collections;

/// <summary>
/// Publishes a front depth image as sensor_msgs/Image (encoding 32FC1) where each pixel is
/// linear eye depth in meters (clamped to maxDepth).
///
/// Requires a shader asset named: "Hidden/LinearDepthToRFloat"
/// (see shader code provided below).
/// </summary>
[RequireComponent(typeof(Camera))]
public class FrontDepthCameraPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string depthTopic = "/drone/camera/front_depth";
    public string frameId = "front_depth_cam";

    [Header("Resolution")]
    public int width = 640;
    public int height = 480;

    [Tooltip("Publish rate (Hz). Real output rate also depends on GPU readback time.")]
    public int publishFps = 20;

    [Header("Depth")]
    [Tooltip("Clamp depth to this max distance (meters).")]
    public float maxDepth = 50f;

    [Header("Capture Mode")]
    [Tooltip("If true, we call cam.Render() ourselves. Recommended if this camera is not part of the main render.")]
    public bool manualRender = true;

    [Tooltip("If manualRender is true, disable the camera so Unity doesn't also render it automatically.")]
    public bool disableCameraIfManual = true;

    [Tooltip("Render into a dummy color RT during manualRender to avoid drawing to screen.")]
    public bool renderToDummyTarget = true;

    [Header("Performance")]
    public bool useAsyncGPUReadback = true;

    [Tooltip("If true, skip publishing while a previous async readback is still in flight.")]
    public bool dropIfBusy = true;

    [Header("Debug")]
    public bool logOccasionally = false;

    [Tooltip("Log every N published frames (only if logOccasionally=true).")]
    public int logEveryN = 60;

    private ROSConnection ros;
    private Camera cam;

    private RenderTexture depthRT;
    private RenderTexture dummyColorRT;
    private Texture2D depthTex;
    private Rect rect;

    private float[] depthBuffer;
    private byte[] byteBuffer;

    private float period;
    private bool requestInFlight = false;

    private Material depthMat;
    private CommandBuffer depthCB;

    private int publishedCount = 0;

    void Start()
    {
        cam = GetComponent<Camera>();

        // Sanity
        width = Mathf.Clamp(width, 16, 4096);
        height = Mathf.Clamp(height, 16, 4096);
        publishFps = Mathf.Clamp(publishFps, 1, 60);
        period = 1f / publishFps;
        logEveryN = Mathf.Max(1, logEveryN);

        // ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(depthTopic);

        // Ensure depth texture exists
        cam.depthTextureMode |= DepthTextureMode.Depth;

        // Shader + material
        Shader sh = Shader.Find("Hidden/LinearDepthToRFloat");
        if (sh == null)
        {
            Debug.LogError("[FrontDepthCameraPublisher] Shader not found: Hidden/LinearDepthToRFloat. " +
                           "Add the shader file to your project (code provided).");
            enabled = false;
            return;
        }
        depthMat = new Material(sh);

        // Depth output RT (RFloat)
        depthRT = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat)
        {
            name = "FrontDepth_RFloat",
            enableRandomWrite = false,
            useMipMap = false,
            autoGenerateMips = false
        };
        depthRT.Create();

        // Dummy color target (optional; used only for manualRender)
        if (manualRender && renderToDummyTarget)
        {
            dummyColorRT = new RenderTexture(width, height, 16, RenderTextureFormat.ARGB32)
            {
                name = "FrontDepth_DummyColor",
                useMipMap = false,
                autoGenerateMips = false
            };
            dummyColorRT.Create();
        }

        // CPU-side readback texture for sync path
        rect = new Rect(0, 0, width, height);
        depthTex = new Texture2D(width, height, TextureFormat.RFloat, false, true);

        // Buffers
        int n = width * height;
        depthBuffer = new float[n];
        byteBuffer = new byte[n * sizeof(float)];

        // CommandBuffer: convert _CameraDepthTexture -> RFloat meters into depthRT
        BuildOrRebuildCommandBuffer();

        // If we render manually, optionally disable camera so it doesn't render twice
        if (manualRender && disableCameraIfManual)
            cam.enabled = false;

        StartCoroutine(PublishLoop());
    }

    void OnDestroy()
    {
        try
        {
            if (cam != null && depthCB != null)
            {
                cam.RemoveCommandBuffer(CameraEvent.AfterDepthTexture, depthCB);
            }
        }
        catch { /* ignore */ }

        if (depthCB != null)
        {
            depthCB.Release();
            depthCB = null;
        }

        if (depthMat != null)
        {
            Destroy(depthMat);
            depthMat = null;
        }

        if (depthRT != null)
        {
            depthRT.Release();
            Destroy(depthRT);
            depthRT = null;
        }

        if (dummyColorRT != null)
        {
            dummyColorRT.Release();
            Destroy(dummyColorRT);
            dummyColorRT = null;
        }

        if (depthTex != null)
        {
            Destroy(depthTex);
            depthTex = null;
        }
    }

    private void BuildOrRebuildCommandBuffer()
    {
        // Remove old if exists
        if (depthCB != null)
        {
            cam.RemoveCommandBuffer(CameraEvent.AfterDepthTexture, depthCB);
            depthCB.Release();
        }

        depthCB = new CommandBuffer { name = "DepthToRFloatMeters" };

        // Shader param
        depthMat.SetFloat("_MaxDepthMeters", maxDepth);

        // Source doesn't matter (shader samples _CameraDepthTexture), but CameraTarget is safe
        depthCB.Blit(BuiltinRenderTextureType.CameraTarget, new RenderTargetIdentifier(depthRT), depthMat);

        cam.AddCommandBuffer(CameraEvent.AfterDepthTexture, depthCB);
    }

    private IEnumerator PublishLoop()
    {
        var wait = new WaitForSecondsRealtime(period);

        while (enabled && gameObject.activeInHierarchy)
        {
            yield return wait;

            // Keep material param fresh if maxDepth changed in inspector
            if (depthMat != null)
                depthMat.SetFloat("_MaxDepthMeters", maxDepth);

            // Manual render guarantees _CameraDepthTexture + command buffer execution
            if (manualRender)
            {
                var prevTarget = cam.targetTexture;
                if (renderToDummyTarget && dummyColorRT != null)
                    cam.targetTexture = dummyColorRT;

                cam.Render();

                // Restore
                cam.targetTexture = prevTarget;
            }

            if (useAsyncGPUReadback)
            {
                if (requestInFlight && dropIfBusy)
                    continue;

                requestInFlight = true;
                try
                {
                    AsyncGPUReadback.Request(depthRT, 0, TextureFormat.RFloat, OnReadbackComplete);
                }
                catch
                {
                    requestInFlight = false;
                }
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

        if (!enabled || req.hasError)
            return;

        var data = req.GetData<float>();
        if (!data.IsCreated || data.Length < depthBuffer.Length)
            return;

        data.CopyTo(depthBuffer);

        // Clamp & sanitize
        ClampAndSanitizeDepth(depthBuffer, maxDepth);

        PublishDepth(depthBuffer);
    }

    private void CaptureSyncAndPublish()
    {
        // Read back depthRT into CPU
        var prev = RenderTexture.active;
        RenderTexture.active = depthRT;

        depthTex.ReadPixels(rect, 0, 0, false);
        depthTex.Apply(false, false);

        RenderTexture.active = prev;

        var raw = depthTex.GetRawTextureData<float>();
        if (raw.Length < depthBuffer.Length)
            return;

        raw.CopyTo(depthBuffer);

        ClampAndSanitizeDepth(depthBuffer, maxDepth);

        PublishDepth(depthBuffer);
    }

    private static void ClampAndSanitizeDepth(float[] buf, float maxD)
    {
        // Ensure we don't publish NaN/Inf and we clamp to [0, maxD]
        for (int i = 0; i < buf.Length; i++)
        {
            float v = buf[i];

            if (float.IsNaN(v) || float.IsInfinity(v))
                v = maxD;

            if (v < 0f) v = 0f;
            if (v > maxD) v = maxD;

            buf[i] = v;
        }
    }

    private void PublishDepth(float[] metersDepth)
    {
        // Convert float[] -> byte[] without allocations
        Buffer.BlockCopy(metersDepth, 0, byteBuffer, 0, byteBuffer.Length);

        // Timestamp (simulation time)
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
            encoding = "32FC1",
            is_bigendian = 0,
            step = (uint)(width * sizeof(float)),
            data = byteBuffer
        };

        ros.Publish(depthTopic, msg);

        publishedCount++;
        if (logOccasionally && (publishedCount % logEveryN == 0))
        {
            // Quick sanity: sample a few pixels
            float center = metersDepth[(height / 2) * width + (width / 2)];
            Debug.Log($"[FrontDepthCameraPublisher] Published #{publishedCount} depth. Center={center:F2} m");
        }
    }
}

