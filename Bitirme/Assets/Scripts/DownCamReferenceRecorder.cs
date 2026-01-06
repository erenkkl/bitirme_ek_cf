using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Threading;
using UnityEngine;
using Unity.Collections;

[RequireComponent(typeof(Camera))]
public class DownCamReferenceRecorder : MonoBehaviour
{
    [Header("Mode")]
    [Tooltip("True ise dataset'i her Play'de sıfırdan başlatır ve kayıt alır. False ise dosyalara dokunmaz.")]
    public bool enableRecording = true;

    [Header("References")]
    [Tooltip("Poz/yönelim kaynağı. Genelde drone gövdesi. Boşsa kamera transformu kullanılır.")]
    public Transform poseSource;

    [Header("Capture Policy (Hybrid)")]
    public float minDt = 0.2f;
    public float maxDt = 1.5f;
    public float distanceThreshold = 1.0f;
    public float yawThresholdDeg = 15f;

    [Header("Image Settings")]
    public int width = 640;
    public int height = 480;
    public bool grayscale = true;
    public bool flipVertical = true;
    public bool forceRenderOnCapture = true;
    public bool useExistingTargetTextureIfPossible = true;

    [Header("Output (Single Dataset)")]
    [Tooltip("Boşsa home altına yazar: ~/bitirme_cetin_dataset. Tam yol vermek istersen doldur.")]
    public string outputBasePathOverride = "";

    [Tooltip("Override boşsa, home altında kullanılacak klasör adı.")]
    public string homeFolderName = "bitirme_cetin_dataset";

    [Tooltip("Play başında images klasörünü temizle.")]
    public bool clearImagesOnStart = true;

    public int writerQueueCapacity = 256;

    // --- Internal ---
    private Camera cam;
    private RenderTexture rt;

    private Texture2D tex;        // RGB24 for ReadPixels + Encode
    private byte[] rgbBuffer;     // managed raw buffer (w*h*3)
    private byte[] tempRow;       // for flip

    private Vector3 lastSavedPos;
    private float lastSavedYawDeg;
    private float lastSavedTime;
    private int seq;

    private string datasetDir;
    private string imagesDir;
    private string csvPath;

    private BlockingCollection<FrameRecord> queue;
    private Thread writerThread;
    private volatile bool isShuttingDown;

    private StreamWriter csvWriter;
    private readonly object csvLock = new object();

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private struct FrameRecord
    {
        public int seq;
        public Vector3 pos;
        public string imageFileName;
        public byte[] pngBytes;
    }

    void Awake()
    {
        // Component kapalıysa Unity zaten Awake çağırmaz.
        // Ama kullanıcı "enableRecording=false" yaparsa, dosyaya dokunmadan tamamen devre dışı kalalım.
        if (!enableRecording)
        {
            // Dosyalara dokunma.
            enabled = false;
            return;
        }

        cam = GetComponent<Camera>();
        if (poseSource == null) poseSource = transform;

        SetupOutputSingleDataset_Reset(); // <-- burada sıfırlama/temizleme var (sadece recording açıkken)
        SetupBuffers();
        StartWriter();

        seq = 0;
        lastSavedPos = poseSource.position;
        lastSavedYawDeg = poseSource.rotation.eulerAngles.y;
        lastSavedTime = Time.realtimeSinceStartup;

        Debug.Log($"[DownCamReferenceRecorder] Recording ENABLED. Dataset dir: {datasetDir}");
        Debug.Log($"[DownCamReferenceRecorder] Images dir: {imagesDir}");
        Debug.Log($"[DownCamReferenceRecorder] CSV path: {csvPath}");
    }

    void OnDisable() => ShutdownAndCleanup();
    void OnDestroy() => ShutdownAndCleanup();
    void OnApplicationQuit() => ShutdownAndCleanup();

    private void ShutdownAndCleanup()
    {
        if (isShuttingDown) return;
        isShuttingDown = true;

        ShutdownWriter();
        CleanupBuffers();
    }

    void Update()
    {
        if (isShuttingDown) return;

        float now = Time.realtimeSinceStartup;
        float dt = now - lastSavedTime;
        if (dt < minDt) return;

        Vector3 pos = poseSource.position;
        float dist = Vector3.Distance(pos, lastSavedPos);

        float yawNow = poseSource.rotation.eulerAngles.y;
        float yawDelta = Mathf.Abs(Mathf.DeltaAngle(lastSavedYawDeg, yawNow));

        bool dueToMaxDt = dt >= maxDt;
        bool dueToDist = dist >= distanceThreshold;
        bool dueToYaw = yawThresholdDeg > 0f && yawDelta >= yawThresholdDeg;

        if (!(dueToMaxDt || dueToDist || dueToYaw))
            return;

        int thisSeq = seq++;
        string fileName = $"downcam_{thisSeq:D06}.png";

        lastSavedPos = pos;
        lastSavedYawDeg = yawNow;
        lastSavedTime = now;

        StartCoroutine(CaptureAndEnqueue(thisSeq, pos, fileName));
    }

    private System.Collections.IEnumerator CaptureAndEnqueue(int thisSeq, Vector3 pos, string fileName)
    {
        yield return new WaitForEndOfFrame();

        if (forceRenderOnCapture)
        {
            cam.targetTexture = rt;
            cam.Render();
        }

        var prevActive = RenderTexture.active;
        RenderTexture.active = rt;

        tex.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
        tex.Apply(false, false);

        RenderTexture.active = prevActive;

        NativeArray<byte> raw = tex.GetRawTextureData<byte>();
        if (rgbBuffer == null || rgbBuffer.Length != raw.Length)
            rgbBuffer = new byte[raw.Length];

        raw.CopyTo(rgbBuffer);

        if (grayscale)
        {
            for (int i = 0; i < rgbBuffer.Length; i += 3)
            {
                byte r = rgbBuffer[i + 0];
                byte g = rgbBuffer[i + 1];
                byte b = rgbBuffer[i + 2];
                byte y = (byte)((r * 77 + g * 150 + b * 29) >> 8);
                rgbBuffer[i + 0] = y;
                rgbBuffer[i + 1] = y;
                rgbBuffer[i + 2] = y;
            }
        }

        if (flipVertical)
        {
            int rowBytes = width * 3;
            if (tempRow == null || tempRow.Length != rowBytes)
                tempRow = new byte[rowBytes];

            for (int y = 0; y < height / 2; y++)
            {
                int top = y * rowBytes;
                int bottom = (height - 1 - y) * rowBytes;

                Buffer.BlockCopy(rgbBuffer, top, tempRow, 0, rowBytes);
                Buffer.BlockCopy(rgbBuffer, bottom, rgbBuffer, top, rowBytes);
                Buffer.BlockCopy(tempRow, 0, rgbBuffer, bottom, rowBytes);
            }
        }

        tex.LoadRawTextureData(rgbBuffer);
        tex.Apply(false, false);

        byte[] pngBytes = tex.EncodeToPNG();

        if (isShuttingDown || queue == null || queue.IsAddingCompleted)
            yield break;

        queue.Add(new FrameRecord
        {
            seq = thisSeq,
            pos = pos,
            imageFileName = fileName,
            pngBytes = pngBytes
        });
    }

    // Recorder aktifken: dataset'i sıfırdan başlat
    private void SetupOutputSingleDataset_Reset()
    {
        string basePath;
        if (!string.IsNullOrWhiteSpace(outputBasePathOverride))
        {
            basePath = outputBasePathOverride;
        }
        else
        {
            string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
            basePath = Path.Combine(home, homeFolderName);
        }

        datasetDir = basePath;
        imagesDir = Path.Combine(datasetDir, "images");
        csvPath = Path.Combine(datasetDir, "references.csv");

        Directory.CreateDirectory(imagesDir);

        if (clearImagesOnStart)
        {
            try
            {
                foreach (var f in Directory.GetFiles(imagesDir, "*.png"))
                    File.Delete(f);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[DownCamReferenceRecorder] Could not clear images folder: {ex.Message}");
            }
        }

        // CSV'yi her run'da sıfırla (tam istediğin davranış)
        csvWriter = new StreamWriter(csvPath, append: false);
        csvWriter.WriteLine("seq,pos_x,pos_y,pos_z,image_rel");
        csvWriter.Flush();
    }

    private void SetupBuffers()
    {
        RenderTexture existing = cam.targetTexture;

        if (useExistingTargetTextureIfPossible &&
            existing != null &&
            existing.width == width &&
            existing.height == height)
        {
            rt = existing;
        }
        else
        {
            rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32)
            {
                name = "DownCamRecorderRT"
            };
            cam.targetTexture = rt;
        }

        tex = new Texture2D(width, height, TextureFormat.RGB24, mipChain: false, linear: false);

        rgbBuffer = new byte[width * height * 3];
        tempRow = new byte[width * 3];
    }

    private void CleanupBuffers()
    {
        if (tex != null) Destroy(tex);

        if (rt != null && rt.name == "DownCamRecorderRT")
        {
            cam.targetTexture = null;
            rt.Release();
            Destroy(rt);
        }

        tex = null;
        rt = null;
        rgbBuffer = null;
        tempRow = null;
    }

    private void StartWriter()
    {
        queue = new BlockingCollection<FrameRecord>(Mathf.Max(16, writerQueueCapacity));
        writerThread = new Thread(WriterLoop)
        {
            IsBackground = true,
            Name = "DownCamReferenceRecorderWriter"
        };
        writerThread.Start();
    }

    private void ShutdownWriter()
    {
        try
        {
            if (queue != null && !queue.IsAddingCompleted)
                queue.CompleteAdding();

            if (writerThread != null && writerThread.IsAlive)
                writerThread.Join();
        }
        catch (Exception ex)
        {
            Debug.LogError($"[DownCamReferenceRecorder] Shutdown exception: {ex}");
        }
        finally
        {
            lock (csvLock)
            {
                csvWriter?.Flush();
                csvWriter?.Dispose();
                csvWriter = null;
            }

            queue?.Dispose();
            queue = null;
            writerThread = null;
        }
    }

    private void WriterLoop()
    {
        try
        {
            foreach (var rec in queue.GetConsumingEnumerable())
            {
                string imgAbs = Path.Combine(imagesDir, rec.imageFileName);
                File.WriteAllBytes(imgAbs, rec.pngBytes);

                string imgRel = "images/" + rec.imageFileName;

                string line =
                    rec.seq.ToString(Inv) + "," +
                    rec.pos.x.ToString("F6", Inv) + "," +
                    rec.pos.y.ToString("F6", Inv) + "," +
                    rec.pos.z.ToString("F6", Inv) + "," +
                    imgRel;

                lock (csvLock)
                {
                    csvWriter.WriteLine(line);
                    csvWriter.Flush();
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[DownCamReferenceRecorder] Writer thread exception: {ex}");
        }
    }
}

