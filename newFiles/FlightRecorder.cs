using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Threading;
using UnityEngine;
using Unity.Collections;

[RequireComponent(typeof(Camera))]
public class FlightRecorder : MonoBehaviour
{
    [Header("Dependencies")]
    public SiftPathFollower pathFollower;

    [Header("Circular Buffer Settings")]
    public int maxFramesToKeep = 60; 

    [Header("Capture Settings")]
    public bool enableRecording = true;
    public float captureInterval = 0.2f; 

    [Header("Image Settings")]
    public int width = 320; 
    public int height = 240;
    public bool grayscale = true;
    public bool flipVertical = true;

    [Header("Output")]
    public string folderName = "flight_dataset";
    public bool clearOnStart = true;
    public int queueCapacity = 50; // Kapasiteyi biraz düşürdük, RAM şişmesin

    // --- Internal ---
    private Camera cam;
    private RenderTexture rt;
    private Texture2D tex;
    private byte[] rgbBuffer;
    private byte[] tempRow;

    private float lastSavedTime;
    private int globalSeqCounter = 0; 

    private string datasetDir;
    private string imagesDir;
    private string csvPath;

    private BlockingCollection<FlightRecord> queue;
    private Thread writerThread;
    private volatile bool isShuttingDown;

    private StreamWriter csvWriter;
    private readonly object csvLock = new object();
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    private struct FlightRecord
    {
        public int seqID;
        public float timestamp;
        public float matchSeq;
        public float matchConf;
        public string fileName;
        public byte[] pngBytes;
    }

    void Awake()
    {
        if (!enableRecording) { enabled = false; return; }

        cam = GetComponent<Camera>();
        // Derinlik ayarını düşürelim ki Main Camera'nın önüne geçmesin
        cam.depth = -10; 
        
        if (pathFollower == null) pathFollower = GetComponentInParent<SiftPathFollower>();

        SetupDirectories();
        SetupBuffers();
        StartWriterThread();

        globalSeqCounter = 0;
        lastSavedTime = Time.time;

        Debug.Log($"[FlightRecorder] 🔴 NON-BLOCKING RECORDER AKTIF.");
    }

    void OnDisable() => Shutdown();
    void OnDestroy() => Shutdown();

    void Update()
    {
        if (isShuttingDown) return;

        // Recovery/Test modunda kayit yapma (Dataset bozulmasin)
        if (pathFollower != null && pathFollower.IsInRecoveryMode) return;

        if (Time.time - lastSavedTime >= captureInterval)
        {
            lastSavedTime = Time.time;

            // Kuyruk doluysa zorlama, bu kareyi atla (Drone fiziği bozulmasın diye)
            if (queue.Count >= queueCapacity) 
            {
                // Debug.LogWarning("Recorder Kuyruğu Dolu! Kare atlandı."); 
                return;
            }

            float mSeq = -1f; 
            float mConf = 0f;
            
            if (pathFollower != null)
            {
                mSeq = pathFollower.GetLastMatchedSeq();
                mConf = pathFollower.GetCurrentConfidence();
            }

            int currentID = globalSeqCounter++;
            string fName = $"flight_{currentID:D06}.png";

            StartCoroutine(CaptureFrame(currentID, Time.time, mSeq, mConf, fName));
        }
    }

    System.Collections.IEnumerator CaptureFrame(int id, float time, float mSeq, float mConf, string fName)
    {
        yield return new WaitForEndOfFrame();

        var oldRT = RenderTexture.active;
        var oldTarget = cam.targetTexture; // Kameranın eski hedefini sakla

        cam.targetTexture = rt;
        cam.Render();
        
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
        tex.Apply(false, false);
        
        // ÖNEMLİ: Kamerayı eski haline döndür ki ROS Publisher bozulmasın
        cam.targetTexture = oldTarget; 
        RenderTexture.active = oldRT;

        NativeArray<byte> raw = tex.GetRawTextureData<byte>();
        if (rgbBuffer == null || rgbBuffer.Length != raw.Length) rgbBuffer = new byte[raw.Length];
        raw.CopyTo(rgbBuffer);

        if (grayscale)
        {
            for (int i = 0; i < rgbBuffer.Length; i += 3)
            {
                byte val = (byte)((rgbBuffer[i] * 77 + rgbBuffer[i + 1] * 150 + rgbBuffer[i + 2] * 29) >> 8);
                rgbBuffer[i] = val; rgbBuffer[i + 1] = val; rgbBuffer[i + 2] = val;
            }
        }

        if (flipVertical)
        {
            int rowLen = width * 3;
            if (tempRow == null || tempRow.Length != rowLen) tempRow = new byte[rowLen];
            for (int y = 0; y < height / 2; y++)
            {
                int top = y * rowLen;
                int btm = (height - 1 - y) * rowLen;
                Buffer.BlockCopy(rgbBuffer, top, tempRow, 0, rowLen);
                Buffer.BlockCopy(rgbBuffer, btm, rgbBuffer, top, rowLen);
                Buffer.BlockCopy(tempRow, 0, rgbBuffer, btm, rowLen);
            }
        }

        tex.LoadRawTextureData(rgbBuffer);
        tex.Apply();
        byte[] png = tex.EncodeToPNG(); // Bu işlem hala biraz yük bindirir ama mecburuz

        if (!isShuttingDown && queue != null && !queue.IsAddingCompleted)
        {
            // TryAdd: Kuyruk doluysa bekleme yapma, false döner ve geçer.
            // Böylece oyun/fizik asla donmaz.
            queue.TryAdd(new FlightRecord
            {
                seqID = id,
                timestamp = time,
                matchSeq = mSeq,
                matchConf = mConf,
                fileName = fName,
                pngBytes = png
            });
        }
    }

    private void SetupDirectories()
    {
        string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
        datasetDir = Path.Combine(home, folderName);
        imagesDir = Path.Combine(datasetDir, "images");
        csvPath = Path.Combine(datasetDir, "references.csv"); 

        Directory.CreateDirectory(imagesDir);

        if (clearOnStart)
        {
            try { foreach (var f in Directory.GetFiles(imagesDir, "*.png")) File.Delete(f); } catch { }
        }

        csvWriter = new StreamWriter(csvPath, append: false);
        csvWriter.WriteLine("seq,timestamp,image_rel,recorded_conf,original_match_seq");
        csvWriter.Flush();
    }

    private void SetupBuffers()
    {
        rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        rgbBuffer = new byte[width * height * 3];
    }

    private void StartWriterThread()
    {
        queue = new BlockingCollection<FlightRecord>(queueCapacity);
        writerThread = new Thread(WriterLoop) { IsBackground = true, Priority = System.Threading.ThreadPriority.BelowNormal };
        writerThread.Start();
    }

    private void WriterLoop()
    {
        foreach (var rec in queue.GetConsumingEnumerable())
        {
            try
            {
                string fullPath = Path.Combine(imagesDir, rec.fileName);
                File.WriteAllBytes(fullPath, rec.pngBytes);

                string line = string.Format(Inv, "{0},{1:F3},{2},{3:F2},{4:F0}",
                    rec.seqID,
                    rec.timestamp,
                    "images/" + rec.fileName,
                    rec.matchConf,
                    rec.matchSeq
                );

                lock (csvLock) { csvWriter.WriteLine(line); csvWriter.Flush(); }

                if (rec.seqID >= maxFramesToKeep)
                {
                    int idToDelete = rec.seqID - maxFramesToKeep;
                    string fileToDelete = $"flight_{idToDelete:D06}.png";
                    string pathToDelete = Path.Combine(imagesDir, fileToDelete);
                    if (File.Exists(pathToDelete)) File.Delete(pathToDelete);
                }
            }
            catch (Exception ex) { Debug.LogError($"[FlightRecorder] Write Error: {ex.Message}"); }
        }
    }

    private void Shutdown()
    {
        isShuttingDown = true;
        try { queue?.CompleteAdding(); writerThread?.Join(); } catch { }
        finally 
        { 
            lock (csvLock) { csvWriter?.Dispose(); csvWriter = null; }
            if (tex) Destroy(tex);
            if (rt) { rt.Release(); Destroy(rt); }
        }
    }
}