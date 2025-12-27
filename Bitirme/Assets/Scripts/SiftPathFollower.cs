using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class SiftPathFollower : MonoBehaviour
{
    [Header("ROS Settings")]
    public string poseTopic = "/drone/sift_pose";
    public string referencesCsvPath = "/home/fidan/bitirme_dataset/references.csv";

    [Header("Drone References")]
    public DronePhysics drone;
    public Transform droneTransform;

    [Header("Navigation Parameters")]
    public int lookAhead = 10;          // Hedefin kaç frame ileride olduğu
    [Range(0f, 1f)]
    public float minConfidence = 0.4f;  // Güven eşiği (0.35'ten 0.4'e çektik, daha güvenli olsun)
    
    [Header("Safety & Filters")]
    public bool maxAllowedJumpActive = true;
    public float maxSeqJump = 50f;      // Ani frame atlaması limiti
    public int jumpTimeOutFrames = 5;   // KAÇ KEZ ısrar ederse zıplamayı kabul edelim? (YENİ)
    public int seqWindowSize = 3;       // Gecikme olmasın diye 3'e düşürdük

    [Header("Control Gains")]
    public float yawGain = 1.0f;
    public float lateralGain = 1.2f;
    public float maxTiltDeg = 20f;      // 25 biraz sertti, 20 daha stabil yapar

    [Header("Debug")]
    public bool debugLogs = true;
    public float debugLogPeriod = 0.5f; // Saniyede 2 kere log basar

    // --- Internal State ---
    private ROSConnection ros;
    private List<Vector3> referencePath = new List<Vector3>();
    private Queue<float> seqHistory = new Queue<float>();
    
    // Anlık Veriler
    private Vector3 currentEstPos = Vector3.zero;
    private float currentSeq = -1;
    private float rawSeq = -1;
    private float currentConf = 0;
    
    // Zıplama (Jump) Sayacı
    private int jumpRejectionCounter = 0; 

    // Log Zamanlayıcısı
    private float lastLogTime;
    private bool isInitialized = false;

    void Start()
    {
        if (drone == null) drone = GetComponent<DronePhysics>();
        if (droneTransform == null && drone != null) droneTransform = drone.transform;

        // GPS'i kapat, tamamen SIFT'e güveneceğiz
        if (drone != null) drone.useGPS = false;

        LoadReferenceCSV();

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32MultiArrayMsg>(poseTopic, OnPoseReceived);

        isInitialized = true;
        Debug.Log("[SiftPathFollower] Başlatıldı. Veri bekleniyor...");
    }

    // CSV Okuma (Sadece Konum İçin)
    void LoadReferenceCSV()
    {
        referencePath.Clear();
        if (!File.Exists(referencesCsvPath))
        {
            Debug.LogError($"[SiftPathFollower] CSV Bulunamadı: {referencesCsvPath}");
            return;
        }

        try
        {
            string[] lines = File.ReadAllLines(referencesCsvPath);
            // Header atla (i=1)
            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i];
                if (string.IsNullOrWhiteSpace(line)) continue;
                string[] parts = line.Split(',');
                
                if (parts.Length >= 4)
                {
                    // CSV formatı: seq, x, y, z, img_path
                    float x = float.Parse(parts[1]);
                    float y = float.Parse(parts[2]);
                    float z = float.Parse(parts[3]);
                    referencePath.Add(new Vector3(x, y, z));
                }
            }
            Debug.Log($"[SiftPathFollower] {referencePath.Count} waypoint yüklendi.");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[SiftPathFollower] CSV Hatası: {e.Message}");
        }
    }

    // ROS Callback
    void OnPoseReceived(Float32MultiArrayMsg msg)
    {
        // msg.data = [x, y, z, seq, confidence, match_count]
        if (msg.data.Length < 6) return;

        float x = msg.data[0];
        float y = msg.data[1];
        float z = msg.data[2];
        float incomingSeq = msg.data[3];
        float conf = msg.data[4];
        
        // Sadece loglama için sakla
        rawSeq = incomingSeq; 

        // 1. Güven Kontrolü (Confidence Check)
        if (conf < minConfidence)
        {
            currentConf = conf; // Düşük güven olduğunu bilmek için kaydet
            return; // Pozisyonu güncelleme, son bilinen yerde kal
        }

        // 2. Akıllı Zıplama Kontrolü (Smart Jump Logic)
        bool acceptData = true;

        if (maxAllowedJumpActive && currentSeq >= 0)
        {
            float jumpDiff = Mathf.Abs(incomingSeq - currentSeq);

            if (jumpDiff > maxSeqJump)
            {
                // Ani zıplama tespit edildi!
                jumpRejectionCounter++;

                if (jumpRejectionCounter < jumpTimeOutFrames)
                {
                    // Henüz limit dolmadı, bu veriyi "gürültü" say ve reddet
                    acceptData = false;
                }
                else
                {
                    // Limit doldu! Demek ki gerçekten oradayız (Israrcı Hata kuralı)
                    Debug.LogWarning($"[SiftPathFollower] BÜYÜK ZIPLAMA KABUL EDİLDİ! (Diff: {jumpDiff})");
                    
                    // Eski filtre geçmişini temizle ki yeni konuma hemen adapte olsun
                    seqHistory.Clear();
                    jumpRejectionCounter = 0;
                    acceptData = true;
                }
            }
            else
            {
                // Zıplama yok, her şey normal. Sayacı sıfırla.
                jumpRejectionCounter = 0;
                acceptData = true;
            }
        }

        if (acceptData)
        {
            // Pozisyonu hafif yumuşatarak al (Lerp) - Titremeyi azaltır
            Vector3 newPos = new Vector3(x, y, z);
            if (currentEstPos == Vector3.zero) currentEstPos = newPos;
            else currentEstPos = Vector3.Lerp(currentEstPos, newPos, 0.5f); // %50 yeni, %50 eski

            currentConf = conf;

            // Hareketli Ortalama (Moving Average) Filtresi
            seqHistory.Enqueue(incomingSeq);
            if (seqHistory.Count > seqWindowSize) seqHistory.Dequeue();

            float sum = 0;
            foreach (float s in seqHistory) sum += s;
            currentSeq = sum / seqHistory.Count;
        }
    }

    // Fizik ve Kontrol Döngüsü
    void FixedUpdate()
    {
        if (!isInitialized || drone == null) return;

        float pitchCmd = 0;
        float rollCmd = 0;
        float yawCmd = 0;
        string statusTag = "WAIT";

        // Güvenlik Kontrolleri
        if (referencePath.Count == 0 || currentSeq < 0)
        {
            statusTag = "NO_DATA";
            drone.SetExternalCommand(0, 0, 0); // Hover
        }
        else if (currentConf < minConfidence)
        {
            statusTag = "LOW_CONF"; // Güven düşük, yerinde dur
            drone.SetExternalCommand(0, 0, 0);
        }
        else
        {
            statusTag = "FLYING";

            // --- Hedef Belirleme ---
            int targetIndex = Mathf.FloorToInt(currentSeq) + lookAhead;
            // Dizi dışına taşmayı önle
            targetIndex = Mathf.Clamp(targetIndex, 0, referencePath.Count - 1);

            // Son noktaya geldik mi?
            if (targetIndex >= referencePath.Count - 5) statusTag = "ARRIVED";

            Vector3 targetPos = referencePath[targetIndex];
            
            // Hata Vektörü (Hedef - Ben)
            Vector3 errorVec = targetPos - currentEstPos;
            
            // Yüksekliği (Y) yoksay, sadece yatay düzlem (X,Z)
            Vector3 errorFlat = Vector3.ProjectOnPlane(errorVec, Vector3.up);

            // Drone'un burnu nereye bakıyor? (Yaw)
            float droneYaw = droneTransform.eulerAngles.y;
            Quaternion rot = Quaternion.Euler(0, droneYaw, 0);

            // Hata vektörünü Dünya'dan Drone'un Yerel (Local) eksenine çevir
            // Böylece: Z=İleri Hata, X=Sağ Hata olur.
            Vector3 localError = Quaternion.Inverse(rot) * errorFlat;

            // --- Kontrol Yasası (P-Controller) ---

            // 1. Yaw (Burun) Kontrolü: Hedefe dön
            float targetAngle = Mathf.Atan2(localError.x, localError.z) * Mathf.Rad2Deg;
            yawCmd = Mathf.Clamp(targetAngle * yawGain, -drone.maxYawTorqueCmd, drone.maxYawTorqueCmd);

            // 2. Pitch (İleri/Geri) ve Roll (Sağ/Sol)
            // localError.z pozitifse hedef öndedir -> Pitch (İleri git)
            // localError.x pozitifse hedef sağdadır -> Roll (Sağa yat)
            
            pitchCmd = Mathf.Clamp(localError.z * lateralGain, -maxTiltDeg, maxTiltDeg);
            rollCmd = Mathf.Clamp(localError.x * lateralGain, -maxTiltDeg, maxTiltDeg);

            // Komutları Drone Fiziğine Gönder
            drone.SetExternalCommand(pitchCmd, rollCmd, yawCmd);
        }

        // --- Düzenli Loglama ---
        if (debugLogs && Time.time - lastLogTime > debugLogPeriod)
        {
            lastLogTime = Time.time;
            Debug.Log($"[{statusTag}] " +
                      $"Seq: {currentSeq:F0} (Raw:{rawSeq:F0}) | " +
                      $"Target: {(int)(currentSeq + lookAhead)} | " +
                      $"Conf: {currentConf:F2} | " +
                      $"Cmd: (P:{pitchCmd:F1}, R:{rollCmd:F1}, Y:{yawCmd:F1}) | " +
                      $"RejCount: {jumpRejectionCounter}");
        }
    }
}
