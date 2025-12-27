using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DronePhysics : MonoBehaviour
{
    [Header("Mod Ayarları")]
    public bool useGPS = true;
    public bool autoHover = true;

    [Header("Hedefler")]
    public float targetAltitude = 3.0f;
    private Vector3 savedStationPoint;

    [Header("PID Kontrolcüleri")]
    public PIDController throttlePID;
    public PIDController pitchPID;
    public PIDController rollPID;

    [Header("Yumuşatma Ayarları (Yeni)")]
    public float yawSpeed = 1.5f;        // Yaw komut kazancı (açı hatasından torka benzer komut üretir)
    public float turnSmoothness = 2.0f;  // Dönüş yumuşatma (Düşük = daha yumuşak)
    public float moveSmoothness = 5.0f;  // Pitch/Roll hedef yumuşatma

    [Header("Motor Fizik Ayarları")]
    public float maxThrust = 50f;
    public float maxTiltAngle = 35f;
    public Transform[] propellers;
    public float visualRotationSpeed = 4000f;

    [Header("Stabilite (Önerilen)")]
    [Tooltip("Açısal hız sönümü. Salınımı azaltır. 0.2-1.0 arası tipik.")]
    public float angularDamping = 0.5f;

    [Tooltip("Yaw komut limitidir (AddRelativeTorque için).")]
    public float maxYawTorqueCmd = 10f;

    private Rigidbody rb;

    // Anlık uygulanan değerler (Yumuşatılmış)
    private float currentPitch = 0f;
    private float currentRoll = 0f;
    private float currentYawTorque = 0f;

    // Hedeflenen ham değerler
    private float targetPitchRaw = 0f;
    private float targetRollRaw = 0f;
    private float targetYawRaw = 0f;

    // --- Dış Komut ---
    private float overridePitch = 0f;
    private float overrideRoll = 0f;
    private float overrideYaw = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // Mevcut davranışı koruyorum. İstersen bunları public yapıp Inspector’dan yönetebiliriz.
        rb.mass = 1.2f;
        rb.drag = 1f;
        rb.angularDrag = 4f;
    }

    void Start()
    {
        savedStationPoint = transform.position;
    }

    void FixedUpdate()
    {
        float throttleForce = 0f;
        if (autoHover) throttleForce = CalculateHover();

        if (useGPS)
        {
            CalculateGPSPositioning_Smooth();
        }
        else
        {
            // Otonom Mod (Python/ROS)
            targetPitchRaw = overridePitch;
            targetRollRaw = overrideRoll;
            targetYawRaw = overrideYaw;
        }

        // Değerleri Yumuşat (Amortisör)
        currentPitch = Mathf.Lerp(currentPitch, targetPitchRaw, Time.fixedDeltaTime * moveSmoothness);
        currentRoll = Mathf.Lerp(currentRoll, targetRollRaw, Time.fixedDeltaTime * moveSmoothness);
        currentYawTorque = Mathf.Lerp(currentYawTorque, targetYawRaw, Time.fixedDeltaTime * turnSmoothness);

        ApplyPhysics(throttleForce);
    }

    void Update()
    {
        RotatePropellers();
    }

    float CalculateHover()
    {
        // PID: targetAltitude - currentY
        float pidOut = throttlePID.Calculate(targetAltitude, transform.position.y, Time.fixedDeltaTime);

        // Sabit 11.77f yerine dinamik hover (mass ve gravity’ye göre)
        float hoverBase = rb.mass * -Physics.gravity.y;

        float totalForce = hoverBase + pidOut;
        return Mathf.Clamp(totalForce, 0f, maxThrust);
    }

    void CalculateGPSPositioning_Smooth()
    {
        // Hedefe olan vektör (world)
        Vector3 toTarget = savedStationPoint - transform.position;

        // Yalnızca yatay düzlemde çalış (roll/pitch kaynaklı eksen karışmasını azaltır)
        Vector3 toTargetFlat = Vector3.ProjectOnPlane(toTarget, Vector3.up);

        // Hedef çok yakınsa komutları sıfırla
        if (toTargetFlat.sqrMagnitude < 0.01f)
        {
            targetYawRaw = 0f;
            targetPitchRaw = 0f;
            targetRollRaw = 0f;
            return;
        }

        // Yaw-only body frame: drone’un sadece yaw’ını kullanarak hedefi lokale çevir
        Quaternion yawOnly = Quaternion.Euler(0f, transform.eulerAngles.y, 0f);
        Vector3 localFlat = Quaternion.Inverse(yawOnly) * toTargetFlat; // lokal (yaw eksenine göre)

        // 1) YAW: gerçek açı hatası (atan2)
        float yawErrorDeg = Mathf.Atan2(localFlat.x, localFlat.z) * Mathf.Rad2Deg;
        targetYawRaw = Mathf.Clamp(yawErrorDeg * yawSpeed, -maxYawTorqueCmd, maxYawTorqueCmd);

        // 2) PITCH: ileri/geri (eski semantiği koruyor: -z ölçümü ile PID)
        float pitchCommand = pitchPID.Calculate(0f, -localFlat.z, Time.fixedDeltaTime);
        targetPitchRaw = Mathf.Clamp(pitchCommand, -maxTiltAngle, maxTiltAngle);

        // 3) ROLL: yanal dengeleme
        float rollCommand = rollPID.Calculate(0f, -localFlat.x, Time.fixedDeltaTime);
        targetRollRaw = Mathf.Clamp(rollCommand, -maxTiltAngle, maxTiltAngle);
    }

    void ApplyPhysics(float upwardForce)
    {
        // Yukarı kuvvet (throttle)
        rb.AddRelativeForce(Vector3.up * upwardForce);

        // Pitch/Roll açılarını -180..180 aralığına getir
        float curPitchAngle = (transform.eulerAngles.x > 180f) ? transform.eulerAngles.x - 360f : transform.eulerAngles.x;
        float curRollAngle = (transform.eulerAngles.z > 180f) ? transform.eulerAngles.z - 360f : transform.eulerAngles.z;

        // Hata
        float pitchError = currentPitch - curPitchAngle;
        float rollError = -currentRoll - curRollAngle;

        // P (basit) tilt torku
        Vector3 tiltTorqueP = new Vector3(pitchError, 0f, rollError) * 2.0f;

        // Damping: angular velocity’ye karşı koy (local uzayda)
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        Vector3 tiltTorqueD = -localAngVel * angularDamping;

        rb.AddRelativeTorque(tiltTorqueP + tiltTorqueD);

        // Yaw torku (yumuşatılmış)
        rb.AddRelativeTorque(Vector3.up * currentYawTorque);
    }

    void RotatePropellers()
    {
        if (propellers == null) return;
        foreach (var p in propellers)
            if (p) p.Rotate(Vector3.up * visualRotationSpeed * Time.deltaTime);
    }

    public void SetGPSTarget(Vector3 newTarget)
    {
        // İrtifayı targetAltitude’a sabitle
        savedStationPoint = new Vector3(newTarget.x, targetAltitude, newTarget.z);
    }

    public void SetExternalCommand(float pitch, float roll, float yaw)
    {
        // Repeat fazında güvenlik: clamp
        overridePitch = Mathf.Clamp(pitch, -maxTiltAngle, maxTiltAngle);
        overrideRoll = Mathf.Clamp(roll, -maxTiltAngle, maxTiltAngle);
        overrideYaw = Mathf.Clamp(yaw, -maxYawTorqueCmd, maxYawTorqueCmd);
    }
}

