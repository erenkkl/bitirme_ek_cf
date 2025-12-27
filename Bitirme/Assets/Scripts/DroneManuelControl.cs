using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneManualControl : MonoBehaviour
{
    [Header("Motor Güç Ayarları")]
    public float maxThrust = 30f;      // Motor gücü (Newton)
    public float responsiveness = 10f; // Tepki hızı
    public float rotateSpeed = 2.0f;   // Dönüş tork gücü

    [Header("Pervane Görsel Ayarları")]
    public Transform[] propellers;
    public float visualPropSpeed = 4000f;

    [Header("Kontrol (Klavye veya Slider)")]
    public bool useKeyboard = true; // Tıklıysa W,A,S,D çalışır

    // Test için Slider'lar
    [Range(0, 1)] public float throttleInput = 0f; // Space / Shift
    [Range(-1, 1)] public float pitchInput = 0f;   // W / S
    [Range(-1, 1)] public float rollInput = 0f;    // A / D
    [Range(-1, 1)] public float yawInput = 0f;     // Sol Ok / Sağ Ok

    private Rigidbody rb;
    private float finalThrottle;
    private float finalPitch;
    private float finalRoll;
    private float finalYaw;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.mass = 1.2f; 
        rb.drag = 1f; 
        rb.angularDrag = 4f;
    }

    void Update() // Girdileri burada alıyoruz
    {
        if (useKeyboard)
        {
            // Klavye girdilerini al
            pitchInput = Input.GetAxis("Vertical");     // W - S (Öne/Arkaya)
            rollInput = -Input.GetAxis("Horizontal");   // A - D (Sola/Sağa) - Eksi işareti yönü düzeltir
            
            // Yükselme (Space) / Alçalma (Left Control)
            if (Input.GetKey(KeyCode.Space)) throttleInput = 1f;
            else if (Input.GetKey(KeyCode.LeftControl)) throttleInput = 0f;
            // else throttleInput = 0.5f; // Tuşa basmıyorsan havada asılı kalmaya çalışsın (Kabaca)

            // Kendi etrafında dönme (Q ve E tuşları)
            if(Input.GetKey(KeyCode.E)) yawInput = 1f;
            else if(Input.GetKey(KeyCode.Q)) yawInput = -1f;
            else yawInput = 0f;
        }

        RotatePropellers();
    }

    void FixedUpdate() // Fiziği burada uyguluyoruz
    {
        ApplyPhysics();
    }

    void ApplyPhysics()
    {
        // 1. Değerleri yumuşat (Lerp)
        finalThrottle = Mathf.Lerp(finalThrottle, throttleInput, Time.fixedDeltaTime * responsiveness);
        finalPitch = Mathf.Lerp(finalPitch, pitchInput, Time.fixedDeltaTime * responsiveness);
        finalRoll = Mathf.Lerp(finalRoll, rollInput, Time.fixedDeltaTime * responsiveness);
        finalYaw = Mathf.Lerp(finalYaw, yawInput, Time.fixedDeltaTime * responsiveness);

        // 2. Yükselme Kuvveti
        Vector3 thrustForce = Vector3.up * finalThrottle * maxThrust;
        rb.AddRelativeForce(thrustForce);

        // 3. Dönüş Torku
        Vector3 torqueVector = new Vector3(finalPitch, finalYaw, -finalRoll);
        rb.AddRelativeTorque(torqueVector * rotateSpeed); 
    }

    void RotatePropellers()
    {
        if (propellers == null) return;
        
        float spinSpeed = 0f;
        if (finalThrottle > 0.01f)
        {
            spinSpeed = (finalThrottle * visualPropSpeed) + 500f;
        }

        foreach (var p in propellers)
        {
            if(p) p.Rotate(Vector3.up * spinSpeed * Time.deltaTime);
        }
    }
}
