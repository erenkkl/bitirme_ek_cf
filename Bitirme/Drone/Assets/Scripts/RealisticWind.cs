using UnityEngine;

public class RealisticWind : MonoBehaviour
{
    [Header("Hedef")]
    public Rigidbody droneRb;

    [Header("Rüzgar Ayarları")]
    public bool isWindActive = true;
    
    [Tooltip("İtme Gücü (Sürüklenme)")]
    public float windStrength = 15f; 

    [Tooltip("Dikey hava boşluğu")]
    public float verticalTurbulence = 10f; 

    [Header("Yalpalama Ayarları")]
    [Tooltip("Sağa/Sola ve Öne/Arkaya yatırma gücü (ROLL & PITCH)")]
    public float tiltStrength = 8.0f; // <-- SENİN İSTEDİĞİN AYAR BU

    [Tooltip("Kendi ekseninde dönme gücü (YAW)")]
    public float yawStrength = 3.0f; 

    [Tooltip("Rüzgarın değişim hızı")]
    public float windChangeSpeed = 0.5f;

    // Perlin Noise Offsets
    private float pOffX, pOffY, pOffZ; // İtme için
    private float rOffX, rOffY, rOffZ; // Dönme için

    void Start()
    {
        // Rastgelelik tohumları
        pOffX = Random.Range(0f, 100f); pOffY = Random.Range(0f, 100f); pOffZ = Random.Range(0f, 100f);
        rOffX = Random.Range(0f, 100f); rOffY = Random.Range(0f, 100f); rOffZ = Random.Range(0f, 100f);
    }

    void FixedUpdate()
    {
        if (!isWindActive || droneRb == null) return;
        ApplyWindPhysics();
    }

    void ApplyWindPhysics()
    {
        float time = Time.time * windChangeSpeed;

        // --- 1. İTME (FORCE) ---
        float pushX = (Mathf.PerlinNoise(time + pOffX, 0) * 2f - 1f) * windStrength;
        float pushY = (Mathf.PerlinNoise(0, time + pOffY) * 2f - 1f) * verticalTurbulence; 
        float pushZ = (Mathf.PerlinNoise(time + pOffZ, time) * 2f - 1f) * windStrength; 
        
        droneRb.AddForce(new Vector3(pushX, pushY, pushZ), ForceMode.Force);

        // --- 2. YALPALAMA / ROLL (TORQUE) ---
        // Burası drone'un dengesini bozan kısım
        
        // Pitch (Öne/Arkaya sarsıntı - X ekseni)
        float rotPitch = (Mathf.PerlinNoise(time + rOffX, 0) * 2f - 1f) * tiltStrength;
        
        // Yaw (Kendi etrafında - Y ekseni)
        float rotYaw = (Mathf.PerlinNoise(0, time + rOffY) * 2f - 1f) * yawStrength;

        // Roll (Sağa/Sola sarsıntı - Z ekseni) <-- İSTEDİĞİN ETKİ
        float rotRoll = (Mathf.PerlinNoise(time + rOffZ, time) * 2f - 1f) * tiltStrength;

        Vector3 windTorque = new Vector3(rotPitch, rotYaw, rotRoll);
        droneRb.AddTorque(windTorque, ForceMode.Force);
    }
}
