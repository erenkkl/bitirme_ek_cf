using UnityEngine;

[System.Serializable] // Inspector'da ayarları görebilmek için
public class PIDController
{
    [Header("PID Parametreleri")]
    public float pGain = 1f;    // Proportional: Hataya anında tepki (Kaba Kuvvet)
    public float iGain = 0f;    // Integral: Zamanla biriken hatayı düzeltme (Rüzgar/Ağırlık dengesi)
    public float dGain = 0.5f;  // Derivative: Geleceği tahmin etme (Frenleme/Yumuşatma)
    
    [Header("Limitler")]
    public float maxOutput = 1000f; // Motorun verebileceği max tepki (Sınırlandırma)

    // Arka plan değişkenleri
    private float integralSum = 0f;
    private float lastError = 0f;

    // Hesapalama Fonksiyonu (Her fizik adımında çağıracağız)
    public float Calculate(float target, float current, float deltaTime)
    {
        // 1. Hata Hesapla (Hedef - Mevcut)
        float error = target - current;

        // 2. Integral (Biriken Hata)
        integralSum += error * deltaTime;
        
        // 3. Derivative (Hata Değişim Hızı)
        float derivative = (error - lastError) / deltaTime;
        lastError = error;

        // 4. PID Formülü: Output = (P * error) + (I * sum) + (D * deriv)
        float output = (error * pGain) + (integralSum * iGain) + (derivative * dGain);

        // 5. Çıktıyı Sınırla (Motorları yakmamak için)
        return Mathf.Clamp(output, -maxOutput, maxOutput);
    }

    public void Reset()
    {
        integralSum = 0f;
        lastError = 0f;
    }
}
