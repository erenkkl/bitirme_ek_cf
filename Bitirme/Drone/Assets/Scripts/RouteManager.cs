using UnityEngine;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor; // Handles.Label için gerekli olabilir
#endif

public class RouteManager : MonoBehaviour
{
    [Header("Rota Noktaları")]
    public List<Transform> waypoints = new List<Transform>();
    
    [Header("Görsel Ayarlar")]
    public Color routeColor = Color.yellow;
    public float sphereSize = 0.5f;

    // --- MEVCUT FONKSİYONLAR ---

    [ContextMenu("Toplam Mesafeyi Hesapla")]
    public void CalculateTotalDistance()
    {
        if (waypoints.Count < 2)
        {
            Debug.LogWarning("⚠️ Mesafe ölçmek için en az 2 nokta lazım!");
            return;
        }

        float totalDist = 0f;
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            if (waypoints[i] != null && waypoints[i+1] != null)
            {
                totalDist += Vector3.Distance(waypoints[i].position, waypoints[i + 1].position);
            }
        }
        Debug.Log($"📏 Toplam Rota Uzunluğu: {totalDist:F2} metre");
    }

    void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Count == 0) return;

        Gizmos.color = routeColor;

        for (int i = 0; i < waypoints.Count; i++)
        {
            // Eğer liste elemanı boşsa çizmeye çalışma
            if (waypoints[i] == null) continue;

            // Noktayı top olarak çiz
            Gizmos.DrawWireSphere(waypoints[i].position, sphereSize);

            // Bir sonraki noktaya çizgi çek
            if (i < waypoints.Count - 1 && waypoints[i + 1] != null)
            {
                Gizmos.DrawLine(waypoints[i].position, waypoints[i + 1].position);
            }
        }
    }

    // --- YENİ EKLENEN DÜZELTME FONKSİYONLARI ---

    // Listenin içindeki silinmiş (Missing) objeleri temizler
    [ContextMenu("🧹 Listeyi Temizle ve Onar")]
    public void CleanList()
    {
        // Geriye doğru döngü kurarak siliyoruz (Liste kaymasın diye)
        for (int i = waypoints.Count - 1; i >= 0; i--)
        {
            // Eğer kutucuk boşsa veya obje silinmişse (null)
            if (waypoints[i] == null)
            {
                waypoints.RemoveAt(i);
            }
        }
        Debug.Log("✅ Liste temizlendi! Ölü noktalar atıldı.");
    }
    
    // İsimleri WP_0, WP_1 diye baştan isimlendirir
    [ContextMenu("🏷️ İsimleri Sırala")]
    public void RenameWaypoints()
    {
        for (int i = 0; i < waypoints.Count; i++)
        {
            if (waypoints[i] != null)
            {
                waypoints[i].name = "WP_" + i;
            }
        }
        Debug.Log("✅ Tüm noktalar yeniden isimlendirildi.");
    }
}
