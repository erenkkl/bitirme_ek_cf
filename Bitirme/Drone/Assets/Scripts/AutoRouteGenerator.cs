using UnityEngine;
using System.Collections.Generic;

[ExecuteInEditMode]
public class AutoRouteGenerator : MonoBehaviour
{
    [Header("Hedef Depo (Zorunlu)")]
    public RouteManager targetRouteManager;

    [Header("Koordinatlar")]
    public Vector3 startCoordinates = new Vector3(0, 0, 0);
    public Vector3 endCoordinates = new Vector3(0, 0, 3000);

    [Header("Ayarlar")]
    public float pointInterval = 50f; 
    public float flightAltitude = 3.0f; 
    
    [Header("Temizlik")]
    public bool clearOldPoints = true; 

    [ContextMenu("⚡ Koordinatlara Göre Oluştur")]
    public void GenerateRoute()
    {
        if (targetRouteManager == null)
        {
            Debug.LogError("❌ Target Route Manager boş! Lütfen sahnedeki Mission_Route objesini buraya sürükle.");
            return;
        }

        // 1. Eskileri Temizle (Seçiliyse)
        if (clearOldPoints)
        {
            targetRouteManager.waypoints.Clear();
            var tempArray = new GameObject[targetRouteManager.transform.childCount];
            for (int i = 0; i < tempArray.Length; i++) tempArray[i] = targetRouteManager.transform.GetChild(i).gameObject;
            foreach (var child in tempArray) DestroyImmediate(child);
        }

        // Mevcut rota üzerine ekleme yapıyorsak sayaç sıfırdan başlamasın
        // Eğer clearOldPoints=true ise startCount 0 olur.
        // Eğer clearOldPoints=false ise startCount listedeki eleman sayısı olur.
        int startCount = targetRouteManager.waypoints.Count;

        // 2. Yükseklikleri Sabitle
        Vector3 startPos = startCoordinates; startPos.y = flightAltitude;
        Vector3 endPos = endCoordinates; endPos.y = flightAltitude;

        // 3. Mesafe Hesapla
        float totalDistance = Vector3.Distance(startPos, endPos);
        Vector3 direction = (endPos - startPos).normalized;
        int numberOfPoints = Mathf.FloorToInt(totalDistance / pointInterval);

        Debug.Log($"📏 Mesafe: {totalDistance}m | Eklenecek Nokta: {numberOfPoints}");

        // 4. Noktaları Döşe
        for (int i = 0; i <= numberOfPoints; i++)
        {
            float currentDist = i * pointInterval;
            
            if (currentDist > totalDistance) currentDist = totalDistance;

            Vector3 newPos = startPos + (direction * currentDist);

            // İsimlendirme: WP_ + (Mevcut Sayı + i)
            // Böylece liste silinmemişse bile WP_101, WP_102 diye devam eder.
            GameObject wpObj = new GameObject($"WP_{startCount + i}");
            
            wpObj.transform.position = newPos;
            wpObj.transform.parent = targetRouteManager.transform;

            targetRouteManager.waypoints.Add(wpObj.transform);
        }

        // 5. Son Nokta Kontrolü (Tam denk gelmediyse bitiş koordinatını ekle)
        Vector3 lastPointPos = targetRouteManager.waypoints[targetRouteManager.waypoints.Count - 1].position;
        
        // Eğer son konulan nokta, hedeften uzaktaysa (tam üstüne basmadıysa)
        if (Vector3.Distance(lastPointPos, endPos) > 0.1f)
        {
            // BURASI DEĞİŞTİ: WP_Final yerine sıradaki numarayı veriyoruz.
            int nextIndex = targetRouteManager.waypoints.Count; // Listede 50 eleman varsa index 50 olur (0-49 doludur)
            GameObject endWP = new GameObject($"WP_{nextIndex}");
            
            endWP.transform.position = endPos;
            endWP.transform.parent = targetRouteManager.transform;
            targetRouteManager.waypoints.Add(endWP.transform);
        }

        Debug.Log("✅ Rota Oluşturuldu! Son nokta sayısal olarak eklendi.");
    }
}
