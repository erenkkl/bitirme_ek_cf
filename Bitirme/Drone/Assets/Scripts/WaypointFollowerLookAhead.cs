using System.Collections.Generic;
using UnityEngine;

public class WaypointFollowerLookAhead : MonoBehaviour
{
    [Header("References")]
    public RouteManager routeManager;
    public DronePhysics drone;

    [Header("Look-Ahead (Speed Control)")]
    [Tooltip("Düzlükte look-ahead (m). Büyük = daha hızlı.")]
    public float baseLookAhead = 60f;

    [Tooltip("Köşeye yaklaşınca düşeceği minimum look-ahead (m). Küçük = daha çok yavaşlama.")]
    public float minLookAhead = 10f;

    [Tooltip("Köşeye bu kalan yol mesafesinde (segment üzerinde) yavaşlamaya başla (m).")]
    public float cornerSlowRadius = 40f;

    [Tooltip("True ise köşe açısına göre yavaşlama mesafesi adaptif olur.")]
    public bool useAdaptiveCornerSlow = true;

    [Tooltip("Düz segment için slow radius (m).")]
    public float slowRadiusStraight = 25f;

    [Tooltip("Keskin köşe için slow radius (m).")]
    public float slowRadiusSharp = 60f;

    [Header("Corner Behavior")]
    [Tooltip("Köşeye yaklaşırken hedef noktanın waypoint B'nin ötesine geçmesini engeller (önerilir).")]
    public bool preventTargetBeyondCorner = true;

    [Range(0.5f, 1f)]
    [Tooltip("Köşeye yaklaşırken hedefi B'ye ne kadar yakın tutalım (0.9 iyi).")]
    public float cornerCapFactor = 0.9f;

    [Header("Segment Advance")]
    [Tooltip("Waypointe bu XZ mesafede yaklaşınca segment ilerletilir (m).")]
    public float reachDistance = 6f;

    [Tooltip("Son waypointten sonra başa döner.")]
    public bool loop = false;

    [Header("Dwell (Optional)")]
    public bool useDwell = false;
    public float dwellSeconds = 0.25f;

    [Header("Start")]
    public bool startFromNearest = true;

    [Header("Debug")]
    public bool debugDraw = true;
    public bool drawFullPath = false;

    [SerializeField] private int currentSegmentIndex = 0; // segment: i -> i+1
    private float dwellTimer = 0f;
    private bool finished = false;

    void Reset()
    {
        if (!routeManager) routeManager = FindFirstObjectByType<RouteManager>();
        if (!drone) drone = FindFirstObjectByType<DronePhysics>();
    }

    void Start()
    {
        if (!ValidateRefs())
        {
            enabled = false;
            return;
        }

        if (routeManager.waypoints == null || routeManager.waypoints.Count < 2)
        {
            Debug.LogError("[WaypointFollowerLookAhead] Rota en az 2 waypoint içermeli.");
            enabled = false;
            return;
        }

        if (startFromNearest)
            currentSegmentIndex = FindNearestSegmentIndex(drone.transform.position);
        else
            currentSegmentIndex = Mathf.Clamp(currentSegmentIndex, 0, routeManager.waypoints.Count - 2);

        finished = false;
        dwellTimer = 0f;
    }

    void FixedUpdate()
    {
        if (finished) return;
        if (!ValidateRefs()) return;

        if (useDwell && dwellTimer > 0f)
        {
            dwellTimer -= Time.fixedDeltaTime;
            // Dwell sırasında da hedefi tut
            SetTargetUsingCornerSlowdown();
            return;
        }

        SetTargetUsingCornerSlowdown();

        // Segment sonuna yaklaşınca ilerlet
        Transform b = routeManager.waypoints[currentSegmentIndex + 1];
        if (!b) { AdvanceSegment(); return; }

        float distToB = DistanceXZ(drone.transform.position, b.position);
        if (distToB <= reachDistance)
        {
            if (useDwell) dwellTimer = dwellSeconds;
            AdvanceSegment();
        }
    }

    // -----------------------------
    // MAIN: corner-aware target
    // -----------------------------
    private void SetTargetUsingCornerSlowdown()
    {
        Transform aT = routeManager.waypoints[currentSegmentIndex];
        Transform bT = routeManager.waypoints[currentSegmentIndex + 1];
        if (!aT || !bT) return;

        float y = drone.transform.position.y;

        Vector3 a = FlattenY(aT.position, y);
        Vector3 b = FlattenY(bT.position, y);

        // Drone'u segmente projekte et
        float tOnSeg;
        ProjectPointToSegmentXZ(drone.transform.position, a, b, out tOnSeg);

        float segLen = DistanceXZ(a, b);
        if (segLen < 0.001f)
        {
            // bozuk segment -> ilerle
            AdvanceSegment();
            return;
        }

        // Segment üzerinde B'ye kalan mesafe (path üzerinde)
        float remainingOnSeg = (1f - Mathf.Clamp01(tOnSeg)) * segLen;

        // Slow radius (adaptif köşe açısına göre)
        float slowRadius = cornerSlowRadius;
        if (useAdaptiveCornerSlow)
        {
            float cornerFactor = GetCornerSharpness01(currentSegmentIndex); // 0 düz, 1 keskin
            slowRadius = Mathf.Lerp(slowRadiusStraight, slowRadiusSharp, cornerFactor);
        }

        // 0..1: köşeye yaklaştıkça 0'a iner
        float slowT = 1f;
        if (slowRadius > 0.01f)
            slowT = Mathf.Clamp01(remainingOnSeg / slowRadius);

        // Efektif look-ahead: köşeye yaklaşınca azalır
        float effectiveLookAhead = Mathf.Lerp(minLookAhead, baseLookAhead, slowT);

        // Kritik fix: Köşeye yaklaştığında hedefin B'nin ötesine taşmasını engelle
        if (preventTargetBeyondCorner && remainingOnSeg < slowRadius)
        {
            // hedefi B'ye gelmeden önce tut (corner cutting olmasın)
            float cap = Mathf.Max(0.5f, remainingOnSeg * cornerCapFactor);
            effectiveLookAhead = Mathf.Min(effectiveLookAhead, cap);
        }

        // Projeksiyon noktasından başlayarak path üzerinde effectiveLookAhead ilerideki noktayı hedef yap
        Vector3 targetPoint = WalkAlongRouteFrom(currentSegmentIndex, tOnSeg, effectiveLookAhead, y);
        drone.SetGPSTarget(targetPoint);
    }

    // -----------------------------------------
    // Look-ahead point on polyline path
    // -----------------------------------------
    private Vector3 WalkAlongRouteFrom(int segIndex, float tOnSeg, float distance, float y)
    {
        int wpCount = routeManager.waypoints.Count;
        int curSeg = Mathf.Clamp(segIndex, 0, wpCount - 2);
        float curT = Mathf.Clamp01(tOnSeg);

        while (true)
        {
            Transform aT = routeManager.waypoints[curSeg];
            Transform bT = routeManager.waypoints[curSeg + 1];
            if (!aT || !bT) return new Vector3(transform.position.x, y, transform.position.z);

            Vector3 a = FlattenY(aT.position, y);
            Vector3 b = FlattenY(bT.position, y);

            float segLen = DistanceXZ(a, b);
            if (segLen < 0.001f)
            {
                if (!TryAdvanceSegmentIndex(ref curSeg, wpCount)) return b;
                curT = 0f;
                continue;
            }

            float curDistFromA = curT * segLen;
            float remainingOnSeg = segLen - curDistFromA;

            if (distance <= remainingOnSeg)
            {
                float newDistFromA = curDistFromA + distance;
                float newT = newDistFromA / segLen;
                return Vector3.Lerp(a, b, newT);
            }

            distance -= remainingOnSeg;

            if (!TryAdvanceSegmentIndex(ref curSeg, wpCount))
            {
                return b; // path bitti
            }

            curT = 0f;
        }
    }

    // -----------------------------------------
    // Corner sharpness (optional)
    // -----------------------------------------
    private float GetCornerSharpness01(int segIndex)
    {
        // Köşe B noktasında oluşur: A -> B -> C
        int wpCount = routeManager.waypoints.Count;

        int aIdx = segIndex;
        int bIdx = segIndex + 1;
        int cIdx = segIndex + 2;

        if (cIdx >= wpCount) return 0f;

        Transform aT = routeManager.waypoints[aIdx];
        Transform bT = routeManager.waypoints[bIdx];
        Transform cT = routeManager.waypoints[cIdx];
        if (!aT || !bT || !cT) return 0f;

        Vector3 a = aT.position; a.y = 0f;
        Vector3 b = bT.position; b.y = 0f;
        Vector3 c = cT.position; c.y = 0f;

        Vector3 dir1 = (b - a).normalized;
        Vector3 dir2 = (c - b).normalized;

        float angle = Vector3.Angle(dir1, dir2); // 0 düz, 90 keskin
        return Mathf.InverseLerp(10f, 90f, angle);
    }

    // -----------------------------------------
    // Segment helpers
    // -----------------------------------------
    private bool TryAdvanceSegmentIndex(ref int segIndex, int wpCount)
    {
        int lastSeg = wpCount - 2;

        if (segIndex < lastSeg)
        {
            segIndex++;
            return true;
        }

        if (loop)
        {
            segIndex = 0;
            return true;
        }

        return false;
    }

    private void AdvanceSegment()
    {
        int wpCount = routeManager.waypoints.Count;
        int lastSeg = wpCount - 2;

        if (currentSegmentIndex < lastSeg)
        {
            currentSegmentIndex++;
            return;
        }

        if (loop)
        {
            currentSegmentIndex = 0;
            return;
        }

        finished = true;
        Debug.Log("[WaypointFollowerLookAhead] Rota tamamlandı.");
    }

    private int FindNearestSegmentIndex(Vector3 pos)
    {
        int bestWp = 0;
        float bestDist = float.PositiveInfinity;

        List<Transform> wps = routeManager.waypoints;
        for (int i = 0; i < wps.Count; i++)
        {
            if (!wps[i]) continue;
            float d = DistanceXZ(pos, wps[i].position);
            if (d < bestDist)
            {
                bestDist = d;
                bestWp = i;
            }
        }

        return Mathf.Clamp(bestWp - 1, 0, routeManager.waypoints.Count - 2);
    }

    private bool ValidateRefs()
    {
        return routeManager && drone && routeManager.waypoints != null;
    }

    // -----------------------------------------
    // Geometry helpers (XZ)
    // -----------------------------------------
    private static float DistanceXZ(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x;
        float dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    private static Vector3 FlattenY(Vector3 p, float y) => new Vector3(p.x, y, p.z);

    private static Vector3 ProjectPointToSegmentXZ(Vector3 p, Vector3 a, Vector3 b, out float t)
    {
        Vector3 ap = new Vector3(p.x - a.x, 0f, p.z - a.z);
        Vector3 ab = new Vector3(b.x - a.x, 0f, b.z - a.z);

        float ab2 = Vector3.Dot(ab, ab);
        if (ab2 < 1e-6f)
        {
            t = 0f;
            return new Vector3(a.x, p.y, a.z);
        }

        t = Mathf.Clamp01(Vector3.Dot(ap, ab) / ab2);
        return new Vector3(a.x, p.y, a.z) + new Vector3(ab.x, 0f, ab.z) * t;
    }

    // -----------------------------------------
    // Debug
    // -----------------------------------------
    void OnDrawGizmos()
    {
        if (!debugDraw) return;
        if (!routeManager || routeManager.waypoints == null || routeManager.waypoints.Count < 2) return;

        // Aktif segment çizimi (cyan)
        if (currentSegmentIndex >= 0 && currentSegmentIndex < routeManager.waypoints.Count - 1)
        {
            Transform aT = routeManager.waypoints[currentSegmentIndex];
            Transform bT = routeManager.waypoints[currentSegmentIndex + 1];
            if (aT && bT)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(aT.position, bT.position);
                Gizmos.DrawWireSphere(bT.position, reachDistance);
            }
        }

        // Tüm path (opsiyonel)
        if (drawFullPath)
        {
            Gizmos.color = new Color(0f, 1f, 1f, 0.25f);
            for (int i = 0; i < routeManager.waypoints.Count - 1; i++)
            {
                var w0 = routeManager.waypoints[i];
                var w1 = routeManager.waypoints[i + 1];
                if (w0 && w1) Gizmos.DrawLine(w0.position, w1.position);
            }
        }

        // Sarı hedef noktası (look-ahead) + drone->hedef çizgisi
        if (!Application.isPlaying) return;
        if (!drone) return;
        if (currentSegmentIndex < 0 || currentSegmentIndex >= routeManager.waypoints.Count - 1) return;

        Transform aSeg = routeManager.waypoints[currentSegmentIndex];
        Transform bSeg = routeManager.waypoints[currentSegmentIndex + 1];
        if (!aSeg || !bSeg) return;

        float y = drone.transform.position.y;
        Vector3 a = new Vector3(aSeg.position.x, y, aSeg.position.z);
        Vector3 b = new Vector3(bSeg.position.x, y, bSeg.position.z);

        // Drone'u segmente projekte et
        float tOnSeg;
        ProjectPointToSegmentXZ(drone.transform.position, a, b, out tOnSeg);

        float segLen = DistanceXZ(a, b);
        if (segLen < 0.001f) return;

        float remainingOnSeg = (1f - Mathf.Clamp01(tOnSeg)) * segLen;

        // Effective look-ahead hesabı (SetTargetUsingCornerSlowdown ile aynı mantık)
        float slowRadius = cornerSlowRadius;
        if (useAdaptiveCornerSlow)
        {
            float cornerFactor = GetCornerSharpness01(currentSegmentIndex);
            slowRadius = Mathf.Lerp(slowRadiusStraight, slowRadiusSharp, cornerFactor);
        }

        float slowT = 1f;
        if (slowRadius > 0.01f)
            slowT = Mathf.Clamp01(remainingOnSeg / slowRadius);

        float effectiveLookAhead = Mathf.Lerp(minLookAhead, baseLookAhead, slowT);

        if (preventTargetBeyondCorner && remainingOnSeg < slowRadius)
        {
            float cap = Mathf.Max(0.5f, remainingOnSeg * cornerCapFactor);
            effectiveLookAhead = Mathf.Min(effectiveLookAhead, cap);
        }

        Vector3 targetPoint = WalkAlongRouteFrom(currentSegmentIndex, tOnSeg, effectiveLookAhead, y);

        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(targetPoint, 1.0f);
        Gizmos.DrawLine(drone.transform.position, targetPoint);
    }
}

