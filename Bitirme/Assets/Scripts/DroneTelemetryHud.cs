using UnityEngine;

public class DroneTelemetryHUD : MonoBehaviour
{
    public Rigidbody droneRigidbody;
    public Transform droneTransform;
    public bool showKmh = true;
    public int fontSize = 18;

    void OnGUI()
    {
        var rect = new Rect(10, 10, 450, 120);
        var style = new GUIStyle(GUI.skin.label) { fontSize = fontSize };

        if (!droneRigidbody || !droneTransform)
        {
            GUI.Label(rect, "Telemetry: Assign Rigidbody + Transform", style);
            return;
        }

        float speedMs = droneRigidbody.velocity.magnitude;
        float speed = showKmh ? speedMs * 3.6f : speedMs;
        string unit = showKmh ? "km/h" : "m/s";

        float altitude = droneTransform.position.y;
        float yaw = droneTransform.eulerAngles.y;

        GUI.Label(rect,
            $"Speed: {speed:0.0} {unit}\n" +
            $"Altitude: {altitude:0.0} m\n" +
            $"Yaw: {yaw:0.0}°",
            style);
    }
}

