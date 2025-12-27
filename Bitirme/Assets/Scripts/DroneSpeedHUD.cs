using UnityEngine;

public class DroneSpeedHUD : MonoBehaviour
{
    [Header("References")]
    public Rigidbody droneRigidbody;

    [Header("Display")]
    public bool showKmh = true;
    public int fontSize = 18;

    // Sol üstte küçük bir boşluk
    private readonly Rect _rect = new Rect(10, 10, 350, 60);

    void OnGUI()
    {
        if (!droneRigidbody)
        {
            GUI.Label(_rect, "Speed: (Rigidbody not assigned)");
            return;
        }

        float speedMs = droneRigidbody.velocity.magnitude; // m/s
        float speed = showKmh ? speedMs * 3.6f : speedMs;
        string unit = showKmh ? "km/h" : "m/s";

        var style = new GUIStyle(GUI.skin.label)
        {
            fontSize = fontSize
        };

        GUI.Label(_rect, $"Speed: {speed:0.0} {unit}", style);
    }
}

