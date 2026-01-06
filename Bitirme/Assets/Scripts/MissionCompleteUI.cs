using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Simple UI component for displaying mission completion status.
/// Attach this to a Canvas GameObject with a Text or TextMeshPro component.
/// </summary>
public class MissionCompleteUI : MonoBehaviour
{
    [Header("UI Elements")]
    [Tooltip("Assign the Text component (legacy) or leave null if using TextMeshPro")]
    public Text legacyText;
    
    [Tooltip("If using TextMeshPro, assign the TextMeshProUGUI component")]
    public TMPro.TextMeshProUGUI tmpText;

    [Header("Display Settings")]
    public string completionMessage = "✅ MISSION COMPLETE ✅";
    public Color completionColor = Color.green;
    
    [Header("Optional: Background Panel")]
    public Image backgroundPanel;
    public Color panelColor = new Color(0, 0, 0, 0.7f);

    [Header("Optional: Animation")]
    public bool enablePulseAnimation = true;
    public float pulseSpeed = 2.0f;
    public float pulseMinScale = 0.95f;
    public float pulseMaxScale = 1.05f;

    private float pulseTime = 0f;

    void Start()
    {
        // Try to auto-assign if not set
        if (legacyText == null)
            legacyText = GetComponentInChildren<Text>();
        
        if (tmpText == null)
            tmpText = GetComponentInChildren<TMPro.TextMeshProUGUI>();

        if (backgroundPanel == null)
            backgroundPanel = GetComponent<Image>();

        // Setup on enable
        UpdateDisplay();
    }

    void OnEnable()
    {
        UpdateDisplay();
    }

    void Update()
    {
        if (!enablePulseAnimation) return;

        // Pulse animation
        pulseTime += Time.deltaTime * pulseSpeed;
        float scale = Mathf.Lerp(pulseMinScale, pulseMaxScale, 
            (Mathf.Sin(pulseTime) + 1f) * 0.5f);
        transform.localScale = Vector3.one * scale;
    }

    private void UpdateDisplay()
    {
        // Set text
        if (legacyText != null)
        {
            legacyText.text = completionMessage;
            legacyText.color = completionColor;
        }

        if (tmpText != null)
        {
            tmpText.text = completionMessage;
            tmpText.color = completionColor;
        }

        // Set background
        if (backgroundPanel != null)
        {
            backgroundPanel.color = panelColor;
        }
    }

    /// <summary>
    /// Call this to show the indicator with a custom message
    /// </summary>
    public void ShowWithMessage(string message)
    {
        completionMessage = message;
        UpdateDisplay();
        gameObject.SetActive(true);
    }

    /// <summary>
    /// Call this to show the indicator with mission duration
    /// </summary>
    public void ShowWithDuration(float durationSeconds)
    {
        int minutes = Mathf.FloorToInt(durationSeconds / 60f);
        int seconds = Mathf.FloorToInt(durationSeconds % 60f);
        
        completionMessage = $"✅ MISSION COMPLETE ✅\nDuration: {minutes:00}:{seconds:00}";
        UpdateDisplay();
        gameObject.SetActive(true);
    }
}
