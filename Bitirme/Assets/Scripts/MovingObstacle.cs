using UnityEngine;

public class MovingObstacle : MonoBehaviour
{
    [SerializeField] private Vector3 start = new Vector3(90f, 30f, 170f);
    [SerializeField] private Vector3 end   = new Vector3(140f, 30f, 170f);
    [SerializeField] private float speed = 4f; // m/s

    private void Start()
    {
        // Obje başlangıç koordinatına yerleşsin
        transform.position = start;
    }

    private void Update()
    {
        float distance = Vector3.Distance(start, end);   // 50
        float t = Mathf.PingPong(Time.time * speed, distance); // 0..50..0
        transform.position = Vector3.Lerp(start, end, t / distance);
    }
}
