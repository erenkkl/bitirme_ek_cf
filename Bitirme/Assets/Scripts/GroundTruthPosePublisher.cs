using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class GroundTruthPosePublisher : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("PoseStamped topic")]
    public string topicName = "/drone/ground_truth_pose";

    [Tooltip("ROS frame_id (RViz Fixed Frame ile aynı olmalı; genelde 'map')")]
    public string frameId = "map";

    [Tooltip("Yayın frekansı (Hz). 10 => saniyede 10 mesaj.")]
    public float publishHz = 10f;

    [Header("Source")]
    [Tooltip("Ground-truth alınacak transform. Boşsa bu GameObject'in transformu kullanılır.")]
    public Transform poseSource;

    [Header("Coordinate Mapping")]
    [Tooltip("Unity (x,y,z; y-up) -> ROS/RViz (x,y,z; z-up) dönüşümü uygular. (x,y,z)->(x,z,y)")]
    public bool swapUnityToRos = true;

    [Tooltip("2D path için z'yi 0 basar (RViz'de Path daha okunaklı olur).")]
    public bool use2D = true;

    [Header("Orientation")]
    [Tooltip("True ise yaw-only quaternion yayınlar (path için şart değil ama faydalı olabilir). False ise identity.")]
    public bool publishYawOnly = false;

    private ROSConnection ros;
    private float period;
    private Coroutine pubRoutine;

    void OnEnable()
    {
        if (poseSource == null) poseSource = transform;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);

        period = (publishHz <= 0.01f) ? 0.1f : (1.0f / publishHz);
        pubRoutine = StartCoroutine(PublishLoop());
    }

    void OnDisable()
    {
        if (pubRoutine != null)
        {
            StopCoroutine(pubRoutine);
            pubRoutine = null;
        }
    }

    private IEnumerator PublishLoop()
    {
        var wait = new WaitForSecondsRealtime(period);

        while (enabled)
        {
            PublishOnce();
            yield return wait;

            // publishHz değişirse period güncellenebilsin
            float newPeriod = (publishHz <= 0.01f) ? 0.1f : (1.0f / publishHz);
            if (!Mathf.Approximately(newPeriod, period))
            {
                period = newPeriod;
                wait = new WaitForSecondsRealtime(period);
            }
        }
    }

    private void PublishOnce()
    {
        if (ros == null || poseSource == null) return;

        // Unity world position
        Vector3 p = poseSource.position;

        // Mapping: Unity (x,y,z) -> ROS (x,y,z)
        // swapUnityToRos: (x,y,z) -> (x,z,y)
        double rx, ry, rz;

        if (swapUnityToRos)
        {
            rx = p.x;
            ry = p.z;
            rz = use2D ? 0.0 : p.y;
        }
        else
        {
            rx = p.x;
            ry = p.y;
            rz = use2D ? 0.0 : p.z;
        }

        // Orientation (opsiyonel)
        Quaternion q = Quaternion.identity;

        if (publishYawOnly)
        {
            // yaw-only: Unity'de yaw = y ekseni etrafı
            float yawDeg = poseSource.rotation.eulerAngles.y;
            q = Quaternion.Euler(0f, yawDeg, 0f);
        }

        // Quaternion mapping (basit kullanım):
        // Path için orientation kritik değil. Yine de yayınlıyoruz.
        // swapUnityToRos açıkken tam dönüşüm yapmadık; yaw-only zaten pratikte yeterli.
        var orientation = new QuaternionMsg(q.x, q.y, q.z, q.w);

        // Header stamp: ROS-TCP-Connector tarafında "now" yoksa 0 stamp da çalışır.
        // RViz Path için stamp zorunlu değil. Yine de dolu header veriyoruz.
        var header = new HeaderMsg
        {
            frame_id = frameId,
            stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg(0, 0)
        };

        var pose = new PoseMsg(
            new PointMsg(rx, ry, rz),
            orientation
        );

        var msg = new PoseStampedMsg(header, pose);
        ros.Publish(topicName, msg);
    }
}

