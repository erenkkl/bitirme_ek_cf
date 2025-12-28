using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class AvoidanceArbiter : MonoBehaviour
{
    [Header("References")]
    public DronePhysics drone;
    public SiftPathFollower follower;

    [Header("ROS")]
    public string avoidCmdTopic = "/drone/avoid_cmd";

    [Header("Optional Lost Module Hook")]
    public GameObject lostModuleObject; // hazır modül: OnLostRequested() beklenir

    private ROSConnection ros;

    private bool avoidActive = false;
    private float cmdPitch = 0f, cmdRoll = 0f, cmdYaw = 0f;
    private bool lostRequested = false;

    void Start()
    {
        if (drone == null) drone = FindObjectOfType<DronePhysics>();
        if (follower == null) follower = FindObjectOfType<SiftPathFollower>();

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32MultiArrayMsg>(avoidCmdTopic, OnAvoidCmd);
    }

    private void OnAvoidCmd(Float32MultiArrayMsg msg)
    {
        if (msg.data == null || msg.data.Length < 5)
            return;

        avoidActive = msg.data[0] > 0.5f;
        cmdPitch    = msg.data[1];
        cmdRoll     = msg.data[2];
        cmdYaw      = msg.data[3];
        lostRequested = msg.data[4] > 0.5f;
    }

    void Update()
    {
        if (drone == null || follower == null)
            return;

        if (avoidActive)
        {
            follower.controlEnabled = false;
            drone.SetExternalCommand(cmdPitch, cmdRoll, cmdYaw);

            if (lostRequested && lostModuleObject != null)
            {
                // Kayıp modülü "hazır": tek bir mesajla tetiklediğimizi varsayıyoruz
                lostModuleObject.SendMessage("OnLostRequested", SendMessageOptions.DontRequireReceiver);
            }
        }
        else
        {
            follower.controlEnabled = true;
            // follower kendi Update’inde komut basmaya devam edecek
        }
    }
}

