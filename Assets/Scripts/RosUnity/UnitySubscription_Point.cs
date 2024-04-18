using RosMessageTypes;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class UnitySubscription_Point : MonoBehaviour
{
    public GameObject gg;
    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<RosMessageTypes.Geometry.PointMsg>("ros_unity_topic_sub", PointChange);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void PointChange(RosMessageTypes.Geometry.PointMsg point) {
        gg.transform.localPosition = new Vector3((float)point.x, (float)point.y, (float)point.z);
        Debug.Log("ros:" + point);
    }
}
