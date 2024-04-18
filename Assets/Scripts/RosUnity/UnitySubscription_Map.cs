using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class UnitySubscription_Map : MonoBehaviour
{
    public string map_topic = "/map";   // 导航接收建图topic
    public int map_width = 0;
    public int map_height = 0;
    public int max_pixel = 0;
    public int[,] pixel;
    public bool hasNewDataReceive = false;

    public string navi_topic = "/need_goal";    // 导航接收目标点的topic
    public Vector2 oriPosition = Vector2.zero;
    public float map_resolution = 1;

    public void Start()
    {
        
    }

    public void SubscribeTopics()
    {
        if (!ROSConnection.GetOrCreateInstance().HasSubscriber(map_topic))
            ROSConnection.GetOrCreateInstance().
                Subscribe<RosMessageTypes.Nav.OccupancyGridMsg>(map_topic, MapCall);

        ROSConnection.GetOrCreateInstance().RegisterPublisher<RosMessageTypes.Nav.OccupancyGridMsg>(navi_topic);
    }

    public void UnSubscribeTopics()
    {
        ROSConnection.GetOrCreateInstance().Unsubscribe(map_topic);
        ROSConnection.GetOrCreateInstance().Unsubscribe(navi_topic);
    }

    void MapCall(RosMessageTypes.Nav.OccupancyGridMsg occupancyGridMsg)
    {
        hasNewDataReceive = false;
        int width = (int)occupancyGridMsg.info.width;
        int height = (int)occupancyGridMsg.info.height;
        int[,] temp_pixel = new int[height, width];

        int max_data = 0;
        for (int i = 0; i < height; i++)
            for (int j = 0; j < width; j++) {
                byte data = (byte)occupancyGridMsg.data[i * width + j];
                if (data > max_data) {
                    max_data = data;
                }
                temp_pixel[i, j] = data;
            }

        map_width = width;
        map_height = height;
        max_pixel = max_data;
        pixel = temp_pixel;
        map_resolution = occupancyGridMsg.info.resolution;
        oriPosition = new Vector2((float)occupancyGridMsg.info.origin.position.x,
            (float)occupancyGridMsg.info.origin.position.y);
        hasNewDataReceive = true;
        Debug.Log("【MapCall】收到地图");
    }

    public void SendTargetPosition(Vector2 target) 
    {
        RosMessageTypes.Nav.OccupancyGridMsg msg = new RosMessageTypes.Nav.OccupancyGridMsg();
        msg.info.origin.position.x = target.x * map_resolution + oriPosition.x;
        msg.info.origin.position.x = target.y * map_resolution + oriPosition.y;
        ROSConnection.GetOrCreateInstance().Publish(navi_topic, msg);
    }
}
