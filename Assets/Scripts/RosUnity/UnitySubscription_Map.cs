using System;
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


    public string cancelTopicName = "/move_base/cancel";    // 急停topic


    public string navi_nowpostion_topic = "/hhr_map_pose";
    public Vector2 nowPosition = Vector2.zero;
    public bool hasNewNowPositionReceive = false;

    public void Start()
    {
        ROSConnection.GetOrCreateInstance().RegisterPublisher<RosMessageTypes.Actionlib.GoalIDMsg>(cancelTopicName);
        ROSConnection.GetOrCreateInstance().RegisterPublisher<RosMessageTypes.Nav.OccupancyGridMsg>(navi_topic);
    }

    public void SubscribeTopics()
    {
        if (!ROSConnection.GetOrCreateInstance().HasSubscriber(map_topic))
            ROSConnection.GetOrCreateInstance().
                Subscribe<RosMessageTypes.Nav.OccupancyGridMsg>(map_topic, MapCall);

        if (!ROSConnection.GetOrCreateInstance().HasSubscriber(navi_nowpostion_topic))
            ROSConnection.GetOrCreateInstance().
                Subscribe<RosMessageTypes.Geometry.PoseWithCovarianceStampedMsg>(navi_nowpostion_topic, NowPositionCall);
    }

    public void UnSubscribeTopics()
    {
        ROSConnection.GetOrCreateInstance().Unsubscribe(map_topic);
        ROSConnection.GetOrCreateInstance().Unsubscribe(navi_topic);
        ROSConnection.GetOrCreateInstance().Unsubscribe(navi_nowpostion_topic);
    }

    void MapCall(RosMessageTypes.Nav.OccupancyGridMsg occupancyGridMsg)
    {
        int width = (int)occupancyGridMsg.info.width;
        int height = (int)occupancyGridMsg.info.height;
        int[,] temp_pixel = new int[height, width];

        int max_data = 0;
        for (int i = 0; i < height; i++)
            for (int j = 0; j < width; j++) {
                byte data = (byte)occupancyGridMsg.data[(height - 1 - i) * width + j];
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
        oriPosition = new Vector2(
            (float)Math.Abs(occupancyGridMsg.info.origin.position.x) / map_resolution,
            map_height - (float)Math.Abs(occupancyGridMsg.info.origin.position.y) / map_resolution);
        hasNewDataReceive = true;
        Debug.Log("【MapCall】收到地图");
    }

    void NowPositionCall(RosMessageTypes.Geometry.PoseWithCovarianceStampedMsg msg) 
    {
        // nowPosition是小车发来图像中的位置 
        nowPosition = new Vector2((float)(msg.pose.pose.position.x / map_resolution + oriPosition.x),
            (float)(oriPosition.y - msg.pose.pose.position.y / map_resolution));
        hasNewNowPositionReceive = true;
        Debug.Log($"【NowPositionCall】收到当前位置 {nowPosition} {msg.pose.pose.position}");
    }

    public void SendTargetPosition(Vector2 target) 
    {
        RosMessageTypes.Nav.OccupancyGridMsg msg = new RosMessageTypes.Nav.OccupancyGridMsg();
        msg.info.origin.position.x = (target.x - oriPosition.x) * map_resolution;
        msg.info.origin.position.y = (oriPosition.y - target.y) * map_resolution;
        Debug.Log($"【SendTargetPosition】{msg.info.origin.position}");
        ROSConnection.GetOrCreateInstance().Publish(navi_topic, msg);
    }


    public void NaviCancel()
    {
        RosMessageTypes.Actionlib.GoalIDMsg msg = new RosMessageTypes.Actionlib.GoalIDMsg();
        ROSConnection.GetOrCreateInstance().Publish(cancelTopicName, msg);
        Debug.Log("【NaviCancel】发送导航底盘急停");
    }
}
