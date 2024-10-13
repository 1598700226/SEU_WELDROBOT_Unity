using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class UnitySubscription_Map : MonoBehaviour
{
    public string map_topic = "/map";   // 导航接收建图topic
    public string map_serviceName = "/static_map";   // 导航接收建图服务名
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
        
        if (!ROSConnection.GetOrCreateInstance().HasSubscriber(navi_nowpostion_topic))
            ROSConnection.GetOrCreateInstance().
                Subscribe<RosMessageTypes.Geometry.PoseWithCovarianceStampedMsg>(navi_nowpostion_topic, NowPositionCall);

        // 地图服务订阅
        if (!ROSConnection.GetOrCreateInstance().HasSubscriber(map_serviceName))
        {
            ROSConnection.GetOrCreateInstance().
                RegisterRosService<RosMessageTypes.Nav.GetMapRequest, RosMessageTypes.Nav.GetMapResponse>(map_serviceName);
        }
    }

    public void MapServiceCall()
    {
        DebugGUI.Log("【MapServiceCall】请求地图数据");
        RosMessageTypes.Nav.GetMapRequest mapRequest = new RosMessageTypes.Nav.GetMapRequest();
        ROSConnection.GetOrCreateInstance().
            SendServiceMessage<RosMessageTypes.Nav.GetMapResponse>(map_serviceName, mapRequest, MapServiceCallBack);
    }

    void MapServiceCallBack(RosMessageTypes.Nav.GetMapResponse response)
    {
        Debug.Log($"【MapService】收到地图 begin");
        DebugGUI.Log($"【MapService】收到地图 begin");
        RosMessageTypes.Nav.OccupancyGridMsg occupancyGridMsg = response.map;
        int width = (int)occupancyGridMsg.info.width;
        int height = (int)occupancyGridMsg.info.height;
        int[,] temp_pixel = new int[height, width];

        int max_data = 0;
        for (int i = 0; i < height; i++)
            for (int j = 0; j < width; j++)
            {
                byte data = (byte)occupancyGridMsg.data[(height - 1 - i) * width + j];
                if (data > max_data)
                {
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
        Debug.Log($"【MapService】收到地图 map_w:{map_width} map_h:{map_height}");
        DebugGUI.Log($"【MapService】收到地图 map_w:{map_width} map_h:{map_height}");
    }

    public void SubscribeTopics()
    {
        if (!ROSConnection.GetOrCreateInstance().HasSubscriber(map_topic))
            ROSConnection.GetOrCreateInstance().
                Subscribe<RosMessageTypes.Nav.OccupancyGridMsg>(map_topic, MapTopicCall);
    }

    public void UnSubscribeTopics()
    {
        ROSConnection.GetOrCreateInstance().Unsubscribe(map_topic);
    }

    void MapTopicCall(RosMessageTypes.Nav.OccupancyGridMsg occupancyGridMsg)
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
        Debug.Log($"【MapCall】收到地图 map_w:{map_width} map_h:{map_height}");
        DebugGUI.Log($"【MapCall】收到地图 map_w:{map_width} map_h:{map_height}");
        Debug.Log($"【MapCall】关闭地图接收话题");
        DebugGUI.Log($"【MapCall】关闭地图接收话题");
    }

    void NowPositionCall(RosMessageTypes.Geometry.PoseWithCovarianceStampedMsg msg)
    {
        // nowPosition是小车发来图像中的位置 
        nowPosition = new Vector2((float)(msg.pose.pose.position.x / map_resolution + oriPosition.x),
            (float)(oriPosition.y - msg.pose.pose.position.y / map_resolution));
        hasNewNowPositionReceive = true;
        Debug.Log($"【NowPositionCall】收到当前位置pic:{nowPosition}, pose: {msg.pose.pose.position}");
        DebugGUI.Log($"【NowPositionCall】收到当前位置pic:{nowPosition}, pose: {msg.pose.pose.position}");
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
