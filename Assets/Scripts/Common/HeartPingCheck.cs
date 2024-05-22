using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

public class HeartPingCheck : MonoBehaviour
{
    public float interval = 1.0f;

    void Start()
    {
        DebugGUI.Log($"【HeartPingCheck】 开启心跳检测 间隔：{interval}");
        Debug.Log($"【HeartPingCheck】 开启心跳检测 间隔：{interval}");
        StartCoroutine(SendPing());
    }

    IEnumerator SendPing()
    {
        while (true)
        {
            // 等待指定的时间间隔
            yield return new WaitForSeconds(interval);

            // 发送Ping消息
            StartCoroutine(SendPingMessage());
        }
    }

    IEnumerator SendPingMessage()
    {
        // todo

        yield return null;
    }
}
