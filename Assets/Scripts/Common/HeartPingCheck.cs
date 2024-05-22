using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

public class HeartPingCheck : MonoBehaviour
{
    public float interval = 1.0f;

    void Start()
    {
        DebugGUI.Log($"��HeartPingCheck�� ����������� �����{interval}");
        Debug.Log($"��HeartPingCheck�� ����������� �����{interval}");
        StartCoroutine(SendPing());
    }

    IEnumerator SendPing()
    {
        while (true)
        {
            // �ȴ�ָ����ʱ����
            yield return new WaitForSeconds(interval);

            // ����Ping��Ϣ
            StartCoroutine(SendPingMessage());
        }
    }

    IEnumerator SendPingMessage()
    {
        // todo

        yield return null;
    }
}
