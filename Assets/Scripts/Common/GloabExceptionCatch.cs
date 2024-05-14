using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GloabExceptionCatch : MonoBehaviour
{
    void OnEnable()
    {
        Application.logMessageReceived += HandleException;
    }

    void OnDisable()
    {
        Application.logMessageReceived -= HandleException;
    }

    private void HandleException(string logString, string stackTrace, LogType type)
    {
        if (type == LogType.Exception)
        {
            Debug.Log("��GloabExceptionCatch��Global Exception Caught: " + logString);
            Debug.Log("��GloabExceptionCatch��Stack Trace: " + stackTrace);

            DebugGUI.LogString($"��GloabExceptionCatch��{logString}");
            DebugGUI.LogString($"��GloabExceptionCatch��{stackTrace}");
        }
    }
}
