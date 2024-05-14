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
            Debug.Log("¡¾GloabExceptionCatch¡¿Global Exception Caught: " + logString);
            Debug.Log("¡¾GloabExceptionCatch¡¿Stack Trace: " + stackTrace);

            DebugGUI.LogString($"¡¾GloabExceptionCatch¡¿{logString}");
            DebugGUI.LogString($"¡¾GloabExceptionCatch¡¿{stackTrace}");
        }
    }
}
