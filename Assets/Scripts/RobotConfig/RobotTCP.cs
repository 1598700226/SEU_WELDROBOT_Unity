using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class RobotTCP : MonoBehaviour
{
    public static readonly string saveFileName = "Robot_TCP.json";

    void Start()
    {
        Debug.Log(Path.Combine(Application.persistentDataPath, saveFileName));
        DebugGUI.Log(Path.Combine(Application.persistentDataPath, saveFileName));
    }

    public static void SaveJsonData(RobotTCPData robotTCPData)
    {
        JsonSaveSystem.SaveByJson(saveFileName, robotTCPData);
    }

    public static RobotTCPData ReadJsonData()
    {
        RobotTCPData robotTCPData = JsonSaveSystem.LoadFromJson<RobotTCPData>(saveFileName);
        return robotTCPData;
    }
}
