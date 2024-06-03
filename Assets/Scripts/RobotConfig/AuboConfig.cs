using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class AuboConfig : MonoBehaviour
{
    public static readonly string saveFileName = "AuboConfig.json";
    void Start()
    {
        Debug.Log(Path.Combine(Application.persistentDataPath, saveFileName));
        DebugGUI.Log(Path.Combine(Application.persistentDataPath, saveFileName));
    }

    public static void SaveJsonData(AuboConfigData auboConfigData)
    {
        JsonSaveSystem.SaveByJson(saveFileName, auboConfigData);
    }

    public static AuboConfigData ReadJsonData()
    {
        AuboConfigData auboConfigData = JsonSaveSystem.LoadFromJson<AuboConfigData>(saveFileName);
        return auboConfigData;
    }
}
