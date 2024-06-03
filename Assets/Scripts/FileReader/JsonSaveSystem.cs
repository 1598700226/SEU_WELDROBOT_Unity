using System;
using System.IO;
using UnityEngine;

public class JsonSaveSystem
{
    public static void SaveByJson(string saveFileName, object data) 
    {
        var json = JsonUtility.ToJson(data, true);
        var path = Path.Combine(Application.persistentDataPath, saveFileName);

        try
        {
            if (File.Exists(path))
            { 
                File.Delete(path);
            }
            File.WriteAllText(path, json);
            Debug.Log($"��SaveByJson�� Success To Save JsonData to {path}");
            DebugGUI.Log($"��SaveByJson�� Success To Save JsonData to {path}");
        }
        catch (Exception ex) 
        {
            Debug.Log($"��SaveByJson�� Fail To Save JsonData to {path}, \n {ex}");
            DebugGUI.Log($"��SaveByJson�� Fail To Save JsonData to {path}, \n {ex}");
        }
    }

    public static T LoadFromJson<T>(string jsonFileName)
    {
        var path = Path.Combine(Application.persistentDataPath, jsonFileName);

        try
        {
            var json = File.ReadAllText(path);
            var data = JsonUtility.FromJson<T>(json);
            return data;
        }
        catch (Exception ex)
        {
            Debug.Log($"��LoadFromJson�� Fail To Load JsonData at {path}, \n {ex}");
            DebugGUI.Log($"��LoadFromJson�� Fail To Load JsonData at {path}, \n {ex}");
            return default;
        }
    }

    public static void DeleteSaveFile(string jsonFileName)
    {
        var path = Path.Combine(Application.persistentDataPath, jsonFileName);

        try
        {
            if (File.Exists(path))
            {
                File.Delete(path);
            }
        }
        catch (Exception ex)
        {
            Debug.Log($"��DeleteSaveFile�� Fail To Delete JsonData at {path}, \n {ex}");
            DebugGUI.Log($"��DeleteSaveFile�� Fail To Delete JsonData at {path}, \n {ex}");
        }
    }
}