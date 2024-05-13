using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;


public class EyeOnHandCalibration: MonoBehaviour
{
    public static readonly string saveFileName = "EyeOnHand_Calibration.json";

    private void Start()
    {
        Debug.Log(Path.Combine(Application.persistentDataPath, saveFileName));
    }

    public static void SaveJsonData(EyeOnHandCalibrationData eyeOnHandCalibrationData)
    {
        JsonSaveSystem.SaveByJson(saveFileName, eyeOnHandCalibrationData);
    }

    public static EyeOnHandCalibrationData ReadJsonData()
    {
        EyeOnHandCalibrationData eyeOnHandCalibrationData = JsonSaveSystem.LoadFromJson<EyeOnHandCalibrationData>(saveFileName);
        return eyeOnHandCalibrationData;
    }
}
