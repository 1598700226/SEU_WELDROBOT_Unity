using UnityEngine;

[System.Serializable]
public class EyeOnHandCalibrationData
{
    public string CalibrationMethod;
    public Matrix4x4 CameraToEndPoint;

    public override string ToString()
    {
        return $"CalibrationMethod={CalibrationMethod} \n CameraToEndPoint:{CameraToEndPoint}";
    }
}
