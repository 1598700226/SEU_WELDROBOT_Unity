using Emgu.CV;
using Emgu.CV.Structure;
using MathNet.Numerics;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class CameraRotation : MonoBehaviour
{
    public Transform target; // 目标物体
    public float distance = 5.0f; // 相机与目标物体的距离
    public float rotation_x = 10.0f; 
    public float rotation_y = 10.0f;
    public float rotation_z = 10.0f;
    public bool isUpdate = false;

    private float sum_rotation_x = 0;
    private float sum_rotation_y = 0;
    private float sum_rotation_z = 0;

    // 数据获取和文件写入
    public bool getData = false;
    public string dir = string.Empty;

    void Update()
    {
        if (isUpdate)
        {
            isUpdate = false;
            // 计算相机绕目标物体的旋转
            transform.RotateAround(target.position, Vector3.right, rotation_x);
            transform.RotateAround(target.position, Vector3.up, rotation_y);
            transform.RotateAround(target.position, Vector3.forward, rotation_z);
            sum_rotation_x += rotation_x; 
            sum_rotation_y += rotation_y;
            sum_rotation_z += rotation_z;

            // 更新相机位置，保持固定距离
            transform.position = (transform.position - target.position).normalized * distance + target.position;

            // 使相机始终朝向目标物体
            transform.LookAt(target);
        }

        if (getData) {
            getData = false;
            StartCoroutine(get3DData());
        }
    }


    IEnumerator get3DData() {
        Debug.Log("width:" + Screen.width + "," + Screen.height);
        Ray ray;
        RaycastHit hit;
        string picName = sum_rotation_x + "_" + sum_rotation_y + "_" + sum_rotation_z;
        ScreenCapture.CaptureScreenshot(dir + "/" + picName + ".bmp");

        for (int y = 0; y < Screen.height; y++)
        {
            for (int x = 0; x < Screen.width; x++)
            {
                int pic_x = x;
                int pic_y = Screen.height - y - 1;

                ray = Camera.main.ScreenPointToRay(new Vector3(x, y, 0));
                if (Physics.Raycast(ray, out hit))
                {
                    Vector3 point = hit.point;
                    Vector3 cameraPoint = transform.InverseTransformPoint(point);
                    // 在这里处理每个转换后的点
                    Debug.Log($"Camera Point: {cameraPoint} x:{pic_x} y:{pic_y}");
                    writeFile(dir, picName + ".txt", $"{cameraPoint.x} {cameraPoint.y} {cameraPoint.z} {pic_x} {pic_y}");
                }
            }
            yield return null;
        }
        Debug.Log("get3DData END!!!");
        yield return null;
    }

    void writeFile(string DirPath, string fileName, string content) {
        string path = DirPath + "/" + fileName;
        // 写入文件
        File.AppendAllText(path, content + "\n");
    }
}
