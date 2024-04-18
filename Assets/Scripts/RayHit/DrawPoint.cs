using DataStructures.ViliWonka.KDTree;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// 利用球的预制体 在空间中画点
public class DrawPoint : MonoBehaviour
{
    [Header("点的位置列表")]
    public List<Vector3> drawPointList;
    [Header("是否进行更新")]
    public bool needUpdate;

    [Header("红色材质")]
    public Material Material_Red;
    [Header("蓝色材质")]
    public Material Material_Blue;
    [Header("绿色材质")]
    public Material Material_Green;

    // Point Draw
    private string gameObjec_name = "DrawTeachingPoint";
    private GameObject drawPointGameObject;
    public float sphereRadius = 0.01f;
    public float offset = 0.1f;

    public enum DrawMode
    {
        Line,
        Circle,
        Curve
    }
    public DrawMode drawMode;
    public float drawParam;

    // Start is called before the first frame update
    void Start()
    {
        drawPointList = new List<Vector3>();
        needUpdate = false;
        drawPointGameObject = new GameObject(gameObjec_name);

        List<Vector3> testList = new List<Vector3>();
        testList.Add(new Vector3(10, 10, 0));
        testList.Add(new Vector3(11, 10, 0));
        testList.Add(new Vector3(10, 11, 0));
        testList.Add(new Vector3(9, 10, 0));
        testList.Add(new Vector3(10, 9, 0));
        testList.Add(new Vector3(10, 9, 0));
        testList.Add(new Vector3(10, 9, 0));

        Debug.Log($"【Test nors】{PointCloudHandle.CalNormalVector(testList, new Vector3(0, 0, 10))}");
    }

    // Update is called once per frame
    void Update()
    {
        if (needUpdate) {
            needUpdate = false;
            // 删除旧的物体
            foreach (Transform childTransform in drawPointGameObject.transform)
            {
                Destroy(childTransform.gameObject);
            }

            for (int i = 0; i < drawPointList.Count; i++) {
                GameObject redSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere); // 创建一个球体
                redSphere.transform.position = drawPointList[i];
                redSphere.transform.parent = drawPointGameObject.transform;
                // 设置球的颜色材质，根据示教点的顺序来设置颜色
                Renderer sphereRenderer = redSphere.GetComponent<Renderer>();
                if (i == drawPointList.Count - 1)
                {
                    sphereRenderer.material = Material_Blue;
                }
                else if (i == 0) 
                {
                    sphereRenderer.material =  Material_Green;
                } 
                else
                { 
                    sphereRenderer.material = Material_Red;
                }

                // 设置球的大小
                redSphere.transform.localScale = new Vector3(sphereRadius * 3, sphereRadius * 3, sphereRadius * 3);
            }

            // 绘制点的法向量
            if(drawPointList.Count > 0)
            {
                // 获取当前活动相机的位置
                Vector3 cameraPosition = Camera.main.transform.position;
                Vector3[] nors = getPointsNormals(drawPointList, 30, cameraPosition);
                Debug.Log($"nors {nors[0]} cameraPosition:{cameraPosition} drawPointList:{drawPointList[0]}");
                if (nors != null && nors.Length == drawPointList.Count)
                {
                    for (int i = 0; i < drawPointList.Count; i++)
                    {
                        GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                        cylinder.transform.parent = drawPointGameObject.transform;
                        float length = 0.05f; // 圆柱的长度
                        float diameter = 0.005f; // 圆柱的直径
                        Vector3 start = drawPointList[i];
                        Vector3 end = start + nors[i] * length;

                        // Position cylinder
                        Vector3 offset = end - start;
                        Vector3 position = start + (offset / 2.0f);
                        cylinder.transform.position = position;

                        // Scale cylinder
                        float cylinderHeight = 1.0f; // Default height of a Unity cylinder is 2, scale factor of 0.5 will give us a height of 1
                        cylinder.transform.localScale = new Vector3(diameter, offset.magnitude / 2.0f / cylinderHeight, diameter);

                        // Rotate cylinder
                        cylinder.transform.rotation = Quaternion.FromToRotation(Vector3.up, offset.normalized);

                        // Optional: Change the color of the cylinder to make it stand out
                        Renderer renderer = cylinder.GetComponent<Renderer>();
                        renderer.material = new Material(Shader.Find("Standard"));
                        renderer.material.color = Color.red;
                    }
                }
                else 
                {
                    Debug.Log("【DrawPoints】 获取对应点的法向量失败");
                }
            }
        }
    }

    /// <summary>
    /// 删除指定位置的点
    /// </summary>
    public void deleteDrawPoint(int index) { 
        if (drawPointList.Count > 0)
        {
            index = index > drawPointList.Count - 1 ? drawPointList.Count - 1 : index;
            index = index < 0 ? 0 : index;
            drawPointList.RemoveAt(index);
            needUpdate = true;
        }
    }

    /// <summary>
    /// 删除所有的点
    /// </summary>
    public void deleteAllDrawPoint() {
        drawPointList.Clear();
        needUpdate = true;
    }

    private List<Vector3> interpolationPoints(List<Vector3> pointList, float interpolationDistance) {
        List<Vector3> interpolatedPoints = new List<Vector3>();
        for (int i = 0; i < pointList.Count - 1; i++)
        {
            // 将当前点添加到插值列表
            interpolatedPoints.Add(pointList[i]);
            // 计算两点间的距离
            float distance = Vector3.Distance(pointList[i], pointList[i + 1]);
            // 计算需要插入多少中间点
            int pointsToInsert = Mathf.FloorToInt(distance / interpolationDistance);

            // 插入中间点
            for (int j = 1; j <= pointsToInsert; j++)
            {
                float lerpFactor = j / (float)(pointsToInsert + 1);
                Vector3 interpolatedPoint = Vector3.Lerp(pointList[i], pointList[i + 1], lerpFactor);
                interpolatedPoints.Add(interpolatedPoint);
            }
        }
        // 添加最后一个点
        interpolatedPoints.Add(pointList[pointList.Count - 1]);
        return interpolatedPoints;
    }

    private List<Vector3> CalculateAverageAxis(List<Vector3> vectors)
    {
        List<Vector3> Points = new List<Vector3>();
        float avg = 0f;
        foreach (Vector3 vec in vectors)
        {
            avg += vec.y;
        }
        avg = avg / vectors.Count;
        for (int i = 0; i < vectors.Count; i++) {
            Points.Add(new Vector3(vectors[i].x, avg, vectors[i].z));
        }
        return Points;
    }

    private List<Vector3> SetPointOffset(List<Vector3> vectors) {
        List<Vector3> Points = new List<Vector3>();
        for (int i = 0; i < vectors.Count; i++)
        {
            Points.Add(new Vector3(vectors[i].x, vectors[i].y + offset, vectors[i].z));
        }
        return Points;
    }

    public List<Vector3> GetPoints(float interpolation_distance) {
        List<Vector3> points = new List<Vector3>();

        switch (drawMode) {
            case DrawMode.Line:
                points = CalculateAverageAxis(drawPointList);
                points = SetPointOffset(points);
                points = interpolationPoints(points, interpolation_distance);
                break;
            case DrawMode.Circle:
                points = CalculateAverageAxis(drawPointList);
                points = SetPointOffset(points);
                break;
            case DrawMode.Curve:
                points = CalculateAverageAxis(drawPointList);
                points = SetPointOffset(points);
                points = interpolationPoints(points, interpolation_distance);
                break;
        }

        return points;
    }

    public Vector3[] getPointsNormals(List<Vector3> points, int kNum, Vector3 viewPoint)
    {
        long nowTime = ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds();

        Vector3[] pointClouds = null;
        if (!DataCommon.isRosCameraSubscription) 
        {
            PointCloudShow pointCloudShow = GameObject.Find("PointCloudShow").GetComponent<PointCloudShow>();
            pointClouds = pointCloudShow.GetPointCloudVector3Array();
        }
        else
        {
            UnitySubscription_PointCloud unitySubscription_PointCloud = GameObject.Find("RosColorDepthData").GetComponent<UnitySubscription_PointCloud>();
            pointClouds = unitySubscription_PointCloud.GetPointCloudVector3Array();
        }
        if (pointClouds == null || pointClouds.Length == 0) 
        {
            Debug.LogError("【DrawPoints getPointNormals】获取的点云数量为0");
            return null;
        }

        KDTree kDTree = new KDTree(pointClouds);
        Vector3[] normals = new Vector3[points.Count];
        KDQuery query = new KDQuery();
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 queryPoint = points[i];
            List<int> ret = new List<int>();
            query.KNearest(kDTree, queryPoint, kNum, ret);
            List<Vector3> nearPoints = ret.Select(index => pointClouds[index]).ToList();
            normals[i] = PointCloudHandle.CalNormalVector(nearPoints, viewPoint);
        }

        long endTime = ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds();
        Debug.Log("【DrawPoints】法线计算完成 cost time:" + (endTime - nowTime) + "ms");

        return normals;
    }
}
