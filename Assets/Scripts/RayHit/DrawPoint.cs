using DataStructures.ViliWonka.KDTree;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// �������Ԥ���� �ڿռ��л���
public class DrawPoint : MonoBehaviour
{
    [Header("���λ���б�")]
    public List<Vector3> drawPointList;
    [Header("�Ƿ���и���")]
    public bool needUpdate;

    [Header("��ɫ����")]
    public Material Material_Red;
    [Header("��ɫ����")]
    public Material Material_Blue;
    [Header("��ɫ����")]
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

        Debug.Log($"��Test nors��{PointCloudHandle.CalNormalVector(testList, new Vector3(0, 0, 10))}");
    }

    // Update is called once per frame
    void Update()
    {
        if (needUpdate) {
            needUpdate = false;
            // ɾ���ɵ�����
            foreach (Transform childTransform in drawPointGameObject.transform)
            {
                Destroy(childTransform.gameObject);
            }

            for (int i = 0; i < drawPointList.Count; i++) {
                GameObject redSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere); // ����һ������
                redSphere.transform.position = drawPointList[i];
                redSphere.transform.parent = drawPointGameObject.transform;
                // ���������ɫ���ʣ�����ʾ�̵��˳����������ɫ
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

                // ������Ĵ�С
                redSphere.transform.localScale = new Vector3(sphereRadius * 3, sphereRadius * 3, sphereRadius * 3);
            }

            // ���Ƶ�ķ�����
            if(drawPointList.Count > 0)
            {
                // ��ȡ��ǰ������λ��
                Vector3 cameraPosition = Camera.main.transform.position;
                Vector3[] nors = getPointsNormals(drawPointList, 30, cameraPosition);
                Debug.Log($"nors {nors[0]} cameraPosition:{cameraPosition} drawPointList:{drawPointList[0]}");
                if (nors != null && nors.Length == drawPointList.Count)
                {
                    for (int i = 0; i < drawPointList.Count; i++)
                    {
                        GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                        cylinder.transform.parent = drawPointGameObject.transform;
                        float length = 0.05f; // Բ���ĳ���
                        float diameter = 0.005f; // Բ����ֱ��
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
                    Debug.Log("��DrawPoints�� ��ȡ��Ӧ��ķ�����ʧ��");
                }
            }
        }
    }

    /// <summary>
    /// ɾ��ָ��λ�õĵ�
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
    /// ɾ�����еĵ�
    /// </summary>
    public void deleteAllDrawPoint() {
        drawPointList.Clear();
        needUpdate = true;
    }

    private List<Vector3> interpolationPoints(List<Vector3> pointList, float interpolationDistance) {
        List<Vector3> interpolatedPoints = new List<Vector3>();
        for (int i = 0; i < pointList.Count - 1; i++)
        {
            // ����ǰ����ӵ���ֵ�б�
            interpolatedPoints.Add(pointList[i]);
            // ���������ľ���
            float distance = Vector3.Distance(pointList[i], pointList[i + 1]);
            // ������Ҫ��������м��
            int pointsToInsert = Mathf.FloorToInt(distance / interpolationDistance);

            // �����м��
            for (int j = 1; j <= pointsToInsert; j++)
            {
                float lerpFactor = j / (float)(pointsToInsert + 1);
                Vector3 interpolatedPoint = Vector3.Lerp(pointList[i], pointList[i + 1], lerpFactor);
                interpolatedPoints.Add(interpolatedPoint);
            }
        }
        // ������һ����
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
            Debug.LogError("��DrawPoints getPointNormals����ȡ�ĵ�������Ϊ0");
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
        Debug.Log("��DrawPoints�����߼������ cost time:" + (endTime - nowTime) + "ms");

        return normals;
    }
}
