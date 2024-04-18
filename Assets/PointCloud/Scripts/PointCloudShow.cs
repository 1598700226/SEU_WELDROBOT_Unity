using UnityEngine;
using System.Collections;
using System.IO;
using System.Threading;
using System;
using System.Collections.Generic;
using Unity.VisualScripting.Antlr3.Runtime.Tree;
using Unity.VisualScripting;
using UnityEngine.UIElements;
using System.Linq;
using Algorithm.Delauntor;
using Algorithm.Delauntor.Interfaces;
using DataStructures.ViliWonka.KDTree;

public class PointCloudShow : MonoBehaviour
{

    // File 获取文件夹下最新的点云文件，以.off_unity结尾
    public static string pointCloudDirPath;
    public string filename = "PointCloud_BackGround";
    public string fileSuffix = ".off_unity";
    public Material matVertex;
    // PointCloud
    private GameObject pointCloud;

    public static float scale = 0.001F;
    public static bool invertYZ = false;
    public static bool isPointTriangle = false;
    public static bool realRefreshPotinCloud = false;
    public static bool pointCloudShowOnce = false;
    public static int pointCloudDelayShow = 15;
    public int numPoints;
    public int numTriangles;

    private int limitPoints = 65001;

    private Vector3[] points;
    private Color[] colors;
    private Vector3[] triangles;
    private Vector3[] normals;

    void Start()
    {
        StartCoroutine(loadSceneForWaitSecond());
    }

    void Update()
    {
        // 单次更新
        if (!realRefreshPotinCloud && pointCloudShowOnce)
        {
            pointCloudShowOnce = false;
            loadSceneOnce();
        }
    }

    public string getLatestPointCloudFile(String dirPath)
    {
        if (!Directory.Exists(dirPath))
        {
            Debug.Log("【getLatestPointCloudFile】文件夹不存在 dirPath:" + dirPath);
            return null;
        }

        DirectoryInfo directoryInfo = new DirectoryInfo(dirPath);
        FileInfo[] files = directoryInfo.GetFiles("*" + fileSuffix);
        if (files == null || files.Length <= 0)
        {
            Debug.Log("【getLatestPointCloudFile】文件夹下不存在点云文件:" + dirPath);
            return null;
        }

        // 查询获取最新的文件
        FileInfo latestFile = files.OrderByDescending(f => f.LastWriteTime).FirstOrDefault();
        return latestFile.FullName;
    }

    IEnumerator loadSceneForWaitSecond()
    {
        while (true)
        {
            // 持续更新
            if (realRefreshPotinCloud)
            {

                pointCloudShowOnce = false;
                String pointCloudFile = getLatestPointCloudFile(pointCloudDirPath);
                Debug.Log("【PointCloudShow】更新环境 FilePath：" + pointCloudFile);
                loadPointCloud(pointCloudFile);
                yield return new WaitForSeconds(pointCloudDelayShow);
            }
            yield return null;
        }
    }

    void loadSceneOnce()
    {
        String pointCloudFile = getLatestPointCloudFile(pointCloudDirPath);
        Debug.Log("【PointCloudShow】loadSceneOnce更新环境 FilePath：" + pointCloudFile);
        loadPointCloud(pointCloudFile);
    }

    void loadPointCloud(String filePath)
    {
        // Check what file exists
        if (!String.IsNullOrEmpty(filePath) && File.Exists(filePath))
        {
            StartCoroutine(loadOFF(filePath));
        }
        else
        {
            Debug.Log("File '" + filePath + "' could not be found");
        }
    }

    // Start Coroutine of reading the points from the OFF file and creating the meshes
    IEnumerator loadOFF(string dPath)
    {

        IPoint[] picPoints;  // 点云中的点在图像上的位置，用于构造三角化
        // Read file
        using (StreamReader sr = new StreamReader(dPath))
        {
            sr.ReadLine(); // OFF
            string[] buffer = sr.ReadLine().Split(); // nPoints, nFaces

            numPoints = int.Parse(buffer[0]);
            numTriangles = int.Parse(buffer[1]);

            points = new Vector3[numPoints];
            picPoints = new IPoint[numPoints];
            colors = new Color[numPoints];
            triangles = new Vector3[numTriangles];

            for (int i = 0; i < numPoints; i++)
            {
                // 自定义off文件格式， x y z [r g b] [pic_x pic_y]
                buffer = sr.ReadLine().Split();
                if (!invertYZ)
                {
                    points[i] = new Vector3(float.Parse(buffer[0]) * scale, float.Parse(buffer[1]) * scale, float.Parse(buffer[2]) * scale);
                    picPoints[i] = new Algorithm.Delauntor.Models.Point(points[i][0], points[i][1], i);
                }
                else
                {
                    points[i] = new Vector3(float.Parse(buffer[0]) * scale, float.Parse(buffer[2]) * scale, float.Parse(buffer[1]) * scale);
                    picPoints[i] = new Algorithm.Delauntor.Models.Point(points[i][0], points[i][2], i);
                }

                // 判断颜色
                if (buffer.Length >= 6)
                    colors[i] = new Color(int.Parse(buffer[3]) / 255.0f, int.Parse(buffer[4]) / 255.0f, int.Parse(buffer[5]) / 255.0f);
                else
                    colors[i] = Color.cyan;

                // 判断点云点对应图像中的点
                if (buffer.Length == 5 || buffer.Length == 8) {
                    if (buffer.Length == 5)
                        picPoints[i] = new Algorithm.Delauntor.Models.Point(float.Parse(buffer[3]), float.Parse(buffer[4]), i);
                    else 
                        picPoints[i] = new Algorithm.Delauntor.Models.Point(float.Parse(buffer[6]), float.Parse(buffer[7]), i);
                }

                if (i % 10000 == 0)
                {
                    yield return null;
                }
            }

            for (int i = 0; i < numTriangles; i++)
            {
                buffer = sr.ReadLine().Split();
                triangles[i] = new Vector3(int.Parse(buffer[1]), int.Parse(buffer[2]), int.Parse(buffer[3]));

                if (i % 10000 == 0)
                {
                    yield return null;
                }
            }
        }
        //points = BilateralFiltering(10, 100, 30);

        // 更新点云
        if (pointCloud != null)
        {
            Transform parentTransform = pointCloud.transform;
            foreach (Transform childTransform in parentTransform)
            {
                Destroy(childTransform.gameObject);
            }
        }
        else
        {
            pointCloud = new GameObject(filename);
        }

        if (isPointTriangle && numTriangles != 0)
        {
            StartCoroutine(CreatTriangleMesh(filename, triangles, points, colors));
        }
        else
        {
            if (isPointTriangle) {
                var delaunator = new Delaunator(picPoints);
                List<Vector3> trianglelist = new List<Vector3>();
                delaunator.ForEachTriangle(Triangle =>
                {
                    Vector3 pt1 = points[Triangle.Points.ElementAt(0).Index];
                    Vector3 pt2 = points[Triangle.Points.ElementAt(1).Index];
                    Vector3 pt3 = points[Triangle.Points.ElementAt(2).Index];
                    float a = Vector3.Distance(pt1, pt2);
                    float b = Vector3.Distance(pt2, pt3);
                    float c = Vector3.Distance(pt1, pt3);
                    // 计算三角形的角度值，剔除过于尖锐的三角形
                    double cosA = (b * b + c * c - a * a) / (2.0 * b * c);
                    double cosB = (a * a + c * c - b * b) / (2.0 * a * c);
                    double cosC = (b * b + a * a - c * c) / (2.0 * b * a);
                    double angleAInRadians = Math.Acos(cosA) * (180.0 / Math.PI);
                    double angleBInRadians = Math.Acos(cosB) * (180.0 / Math.PI);
                    double angleCInRadians = Math.Acos(cosC) * (180.0 / Math.PI);
                    // 计算半周长
                    double s = (a + b + c) / 2.0;
                    // 使用海伦公式计算面积
                    double area = Math.Sqrt(s * (s - a) * (s - b) * (s - c));
                    // 剔除异常面片
                    if (angleAInRadians < 1 || angleBInRadians < 1 || angleCInRadians < 1 || area > 50)
                    {
                        ;
                    }
                    else
                    {
                        trianglelist.Add(new Vector3(Triangle.Points.ElementAt(0).Index,
                            Triangle.Points.ElementAt(1).Index,
                            Triangle.Points.ElementAt(2).Index));
                    }
                });
                StartCoroutine(CreatTriangleMesh(filename, trianglelist.ToArray(), points, colors));
            } else {
                StartCoroutine(CreatPointMesh(filename, points, colors));
            }
        }

        yield return null;
    }

    /// <summary>
    /// 协程：三角形网格化mesh
    /// </summary>
    /// <param name="fileName"></param>
    /// <param name="triangles"></param>
    /// <param name="verticePoints"></param>
    /// <param name="colors"></param>
    /// <returns></returns>
    IEnumerator CreatTriangleMesh(String meshFileName, Vector3[] triangles, Vector3[] verticePoints, Color[] colors)
    {

        GameObject pointGroup;
        Mesh mesh;

        Dictionary<int, int> mapIndexs = new Dictionary<int, int>();
        List<Vector3> vector3s = new List<Vector3>();
        List<Color> myColors = new List<Color>();
        List<int> indecies = new List<int>();
        int index_num = 0;
        List<int> triangle_index = new List<int>();

        int meshGroupId = 0;
        foreach (Vector3 i in triangles)
        {
            // 获取原始文件的索引，并重新映射到新的索引
            //x
            if (mapIndexs.ContainsKey((int)i.x))
            {

                int mapIndex = mapIndexs[(int)i.x];
                triangle_index.Add(mapIndex);
            }
            else
            {
                vector3s.Add(verticePoints[(int)i.x]);
                myColors.Add(colors[(int)i.x]);
                mapIndexs[(int)i.x] = index_num;
                indecies.Add(index_num);
                triangle_index.Add(index_num);
                index_num++;
            }
            //y
            if (mapIndexs.ContainsKey((int)i.y))
            {
                int mapIndex = mapIndexs[(int)i.y];
                triangle_index.Add(mapIndex);
            }
            else
            {
                vector3s.Add(verticePoints[(int)i.y]);
                myColors.Add(colors[(int)i.y]);
                mapIndexs[(int)i.y] = index_num;
                indecies.Add(index_num);
                triangle_index.Add(index_num);
                index_num++;
            }
            //z
            if (mapIndexs.ContainsKey((int)i.z))
            {
                int mapIndex = mapIndexs[(int)i.z];
                triangle_index.Add(mapIndex);
            }
            else
            {
                vector3s.Add(verticePoints[(int)i.z]);
                myColors.Add(colors[(int)i.z]);
                mapIndexs[(int)i.z] = index_num;
                indecies.Add(index_num);
                triangle_index.Add(index_num);
                index_num++;
            }

            // 判断是否超过限制
            if (index_num >= limitPoints)
            {
                // Create Mesh
                pointGroup = new GameObject(meshFileName + meshGroupId);
                pointGroup.AddComponent<MeshFilter>();
                pointGroup.AddComponent<MeshRenderer>();
                pointGroup.GetComponent<Renderer>().material = matVertex;
                mesh = new()
                {
                    vertices = vector3s.ToArray(),
                    colors = myColors.ToArray(),
                    triangles = triangle_index.ToArray()
                };
                mesh.SetIndices(triangle_index.ToArray(), MeshTopology.Triangles, 0);
                mesh.uv = new Vector2[indecies.Count];
                //mesh.normals = new Vector3[indecies.Count];
                mesh.RecalculateNormals();

                pointGroup.GetComponent<MeshFilter>().mesh = mesh;
                pointGroup.AddComponent<MeshCollider>().sharedMesh = mesh;
                pointGroup.transform.parent = pointCloud.transform;
                // 清除本次的点和triangle索引
                mapIndexs.Clear();
                vector3s = new List<Vector3>();
                myColors = new List<Color>();
                indecies = new List<int>();
                index_num = 0;
                triangle_index = new List<int>();
                meshGroupId++;
                yield return null;
            }
        }

        // 补齐最后mesh
        pointGroup = new GameObject(meshFileName + meshGroupId);
        pointGroup.AddComponent<MeshFilter>();
        pointGroup.AddComponent<MeshRenderer>();
        pointGroup.GetComponent<Renderer>().material = matVertex;
        mesh = new()
        {
            vertices = vector3s.ToArray(),
            colors = myColors.ToArray(),
            triangles = triangle_index.ToArray()
        };
        mesh.SetIndices(triangle_index.ToArray(), MeshTopology.Triangles, 0);
        mesh.uv = new Vector2[indecies.Count];
        //mesh.normals = new Vector3[indecies.Count];
        mesh.RecalculateNormals();
        pointGroup.GetComponent<MeshFilter>().mesh = mesh;
        pointGroup.AddComponent<MeshCollider>().sharedMesh = mesh;
        pointGroup.transform.parent = pointCloud.transform;
        yield return null;
    }

    IEnumerator CreatPointMesh(String meshFileName, Vector3[] verticePoints, Color[] colors)
    {
        // 计算点云点的mesh个数
        int groupNum = verticePoints.Length / limitPoints + 1;

        for (int meshId = 0; meshId < groupNum; meshId++)
        {
            int numPoints = limitPoints;
            // 最后一组需要求一下mesh的点数
            if (meshId == groupNum - 1) {
                numPoints = verticePoints.Length - (groupNum - 1) * limitPoints;
            }
            
            // Create Mesh
            GameObject pointGroup = new GameObject(meshFileName + meshId);
            pointGroup.AddComponent<MeshFilter>();
            pointGroup.AddComponent<MeshRenderer>();
            pointGroup.GetComponent<Renderer>().material = matVertex;
            Mesh mesh = new Mesh();
            List<Vector3> myPoints = new List<Vector3>();
            List<int> indecies = new List<int>();
            List<Color> myColors = new List<Color>();
            for (int i = 0; i < numPoints; ++i)
            {
                myPoints.Add(verticePoints[meshId * limitPoints + i]);
                indecies.Add(i);
                myColors.Add(colors[meshId * limitPoints + i]);
            }

            mesh.vertices = myPoints.ToArray();
            mesh.colors = myColors.ToArray();
            mesh.SetIndices(indecies, MeshTopology.Points, 0);
            mesh.uv = new Vector2[numPoints];
            mesh.normals = new Vector3[numPoints];
            pointGroup.GetComponent<MeshFilter>().mesh = mesh;
            pointGroup.transform.parent = pointCloud.transform;
            yield return null;
        }
        yield return null;
    }

    Vector3[] BilateralFiltering(int k_num, double d, double n) {
        long nowTime = ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds();

        KDTree kdTree = new KDTree(points);
        Vector3[] normals = new Vector3[points.Length];
        KDQuery query = new KDQuery();
        for (int i = 0; i < points.Length; i++)
        {
            Vector3 queryPoint = points[i];
            List<int> ret = new List<int>();
            query.KNearest(kdTree, queryPoint, k_num, ret);
            List<Vector3> nearPoints = ret.Select(index => points[index]).ToList();
            normals[i] = PointCloudHandle.CalNormalVector(nearPoints, queryPoint);
        }

        Vector3[] filterPoints = new Vector3[points.Length];
        for (int i = 0; i < points.Length; i++) {
            Vector3 queryPoint = points[i];
            List<int> ret = new List<int>();
            query.KNearest(kdTree, queryPoint, k_num, ret);
            List<Vector3> nearPoints = ret.Select(index => points[index]).ToList();
            // 双边滤波
            float sum_q = 0;
            float sum_w = 0;
            for (int j = 0; j < nearPoints.Count; j++)
            {
                Vector3 q = nearPoints[j];
                float dd = Vector3.Distance(q, queryPoint);
                float dn = Vector3.Dot((q - queryPoint).normalized, normals[i]);
                float w = (float)(Math.Exp(-(dd * dd) / (d * d * 2.0)) * Math.Exp(-(dn * dn) / (n * n * 2.0)));
                sum_q += w * dn;
                sum_w += w;
            }

            filterPoints[i] = queryPoint + (sum_q / sum_w) * normals[i];
            Debug.Log("滤波后：" + filterPoints[i] + "滤波前：" + queryPoint + "法线：" + normals[i] + "sum_q:" + sum_q + "sum_w:" + sum_w);
        }

        long endTime = ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds();
        Debug.Log("双边滤波计算完成 cost time:" + (endTime - nowTime) + "ms");

        return filterPoints;
    }

    public Vector3[] GetPointCloudVector3Array() {
        return points;
    }
}
