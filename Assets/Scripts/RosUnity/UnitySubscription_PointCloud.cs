using Algorithm.Delauntor;
using Algorithm.Delauntor.Interfaces;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Emgu.CV;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;
using RosMessageTypes.Hhr;
using System.Text;
using System.IO;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System.Drawing;
using Color = UnityEngine.Color;
using TMPro;

public class UnitySubscription_PointCloud : MonoBehaviour
{
    public RawImage rawImage;
    public TMP_Text lableTitle;
    private Texture2D RawImageTexture;

    /*############## RealSense相机内参 ##############*/
    private int Image_Width = 640;
    private int Image_Height = 480;
    private float fx = 608.912109375f;
    private float fy = 607.4900512695312f;
    private float ppx = 316.9831848144531f;
    private float ppy = 245.9309844970703f;

    public bool IsBuliding = false; // 是否正在构建点云
    public int showType = 0; // 0是不显示，1是彩色图 2是深度图
    private Color[] Colors;
    private ushort[] Depths;

    /*############## 构造的点云 ##############*/
    Vector3[] points;                                       // 单次存储的数据
    Color[] pointsColor;                                    // 单次存储的数据
    IPoint[] picPoints;                                     // 单次存储的数据

    List<Vector3[]> points_mulview = new List<Vector3[]>(); // 多视角存储数据
    List<Color[]> colors_mulview = new List<Color[]>();     // 多视角存储数据
    List<ushort[]> depths_mulview = new List<ushort[]>();   // 多视角存储数据
    List<IPoint[]> picPoints_mulview = new List<IPoint[]>();   // 多视角存储数据

    Matrix4x4 InitRT = new Matrix4x4(
        new Vector4(1, 0, 0, 0),
        new Vector4(0, 1, 0, 0),
        new Vector4(0, 0, 1, 0),
        new Vector4(0, 0, 0, 1)
    );
    //Matrix4x4 matrix4x4_camera2endPoint = Matrix4x4.zero;
    Matrix4x4 matrix4x4_camera2endPoint = new Matrix4x4(
            new Vector4(1, 0, 0, 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );
    Matrix4x4 matrix4x4_Ros2Unity = new Matrix4x4(
        new Vector4(0, 1, 0, 0),
        new Vector4(-1, 0, 0, 0),
        new Vector4(0, 0, 1, 0),
        new Vector4(0, 0, 0, 1)
    );

    private GameObject pointCloud;
    public Material matVertex;
    private int limitPoints = 65001;
    public String filename = "ROS_Unity_PointCloud";
    public bool isPointTriangle = false;                        // 点云三角化
    public bool invertYZ = false;                               // 反转YZ轴
    public float scala = 1.0f;                                  // 点云缩放大小
    public float pointCloudUpdateTime = 5.0f;                   // 点云更新时间
    public bool realRefreshPotinCloud = false;                  // 是否实时更新
    public bool IsMultiViewPointCloudShow = false;              // 是否多角度视角保存
    public bool IsSiftMatching = false;                         // 是否进行SIFT配准
    public bool IsClearPointCloud = false;                      // 是否清除点云数据
    public bool IsCalibration = false;                          // 是否进行标定
    public bool IsTest = false;
    public Vector3 pointCloudPosition = new(0, -400, 900);      // 点云的位置信息，单位mm
    public bool IsSavePointCloud = false;                       // 保存点云数据
    public String PointCloudSavaDirPath = "";                   // 保存的位置
    public float PointCloudTriangleAngleLimit = 1f;             // 点云三角化角度限制
    public float PointCloudTriangleAreaLimit = 50f;             // 点云三角化面积限制

    /*############## 棋盘格与相机标定 ##############*/
    private int height_num = 5;
    private int width_num = 4;
    private System.Drawing.Size subCorner_win_size = new System.Drawing.Size(7, 7);  // 棋盘格在视野越小，这个值需要设置小
    private MCvTermCriteria mCvTermCriteria = new MCvTermCriteria(30, 0.1); // 终止条件 次数或重投影误差
    private float actual_dis = 33.0f;  // mm

    private List<Emgu.CV.Matrix<double>> target2Camera_R = new List<Matrix<double>>(); 
    private List<Emgu.CV.Matrix<double>> target2Camera_T = new List<Matrix<double>>();
    private List<Emgu.CV.Matrix<double>> endPoint2Base_R = new List<Matrix<double>>();
    private List<Emgu.CV.Matrix<double>> endPoint2Base_T = new List<Matrix<double>>();

    /*############## ROS话题与服务 ##############*/
    public string color_image_topic = "/camera1/color/image_raw/compressed";
    public string depth_image_topic = "/camera1/aligned_depth_to_color/image_raw";
    public string depth_image_service_topic = "get_aligned_depth_image";

    private void Start()
    {
        RawImageTexture = new Texture2D(Image_Width, Image_Height);
        StartCoroutine(loadRosPointCloudForWaitSecond());
        ReadEyeOnHandCalibrationData();
    }

    IEnumerator loadRosPointCloudForWaitSecond()
    {
        // 若开启ros深度话题，则可以设为true，目前是通过服务获取深度数据
        while (false)
        {
            // 持续更新
            if (realRefreshPotinCloud)
            {
                Vector3[] points;
                Color[] pointsColor;
                IPoint[] picPoints;
                GetPointCloudData(Colors, Depths, Image_Width, Image_Height, out points, out pointsColor, out picPoints);
                points = PointConvert(points, InitRT, scala, invertYZ);
                yield return StartCoroutine(BulidPointCloudObject(points, pointsColor, picPoints));
                yield return new WaitForSeconds(pointCloudUpdateTime);
            }
            yield return null;
        }
    }

    // 订阅深度对齐topic和彩色topic
    public void SubscribeTopics(bool color_topic = true, bool depth_topic = false)
    {
        if(color_topic)
        {
            if (!ROSConnection.GetOrCreateInstance().HasSubscriber(color_image_topic))
                ROSConnection.GetOrCreateInstance().
                    Subscribe<RosMessageTypes.Sensor.CompressedImageMsg>(color_image_topic, CompressedColorImageCall);
            /*RosMessageTypes.Nav.OccupancyGridMsg*/
        }

        if (depth_topic)
        {
            if (!ROSConnection.GetOrCreateInstance().HasSubscriber(depth_image_topic))
                ROSConnection.GetOrCreateInstance().
                    Subscribe<RosMessageTypes.Sensor.ImageMsg>(depth_image_topic, CompressedDepthImageCall);
        }
    }

    public void UnSubscribeTopics()
    {
        ROSConnection.GetOrCreateInstance().Unsubscribe(color_image_topic);
        ROSConnection.GetOrCreateInstance().Unsubscribe(depth_image_topic);
        ROSConnection.GetOrCreateInstance().Unsubscribe(depth_image_service_topic);
    }

    public void DepthImageRegisterServiceCall() {
        if (!ROSConnection.GetOrCreateInstance().HasSubscriber(depth_image_service_topic)) {
            ROSConnection.GetOrCreateInstance().
                RegisterRosService<RsAlignedDepthImageRequest, RsAlignedDepthImageResponse>(depth_image_service_topic);
        }
    }

    public void DepthServiceCall()
    {
        DebugGUI.Log("【DepthServiceCall】请求深度数据");
        RsAlignedDepthImageRequest rsAlignedDepthImageResponse = new RsAlignedDepthImageRequest("unity client call");
        ROSConnection.GetOrCreateInstance().
            SendServiceMessage<RsAlignedDepthImageResponse>(depth_image_service_topic, rsAlignedDepthImageResponse, ServiceCallback_DepthImage);
    }

    void ServiceCallback_DepthImage(RsAlignedDepthImageResponse response) {
        Depths = ConvertBytesToUShort(response.image.data);
        if (showType == 2)
            UpdateRawImageByUshortData(Depths);
        IsBuliding = true;
        DebugGUI.Log("【ServiceCallback_DepthImage】接收到深度数据");
    }

    // Update is called once per frame
    void Update()
    {
        // 单视角生成点云
        if (IsBuliding && !realRefreshPotinCloud && !IsMultiViewPointCloudShow && !IsCalibration) {
            GetPointCloudData(Colors, Depths, Image_Width, Image_Height, out points, out pointsColor, out picPoints);
            // points = PointConvert(points, InitRT, invertYZ); 
            Matrix4x4 matrix4x4_camera2unity = matrix4x4_Ros2Unity * GetMatrixEndPoint2RosBaseFromTF() * matrix4x4_camera2endPoint;
            points = PointConvert(points, matrix4x4_camera2unity, scala, invertYZ);
            IsBuliding = false;

            StartCoroutine(BulidPointCloudObject(points, pointsColor, picPoints));
        }

        // 多视角生成点云
        if (IsBuliding && !realRefreshPotinCloud && IsMultiViewPointCloudShow && !IsCalibration)
        {
            GetPointCloudData(Colors, Depths, Image_Width, Image_Height, out points, out pointsColor, out picPoints);
            //Depths = BilateralFilterDataWithColor(Depths, Colors, Image_Width, Image_Height, 3, 10, 10, 10);
            // 获取标定点云
            System.Drawing.PointF[,] corners = GetCorners(Colors, Image_Width, Image_Height);
            if (corners == null)
            {
                Debug.Log("未检测到棋盘格标定板角点！");
                DebugGUI.Log("未检测到棋盘格标定板角点！");
                if (matrix4x4_camera2endPoint == Matrix4x4.zero)
                    return;
                Matrix4x4 matrix4x4_camera2unity = matrix4x4_Ros2Unity * GetMatrixEndPoint2RosBaseFromTF() * matrix4x4_camera2endPoint;
                points = PointConvert(points, matrix4x4_camera2unity, scala, invertYZ);
                // SIFT 配准
                if (IsSiftMatching && colors_mulview.Count > 1) 
                {
                    Matrix4x4 matrix4X4_sift = SiftMatchPointCloud(Colors, colors_mulview[0], Depths, depths_mulview[0]);
                    points = PointConvert(points, matrix4X4_sift, 1, false);
                }

                IsBuliding = false;
                DebugGUI.Log("开始三维重建！");
                StartCoroutine(BulidMultiPointCloudObject(points, pointsColor, picPoints));
            }
            else
            {
                Matrix4x4 matrix4x4;
                ChessBoardCalculateRT(corners, Depths, out matrix4x4);
                points = PointConvert(points, matrix4x4, scala, invertYZ);
                IsBuliding = false;
                StartCoroutine(BulidMultiPointCloudObject(points, pointsColor, picPoints));
            }

            // 保存多视角数据
            points_mulview.Add(points);
            colors_mulview.Add(Colors);
            depths_mulview.Add(Depths);
            picPoints_mulview.Add(picPoints);
        }

        // 图片控件显示
        if (showType <= 0)
        {
            rawImage.enabled = false;
            lableTitle.enabled = false;
        }
        else
        {
            rawImage.enabled = true;
            lableTitle.enabled = true;
        }

        // 清除环境中的点云游戏对象
        if (IsClearPointCloud) {
            IsClearPointCloud = false;
            if (pointCloud == null) {
                return;
            }
            Transform parentTransform = pointCloud.transform;
            foreach (Transform childTransform in parentTransform)
            {
                Destroy(childTransform.gameObject);
            }
            points_mulview.Clear();
            colors_mulview.Clear();
            depths_mulview.Clear();

            target2Camera_R.Clear();
            target2Camera_T.Clear();
            endPoint2Base_R.Clear();
            endPoint2Base_T.Clear();
        }

        /*        // 获取相机到机械臂基座的变换矩阵——测量法
                if (IsBuliding && IsCalibration)
                {
                    GetPointCloudData(Colors, Depths, Image_Width, Image_Height, out points, out pointsColor, out picPoints);
                    //Depths = BilateralFilterDataWithColor(Depths, Colors, Image_Width, Image_Height, 3, 10, 10, 10);
                    // 获取标定点云
                    System.Drawing.PointF[,] corners = GetCorners(Colors, Image_Width, Image_Height);
                    if (corners == null)
                    {
                        Debug.LogString("未检测到棋盘格标定板角点！");
                        IsCalibration = false;
                        IsBuliding = false;
                        return;
                    }
                    Matrix4x4 matrix4x4_camera2unity;
                    bool isSuccess = ChessBoardCalculateRT(corners, Depths, out matrix4x4_camera2unity);
                    if (!isSuccess)
                    {
                        return;
                    }
                    Matrix4x4 matrix4x4_endpoint2rosbase = GetMatrixEndPoint2RosBaseFromTF();
                    matrix4x4_camera2endPoint = matrix4x4_endpoint2rosbase.inverse * matrix4x4_Ros2Unity.inverse * matrix4x4_camera2unity;
                    Debug.LogString($"【相机标定CE】camera2endPoint: \n{matrix4x4_camera2endPoint}");
                    Debug.LogString($"【相机标定TF】matrix4x4_endpoint2rosbase: \n{matrix4x4_endpoint2rosbase}");
                    Debug.LogString($"【相机标定TF】matrix4x4_camera2endPoint: \n " +
                        $"平移:\n {ExtractPositionFromMatrix(matrix4x4_camera2endPoint)}" +
                        $"旋转:\n {ExtractRotationFromMatrix(matrix4x4_camera2endPoint)}");
                    IsCalibration = false;
                    IsBuliding = false;
                }*/

        // 获取相机到机械臂基座的变换矩阵——多次测量法
        if (IsBuliding && IsCalibration)
        {
            GetPointCloudData(Colors, Depths, Image_Width, Image_Height, out points, out pointsColor, out picPoints);
            Matrix4x4 end2BaseTF = GetMatrixEndPoint2RosBaseFromTF();
            Debug.Log(end2BaseTF);
            // 获取标定点云
            System.Drawing.PointF[,] corners = GetCorners(Colors, Image_Width, Image_Height);
            if (corners == null)
            {
                Debug.Log("未检测到棋盘格标定板角点！");
                IsCalibration = false;
                IsBuliding = false;
                return;
            }
            Emgu.CV.Matrix<double> r;
            Emgu.CV.Matrix<double> t;
            Matrix4x4 matrix4X4;
            /*            // SVD方法求解target to Camera变换关系
                        ChessBoard2CameraRT(corners, Depths, out matrix4X4);
                        Matrix4x4ToEmgucvMatrixRT(matrix4X4, out r, out t);*/
            // 角点检测SolvePnp计算外参
            CameraExternalRT(corners, out r, out t);
            target2Camera_R.Add(r);
            target2Camera_T.Add(t);
            Emgu.CV.Matrix<double> tf_r;
            Emgu.CV.Matrix<double> tf_t;
            Matrix4x4ToEmgucvMatrixRT(end2BaseTF, out tf_r, out tf_t);
            endPoint2Base_R.Add(tf_r);
            endPoint2Base_T.Add(tf_t);

            IsBuliding = false;
            Debug.Log($"【相机标定】第 {target2Camera_R.Count} 次位置");
            DebugGUI.LogString($"【相机标定】第 {target2Camera_R.Count} 次位置");
        }

        if (IsSavePointCloud) {
            IsSavePointCloud = false;
            //StartCoroutine(SavePointCloud2OffFile(PointCloudSavaDirPath));
            StartCoroutine(SavePointCloud2OffFile2PersistentDataPath());
        }
        
        if (IsTest)
        {
            Matrix4x4 end2BaseTF = GetMatrixEndPoint2RosBaseFromTF();
            Debug.Log(end2BaseTF);
            Emgu.CV.Matrix<double> tf_r;
            Emgu.CV.Matrix<double> tf_t;
            Matrix4x4ToEmgucvMatrixRT(end2BaseTF, out tf_r, out tf_t);
        }
    }

    /// <summary>
    /// ROS彩色数据话题的订阅
    /// </summary>
    /// <param name="CompressedColorImage"></param>
    void CompressedColorImageCall(RosMessageTypes.Sensor.CompressedImageMsg CompressedColorImage)
    {
        if(!IsBuliding)
        {
            Debug.Log("【CompressedColorImageCall】获得压缩图片");
            Colors = JPEGToRGB(CompressedColorImage.data);

            if (showType == 1)
                UpdateCompressedColorImage(CompressedColorImage.data);
        }
    }

    private void UpdateCompressedColorImage(byte[] data)
    {
        if (rawImage == null)
        {
            Debug.Log("【UnitySubscription_PointCloud error】rawImage控件为空");
            return;
        }

        RawImageTexture.LoadImage(data);
        // 将Texture2D显示在RawImage上
        RawImageTexture.Apply();  // 应用像素更改
        rawImage.texture = RawImageTexture;
    }

    private Color[] JPEGToRGB(byte[] jpegData)
    {
        Texture2D texture = new Texture2D(2, 2);
        texture.LoadImage(jpegData);
        Color[] rgbArray = texture.GetPixels();
        Color[] ret = new Color[rgbArray.Length];
        for (int w = 0; w < Image_Width; w++)
            for (int h = 0; h < Image_Height; h++) {
                // 需要换乘左上角开始，GetPixels()是左下角的数据
                ret[w + h * Image_Width] = rgbArray[w + (Image_Height - 1 - h) * Image_Width];
            }

        Destroy(texture);
        return ret;
    }

    /// <summary>
    /// ROS深度数据话题的订阅
    /// </summary>
    /// <param name="DepthImage"></param>
    void CompressedDepthImageCall(RosMessageTypes.Sensor.ImageMsg DepthImage)
    {
        if (!IsBuliding)
        {
            Depths = ConvertBytesToUShort(DepthImage.data);
            if(showType == 2)
                UpdateRawImageByUshortData(Depths);
        }
    }

    ushort[] ConvertBytesToUShort(byte[] bytesData)
    {
        if (bytesData.Length % 2 != 0)
        {
            Debug.LogError("Invalid byte array length for 16UC1 data.");
            return null;
        }
        ushort[] ushortData = new ushort[bytesData.Length / 2];

        ushort max = ushort.MinValue;
        ushort min = ushort.MaxValue;
        for (int i = 0; i < ushortData.Length; i++)
        {
            // 根据端序将 byte[] 转为 ushort
            if (System.BitConverter.IsLittleEndian)
            {
                ushortData[i] = (ushort)(bytesData[2 * i] | (bytesData[2 * i + 1] << 8));
            }
            else
            {
                ushortData[i] = (ushort)((bytesData[2 * i] << 8) | bytesData[2 * i + 1]);
            }

            if(ushortData[i] > max) 
                max = ushortData[i];
            if (ushortData[i] < min)
                min = ushortData[i];
        }
        return ushortData;
    }

    private void UpdateRawImageByUshortData(ushort[] data)
    {
        if (rawImage == null)
        {
            Debug.Log("【UnitySubscription_PointCloud error】rawImage控件为空");
            return;
        }
        RawImageTexture = new Texture2D(Image_Width, Image_Height, TextureFormat.RGBA32, false);
        ushort max = 0;
        for (int i = 0; i < data.Length; i++)
        { 
            if(data[i] > max)
                max = data[i];
        }
        // 将 ushort[] 数据转换为颜色信息
        Color[] colors = new Color[data.Length];
        for (int i = 0; i < data.Length; i++)
        {
            float normalizedValue = (float)data[i] / max;
            colors[i] = new Color(normalizedValue, normalizedValue, normalizedValue, 255);
        }
        // 将颜色信息应用到 Texture2D 上, 需要颠倒显示
        for (int y = 0; y < Image_Height; y++)
        {
            for (int x = 0; x < Image_Width; x++)
            {
                RawImageTexture.SetPixel(x, Image_Height - 1 - y, colors[x + y * Image_Width]);  // 需要翻转y轴以匹配Unity坐标系
            }
        }
        RawImageTexture.Apply();  // 应用像素更改
        rawImage.texture = RawImageTexture;
    }

    /// <summary>
    /// 双边滤波
    /// </summary>
    public static ushort[] BilateralFilterDataWithColor(ushort[] depthData, Color[] colorData, int img_width, int img_height, int size, float spatialSigma, float depthSigma, float colorSigma)
    {
        int length = depthData.Length;
        ushort[] result = new ushort[length];

        for (int h = 0; h < img_height; h++)
        {
            for (int w = 0; w < img_width; w++)
            {
                double weightedSum = 0f;
                double totalWeight = 0f;
                int index = w + h * img_width;

                for (int ih = -size; ih <= size; ih++)  
                {
                    for (int iw = -size; iw <= size; iw++)
                    {
                        if (h + ih >= img_height || h + ih < 0 || w + iw >= img_width || w + iw < 0)
                            continue;
                        int jndex = w + iw + (h + ih) * img_width;
                        if (depthData[jndex] <= 0)
                            continue;

                        double spatialDistance = Math.Sqrt(iw * iw + ih * ih);
                        double depthDistance = 0;
                        if (depthData[index] > 0)
                        {
                            depthDistance = Math.Abs(depthData[index] - depthData[jndex]);
                        }
                        double colorDistance = (Math.Abs(colorData[index].r - colorData[jndex].r) +
                            Math.Abs(colorData[index].g - colorData[jndex].g) +
                            Math.Abs(colorData[index].b - colorData[jndex].b)) / 3;

                        // 更新双边滤波权重计算，包括颜色信息
                        double spatialWeight = Math.Exp(-spatialDistance / (2 * spatialSigma * spatialSigma));
                        double depthWeight = Math.Exp(-depthDistance / (2 * depthSigma * depthSigma));
                        double colorWeight = Math.Exp(-colorDistance / (2 * colorSigma * colorSigma));
                        double weight = spatialWeight * depthWeight * colorWeight;

                        weightedSum += depthData[jndex] * weight;
                        totalWeight += weight;
                    }
                }
                if (totalWeight > 0)
                    result[index] = (ushort)(weightedSum / totalWeight);
                else
                    result[index] = depthData[index];
            }
        }
        return result;
    }

    /// <summary>
    /// 相机数据获得构造点云需要的数据
    /// </summary>
    /// <param name="colors"></param>
    /// <param name="depth"></param>
    /// <param name="img_w"></param>
    /// <param name="img_h"></param>
    /// <param name="points"></param>
    /// <param name="pointsColor"></param>
    /// <param name="picPoints"></param>
    private void GetPointCloudData(Color[] colors, ushort[] depth, int img_w, int img_h, out Vector3[] points, out Color[] pointsColor, out IPoint[] picPoints) {
        //ushort[] Depth = BilateralFilterData(depth, Image_Width, Image_Height, 2, 10, 10);  // 对深度数据进行双边滤波

        List<Vector3> pointList = new List<Vector3>();
        List<Color> colorList = new List<Color>();
        List<IPoint> picPointList = new List<IPoint>();
        int index = 0;
        for (int h = 0; h < img_h; h++)
        {
            for (int w = 0; w < img_w; w++)
            {
                float real_z = depth[w + h * img_w];
                if (real_z <= 0)
                    continue;

                float real_x = (w - ppx) / fx * real_z;
                float real_y = (h - ppy) / fy * real_z;
                pointList.Add(new Vector3(real_x, real_y, real_z));
                colorList.Add(colors[w + h * img_w]);
                picPointList.Add(new Algorithm.Delauntor.Models.Point(w, h, index));
                index++;
            }
        }

        points = pointList.ToArray();
        pointsColor = colorList.ToArray(); 
        picPoints = picPointList.ToArray();
    }

    private Vector3[] PointConvert(Vector3[] points, Matrix4x4 matrix, float scala, bool invertYZ = false) {
        Vector3[] result = new Vector3[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            result[i] = matrix.MultiplyPoint3x4(points[i]) * scala;
        }

        if (invertYZ) {
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 itemPoint = new Vector3(result[i].x, result[i].z, result[i].y);
                result[i] = itemPoint;
            }
        }
        return result;
    }

    private IEnumerator BulidPointCloudObject(Vector3[] points, Color[] pointsColor, IPoint[] picPoints) {
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

        if (isPointTriangle)
        {
            var delaunator = new Delaunator(picPoints.ToArray());
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
                if (angleAInRadians < PointCloudTriangleAngleLimit ||
                    angleBInRadians < PointCloudTriangleAngleLimit ||
                    angleCInRadians < PointCloudTriangleAngleLimit || 
                    area > PointCloudTriangleAreaLimit)
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
            StartCoroutine(CreatTriangleMesh(filename, trianglelist.ToArray(), points, pointsColor));
        }
        else
        {
            StartCoroutine(CreatPointMesh(filename, points, pointsColor));
        }
        
        yield return null;
    }

    private IEnumerator BulidMultiPointCloudObject(Vector3[] points, Color[] pointsColor, IPoint[] picPoints)
    {
        // 更新点云
        if (pointCloud != null)
        {
            Transform parentTransform = pointCloud.transform;
        }
        else
        {
            pointCloud = new GameObject(filename);
        }

        if (isPointTriangle)
        {
            var delaunator = new Delaunator(picPoints.ToArray());
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
                if (angleAInRadians < PointCloudTriangleAngleLimit || 
                    angleBInRadians < PointCloudTriangleAngleLimit || 
                    angleCInRadians < PointCloudTriangleAngleLimit || 
                    area > PointCloudTriangleAreaLimit)
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
            StartCoroutine(CreatTriangleMesh(filename, trianglelist.ToArray(), points, pointsColor));
        }
        else
        {
            StartCoroutine(CreatPointMesh(filename, points, pointsColor));
        }

        yield return null;
    }

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
                //pointGroup.transform.localScale = new Vector3((float)scala, (float)scala, (float)scala);
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
        //pointGroup.transform.localScale = new Vector3((float)scala, (float)scala, (float)scala);
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
            if (meshId == groupNum - 1)
            {
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
            //pointGroup.transform.localScale = new Vector3((float)scala, (float)scala, (float)scala);
            pointGroup.transform.parent = pointCloud.transform;
            yield return null;
        }
        yield return null;
    }

    /**********************************************************-- 多视角点云融合 --********************************************************/
    // todo
    Vector3 ExtractPositionFromMatrix(Matrix4x4 matrix)
    {
        // 提取位置 - 矩阵的最后一列
        return new Vector3(matrix.m03, matrix.m13, matrix.m23);
    }

    Quaternion ExtractRotationFromMatrix(Matrix4x4 matrix)
    {
        // 假设没有缩放 - 提取3x3矩阵
        Vector3 forward = new Vector3(matrix.m02, matrix.m12, matrix.m22);
        Vector3 upwards = new Vector3(matrix.m01, matrix.m11, matrix.m21);
        return Quaternion.LookRotation(forward, upwards);
    }

    private Matrix4x4 SiftMatchPointCloud(Color[] sourceColors, Color[] targetColors, ushort[] sourceDepths, ushort[] targetDepths)
    {
        List<Vector2> match_pt1;
        List<Vector2> match_pt2;
        Sift.SearchMatchPoint(sourceColors, targetColors, Image_Width, Image_Height, 10, out match_pt1, out match_pt2);
        if (match_pt1.Count < 4) 
        {
            Debug.Log("【SIFT】特征点匹配的数量过少");
            return Matrix4x4.identity;
        }

        List<Vector3> source_pt1 = new List<Vector3>();
        List<Vector3> target_pt2 = new List<Vector3>();
        for (int i = 0; i < match_pt1.Count; i++)
        {
            Vector2 pt1 = match_pt1[i];
            Vector2 pt2 = match_pt2[i];
            float zw = BitmapInterpolation.bilinear(sourceDepths, Image_Width, Image_Height, pt1.x, pt1.y);
            Vector3 pt1_3d = new (((int)pt1.x - ppx) / fx * zw, ((int)pt1.y - ppy) / fy * zw, zw);
            zw = BitmapInterpolation.bilinear(targetDepths, Image_Width, Image_Height, pt2.x, pt2.y);
            Vector3 pt2_3d = new(((int)pt2.x - ppx) / fx * zw, ((int)pt2.y - ppy) / fy * zw, zw);
            source_pt1.Add(pt1_3d);
            target_pt2.Add(pt2_3d);
        }

        PointCloudHandle.RegisterPointCloud(source_pt1, target_pt2,
        out MathNet.Numerics.LinearAlgebra.Matrix<double> rotation_matrix, 
        out MathNet.Numerics.LinearAlgebra.Vector<double> translation_vector);

        return PointCloudHandle.BuildMatrix4x4ByRT(rotation_matrix, translation_vector);
    }

    /**********************************************************-- 标定相关代码 --********************************************************/
    Image<Bgr, byte> ConvertColorArrayToImage(Color[] colors, int width, int height)
    {
        Image<Bgr, byte> image = new Image<Bgr, byte>(width, height);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Color color = colors[y * width + x];
                Bgr bgrColor = new Bgr(color.b * 255, color.g * 255, color.r * 255);
                image[y, x] = bgrColor;
            }
        }
        return image;
    }

    public System.Drawing.PointF[,] GetCorners(Color[] input_colors, int width, int height)
    {
        System.Drawing.PointF[,] corner = new System.Drawing.PointF[height_num, width_num];
        Image<Bgr, byte> image = ConvertColorArrayToImage(input_colors, width, height);
        // 定义棋盘格的尺寸
        System.Drawing.Size patternSize = new System.Drawing.Size(width_num, height_num);
        // 查找棋盘格的角点
        VectorOfPointF corners = new VectorOfPointF();
        bool found = CvInvoke.FindChessboardCorners(image, patternSize, corners, CalibCbType.AdaptiveThresh);
        if (found)
        {
            Mat grayImage = new Mat();
            CvInvoke.CvtColor(image, grayImage, ColorConversion.Bgr2Gray);
            // 亚像素精确化 MCvTermCriteria(迭代次数，阈值)
            CvInvoke.CornerSubPix(grayImage, corners, new System.Drawing.Size(7, 7), subCorner_win_size, mCvTermCriteria);
            // 绘制角点
            Image<Bgr, byte> imageWithCorners = image.Clone();
            CvInvoke.DrawChessboardCorners(imageWithCorners, patternSize, corners, found);
            System.Drawing.PointF[] cornorsPF = corners.ToArray();
            for (int index = 0; index < cornorsPF.Count(); index++)
            {
                System.Drawing.PointF pt = cornorsPF[index];
                int wi = index % width_num;
                int hi = index / width_num;
                corner[hi, wi] = pt;
            }
            return corner;
        }
        else
        {
            return null;
        }
    }

    /// <summary>
    /// 测量法计算的RT
    /// </summary>
    private bool ChessBoardCalculateRT(System.Drawing.PointF[,] corners, ushort[] points_depth, out Matrix4x4 matrixRT) {
        List<Vector3> actualPts = new List<Vector3>();
        List<Vector3> realSensePts = new List<Vector3>();
        for (int hi = 0; hi < height_num; hi++)
        {
            for (int wi = 0; wi < width_num; wi++)
            {
                // 实际标定板的角点, 加上基坐标的相对位置
                Vector3 pt_actual = new(wi * actual_dis + pointCloudPosition.x, -hi * actual_dis + pointCloudPosition.z, 0 + pointCloudPosition.y);
                // realsense获得的角点
                System.Drawing.PointF pt = corners[hi, wi];
                float zw = BitmapInterpolation.bilinear(points_depth, Image_Width, Image_Height, pt.X, pt.Y);
                Vector3 pt_realsense = new(((int)pt.X - ppx) / fx * zw, 
                    ((int)pt.Y - ppy) / fy * zw, zw);
                if (pt_realsense.z > 0)
                {
                    actualPts.Add(pt_actual);
                    realSensePts.Add(new Vector3(pt_realsense.x, pt_realsense.y, pt_realsense.z));
                }
            }
        }

        matrixRT = Matrix4x4.identity;
        if (actualPts.Count >= 4 && realSensePts.Count >= 4 && actualPts.Count == realSensePts.Count)
        {
            PointCloudHandle.RegisterPointCloud(realSensePts, actualPts, 
                out MathNet.Numerics.LinearAlgebra.Matrix<double> rotation_matrix, out MathNet.Numerics.LinearAlgebra.Vector<double> translation_vector);
            Debug.Log("旋转矩阵：\n" + rotation_matrix);
            Debug.Log("平移矩阵：\n" + translation_vector);

            matrixRT = PointCloudHandle.BuildMatrix4x4ByRT(rotation_matrix, translation_vector);
            List<Vector3> item_pts = new List<Vector3>();
            for (int i = 0; i < realSensePts.Count; i++)
                item_pts.Add(matrixRT.MultiplyPoint3x4(realSensePts[i]));

            double error_dis = 0;
            for (int i = 0; i < item_pts.Count; i++)
            {
                error_dis += Vector3.Distance(item_pts[i], actualPts[i]);
            }
            Debug.Log($"reprojection sum error: {error_dis}, avg error: {error_dis / item_pts.Count}" );
            return true;
        }
        else
        {
            Debug.Log("【UnitySubscription_PointCloud】未找到关键点，获取位姿关系失败");
            return false;
        }
    }

    private bool ChessBoard2CameraRT(System.Drawing.PointF[,] corners, ushort[] points_depth, out Matrix4x4 matrixRT)
    {
        List<Vector3> actualPts = new List<Vector3>();
        List<Vector3> realSensePts = new List<Vector3>();
        for (int hi = 0; hi < height_num; hi++)
        {
            for (int wi = 0; wi < width_num; wi++)
            {
                // 实际标定板的角点, 加上基坐标的相对位置
                Vector3 pt_actual = new(wi * actual_dis + pointCloudPosition.x, -hi * actual_dis + pointCloudPosition.z, 0 + pointCloudPosition.y);
                // realsense获得的角点
                System.Drawing.PointF pt = corners[hi, wi];
                float zw = BitmapInterpolation.bilinear(points_depth, Image_Width, Image_Height, pt.X, pt.Y);
                Vector3 pt_realsense = new(((int)pt.X - ppx) / fx * zw,
                    ((int)pt.Y - ppy) / fy * zw, zw);
                if (pt_realsense.z > 0)
                {
                    actualPts.Add(pt_actual);
                    realSensePts.Add(new Vector3(pt_realsense.x, pt_realsense.y, pt_realsense.z));
                }
            }
        }

        matrixRT = Matrix4x4.identity;
        if (actualPts.Count >= 4 && realSensePts.Count >= 4 && actualPts.Count == realSensePts.Count)
        {
            PointCloudHandle.RegisterPointCloud(actualPts, realSensePts,
                out MathNet.Numerics.LinearAlgebra.Matrix<double> rotation_matrix, out MathNet.Numerics.LinearAlgebra.Vector<double> translation_vector);
            Debug.Log("旋转矩阵：\n" + rotation_matrix);
            Debug.Log("平移矩阵：\n" + translation_vector);
            return true;
        }
        else
        {
            Debug.Log("【UnitySubscription_PointCloud】未找到关键点，获取位姿关系失败");
            return false;
        }
    }

    /// <summary>
    /// 标定获得的相机外参，3*3旋转矩阵 3*1的平移向量, 返回图像的重投影误差
    /// </summary>
    /// <param name="input_bitmap"></param>
    /// <param name="r"></param>
    /// <param name="t"></param>
    /// <returns></returns>
    private double CameraExternalRT(PointF[,] corners, out Emgu.CV.Matrix<double> r, out Emgu.CV.Matrix<double> t)
    {
        // 1. 参数1：初始化标定板上角点的三维坐标
        List<MCvPoint3D32f> object_list = new List<MCvPoint3D32f>();
        for (int i = 0; i < height_num; i++)
        {
            for (int j = 0; j < width_num; j++)
            {
                object_list.Add(new MCvPoint3D32f(j * actual_dis, i * -actual_dis, 0));
            }
        }
        MCvPoint3D32f[] object_points = object_list.ToArray();

        // 2. 参数2，角点的图像坐标并转为一维数组
        int rows = corners.GetLength(0);
        int cols = corners.GetLength(1);
        PointF[] imagePoints = new PointF[rows * cols];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                imagePoints[i * cols + j] = corners[i, j];
            }
        }

        // 创建变量来存储求解结果
        Mat rvec = new Mat(); // 旋转向量
        Mat tvec = new Mat(); // 平移向量
                              // 使用SolvePnP计算相机外参
        Matrix<double> cameraInerMatrix = new Matrix<double>(new double[,]
        {
                { fx, 0, ppx},
                { 0, fy, ppy},
                { 0, 0, 1 }
        });
        Matrix<double> dff = new Matrix<double>(new double[] { 0, 0, 0, 0, 0 });

        // 使用 SolvePnP 求解位姿
        bool success = CvInvoke.SolvePnP(object_points, imagePoints, cameraInerMatrix, dff, rvec, tvec);
        // 旋转矩阵和平移矩阵
        r = new Emgu.CV.Matrix<double>(3, 3);
        CvInvoke.Rodrigues(rvec, r);
        t = new Emgu.CV.Matrix<double>(3, 1);
        tvec.CopyTo(t);

        PointF[] proPoints = CvInvoke.ProjectPoints(object_points, rvec, tvec, cameraInerMatrix, dff);
        double errorSum = 0.0;
        for (int i = 0; i < proPoints.Length; ++i)
        {
            errorSum += Math.Abs(imagePoints[i].X - proPoints[i].X) + Math.Abs(imagePoints[i].Y - proPoints[i].Y);
        }
        double totalError = errorSum / proPoints.Length;

        Debug.Log($"【SolvePnp】平均重投影误差为： {totalError}");
        return totalError;
    }

    /// <summary>
    /// 眼在手上标定，输入的是3*3的旋转矩阵
    /// </summary>
    private void EyeOnHand(List<Matrix<double>> endPoint2BaseR, List<Matrix<double>> endPoint2BaseT,
        List<Matrix<double>> target2CameraR, List<Matrix<double>> target2CameraT,
        out Matrix<double> camera2EndpointR, out Matrix<double> camera2EndpointT)
    {
        // 旋转矩阵转为旋转向量
        VectorOfMat vMatEndPoint2BaseR = new VectorOfMat();
        VectorOfMat vMatEndPoint2BaseT = new VectorOfMat();
        VectorOfMat vMatTarget2CameraR = new VectorOfMat();
        VectorOfMat vMatTarget2CameraT = new VectorOfMat();
        for (int i = 0; i < endPoint2BaseR.Count; i++)
        {
            vMatEndPoint2BaseR.Push(endPoint2BaseR[i]);
            vMatTarget2CameraR.Push(target2CameraR[i]);
            vMatEndPoint2BaseT.Push(endPoint2BaseT[i]);
            vMatTarget2CameraT.Push(target2CameraT[i]);
        }

        Mat temp_r = new Mat();
        Mat temp_t = new Mat();
        CvInvoke.CalibrateHandEye(vMatEndPoint2BaseR, vMatEndPoint2BaseT,
            vMatTarget2CameraR, vMatTarget2CameraT,
            temp_r, temp_t,
            HandEyeCalibrationMethod.Tsai);

        // 旋转矩阵和平移矩阵
        camera2EndpointR = new Matrix<double>(3, 3);
        temp_r.CopyTo(camera2EndpointR);

        camera2EndpointT = new Matrix<double>(3, 1);
        temp_t.CopyTo(camera2EndpointT);
    }

    public void EyeOnHandCalibrationConfrim() {
        if (IsCalibration)
        {
            if (endPoint2Base_R.Count < 4 || endPoint2Base_T.Count < 4 ||
                target2Camera_R.Count < 4 || target2Camera_T.Count < 4)
            {
                Debug.Log($"【EyeOnHandCalibrationConfrim】当前标定次数不满足 count < 4");
                return;
            }

            Emgu.CV.Matrix<double> temp_r;
            Emgu.CV.Matrix<double> temp_t;
            EyeOnHand(endPoint2Base_R, endPoint2Base_T,
                target2Camera_R, target2Camera_T,
                out temp_r, out temp_t);
            matrix4x4_camera2endPoint = PointCloudHandle.BuildMatrix4x4ByRT(temp_r, temp_t);
            Debug.Log($"【相机标定CE】camera2endPoint: \n{matrix4x4_camera2endPoint}");
            Debug.Log($"【相机标定TF】matrix4x4_camera2endPoint: \n " +
                        $"平移:\n {ExtractPositionFromMatrix(matrix4x4_camera2endPoint)}" +
                        $"旋转:\n {ExtractRotationFromMatrix(matrix4x4_camera2endPoint)}");

            EyeOnHandCalibrationData eyeOnHandCalibrationData = new EyeOnHandCalibrationData();
            eyeOnHandCalibrationData.CameraToEndPoint = matrix4x4_camera2endPoint;
            eyeOnHandCalibrationData.CalibrationMethod = "OpenCV_Tasi";
            EyeOnHandCalibration.SaveJsonData(eyeOnHandCalibrationData);
        }
        else 
        {
            Debug.Log("【EyeOnHandCalibrationConfrim】当前并不处于手眼标定模式");
        }
    }

    /// <summary>
    /// 获得角点在realsense下的坐标
    /// </summary>
    private List<Vector3> GetCornersPositionInCamrera(System.Drawing.PointF[,] corners, ushort[] points_depth) 
    {
        List<Vector3> points = new List<Vector3>();
        for (int hi = 0; hi < height_num; hi++)
        {
            for (int wi = 0; wi < width_num; wi++)
            {
                // realsense获得的角点
                System.Drawing.PointF pt = corners[hi, wi];
                int index = (int)pt.X + Image_Width * (int)pt.Y; // todo 可以优化
                Vector3 pt_realsense = new(((int)pt.X - ppx) / fx * points_depth[index],
                    ((int)pt.Y - ppy) / fy * points_depth[index],
                    points_depth[index]);
                if (pt_realsense.z > 0)
                {
                    points.Add(pt_realsense);
                }
            }
        }
        return points;
    }

    /// <summary>
    /// matrix转为字符串并用"，"分割
    /// </summary>
    private string Matrix4x4ToString(Matrix4x4 matrix) 
    {
        string[] elements = new string[16];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                elements[i * 4 + j] = matrix[i, j].ToString("F3");
            }
        }
        string matrixString = string.Join(",", elements);
        return matrixString;
    }
    private string ListVector3ToString(List<Vector3> vectors)
    {
        List<string> elements = new List<string>();
        foreach (Vector3 vec in vectors)
        {
            // 将每个向量的x,y,z坐标添加到元素列表中
            elements.Add(vec.x.ToString("F3"));
            elements.Add(vec.y.ToString("F3"));
            elements.Add(vec.z.ToString("F3"));
        }
        // 使用逗号将元素列表中的所有字符串连接起来
        return string.Join(",", elements);
    }

    /// <summary>
    /// 计算相机到末端的位姿变换矩阵
    /// </summary>
    /// <param name="M_Camera_to_UnityBase">相机到Unity坐标系变换矩阵，由标定得到</param>
    /// <param name="M_RosBase_to_UnityBase">ROS坐标系到Unity坐标系，直接确定</param>
    /// <param name="M_RosEndPoint_to_RosBase">TF树获得</param>
    /// <returns></returns>
    private Matrix4x4 CalculateRT_CameraToEndPoint(Matrix4x4 M_Camera_to_UnityBase, Matrix4x4 M_RosBase_to_UnityBase, Matrix4x4 M_RosEndPoint_to_RosBase) {
        Matrix4x4 M_Camera_to_RosEndPoint = Matrix4x4.identity;
        M_Camera_to_RosEndPoint = M_RosEndPoint_to_RosBase.inverse * M_RosBase_to_UnityBase.inverse * M_Camera_to_UnityBase;
        return M_Camera_to_RosEndPoint;
    }

    private Matrix4x4 BulidMatrixFromTF(Vector3 T, Quaternion Q) {
        return Matrix4x4.TRS(T, Q, Vector3.one);
    }

    private Matrix4x4 GetMatrixEndPoint2RosBaseFromTF() {
        // todo 从TF树获取
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        RosMessageTypes.Geometry.PoseMsg endPoint2Base = auboControl.m_Transform;
        Vector3 T = new Vector3((float)endPoint2Base.position.x * 1000, 
            (float)endPoint2Base.position.y * 1000, 
            (float)endPoint2Base.position.z * 1000);
        Quaternion Q = new Quaternion((float)endPoint2Base.orientation.x,
            (float)endPoint2Base.orientation.y, 
            (float)endPoint2Base.orientation.z,
            (float)endPoint2Base.orientation.w);
            return Matrix4x4.TRS(T, Q, Vector3.one);
    }

    private void Matrix4x4ToEmgucvMatrixRT(Matrix4x4 matrix, out Emgu.CV.Matrix<double> R, out Emgu.CV.Matrix<double> T)
    {
        R = new Emgu.CV.Matrix<double>(3, 3);
        T = new Emgu.CV.Matrix<double>(3, 1);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.Data[i, j] = matrix[i, j];
            }
        }
        // 提取平移矩阵
        T[0, 0] = matrix[0, 3];
        T[1, 0] = matrix[1, 3];
        T[2, 0] = matrix[2, 3];
    }

    public void ReadEyeOnHandCalibrationData()
    {
        // 读取手眼标定配置文件
        EyeOnHandCalibrationData eyeOnHandCalibrationData = EyeOnHandCalibration.ReadJsonData();
        if (eyeOnHandCalibrationData != null)
        {
            Debug.Log($"【UnitySubscription_PointCloud】matrix4x4_camera2endPoint ReadJsonData \n {eyeOnHandCalibrationData}");
            DebugGUI.LogString($"【UnitySubscription_PointCloud】matrix4x4_camera2endPoint ReadJsonData \n {eyeOnHandCalibrationData}");
            matrix4x4_camera2endPoint = eyeOnHandCalibrationData.CameraToEndPoint;
        }
        else
        {
            Debug.Log("【UnitySubscription_PointCloud】获取手眼标定数据失败，不存在标定文件");
            DebugGUI.LogString("【UnitySubscription_PointCloud】获取手眼标定数据失败，不存在标定文件");
        }
    }
    /**********************************************************-- 点云保存相关代码 --********************************************************/
    IEnumerator SavePointCloud2OffFile(string PointCloudSavaDirPath) {
        if (Directory.Exists(PointCloudSavaDirPath))
        {
            // 获取当前时间戳到毫秒
            long timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            // 创建文件路径
            string filePath = Path.Combine(PointCloudSavaDirPath, $"pointCloud_{timestamp}.off_unity");
            StringBuilder fileContent = new StringBuilder();
            // 写入OFF文件头
            fileContent.AppendLine("OFF");
            fileContent.AppendLine($"{points.Length} 0 0");

            // 写入顶点数据
            for (int i = 0; i < points.Length; i++) 
            {
                int r = (int)(pointsColor[i].r * 255.0f);
                int g = (int)(pointsColor[i].g * 255.0f);
                int b = (int)(pointsColor[i].b * 255.0f);
                fileContent.AppendLine($"{points[i].x} {points[i].y} {points[i].z} {r} {g} {b} {picPoints[i].X} {picPoints[i].Y}");
                if (i % 10000 == 0)
                {
                    yield return null;
                }
            }

            // 将字符串写入文件
            File.WriteAllText(filePath, fileContent.ToString());
            yield return null;

        } else
        {
            Debug.Log($"SavePointCloud2OffFile 文件夹 {PointCloudSavaDirPath} 不存在");
            yield return null;
        }
    }
    IEnumerator SavePointCloud2OffFile2PersistentDataPath()
    {
        if (Directory.Exists(Application.persistentDataPath))
        {
            // 判断当前是单视角还是多视角
            if (!IsMultiViewPointCloudShow)
            {
                if (points == null || pointsColor == null || picPoints == null)
                    yield break;

                // 获取当前时间戳到秒
                long timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
                // 创建文件路径
                string filePath = Path.Combine(Application.persistentDataPath, $"pointCloud_{timestamp}.off_unity");
                StringBuilder fileContent = new StringBuilder();
                // 写入OFF文件头
                fileContent.AppendLine("OFF");
                fileContent.AppendLine($"{points.Length} 0 0");

                // 写入顶点数据
                for (int i = 0; i < points.Length; i++)
                {
                    int r = (int)(pointsColor[i].r * 255.0f);
                    int g = (int)(pointsColor[i].g * 255.0f);
                    int b = (int)(pointsColor[i].b * 255.0f);
                    fileContent.AppendLine($"{points[i].x} {points[i].y} {points[i].z} {r} {g} {b} {picPoints[i].X} {picPoints[i].Y}");
                    if (i % 10000 == 0)
                    {
                        yield return null;
                    }
                }

                // 将字符串写入文件
                File.WriteAllText(filePath, fileContent.ToString());
                DebugGUI.LogString($"【SavePointCloud】 保存文件至:{filePath}");
                yield return null;
            }
            else
            {
                // 获取当前时间戳到秒
                long timestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
                for (int index = 0; index < points_mulview.Count; index++)
                {
                    // 创建文件路径
                    string filePath = Path.Combine(Application.persistentDataPath, $"pointCloudMul_{index}_{timestamp}.off_unity");
                    StringBuilder fileContent = new StringBuilder();
                    // 写入OFF文件头
                    fileContent.AppendLine("OFF");
                    fileContent.AppendLine($"{points_mulview[index].Length} 0 0");

                    // 写入顶点数据
                    for (int i = 0; i < points_mulview[index].Length; i++)
                    {
                        int r = (int)(colors_mulview[index][i].r * 255.0f);
                        int g = (int)(colors_mulview[index][i].g * 255.0f);
                        int b = (int)(colors_mulview[index][i].b * 255.0f);
                        fileContent.AppendLine($"{points_mulview[index][i].x} {points_mulview[index][i].y} {points_mulview[index][i].z} {r} {g} {b} {picPoints_mulview[index][i].X} {picPoints_mulview[index][i].Y}");
                        if (i % 10000 == 0)
                        {
                            yield return null;
                        }
                    }

                    // 将字符串写入文件
                    File.WriteAllText(filePath, fileContent.ToString());
                    DebugGUI.LogString($"【SavePointCloud】 保存文件至:{filePath}");
                    yield return null;
                }
            }
        }
        else
        {
            Debug.Log($"【SavePointCloud】 文件夹 {Application.persistentDataPath} 不存在");
            DebugGUI.LogString($"【SavePointCloud】 文件夹 {Application.persistentDataPath} 不存在");
            yield return null;
        }
    }

    public Vector3[] GetPointCloudVector3Array()
    {
        List<Vector3> points = new List<Vector3>();
        // 判断当前的状态，来获取点云数据
        if (!IsMultiViewPointCloudShow)
        {
            points.AddRange(this.points);
            return points.ToArray();
        }
        else 
        { 
            for (int i = 0; i < points_mulview.Count; i++)
            {
                points.AddRange(points_mulview[i]);
            }
            return points.ToArray();
        }
    }

    /**********************************************************-- 调用python代码 --********************************************************/
    private string RunPythonAndGetOutput(string scriptName, string tf1, string tf2, string p1, string p2, string initRT)
    {
        // 设置Python脚本路径
        string scriptPath = Path.Combine(Application.dataPath, scriptName);

        // 创建ProcessStartInfo
        ProcessStartInfo start = new ProcessStartInfo
        {
            FileName = "python", // 确保这个路径在你的系统环境变量中配置正确
            Arguments = $"\"{scriptPath}\" {tf1} {tf2} {p1} {p2} {initRT}",
            UseShellExecute = false,
            RedirectStandardOutput = true,
            CreateNoWindow = true
        };

        // 启动进程并获取输出
        using (Process process = Process.Start(start))
        {
            using (StreamReader reader = process.StandardOutput)
            {
                string result = reader.ReadToEnd();
                Debug.Log("脚本调用结果:" + result);
                return result.Trim(); // Trim用于去除字符串两端的空白字符
            }
        }
    }
}