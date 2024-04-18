using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class UnitySubscription_AvoidanceCamrea : MonoBehaviour
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

    public int showType = 0; // 0是不显示；1是点云彩色图；2是点云深度图；3是导航相机彩色图；4是导航相机深度图
    private Color[] Colors;
    private ushort[] Depths;

    /*############## ROS话题与服务 ##############*/
    public string color_image_topic = "/camera2/color/image_raw/compressed";
    public string depth_image_topic = "/camera2/aligned_depth_to_color/image_raw";

    /*********************** 避障相关 **********************/
    public bool hasObstacles = false;
    public float depthThreshold_mm = 1000f;
    public float depthExceedPecent = 0.3f;

    // Start is called before the first frame update
    void Start()
    {
        RawImageTexture = new Texture2D(Image_Width, Image_Height);
        // 开启Realsense障碍物检测
        StartCoroutine(CheckForObstacles());
    }

    // Update is called once per frame
    void Update()
    {
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
    }

    public void SubscribeTopics(bool color_topic = true, bool depth_topic = true)
    {
        if (color_topic)
        {
            if (!ROSConnection.GetOrCreateInstance().HasSubscriber(color_image_topic))
                ROSConnection.GetOrCreateInstance().
                    Subscribe<RosMessageTypes.Sensor.CompressedImageMsg>(color_image_topic, CompressedColorImageCall);
        }

        if (depth_topic)
        {
            if (!ROSConnection.GetOrCreateInstance().HasSubscriber(depth_image_topic))
                ROSConnection.GetOrCreateInstance().
                    Subscribe<RosMessageTypes.Sensor.ImageMsg>(depth_image_topic, DepthImageCall);
        }
    }

    public void UnSubscribeTopics()
    {
        ROSConnection.GetOrCreateInstance().Unsubscribe(color_image_topic);
        ROSConnection.GetOrCreateInstance().Unsubscribe(depth_image_topic);
    }

    /// <summary>
    /// ROS彩色数据话题的订阅
    /// </summary>
    /// <param name="CompressedColorImage"></param>
    void CompressedColorImageCall(RosMessageTypes.Sensor.CompressedImageMsg CompressedColorImage)
    {
        Colors = JPEGToRGB(CompressedColorImage.data);
        if (showType == 3)
            UpdateCompressedColorImage(CompressedColorImage.data);
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
            for (int h = 0; h < Image_Height; h++)
            {
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
    void DepthImageCall(RosMessageTypes.Sensor.ImageMsg DepthImage)
    {
        Depths = ConvertBytesToUShort(DepthImage.data);
        if (showType == 4)
            UpdateRawImageByUshortData(Depths);
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

            if (ushortData[i] > max)
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
            if (data[i] > max)
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

    /************************************* 障碍物检测 ****************************************/
    IEnumerator CheckForObstacles() 
    {
        while (true)
        {
            if (!ROSConnection.GetOrCreateInstance().HasSubscriber(depth_image_topic))
            {
                hasObstacles = false;
            }
            else
            {
                if (Depths == null)
                {
                    hasObstacles = false;
                }
                else
                {
                    // 检测中间一条线的深度阈值比例
                    int sum = Image_Width;
                    int exceedNum = 0;
                    for (int i = 0; i < Image_Width; i++)
                    {
                        if (Depths[Image_Width * Image_Height / 2 + i] == 0 ||
                            Depths[Image_Width * Image_Height / 2 + i] < depthThreshold_mm)
                        {
                            exceedNum++;
                        }
                    }
                    float exceedPecent = (float)exceedNum / sum;
                    if (exceedPecent > depthExceedPecent)
                    {
                        Debug.Log($"检测到障碍，当前阈值{exceedPecent}");
                        hasObstacles = true;
                    }
                }
            }

            yield return new WaitForSeconds(0.5f);
        }
    }
}