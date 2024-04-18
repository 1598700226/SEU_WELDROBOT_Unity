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

    /*############## RealSense����ڲ� ##############*/
    private int Image_Width = 640;
    private int Image_Height = 480;
    private float fx = 608.912109375f;
    private float fy = 607.4900512695312f;
    private float ppx = 316.9831848144531f;
    private float ppy = 245.9309844970703f;

    public int showType = 0; // 0�ǲ���ʾ��1�ǵ��Ʋ�ɫͼ��2�ǵ������ͼ��3�ǵ��������ɫͼ��4�ǵ���������ͼ
    private Color[] Colors;
    private ushort[] Depths;

    /*############## ROS��������� ##############*/
    public string color_image_topic = "/camera2/color/image_raw/compressed";
    public string depth_image_topic = "/camera2/aligned_depth_to_color/image_raw";

    /*********************** ������� **********************/
    public bool hasObstacles = false;
    public float depthThreshold_mm = 1000f;
    public float depthExceedPecent = 0.3f;

    // Start is called before the first frame update
    void Start()
    {
        RawImageTexture = new Texture2D(Image_Width, Image_Height);
        // ����Realsense�ϰ�����
        StartCoroutine(CheckForObstacles());
    }

    // Update is called once per frame
    void Update()
    {
        // ͼƬ�ؼ���ʾ
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
    /// ROS��ɫ���ݻ���Ķ���
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
            Debug.Log("��UnitySubscription_PointCloud error��rawImage�ؼ�Ϊ��");
            return;
        }

        RawImageTexture.LoadImage(data);
        // ��Texture2D��ʾ��RawImage��
        RawImageTexture.Apply();  // Ӧ�����ظ���
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
                // ��Ҫ�������Ͻǿ�ʼ��GetPixels()�����½ǵ�����
                ret[w + h * Image_Width] = rgbArray[w + (Image_Height - 1 - h) * Image_Width];
            }

        Destroy(texture);
        return ret;
    }

    /// <summary>
    /// ROS������ݻ���Ķ���
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
            // ���ݶ��� byte[] תΪ ushort
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
            Debug.Log("��UnitySubscription_PointCloud error��rawImage�ؼ�Ϊ��");
            return;
        }
        RawImageTexture = new Texture2D(Image_Width, Image_Height, TextureFormat.RGBA32, false);
        ushort max = 0;
        for (int i = 0; i < data.Length; i++)
        {
            if (data[i] > max)
                max = data[i];
        }
        // �� ushort[] ����ת��Ϊ��ɫ��Ϣ
        Color[] colors = new Color[data.Length];
        for (int i = 0; i < data.Length; i++)
        {
            float normalizedValue = (float)data[i] / max;
            colors[i] = new Color(normalizedValue, normalizedValue, normalizedValue, 255);
        }
        // ����ɫ��ϢӦ�õ� Texture2D ��, ��Ҫ�ߵ���ʾ
        for (int y = 0; y < Image_Height; y++)
        {
            for (int x = 0; x < Image_Width; x++)
            {
                RawImageTexture.SetPixel(x, Image_Height - 1 - y, colors[x + y * Image_Width]);  // ��Ҫ��תy����ƥ��Unity����ϵ
            }
        }
        RawImageTexture.Apply();  // Ӧ�����ظ���
        rawImage.texture = RawImageTexture;
    }

    /************************************* �ϰ����� ****************************************/
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
                    // ����м�һ���ߵ������ֵ����
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
                        Debug.Log($"��⵽�ϰ�����ǰ��ֵ{exceedPecent}");
                        hasObstacles = true;
                    }
                }
            }

            yield return new WaitForSeconds(0.5f);
        }
    }
}