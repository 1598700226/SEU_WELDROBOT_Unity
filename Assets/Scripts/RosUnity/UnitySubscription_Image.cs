using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;

public class UnitySubscription_Image : MonoBehaviour
{
    public RawImage rawImage;
    // Start is called before the first frame update
    bool isShowImage = false;

    void Start()
    {
        //ROSConnection.GetOrCreateInstance().Subscribe<RosMessageTypes.Sensor.ImageMsg>("/camera/color/image_raw", ColorImageCall);
        ROSConnection.GetOrCreateInstance().Subscribe<RosMessageTypes.Sensor.CompressedImageMsg>("/camera/color/image_raw/compressed", CompressedColorImageCall);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void ColorImageCall(RosMessageTypes.Sensor.ImageMsg colorImage) {
        if (!isShowImage) {
            isShowImage = true;
            UpdateRawImage((int)colorImage.width, (int)colorImage.height, colorImage.data);
        }
    }

    private void UpdateRawImage(int img_width, int img_height, byte[] data)
    {
        if (rawImage == null)
        {
            Debug.Log("【UnitySubscription_Image error】rawImage控件为空");
            return;
        }
        Texture2D texture = new Texture2D(img_width, img_height, TextureFormat.RGB24, false);
        texture.LoadImage(data);
        // 将Texture2D显示在RawImage上
        texture.Apply();  // 应用像素更改
        rawImage.texture = texture;
    }

    void CompressedColorImageCall(RosMessageTypes.Sensor.CompressedImageMsg CompressedColorImage)
    {
        if (!isShowImage)
        {
            isShowImage = true;
            UpdateCompressedRawImage(CompressedColorImage.data);
            isShowImage = false;
        }
        else {
            Debug.Log("正在加载图片");
        }
    }

    private void UpdateCompressedRawImage(byte[] data)
    {
        if (rawImage == null)
        {
            Debug.Log("【UnitySubscription_Image error】rawImage控件为空");
            return;
        }
        Texture2D texture = new Texture2D(640, 480);
        texture.LoadImage(data);
        // 将Texture2D显示在RawImage上
        texture.Apply();  // 应用像素更改
        rawImage.texture = texture;
    }
}
