using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SocialPlatforms;
using UnityEngine.UI;
using UnityEngine.UIElements;

public class PGMFileReader : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler, IPointerDownHandler
{
    public string pgmFileDir;   // PGM文件所在文件夹的路径
    public string pgmFilePath;  // PGM文件的路径
    public RawImage rawImage;   // 用于显示图像的UI RawImage
    public TMP_Text lableTitle;     
    private Texture2D pgmTexture2D;
    private int updateCoroutinePixelLimit = 250000;     // 协程更新像素点数量的限制
    private bool isUpdatePgmTextureRunning = false;   // 当前是否正在更新PGM地图
    private Texture2D pgmCarPositionTexture2D;
    public bool isUpdatePGMMapOnce = true;
    public bool isReceviceRosMapTopic = false;
    public GameObject RosMapTopicsServer;
    public int AstarNodeSize = 20;
    public int AstarObstacleGray = 255;

    // PGM图片的信息
    string fileSuffix = ".pgm";
    int width = 0;
    int height = 0;
    int maxPixelValue = 0;
    int[,] pixels = null;                // 【行，列】
    float imageShow_width_scala = 0.0f;  //缩放系数 图像像素位置*scala = 实际图像像素位置
    float imageShow_height_scala = 0.0f; //缩放系数 图像像素位置*scala = 实际图像像素位置..

    // 鼠标控制
    bool isMouseEnter = false;
    float currentZoom = 1.0f;
    float zoomSpeed = 3.0f;
    public Vector2 mouseDownPositon = Vector2.zero;         // unity图片坐标系下的目标点，坐标系原点是左下角
    public Vector2 mouseDownPositon_actual = Vector2.zero;  // 真实图片坐标系是左上角
    public bool isLocalMagnification = true;

    // 定位点显示
    public int PointShowSize = 20; 
    public Vector2 nowPosition = Vector2.zero;                     // unity图片坐标系下的点
    public Vector2 nowPosition_actual = Vector2.zero;
    private bool isShowPositionRunning = false;                    // 更新坐标点显示的协程

    // Start is called before the first frame update
    void Start()
    {
        width = rawImage.texture.width;
        height = rawImage.texture.height;
        RosMapTopicsServer = GameObject.Find("RosMapData");
    }

    // Update is called once per frame
    void Update()
    {
        if (isUpdatePGMMapOnce && !isReceviceRosMapTopic) {
            isUpdatePGMMapOnce = false;
            string updateFilePath;
            if (string.IsNullOrEmpty(pgmFilePath))
            {
                updateFilePath = getLatestPGMFile(pgmFileDir);
            }
            else
            { 
                updateFilePath = pgmFilePath;
            }

            bool ret = UploadPGMMap(updateFilePath);
            if (ret) {
                Debug.Log("【PGMFileReader】更新PGM地图文件：" + updateFilePath);
                DebugGUI.Log("【PGMFileReader】更新PGM地图文件：" + updateFilePath);
                StartCoroutine(UpdateRawImage(width, height, pixels, maxPixelValue));
            }
        }

        if (isMouseEnter) {
            if (rawImage == null) return;

            if (isLocalMagnification)
            {
                float scroll = Input.GetAxis("Mouse ScrollWheel");
                if (scroll != 0)
                {
                    // 更新缩放比例
                    currentZoom = Mathf.Clamp(currentZoom + scroll * zoomSpeed, 1f, 10f);

                    Vector2 point = MousePositionRelativeToImagePosition();
                    Vector2 normalized = new(point.x / width, point.y / height);

                    // 更新uvRect来放大纹理
                    float size = 1f / currentZoom;
                    float x = Mathf.Clamp(normalized.x - size * 0.5f, 0f, 1f - size);
                    float y = Mathf.Clamp(normalized.y - size * 0.5f, 0f, 1f - size);
                    rawImage.uvRect = new Rect(x, y, size, size);
                }
            }
            else 
            {
                // 获取鼠标滚轮的输入
                float scroll = Input.GetAxis("Mouse ScrollWheel");
                if (scroll != 0)
                {
                    // 根据滚轮输入调整RawImage的sizeDelta
                    rawImage.rectTransform.sizeDelta += new Vector2(scroll, scroll) * zoomSpeed * 50;
                }
            }

        }

        UnitySubscription_Map unitySubscription_Map = RosMapTopicsServer.GetComponent<UnitySubscription_Map>();
        if (isReceviceRosMapTopic) {
            if (unitySubscription_Map.hasNewDataReceive) {
                unitySubscription_Map.hasNewDataReceive = false;
                pixels = unitySubscription_Map.pixel;
                width = unitySubscription_Map.map_width;
                height = unitySubscription_Map.map_height;
                maxPixelValue = unitySubscription_Map.max_pixel;

                StartCoroutine(UpdateRawImage(width, height, pixels, maxPixelValue));
            }
        }

        if (unitySubscription_Map.hasNewNowPositionReceive)
        {
            unitySubscription_Map.hasNewNowPositionReceive = false;
            nowPosition_actual = unitySubscription_Map.nowPosition;
            nowPosition = new Vector2(nowPosition_actual.x, height - nowPosition_actual.y);
            List<Vector2> points = new List<Vector2>
                {
                    nowPosition
                };

            // Astar search path
            if (nowPosition == Vector2.zero || mouseDownPositon == Vector2.zero)
            {
                return;
            }
            AStarAlgorithm.Astar astar = new AStarAlgorithm.Astar(pixels,
                width, height, AstarNodeSize, AstarObstacleGray);
            List<Vector2> pathPoints = astar.getPath(nowPosition, mouseDownPositon);
            if (pathPoints == null)
            {
                Debug.Log("【Astar】hasNewNowPositionReceive 路径规划失败，可能终点不可达");
                DebugGUI.Log("【Astar】hasNewNowPositionReceive 路径规划失败，可能终点不可达");
            }
            else
            {
                points.AddRange(pathPoints);
            }
            points.Add(mouseDownPositon);
            StartCoroutine(UpdatePostionShow(points, PointShowSize, true));
        }
    }

    private bool UploadPGMMap(String filePath) {
        // 检查文件是否存在
        if (File.Exists(filePath))
        {
            // 读取PGM文件内容
            string[] lines = File.ReadAllLines(filePath);

            bool readingPixels = false;
            foreach (string line in lines)
            {
                if (readingPixels)
                {
                    // 文本方式读取像素值
                    /*                    string[] pixelValues = line.Split(' ');
                                        if (pixels == null)
                                            pixels = new int[height, width];

                                        for (int j = 0; j < height; j++)
                                        {
                                            for (int i = 0; i < width; i++)
                                            {
                                                pixels[j, i] = int.Parse(pixelValues[i + j * width]);
                                            }
                                        }*/

                    // 二进制方式读取
                    using (FileStream fileStream = new FileStream(filePath, FileMode.Open))
                    {
                        using (BinaryReader binaryReader = new BinaryReader(fileStream)) 
                        {
                            byte[] binaryData = binaryReader.ReadBytes((int)fileStream.Length);
                            int header_line_num = 0;
                            int header_line_max = 4; // 头信息的行数
                            int start_index = 0;
                            // 找到数据所在的byte索引
                            for (int i = 0; i < binaryData.Length; i++)
                            {
                                if (binaryData[i] == 10)
                                {
                                    header_line_num++;
                                    if (header_line_num >= header_line_max)
                                    {
                                        start_index = i + 1;
                                        break;
                                    }
                                }
                            }

                            for (int y = 0; y < height; y++)
                            {
                                for (int x = 0; x < width; x++)
                                {
                                    pixels[y, x] = (int)binaryData[x + y * width + start_index];
                                }
                            }
                        }
                    }
                }
                else if (line.StartsWith("#"))
                {
                    // 跳过注释行
                }
                else if (line.StartsWith("P2") || line.StartsWith("P5"))
                {
                    // 读取文件格式标识符（P2或P5）
                }
                else if (line.Contains(" "))
                {
                    // 读取宽度和高度
                    string[] dimensions = line.Split(' ');
                    width = int.Parse(dimensions[0]);
                    height = int.Parse(dimensions[1]);
                    pixels = new int[height, width];
                }
                else
                {
                    // 读取最大像素值
                    maxPixelValue = int.Parse(line);
                    readingPixels = true;
                }
            }
            return true;
        }
        else
        {
            Debug.Log("【PGMFileReader】PGM文件不存在。" + filePath);
            DebugGUI.Log("【PGMFileReader】PGM文件不存在。" + filePath);
            return false;
        }
    }

    private IEnumerator UpdateRawImage(int img_width, int img_height, int[,] data, int maxPixelValue) {
        if (rawImage == null)
        {
            Debug.Log("【PGMFileReader】rawImage控件为空");
            DebugGUI.Log("【PGMFileReader】rawImage控件为空");
            yield break;
        }
        if (isUpdatePgmTextureRunning)
        {
            Debug.Log("【PGMFileReader】UpdateRawImage 当前正在更新地图");
            DebugGUI.Log("【PGMFileReader】UpdateRawImage 当前正在更新地图");
            yield break;
        }

        isUpdatePgmTextureRunning = true;
        // 创建一个Texture2D来加载像素数据
        // 销毁旧的Texture2D以防止内存泄漏
        if (pgmTexture2D != null)
        {
            Destroy(pgmTexture2D);
        }
        pgmTexture2D = new Texture2D(img_width, img_height);
        // 将像素数据填充到Texture2D
        int count = 0;
        for (int y = 0; y < img_height; y++)
        {
            for (int x = 0; x < img_width; x++)
            {
                float normalizedValue = (float)data[y, x] / (float)maxPixelValue;
                Color color = new Color(1 - normalizedValue, 1 - normalizedValue, 1 - normalizedValue);
                pgmTexture2D.SetPixel(x, img_height - 1 - y, color);  // 需要翻转y轴以匹配Unity坐标系
                count++;
                if(count % updateCoroutinePixelLimit == 0)
                {
                    yield return null;
                }
            }
        }

        // 将Texture2D显示在RawImage上
        pgmTexture2D.Apply();  // 应用像素更改
        rawImage.texture = pgmTexture2D;
        // 更新图像控件的大小并获取缩放比例
        imageShow_width_scala = img_width / rawImage.rectTransform.rect.width;
        imageShow_height_scala = img_height / rawImage.rectTransform.rect.height;
        //PointShowSize = img_width / 80;

        isUpdatePgmTextureRunning = false;
    }

    private Vector2 MousePositionRelativeToImagePosition() 
    {
        if(width <= 0 || height <= 0)
            return Vector2.zero;

        Vector2 mousePosition = Input.mousePosition;
        float start_x = rawImage.uvRect.x * width;
        float start_y = rawImage.uvRect.y * height;

        Vector2 localCursor;
        RectTransformUtility.ScreenPointToLocalPointInRectangle(rawImage.rectTransform, mousePosition, null, out localCursor);
        Vector2 normalized = Rect.PointToNormalized(rawImage.rectTransform.rect, localCursor);
        float offset_x = normalized.x * width * rawImage.uvRect.width;
        float offset_y = normalized.y * height * rawImage.uvRect.height;

        return new Vector2(start_x + offset_x, start_y + offset_y);
    }

    public string getLatestPGMFile(String dirPath)
    {
        if (!Directory.Exists(dirPath))
        {
            Debug.Log("【getLatestPGMFile】文件夹不存在 dirPath:" + dirPath);
            return null;
        }

        DirectoryInfo directoryInfo = new DirectoryInfo(dirPath);
        FileInfo[] files = directoryInfo.GetFiles("*" + fileSuffix);
        if (files == null || files.Length <= 0)
        {
            Debug.Log("【getLatestPGMFile】文件夹下不存在PGM文件:" + dirPath);
            return null;
        }

        // 查询获取最新的文件
        FileInfo latestFile = files.OrderByDescending(f => f.LastWriteTime).FirstOrDefault();
        return latestFile.FullName;
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        // 当鼠标进入游戏对象时触发
        isMouseEnter = true;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        // 当鼠标离开游戏对象时触发
        isMouseEnter = false;
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if(eventData.button == PointerEventData.InputButton.Left)
        {
            Vector2 retPoint = Vector2.zero;
            float start_x = rawImage.uvRect.x * width;
            float start_y = rawImage.uvRect.y * height;

            Vector2 normalized = Rect.PointToNormalized(rawImage.rectTransform.rect, eventData.position);
            float offset_x = normalized.x * width * rawImage.uvRect.width;
            float offset_y = normalized.y * height * rawImage.uvRect.height;

            mouseDownPositon = new Vector2(start_x + offset_x, start_y + offset_y);
            mouseDownPositon_actual = new Vector2(mouseDownPositon.x, height - mouseDownPositon.y);
            Debug.Log($"【PGMFileReader】mouseDownPositon:{mouseDownPositon} mouseDownPositon_actual: {mouseDownPositon_actual}");
            DebugGUI.Log($"【PGMFileReader】mouseDownPositon:{mouseDownPositon} mouseDownPositon_actual: {mouseDownPositon_actual}");
        }
        if(eventData.button == PointerEventData.InputButton.Right)
        {
            mouseDownPositon = Vector2.zero;
            mouseDownPositon_actual = Vector2.zero;
        }

        List<Vector2> points = new List<Vector2>();
        points.Add(nowPosition);
        points.Add(mouseDownPositon);
        StartCoroutine(UpdatePostionShow(points, PointShowSize, true));
    }

    public IEnumerator UpdatePostionShow(List<Vector2> positions, int areaSize, bool isShow) {
        if (isShowPositionRunning)
        {
            Debug.Log("【PGMFileReader】UpdatePostionShow 当前正在更新点");
            yield break;
        }

        if (pgmTexture2D == null)
        {
            Debug.Log("【PGMFileReader】UpdatePostionShow pgmTexture2D == null");
            DebugGUI.Log("【PGMFileReader】UpdatePostionShow pgmTexture2D == null");
            yield break;
        }

        // 开始绘制点
        isShowPositionRunning = true;
        // 创建一个新的Texture2D对象，尺寸和格式与原始纹理相同
        if (pgmCarPositionTexture2D != null) {
            Destroy(pgmCarPositionTexture2D);
        }
        pgmCarPositionTexture2D = new Texture2D(pgmTexture2D.width, pgmTexture2D.height, pgmTexture2D.format, pgmTexture2D.mipmapCount > 1);
        pgmCarPositionTexture2D.SetPixels(pgmTexture2D.GetPixels());

        if (isShow)
        {
            // 设置红色区域
            Color[] redPixels = new Color[areaSize * areaSize];
            for (int i = 0; i < redPixels.Length; i++)
                redPixels[i] = Color.red;
            // 设置绿色区域
            Color[] greenPixels = new Color[areaSize * areaSize];
            for (int i = 0; i < greenPixels.Length; i++)
                greenPixels[i] = Color.green;
            // 设置蓝色区域
            Color[] bluePixels = new Color[areaSize * areaSize];
            for (int i = 0; i < bluePixels.Length; i++)
                bluePixels[i] = Color.blue;

            for (int i = 0; i < positions.Count; i++)
            {
                int px = (int)positions[i].x - areaSize / 2;
                int py = (int)positions[i].y - areaSize / 2;
                px = px < 0 ? 0 : px;
                py = py < 0 ? 0 : py;

                // 防止出界
                if ((int)positions[i].x + areaSize / 2 >= pgmTexture2D.width ||
                    (int)positions[i].y + areaSize / 2 >= pgmTexture2D.height)
                {
                    continue;
                }

                if (i == 0)
                {
                    pgmCarPositionTexture2D.SetPixels(px, py,
                        areaSize, areaSize, redPixels);
                }
                else if (i == positions.Count - 1)
                {
                    pgmCarPositionTexture2D.SetPixels(px, py,
                        areaSize, areaSize, bluePixels);
                }
                else 
                {
                    pgmCarPositionTexture2D.SetPixels(px, py,
                        areaSize, areaSize, greenPixels);
                }
            }

            pgmCarPositionTexture2D.Apply();
            rawImage.texture = pgmCarPositionTexture2D;
        }
        else {
            pgmCarPositionTexture2D.Apply();
            rawImage.texture = pgmCarPositionTexture2D;
        }

        isShowPositionRunning = false;
    }

    public void AstarPlan() 
    {
        if(nowPosition == Vector2.zero || mouseDownPositon == Vector2.zero)
        {
            return;
        }

        AStarAlgorithm.Astar astar = new AStarAlgorithm.Astar(pixels,
            width, height, AstarNodeSize, AstarObstacleGray);
        List<Vector2> pathPoints = astar.getPath(nowPosition, mouseDownPositon);
        if (pathPoints == null)
        {
            Debug.Log("【Astar】路径规划失败，可能终点不可达");
            DebugGUI.Log("【Astar】路径规划失败，可能终点不可达");
            return;
        }
        StartCoroutine(UpdatePostionShow(pathPoints, PointShowSize, true));
    }
}
