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
    public string pgmFileDir;   // PGM�ļ������ļ��е�·��
    public string pgmFilePath;  // PGM�ļ���·��
    public RawImage rawImage;   // ������ʾͼ���UI RawImage
    public TMP_Text lableTitle;     
    private Texture2D pgmTexture2D;
    private int updateCoroutinePixelLimit = 250000;     // Э�̸������ص�����������
    private bool isUpdatePgmTextureRunning = false;   // ��ǰ�Ƿ����ڸ���PGM��ͼ
    private Texture2D pgmCarPositionTexture2D;
    public bool isUpdatePGMMapOnce = true;
    public bool isReceviceRosMapTopic = false;
    public GameObject RosMapTopicsServer;
    public int AstarNodeSize = 20;
    public int AstarObstacleGray = 255;

    // PGMͼƬ����Ϣ
    string fileSuffix = ".pgm";
    int width = 0;
    int height = 0;
    int maxPixelValue = 0;
    int[,] pixels = null;                // ���У��С�
    float imageShow_width_scala = 0.0f;  //����ϵ�� ͼ������λ��*scala = ʵ��ͼ������λ��
    float imageShow_height_scala = 0.0f; //����ϵ�� ͼ������λ��*scala = ʵ��ͼ������λ��..

    // ������
    bool isMouseEnter = false;
    float currentZoom = 1.0f;
    float zoomSpeed = 3.0f;
    public Vector2 mouseDownPositon = Vector2.zero;         // unityͼƬ����ϵ�µ�Ŀ��㣬����ϵԭ�������½�
    public Vector2 mouseDownPositon_actual = Vector2.zero;  // ��ʵͼƬ����ϵ�����Ͻ�
    public bool isLocalMagnification = true;

    // ��λ����ʾ
    public int PointShowSize = 20; 
    public Vector2 nowPosition = Vector2.zero;                     // unityͼƬ����ϵ�µĵ�
    public Vector2 nowPosition_actual = Vector2.zero;
    private bool isShowPositionRunning = false;                    // �����������ʾ��Э��

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
                Debug.Log("��PGMFileReader������PGM��ͼ�ļ���" + updateFilePath);
                DebugGUI.Log("��PGMFileReader������PGM��ͼ�ļ���" + updateFilePath);
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
                    // �������ű���
                    currentZoom = Mathf.Clamp(currentZoom + scroll * zoomSpeed, 1f, 10f);

                    Vector2 point = MousePositionRelativeToImagePosition();
                    Vector2 normalized = new(point.x / width, point.y / height);

                    // ����uvRect���Ŵ�����
                    float size = 1f / currentZoom;
                    float x = Mathf.Clamp(normalized.x - size * 0.5f, 0f, 1f - size);
                    float y = Mathf.Clamp(normalized.y - size * 0.5f, 0f, 1f - size);
                    rawImage.uvRect = new Rect(x, y, size, size);
                }
            }
            else 
            {
                // ��ȡ�����ֵ�����
                float scroll = Input.GetAxis("Mouse ScrollWheel");
                if (scroll != 0)
                {
                    // ���ݹ����������RawImage��sizeDelta
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
                Debug.Log("��Astar��hasNewNowPositionReceive ·���滮ʧ�ܣ������յ㲻�ɴ�");
                DebugGUI.Log("��Astar��hasNewNowPositionReceive ·���滮ʧ�ܣ������յ㲻�ɴ�");
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
        // ����ļ��Ƿ����
        if (File.Exists(filePath))
        {
            // ��ȡPGM�ļ�����
            string[] lines = File.ReadAllLines(filePath);

            bool readingPixels = false;
            foreach (string line in lines)
            {
                if (readingPixels)
                {
                    // �ı���ʽ��ȡ����ֵ
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

                    // �����Ʒ�ʽ��ȡ
                    using (FileStream fileStream = new FileStream(filePath, FileMode.Open))
                    {
                        using (BinaryReader binaryReader = new BinaryReader(fileStream)) 
                        {
                            byte[] binaryData = binaryReader.ReadBytes((int)fileStream.Length);
                            int header_line_num = 0;
                            int header_line_max = 4; // ͷ��Ϣ������
                            int start_index = 0;
                            // �ҵ��������ڵ�byte����
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
                    // ����ע����
                }
                else if (line.StartsWith("P2") || line.StartsWith("P5"))
                {
                    // ��ȡ�ļ���ʽ��ʶ����P2��P5��
                }
                else if (line.Contains(" "))
                {
                    // ��ȡ��Ⱥ͸߶�
                    string[] dimensions = line.Split(' ');
                    width = int.Parse(dimensions[0]);
                    height = int.Parse(dimensions[1]);
                    pixels = new int[height, width];
                }
                else
                {
                    // ��ȡ�������ֵ
                    maxPixelValue = int.Parse(line);
                    readingPixels = true;
                }
            }
            return true;
        }
        else
        {
            Debug.Log("��PGMFileReader��PGM�ļ������ڡ�" + filePath);
            DebugGUI.Log("��PGMFileReader��PGM�ļ������ڡ�" + filePath);
            return false;
        }
    }

    private IEnumerator UpdateRawImage(int img_width, int img_height, int[,] data, int maxPixelValue) {
        if (rawImage == null)
        {
            Debug.Log("��PGMFileReader��rawImage�ؼ�Ϊ��");
            DebugGUI.Log("��PGMFileReader��rawImage�ؼ�Ϊ��");
            yield break;
        }
        if (isUpdatePgmTextureRunning)
        {
            Debug.Log("��PGMFileReader��UpdateRawImage ��ǰ���ڸ��µ�ͼ");
            DebugGUI.Log("��PGMFileReader��UpdateRawImage ��ǰ���ڸ��µ�ͼ");
            yield break;
        }

        isUpdatePgmTextureRunning = true;
        // ����һ��Texture2D��������������
        // ���پɵ�Texture2D�Է�ֹ�ڴ�й©
        if (pgmTexture2D != null)
        {
            Destroy(pgmTexture2D);
        }
        pgmTexture2D = new Texture2D(img_width, img_height);
        // ������������䵽Texture2D
        int count = 0;
        for (int y = 0; y < img_height; y++)
        {
            for (int x = 0; x < img_width; x++)
            {
                float normalizedValue = (float)data[y, x] / (float)maxPixelValue;
                Color color = new Color(1 - normalizedValue, 1 - normalizedValue, 1 - normalizedValue);
                pgmTexture2D.SetPixel(x, img_height - 1 - y, color);  // ��Ҫ��תy����ƥ��Unity����ϵ
                count++;
                if(count % updateCoroutinePixelLimit == 0)
                {
                    yield return null;
                }
            }
        }

        // ��Texture2D��ʾ��RawImage��
        pgmTexture2D.Apply();  // Ӧ�����ظ���
        rawImage.texture = pgmTexture2D;
        // ����ͼ��ؼ��Ĵ�С����ȡ���ű���
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
            Debug.Log("��getLatestPGMFile���ļ��в����� dirPath:" + dirPath);
            return null;
        }

        DirectoryInfo directoryInfo = new DirectoryInfo(dirPath);
        FileInfo[] files = directoryInfo.GetFiles("*" + fileSuffix);
        if (files == null || files.Length <= 0)
        {
            Debug.Log("��getLatestPGMFile���ļ����²�����PGM�ļ�:" + dirPath);
            return null;
        }

        // ��ѯ��ȡ���µ��ļ�
        FileInfo latestFile = files.OrderByDescending(f => f.LastWriteTime).FirstOrDefault();
        return latestFile.FullName;
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        // ����������Ϸ����ʱ����
        isMouseEnter = true;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        // ������뿪��Ϸ����ʱ����
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
            Debug.Log($"��PGMFileReader��mouseDownPositon:{mouseDownPositon} mouseDownPositon_actual: {mouseDownPositon_actual}");
            DebugGUI.Log($"��PGMFileReader��mouseDownPositon:{mouseDownPositon} mouseDownPositon_actual: {mouseDownPositon_actual}");
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
            Debug.Log("��PGMFileReader��UpdatePostionShow ��ǰ���ڸ��µ�");
            yield break;
        }

        if (pgmTexture2D == null)
        {
            Debug.Log("��PGMFileReader��UpdatePostionShow pgmTexture2D == null");
            DebugGUI.Log("��PGMFileReader��UpdatePostionShow pgmTexture2D == null");
            yield break;
        }

        // ��ʼ���Ƶ�
        isShowPositionRunning = true;
        // ����һ���µ�Texture2D���󣬳ߴ�͸�ʽ��ԭʼ������ͬ
        if (pgmCarPositionTexture2D != null) {
            Destroy(pgmCarPositionTexture2D);
        }
        pgmCarPositionTexture2D = new Texture2D(pgmTexture2D.width, pgmTexture2D.height, pgmTexture2D.format, pgmTexture2D.mipmapCount > 1);
        pgmCarPositionTexture2D.SetPixels(pgmTexture2D.GetPixels());

        if (isShow)
        {
            // ���ú�ɫ����
            Color[] redPixels = new Color[areaSize * areaSize];
            for (int i = 0; i < redPixels.Length; i++)
                redPixels[i] = Color.red;
            // ������ɫ����
            Color[] greenPixels = new Color[areaSize * areaSize];
            for (int i = 0; i < greenPixels.Length; i++)
                greenPixels[i] = Color.green;
            // ������ɫ����
            Color[] bluePixels = new Color[areaSize * areaSize];
            for (int i = 0; i < bluePixels.Length; i++)
                bluePixels[i] = Color.blue;

            for (int i = 0; i < positions.Count; i++)
            {
                int px = (int)positions[i].x - areaSize / 2;
                int py = (int)positions[i].y - areaSize / 2;
                px = px < 0 ? 0 : px;
                py = py < 0 ? 0 : py;

                // ��ֹ����
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
            Debug.Log("��Astar��·���滮ʧ�ܣ������յ㲻�ɴ�");
            DebugGUI.Log("��Astar��·���滮ʧ�ܣ������յ㲻�ɴ�");
            return;
        }
        StartCoroutine(UpdatePostionShow(pathPoints, PointShowSize, true));
    }
}
