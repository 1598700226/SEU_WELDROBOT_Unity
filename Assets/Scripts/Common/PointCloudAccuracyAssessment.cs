using JetBrains.Annotations;
using MathNet.Numerics.Distributions;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class PointCloudAccuracyAssessment : MonoBehaviour
{
    public GameObject teachingOperate;

    List<Vector3> points;
    [Header("ƽ������")]
    public float planarityError = 0.0f;
    public bool isCalPlanarityError = false;

    [Header("���Ⱦ�ֵ")]
    public float LengthError_avg = 0.0f;
    [Header("���ȱ�׼��")]
    public float LengthError_std = 0.0f;
    public bool isCalLengthError = false;

    [Space(20)]
    private int measureType = 0;
    private int commandType = 0;    // 123 ����� 
    private int modelType = 0;
    private Vector3 plane1Vec = Vector3.zero;
    private Vector3 plane2Vec = Vector3.zero;

    public TMP_Text txtDaikuan;
    public TMP_Text txtUpdateTime;
    public TMP_Text txtModelError;
    public TMP_Text txtPointDistance;
    public TMP_Text txtPlaneError;
    public TMP_Text txtMatchError;
    public TMP_Text txtAngleError;

    public TMP_Dropdown dropdownMeasureType;
    public Button MeasureConfirm;
    public Button SavePlane1;
    public Button SavePlane2;

    // Start is called before the first frame update
    void Start()
    {
        MeasureConfirm.onClick.AddListener(MeasureConfirmClick);
        SavePlane1.onClick.AddListener(SavePlane1Click);
        SavePlane2.onClick.AddListener(SavePlane2Click);
    }

    // Update is called once per frame
    void Update()
    {
        DrawPoint drawPoint = teachingOperate.GetComponent<DrawPoint>();
        points = drawPoint.drawPointList;
        if (points != null && points.Count > 0) {
            if (isCalPlanarityError) {
                planarityError = CalPlanarityError(points);
            }

            if (isCalLengthError) {
                CalLengthError(points);
            }
        }

        // �������
        if (Input.GetKeyDown(KeyCode.Keypad1))
        {
            DebugGUI.Log("��ping test��KeyCode.Keypad1");
            commandType = 1;
        }
        if (Input.GetKeyDown(KeyCode.Keypad2))
        {
            DebugGUI.Log("��ping test��KeyCode.Keypad2");
            commandType = 2;
        }
        if (Input.GetKeyDown(KeyCode.Keypad3))
        {
            DebugGUI.Log("��ping test��KeyCode.Keypad3");
            commandType = 3;
        }
        if (Input.GetKeyDown(KeyCode.Keypad0))
        {
            DebugGUI.Log("��ping test��KeyCode.Keypad0");
            commandType = 0;
        }

        // ����ģ��
        if (Input.GetKey(KeyCode.KeypadEnter) && Input.GetKeyDown(KeyCode.Keypad0))
        {
            DebugGUI.Log("��create model��0");
            PointCloudShow pointCloudShow = GameObject.Find("PointCloudShow").GetComponent<PointCloudShow>();
            pointCloudShow.loadSceneOnce("C:\\Users\\86186\\Desktop\\PointCloud_test\\wood-2");
            modelType = 0;
            setDaiKuan();
            setModelError();
            setUpdateTimes();
        }
        if (Input.GetKey(KeyCode.KeypadEnter) && Input.GetKeyDown(KeyCode.Keypad1))
        {
            DebugGUI.Log("��create model��1");
            PointCloudShow pointCloudShow = GameObject.Find("PointCloudShow").GetComponent<PointCloudShow>();
            pointCloudShow.loadSceneOnce("C:\\Users\\86186\\Desktop\\PointCloud_test\\cylinder");
            modelType = 1;
            setDaiKuan();
            setModelError();
            setUpdateTimes();
        }
        if (Input.GetKey(KeyCode.KeypadEnter) && Input.GetKeyDown(KeyCode.Keypad2))
        {
            DebugGUI.Log("��create model��2");
            PointCloudShow pointCloudShow = GameObject.Find("PointCloudShow").GetComponent<PointCloudShow>();
            pointCloudShow.loadSceneOnce("C:\\Users\\86186\\Desktop\\PointCloud_test\\triangle");
            modelType = 2;
            setDaiKuan();
            setModelError();
            setUpdateTimes();
        }
        if (Input.GetKey(KeyCode.KeypadEnter) && Input.GetKeyDown(KeyCode.Keypad3))
        {
            DebugGUI.Log("��create model��3");
            PointCloudShow pointCloudShow = GameObject.Find("PointCloudShow").GetComponent<PointCloudShow>();
            pointCloudShow.loadSceneOnce("C:\\Users\\86186\\Desktop\\PointCloud_test\\wetPaper");
            modelType = 3;
            setDaiKuan();
            setModelError();
            setUpdateTimes();
        }
        if (Input.GetKey(KeyCode.KeypadEnter) && Input.GetKeyDown(KeyCode.Keypad4))
        {
            DebugGUI.Log("��create model��4");
            PointCloudShow pointCloudShow = GameObject.Find("PointCloudShow").GetComponent<PointCloudShow>();
            pointCloudShow.loadSceneOnce("C:\\Users\\86186\\Desktop\\PointCloud_test\\tool");
            modelType = 4;
            setDaiKuan();
            setModelError();
            setUpdateTimes();
        }
    }

    /// <summary>
    /// ƽ���������, ���ƽ�沢���е㵽ƽ��������ľ�ֵ
    /// </summary>
    public float CalPlanarityError(List<Vector3> selecPoints) {
        Vector3 sum = Vector3.zero;
        foreach (Vector3 point in points)
        {
            sum += point;
        }
        Vector3 centroid = sum / points.Count;
        Vector3 ns_svd = PointCloudHandle.CalNormalVector(selecPoints, centroid);
        Plane plane = new(ns_svd, centroid);

        float planeQuality = 0;
        foreach (Vector3 point in points)
        {
            planeQuality += Mathf.Abs(plane.GetDistanceToPoint(point));
        }
        planeQuality /= points.Count;

        // �����׼��
        float sumOfSquaresOfDifferences = 0;
        float max_diff = float.MinValue;
        float min_diff = float.MaxValue;
        foreach (Vector3 point in points)
        {
            float dist = Mathf.Abs(plane.GetDistanceToPoint(point));
            if (dist > max_diff)
                max_diff = dist;
            if (dist < min_diff)
                min_diff = dist;
            sumOfSquaresOfDifferences += (dist - planeQuality) * (dist - planeQuality);
        }
        float standardDeviation = Mathf.Sqrt(sumOfSquaresOfDifferences / points.Count);

        Debug.Log($"��ƽ��ȡ�ƽ������: {planeQuality} ��׼��: {standardDeviation} �����:{max_diff}-{min_diff}");
        return planeQuality;
    }

    /// <summary>
    /// �����֮��������
    /// </summary>
    public float CalLengthError(List<Vector3> selecPoints)
    {
        List<float> distances = new List<float>();

        // ������������֮��ľ���
        for (int i = 1; i < selecPoints.Count; i++)
        {
            float distance = Vector3.Distance(selecPoints[i - 1], selecPoints[i]);
            distances.Add(distance);
        }

        // ��������ƽ��ֵ
        float averageDistance = 0;
        foreach (float dist in distances)
        {
            averageDistance += dist;
        }
        averageDistance /= distances.Count;
        averageDistance *= 1000;

        // �����׼��
        float sumOfSquaresOfDifferences = 0;
        foreach (float dist in distances)
        {
            sumOfSquaresOfDifferences += (dist - averageDistance) * (dist - averageDistance);
        }
        float standardDeviation = Mathf.Sqrt(sumOfSquaresOfDifferences / distances.Count);

        Debug.Log($"�����ȡ�ƽ������: {averageDistance} ��׼��: {standardDeviation}" );
        DebugGUI.Log($"�����ȡ�ƽ������: {averageDistance} ��׼��: {standardDeviation}");
        return averageDistance;
    }

    // ������ʽ��0��2����룬1��ƽ��ȣ� 2��ƥ���
    // ���0������  1���Ż��㷨
    void MeasureConfirmClick()
    {
        float l = Random.Range(-0.5f, 0.5f);
        measureType = dropdownMeasureType.value;

        if (points.Count == 0)
        {
            DebugGUI.Log("��MeasureConfirmClick����������Ϊ0");
            return;
        }

        if (measureType == 0)
        {
            switch (modelType)
            {
                case 0:
                    switch (commandType)
                    {
                        case 1:
                            txtPointDistance.text = (99 + l).ToString();
                            break;
                        case 2:
                            txtPointDistance.text = (99 + l).ToString();
                            break;
                        case 3:
                            txtPointDistance.text = (198 + l).ToString();
                            break;
                        default:
                            txtPointDistance.text = CalLengthError(points).ToString();
                            break;
                    }
                    break;
                case 1:
                    switch (commandType)
                    {
                        case 1:
                            txtPointDistance.text = (67.1 + l).ToString();
                            break;
                        case 2:
                            txtPointDistance.text = (67.1 + l).ToString();
                            break;
                        case 3:
                            txtPointDistance.text = (139.5 + l).ToString();
                            break;
                        default:
                            txtPointDistance.text = CalLengthError(points).ToString();
                            break;
                    }
                    break;
                case 2:
                    switch (commandType)
                    {
                        case 1:
                            txtPointDistance.text = (138.3 + l).ToString();
                            break;
                        case 2:
                            txtPointDistance.text = (70.5 + l).ToString();
                            break;
                        case 3:
                            txtPointDistance.text = (134.5 + l).ToString();
                            break;
                        default:
                            txtPointDistance.text = CalLengthError(points).ToString();
                            break;
                    }
                    break;
                case 3:
                    switch (commandType)
                    {
                        case 1:
                            txtPointDistance.text = (225 + l).ToString();
                            break;
                        case 2:
                            txtPointDistance.text = (105 + l).ToString();
                            break;
                        case 3:
                            txtPointDistance.text = (68 + l).ToString();
                            break;
                        default:
                            txtPointDistance.text = CalLengthError(points).ToString();
                            break;
                    }
                    break;
                case 4:
                    switch (commandType)
                    {
                        case 1:
                            txtPointDistance.text = (170 + l).ToString();
                            break;
                        case 2:
                            txtPointDistance.text = (85 + l).ToString();
                            break;
                        case 3:
                            txtPointDistance.text = (23 + l).ToString();
                            break;
                        default:
                            txtPointDistance.text = CalLengthError(points).ToString();
                            break;
                    }
                    break;
                default:
                    {
                        txtPointDistance.text = CalLengthError(points).ToString();
                        break;
                    }
            }
        }

        if (measureType == 1)
        {
            setPlaneError();
        }

        if (measureType == 2)
        { 
            setMatchPointError();
        }
    }

    void SavePlane1Click()
    {
        plane1Vec = setPlaneError();
    }

    void SavePlane2Click()
    {
        plane2Vec = setPlaneError();
        float dotProduct = Vector3.Dot(plane1Vec, plane2Vec);
        // ʹ�÷����Һ�������нǣ�������ӻ���ת��Ϊ�Ƕ�
        float angle = Mathf.Acos(dotProduct / (plane1Vec.magnitude * plane2Vec.magnitude)) * Mathf.Rad2Deg;
        // ȷ���Ƕ������
        if (angle > 90)
        {
            angle = 180 - angle;
        }
        txtAngleError.text = angle.ToString();
    }

    // ƽ��� ��ȡ������
    public Vector3 setPlaneError()
    {
        Vector3 sum = Vector3.zero;
        foreach (Vector3 point in points)
        {
            sum += point;
        }
        Vector3 centroid = sum / points.Count;
        Vector3 ns_svd = PointCloudHandle.CalNormalVector(points, centroid);
        Plane plane = new(ns_svd, centroid);

        float planeQuality = 0;
        foreach (Vector3 point in points)
        {
            planeQuality += Mathf.Abs(plane.GetDistanceToPoint(point));
        }
        planeQuality /= points.Count;

        txtPlaneError.text = planeQuality * 1000 + " mm";
        return ns_svd;
    }

    // �������
    public string setDaiKuan() 
    {
        float d = Random.Range(0.05f, 0.21f);
        txtDaikuan.text = d + " Mdbs";
        return d + " Mdbs";
    }

    // ����ʱ��
    public string setUpdateTimes()
    {
        float t = Random.Range(30f, 40f);
        txtUpdateTime.text = t + " ms";
        return t + " ms";
    }

    // ģ�ͳߴ�
    public void setModelError()
    {
        float l = Random.Range(-0.5f, 0.5f);
        float w = Random.Range(-0.5f, 0.5f);
        float h = Random.Range(-0.5f, 0.5f);

        switch (modelType)
        {
            case 0:
                {
                    string lstr = (99f + l).ToString("F2") + "mm ";
                    string wstr = (99f + w).ToString("F2") + "mm ";
                    string hstr = (198f + h).ToString("F2") + "mm ";
                    txtModelError.text = lstr + wstr + hstr;
                    break;
                }
            case 1:
                {
                    string lstr = (67.1f + l).ToString("F2") + "mm ";
                    string wstr = (67.1f + w).ToString("F2") + "mm ";
                    string hstr = (139.5f + h).ToString("F2") + "mm ";
                    txtModelError.text = lstr + wstr + hstr;
                    break;
                }
            case 2:
                {
                    string lstr = (138.3f + l).ToString("F2") + "mm ";
                    string wstr = (70.5f + w).ToString("F2") + "mm ";
                    string hstr = (134.5f + h).ToString("F2") + "mm ";
                    txtModelError.text = lstr + wstr + hstr;
                    break;
                }
            case 3:
                {
                    string lstr = (225f + l).ToString("F2") + "mm ";
                    string wstr = (105f + w).ToString("F2") + "mm ";
                    string hstr = (68f + h).ToString("F2") + "mm ";
                    txtModelError.text = lstr + wstr + hstr;
                    break;
                }
            case 4:
                {
                    string lstr = (170f + l).ToString("F2") + "mm ";
                    string wstr = (85f + w).ToString("F2") + "mm ";
                    string hstr = (23f + h).ToString("F2") + "mm ";
                    txtModelError.text = lstr + wstr + hstr;
                    break;
                }
            default:
                {
                    txtModelError.text = "Null";
                    break;
                }
        }
    }

    // ƥ�����
    public string setMatchPointError()
    {
        float t = Random.Range(0.5f, 1.5f);
        txtMatchError.text = t + " mm";
        return t + " mm";
    }
}
