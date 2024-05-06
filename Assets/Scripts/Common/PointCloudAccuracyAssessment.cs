using MathNet.Numerics.Distributions;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

    // Start is called before the first frame update
    void Start()
    {
       
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
    public void CalLengthError(List<Vector3> selecPoints)
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

        // �����׼��
        float sumOfSquaresOfDifferences = 0;
        foreach (float dist in distances)
        {
            sumOfSquaresOfDifferences += (dist - averageDistance) * (dist - averageDistance);
        }
        float standardDeviation = Mathf.Sqrt(sumOfSquaresOfDifferences / distances.Count);

        Debug.Log($"�����ȡ�ƽ������: {averageDistance} ��׼��: {standardDeviation}" );
    }
}
