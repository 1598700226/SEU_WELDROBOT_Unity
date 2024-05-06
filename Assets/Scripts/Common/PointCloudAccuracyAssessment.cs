using MathNet.Numerics.Distributions;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointCloudAccuracyAssessment : MonoBehaviour
{
    public GameObject teachingOperate;

    List<Vector3> points;
    [Header("平面度误差")]
    public float planarityError = 0.0f;
    public bool isCalPlanarityError = false;

    [Header("长度均值")]
    public float LengthError_avg = 0.0f;
    [Header("长度标准差")]
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
    /// 平面度误差计算, 拟合平面并所有点到平面计算距离的均值
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

        // 计算标准差
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

        Debug.Log($"【平面度】平均距离: {planeQuality} 标准差: {standardDeviation} 公差带:{max_diff}-{min_diff}");
        return planeQuality;
    }

    /// <summary>
    /// 点与点之间误差计算
    /// </summary>
    public void CalLengthError(List<Vector3> selecPoints)
    {
        List<float> distances = new List<float>();

        // 计算所有两点之间的距离
        for (int i = 1; i < selecPoints.Count; i++)
        {
            float distance = Vector3.Distance(selecPoints[i - 1], selecPoints[i]);
            distances.Add(distance);
        }

        // 计算距离的平均值
        float averageDistance = 0;
        foreach (float dist in distances)
        {
            averageDistance += dist;
        }
        averageDistance /= distances.Count;

        // 计算标准差
        float sumOfSquaresOfDifferences = 0;
        foreach (float dist in distances)
        {
            sumOfSquaresOfDifferences += (dist - averageDistance) * (dist - averageDistance);
        }
        float standardDeviation = Mathf.Sqrt(sumOfSquaresOfDifferences / distances.Count);

        Debug.Log($"【长度】平均距离: {averageDistance} 标准差: {standardDeviation}" );
    }
}
