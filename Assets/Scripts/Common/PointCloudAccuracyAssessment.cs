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
    [Header("长度误差")]
    public float LengthError = 0.0f;
    public bool isCalLength = false;

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
        return planeQuality;
    }

/*    /// <summary>
    /// 边长误差计算
    /// </summary>
    public float CalLengthError(List<Vector3> selecPoints) { 
        
    }*/
}
