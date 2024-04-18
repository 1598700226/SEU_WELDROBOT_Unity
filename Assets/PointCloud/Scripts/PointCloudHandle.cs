using DataStructures.ViliWonka.KDTree;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class PointCloudHandle
{
    static Matrix<double> GeneratePointCloudByWorldPosition(List<Vector3> pointClouds)
    {
        Matrix<double> pointCloud = Matrix<double>.Build.Dense(pointClouds.Count, 3);
        for (int i = 0; i < pointClouds.Count; i++)
        {
            pointCloud[i, 0] = pointClouds[i].x;
            pointCloud[i, 1] = pointClouds[i].y;
            pointCloud[i, 2] = pointClouds[i].z;
        }
        return pointCloud;
    }

    static Vector<double> CalculateCentroid(Matrix<double> pointCloud)
    {
        int numPoints = pointCloud.RowCount;
        Vector<double> centroid = Vector<double>.Build.Dense(3);
        for (int i = 0; i < numPoints; i++)
        {
            centroid += pointCloud.Row(i);
        }
        centroid /= numPoints;
        return centroid;
    }

    static Matrix<double> CenterPointCloud(Matrix<double> pointCloud, Vector<double> centroid)
    {
        int numPoints = pointCloud.RowCount;
        Matrix<double> centeredPointCloud = Matrix<double>.Build.Dense(numPoints, 3);
        for (int i = 0; i < numPoints; i++)
        {
            centeredPointCloud.SetRow(i, pointCloud.Row(i) - centroid);
        }
        return centeredPointCloud;
    }

    static Matrix<double> CalculateCrossCovariance(Matrix<double> sourcePointCloud, Matrix<double> targetPointCloud)
    {
        int numPoints = sourcePointCloud.RowCount;
        Matrix<double> crossCovariance = Matrix<double>.Build.Dense(3, 3);
        for (int i = 0; i < numPoints; i++)
        {
            crossCovariance += sourcePointCloud.Row(i).ToColumnMatrix() * targetPointCloud.Row(i).ToRowMatrix();
        }
        return crossCovariance;
    }

    // 计算点云中某一点的法向量——SVD
    public static Vector3 CalNormalVector(List<Vector3> points, Vector3 point)
    {
        Vector3 n = new(0, 0, 0);

        // 生成用于运算的点云
        Matrix<double> nearPoints = GeneratePointCloudByWorldPosition(points);

        // 计算中心点
        Vector<double> centroid = CalculateCentroid(nearPoints);

        // 每个点云点与中心点做差
        Matrix<double> centeredSourceCloud = CenterPointCloud(nearPoints, centroid);

        // 计算协方差矩阵
        Matrix<double> crossCovariance = CalculateCrossCovariance(centeredSourceCloud, centeredSourceCloud);

        // SVD
        Svd<Double> svd = crossCovariance.Svd();

        // 判断法线方向
        Vector<double> ns = svd.U.Column(2);
        Vector<double> nsc = Vector<double>.Build.Dense(3);
        nsc[0] = point.x - centroid[0];
        nsc[1] = point.y - centroid[1];
        nsc[2] = point.z - centroid[2];
/*        nsc[0] = -point.x;
        nsc[1] = -point.y;
        nsc[2] = -point.z;*/
        nsc = nsc.Normalize(1);
        double angle = Math.Round(ns.DotProduct(nsc), 6);
        // 法向量改为和视角的方向一直
        if (angle > 0)
        {
            ns = -ns;
        }
        n.x = (float)Math.Round(ns[0], 6);
        n.y = (float)Math.Round(ns[1], 6);
        n.z = (float)Math.Round(ns[2], 6);
        return n;
    }

    // 计算点云的法向量——PCA
    public static Vector3 CalNormalVectorPCA(List<Vector3> points) {
        Vector3 sum = Vector3.zero;
        foreach (Vector3 point in points)
        {
            sum += point;
        }
        Vector3 centroid = sum / points.Count;

        // 计算协方差矩阵
        float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        foreach (Vector3 point in points)
        {
            Vector3 r = point - centroid;
            xx += r.x * r.x;
            xy += r.x * r.y;
            xz += r.x * r.z;
            yy += r.y * r.y;
            yz += r.y * r.z;
            zz += r.z * r.z;
        }
        float det_x = yy * zz - yz * yz;
        float det_y = xx * zz - xz * xz;
        float det_z = xx * yy - xy * xy;
        float det_max = Mathf.Max(det_x, det_y, det_z);
        if (det_max <= 0) 
            return Vector3.zero; // 点集不足以构成平面

        // 主成分分析
        Vector3 normal;
        if (det_max == det_x)
        {
            normal = new Vector3(det_x, xz * yz - xy * zz, xy * yz - xz * yy);
        }
        else if (det_max == det_y)
        {
            normal = new Vector3(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
        }
        else
        {
            normal = new Vector3(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
        }
        normal.Normalize();
        return normal;
    }

    public static IEnumerator GetPointsNormalsByKnum(Vector3[] points, KDTree kdTree, int k_num, Action<Vector3[]> callback)
    {
        long nowTime = ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds();

        Vector3[] normals = new Vector3[points.Length];
        KDQuery query = new KDQuery();
        for (int i = 0; i < points.Length; i++) {
            Vector3 queryPoint = points[i];
            List<int> ret = new List<int>();
            query.KNearest(kdTree, queryPoint, k_num, ret);
            List<Vector3> nearPoints = ret.Select(index => points[index]).ToList();
            normals[i] = CalNormalVector(nearPoints, queryPoint);

            if (i % 1000 == 0) {
                yield return null;
            }
        }
        callback?.Invoke(normals);

        long endTime = ((DateTimeOffset)DateTime.Now).ToUnixTimeMilliseconds();
        Debug.Log("法线计算完成 cost time:" + (endTime - nowTime) + "ms");
    }

    public static void RegisterPointCloud(List<Vector3> sourceClouds, List<Vector3> targetClouds, 
        out Matrix<double> rotation, out Vector<double> translation)
    {
        // 生成用于运算的点云
        Matrix<double> sourceCloud = GeneratePointCloudByWorldPosition(sourceClouds);
        Matrix<double> targetCloud = GeneratePointCloudByWorldPosition(targetClouds);

        // 计算中心点
        Vector<double> sourceCentroid = CalculateCentroid(sourceCloud);
        Vector<double> targetCentroid = CalculateCentroid(targetCloud);

        // 每个点云点与中心点做差
        Matrix<double> centeredSourceCloud = CenterPointCloud(sourceCloud, sourceCentroid);
        Matrix<double> centeredTargetCloud = CenterPointCloud(targetCloud, targetCentroid);

        // 计算协方差矩阵
        Matrix<double> crossCovariance = CalculateCrossCovariance(centeredSourceCloud, centeredTargetCloud);

        // SVD
        Svd<double> svd = crossCovariance.Svd();

        // 旋转矩阵
        Matrix<double> E = Matrix<double>.Build.DenseIdentity(3);
        Matrix<double> rotationMatrix = svd.VT.Transpose() * E * svd.U.Transpose();
        double det_R = rotationMatrix.Determinant();
        // 缩放
        Matrix<double> s = svd.W;
        // 确保R是一个旋转矩阵：行列式应该为1。
        if (det_R < 0)
        {
            // 如果行列式为-1，将Σ中最小的值取反，然后重新计算R。
            //s[s.RowCount - 1, s.ColumnCount - 1] *= -1;
            E[E.RowCount - 1, E.ColumnCount - 1] *= -1;
            rotationMatrix = svd.VT.Transpose() * E * svd.U.Transpose();
        }

        // 平移矩阵
        Vector<double> translationVector = targetCentroid - rotationMatrix * sourceCentroid;

        Debug.Log($"Rotation Matrix before det_R: {det_R} after det_R:{rotationMatrix.Determinant()}");
        rotation = rotationMatrix;
        translation = translationVector;
    }

    public static Matrix4x4 BuildMatrix4x4ByRT(Matrix<double> rotation, Vector<double> translation) {
        Matrix4x4 matrix = new Matrix4x4(
            new Vector4((float)rotation.At(0, 0), (float)rotation.At(1, 0), (float)rotation.At(2, 0), 0),
            new Vector4((float)rotation.At(0, 1), (float)rotation.At(1, 1), (float)rotation.At(2, 1), 0),
            new Vector4((float)rotation.At(0, 2), (float)rotation.At(1, 2), (float)rotation.At(2, 2), 0),
            new Vector4((float)translation.At(0), (float)translation.At(1), (float)translation.At(2), 1)
            );
        return matrix;
    }

    public static Matrix4x4 BuildMatrix4x4ByRT(Emgu.CV.Matrix<double> rotation, Emgu.CV.Matrix<double> translation)
    {
        Matrix4x4 matrix = new Matrix4x4(
            new Vector4((float)rotation.Data[0, 0], (float)rotation.Data[1, 0], (float)rotation.Data[2, 0], 0),
            new Vector4((float)rotation.Data[0, 1], (float)rotation.Data[1, 1], (float)rotation.Data[2, 1], 0),
            new Vector4((float)rotation.Data[0, 2], (float)rotation.Data[1, 2], (float)rotation.Data[2, 2], 0),
            new Vector4((float)translation.Data[0, 0], (float)translation.Data[1, 0], (float)translation.Data[2, 0], 1)
            );
        return matrix;
    }
}
