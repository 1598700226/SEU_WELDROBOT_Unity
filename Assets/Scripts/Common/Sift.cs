using Emgu.CV;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.VisualScripting;
using UnityEngine;
using Color = UnityEngine.Color;

public class Sift
{
    private static Image<Bgr, Byte> GetEmgucvImage(Color[] colors, int img_width, int img_height)
    {
        // 创建一个新的 Emgu CV 图像
        Image<Bgr, byte> emguImage = new Image<Bgr, byte>(img_width, img_height);

        // 填充 Emgu CV 图像
        for (int y = 0; y < img_height; y++)
        {
            for (int x = 0; x < img_width; x++)
            {
                // 获取 Unity 颜色数组中的颜色
                Color color = colors[y * img_width + x];

                // 将颜色值从 [0, 1] 范围转换为 [0, 255] 范围
                byte r = (byte)(color.r * 255);
                byte g = (byte)(color.g * 255);
                byte b = (byte)(color.b * 255);

                // 设置 Emgu CV 图像的像素值
                emguImage.Data[y, x, 0] = b; // 蓝色通道
                emguImage.Data[y, x, 1] = g; // 绿色通道
                emguImage.Data[y, x, 2] = r; // 红色通道
            }
        }

        return emguImage;
    }

    public static void SearchMatchPoint(Color[] colors1, Color[] colors2, int img_width, int img_height, int topLimit, 
        out List<Vector2> match_pt1, out List<Vector2> match_pt2)
    {
        Image<Bgr, Byte> originPic = GetEmgucvImage(colors1, img_width, img_height);
        Image<Bgr, Byte> deformPic = GetEmgucvImage(colors2, img_width, img_height);
        SIFT sift = new SIFT(100, 15, 0.04, 10, 2);
        //计算特征点
        MKeyPoint[] keyPoints1 = sift.Detect(originPic);
        MKeyPoint[] keyPoints2 = sift.Detect(deformPic);
        VectorOfKeyPoint vkeyPoint1 = new VectorOfKeyPoint(keyPoints1);
        VectorOfKeyPoint vkeyPoint2 = new VectorOfKeyPoint(keyPoints2);
        //计算特征描述符
        Mat descriptors1 = new Mat();
        Mat descriptors2 = new Mat();
        sift.Compute(originPic, vkeyPoint1, descriptors1);
        sift.Compute(deformPic, vkeyPoint2, descriptors2);
        //使用BF匹配器进行暴力匹配
        BFMatcher bFMatcher = new BFMatcher(DistanceType.L2);
        VectorOfVectorOfDMatch matches = new VectorOfVectorOfDMatch();
        //添加特征描述符
        bFMatcher.Add(descriptors1);
        //k最邻近匹配
        bFMatcher.KnnMatch(descriptors2, matches, 1, null);

        // 使用 LINQ 对所有匹配根据距离进行排序
        List<MDMatch> allMatches = new List<MDMatch>();
        for (int i = 0; i < matches.Size; i++)
        {
            allMatches.Add(matches[i][0]);
        }
        List<MDMatch> sortedMatches = allMatches.OrderBy(match => match.Distance).ToList();
        List<MDMatch> bestMatches = sortedMatches.Take(topLimit).ToList();
        //对BF匹配结果进行筛选
        VectorOfVectorOfDMatch good_matches = new VectorOfVectorOfDMatch();
        good_matches.Push(new VectorOfDMatch(bestMatches.ToArray()));

        // 剔除坏点之后，使用RANSAC进一步优化匹配
        Mat maskM = new Mat();
        List<PointF> pts1 = new List<PointF>();
        List<PointF> pts2 = new List<PointF>();
        for (int i = 0; i < good_matches[0].Size; i++)
        {
            if (good_matches[0][i].TrainIdx >= vkeyPoint2.Size || good_matches[0][i].QueryIdx >= vkeyPoint1.Size)
                continue;
            pts1.Add(vkeyPoint1[good_matches[0][i].QueryIdx].Point);
            pts2.Add(vkeyPoint2[good_matches[0][i].TrainIdx].Point);
        }
        // 计算单应性矩阵
        Mat homography = CvInvoke.FindHomography(pts2.ToArray(), pts1.ToArray(), Emgu.CV.CvEnum.RobustEstimationAlgorithm.Ransac, 10, maskM);
        // 使用mask过滤掉RANSAC认为的异常值
        VectorOfVectorOfDMatch goodMatches_Ransac = new VectorOfVectorOfDMatch();
        List<MDMatch> itemMatch = new List<MDMatch>();

        match_pt1 = new List<Vector2>();
        match_pt2 = new List<Vector2>();
        for (int i = 0; i < maskM.Rows; i++)
        {
            if (maskM.GetRawData(i)[0] > 0) // mask值大于0表示这个匹配是好的
            {
                match_pt1.Add(new Vector2(pts1[i].X, pts1[i].Y));
                match_pt2.Add(new Vector2(pts2[i].X, pts2[i].Y));
            }
        }
    }
}
