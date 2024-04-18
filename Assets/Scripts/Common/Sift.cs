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
        // ����һ���µ� Emgu CV ͼ��
        Image<Bgr, byte> emguImage = new Image<Bgr, byte>(img_width, img_height);

        // ��� Emgu CV ͼ��
        for (int y = 0; y < img_height; y++)
        {
            for (int x = 0; x < img_width; x++)
            {
                // ��ȡ Unity ��ɫ�����е���ɫ
                Color color = colors[y * img_width + x];

                // ����ɫֵ�� [0, 1] ��Χת��Ϊ [0, 255] ��Χ
                byte r = (byte)(color.r * 255);
                byte g = (byte)(color.g * 255);
                byte b = (byte)(color.b * 255);

                // ���� Emgu CV ͼ�������ֵ
                emguImage.Data[y, x, 0] = b; // ��ɫͨ��
                emguImage.Data[y, x, 1] = g; // ��ɫͨ��
                emguImage.Data[y, x, 2] = r; // ��ɫͨ��
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
        //����������
        MKeyPoint[] keyPoints1 = sift.Detect(originPic);
        MKeyPoint[] keyPoints2 = sift.Detect(deformPic);
        VectorOfKeyPoint vkeyPoint1 = new VectorOfKeyPoint(keyPoints1);
        VectorOfKeyPoint vkeyPoint2 = new VectorOfKeyPoint(keyPoints2);
        //��������������
        Mat descriptors1 = new Mat();
        Mat descriptors2 = new Mat();
        sift.Compute(originPic, vkeyPoint1, descriptors1);
        sift.Compute(deformPic, vkeyPoint2, descriptors2);
        //ʹ��BFƥ�������б���ƥ��
        BFMatcher bFMatcher = new BFMatcher(DistanceType.L2);
        VectorOfVectorOfDMatch matches = new VectorOfVectorOfDMatch();
        //�������������
        bFMatcher.Add(descriptors1);
        //k���ڽ�ƥ��
        bFMatcher.KnnMatch(descriptors2, matches, 1, null);

        // ʹ�� LINQ ������ƥ����ݾ����������
        List<MDMatch> allMatches = new List<MDMatch>();
        for (int i = 0; i < matches.Size; i++)
        {
            allMatches.Add(matches[i][0]);
        }
        List<MDMatch> sortedMatches = allMatches.OrderBy(match => match.Distance).ToList();
        List<MDMatch> bestMatches = sortedMatches.Take(topLimit).ToList();
        //��BFƥ��������ɸѡ
        VectorOfVectorOfDMatch good_matches = new VectorOfVectorOfDMatch();
        good_matches.Push(new VectorOfDMatch(bestMatches.ToArray()));

        // �޳�����֮��ʹ��RANSAC��һ���Ż�ƥ��
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
        // ���㵥Ӧ�Ծ���
        Mat homography = CvInvoke.FindHomography(pts2.ToArray(), pts1.ToArray(), Emgu.CV.CvEnum.RobustEstimationAlgorithm.Ransac, 10, maskM);
        // ʹ��mask���˵�RANSAC��Ϊ���쳣ֵ
        VectorOfVectorOfDMatch goodMatches_Ransac = new VectorOfVectorOfDMatch();
        List<MDMatch> itemMatch = new List<MDMatch>();

        match_pt1 = new List<Vector2>();
        match_pt2 = new List<Vector2>();
        for (int i = 0; i < maskM.Rows; i++)
        {
            if (maskM.GetRawData(i)[0] > 0) // maskֵ����0��ʾ���ƥ���Ǻõ�
            {
                match_pt1.Add(new Vector2(pts1[i].X, pts1[i].Y));
                match_pt2.Add(new Vector2(pts2[i].X, pts2[i].Y));
            }
        }
    }
}
