using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class RayTest : MonoBehaviour
{
    /*******************�������********************/
    private Ray ray;
    private RaycastHit hit;
    public static Vector3 hitPoint;

    /********************ʾ�����*********************/
    public GameObject teachingOperate;
    public bool isTeachingOperateOpen = false;  // ʾ�̹����Ƿ���
    public enum TeachingOperateMode
    {
        Line,   //  ��
        Circle, //  Բ
        Curve,  //  ������
        Point   //  ��
    }
    public TeachingOperateMode teachingOperateMode; // ʾ��ģʽ
    public float teachingOperateParam; // ʾ�̲���

    void Start()
    {
        Debug.Log("�������߼��");
        hitPoint = new Vector3();
        teachingOperateMode = TeachingOperateMode.Line;
    }

    // Update is called once per frame
    void Update()
    {
        // �������
        if (Input.GetMouseButtonDown(0))
        {
            Physics.queriesHitBackfaces = true;

            // �ж��Ƿ���UI��
            if (EventSystem.current.IsPointerOverGameObject())
            {
                return;
            }

            ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out hit)) 
            {
                Debug.Log("λ�ã�" + hit.point + "name:" + hit.collider.gameObject.name);
                hitPoint = hit.point;

                if (isTeachingOperateOpen && teachingOperate != null)
                {
                    DrawPoint drawPoint = teachingOperate.GetComponent<DrawPoint>();
                    drawPoint.drawParam = teachingOperateParam;
                    switch (teachingOperateMode)
                    {
                        case TeachingOperateMode.Line:
                            drawPoint.drawMode = DrawPoint.DrawMode.Line;
                            drawPoint.drawPointList.Add(hitPoint);
                            drawPoint.needUpdate = true;
                            break;
                        case TeachingOperateMode.Circle:
                            drawPoint.drawMode = DrawPoint.DrawMode.Circle;
                            drawPoint.drawPointList.AddRange(GetCirclePoints(hitPoint, teachingOperateParam));
                            drawPoint.needUpdate = true;
                            break;
                        case TeachingOperateMode.Curve:
                            drawPoint.drawMode = DrawPoint.DrawMode.Curve;
                            drawPoint.drawPointList.Add(hitPoint);
                            if (drawPoint.drawPointList.Count == 3) {
                                drawPoint.drawPointList = GetCurvePoints(drawPoint.drawPointList[0], 
                                    drawPoint.drawPointList[1], 
                                    drawPoint.drawPointList[2], 
                                    teachingOperateParam);
                            }
                            drawPoint.needUpdate = true;
                            break;
                        case TeachingOperateMode.Point:
                            drawPoint.drawMode = DrawPoint.DrawMode.Point;
                            drawPoint.drawPointList.Add(hitPoint);
                            drawPoint.needUpdate = true;
                            break;
                    }
                }
            }

            Physics.queriesHitBackfaces = false;
        }

        // �Ҽ�����, �ر�ʾ�̹���
        if (Input.GetMouseButtonDown(1))
        {
            isTeachingOperateOpen = false;
        }
    }

    private List<Vector3> GetCirclePoints(Vector3 center, float radius) {
        List<Vector3> points = new List<Vector3>();

        //points.Add(center);
        for (int i = 0; i < 360; i += 9)
        {
            float angle = 2 * Mathf.PI * i / 360.0f;
            float x = center.x + radius * Mathf.Cos(angle);
            float z = center.z + radius * Mathf.Sin(angle);

            points.Add(new Vector3(x, center.y, z));
        }

        return points;
    }

    private List<Vector3> GetCurvePoints(Vector3 p1, Vector3 p2, Vector3 p3, float width)
    {
        Vector3 p12p2 = new(p2.x - p1.x, 0, p2.z - p1.z);
        p12p2 = p12p2.normalized;
        Vector3 p12p3 = new(p3.x - p1.x, 0, p3.z - p1.z);
        p12p3 = p12p3.normalized;

        Vector3 rotation_p12 = Quaternion.Euler(0, 90, 0) * p12p2;
        float dotProduct = Vector3.Dot(rotation_p12.normalized, p12p3.normalized);
        if (dotProduct < 0)
        {
            rotation_p12 = Quaternion.Euler(0, -90, 0) * p12p2;
        }

        List<Vector3> points = new List<Vector3>();
        points.Add(p1);
        points.Add(p2);

        Vector3 r0 = p2;
        Vector3 l0 = p1;
        while (true) {
            Vector3 r1 = r0 + width * rotation_p12;
            if (Vector3.Dot((p3 - r1).normalized, rotation_p12.normalized) < 0)
            {
                break;
            }
            else {
                points.Add(r1);
            }

            Vector3 l1 = l0 + width * rotation_p12;
            if (Vector3.Dot((p3 - l1).normalized, rotation_p12.normalized) < 0)
            {
                break;
            }
            else
            {
                points.Add(l1);
            }

            Vector3 l2 = l1 + width * rotation_p12;
            if (Vector3.Dot((p3 - l2).normalized, rotation_p12.normalized) < 0)
            {
                break;
            }
            else
            {
                points.Add(l2);
            }

            Vector3 r2 = r1 + width * rotation_p12;
            if (Vector3.Dot((p3 - r2).normalized, rotation_p12.normalized) < 0)
            {
                break;
            }
            else
            {
                points.Add(r2);
            }

            r0 = r2;
            l0 = l2;
            if (points.Count > 1000) {
                Debug.Log("��GetCurvePoints������ĵ㳬��1000��");
                break;
            }
        }

        return points;
    }
}
