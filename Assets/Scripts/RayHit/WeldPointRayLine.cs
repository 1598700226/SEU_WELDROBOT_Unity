using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class WeldPointRayLine : MonoBehaviour
{
    public GameObject WeldToolPoint;
    private LineRenderer lineRenderer;

    // Start is called before the first frame update
    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = 2;
        // ���� LineRenderer
        lineRenderer.widthMultiplier = 0.01f;  // �����������

        // ���������ò���
        lineRenderer.material = new Material(Shader.Find("Unlit/Color"));
        lineRenderer.material.color = Color.red;  // ���ò�����ɫΪ��ɫ
    }

    // Update is called once per frame
    void Update()
    {
        // �������ߵ����ͷ���
        Vector3 start = WeldToolPoint.transform.position;
        Vector3 direction = -transform.up; // y��ĸ�����
                                           
        float rayLength = 1.0f;
        Vector3 end = start + direction * rayLength; // ���ߵĳ���

        // �������߼��
        RaycastHit hit;
        if (Physics.Raycast(start, direction, out hit))
        {
            if (hit.collider.name.ToLower().Contains("pointcloud"))
            {
                lineRenderer.SetPosition(0, start);
                lineRenderer.SetPosition(1, hit.point);
            }
            else
            {
                lineRenderer.SetPosition(0, start);
                lineRenderer.SetPosition(1, end);
            }
        }
    }
}
