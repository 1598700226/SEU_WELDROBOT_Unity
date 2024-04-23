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
        // 配置 LineRenderer
        lineRenderer.widthMultiplier = 0.01f;  // 设置线条宽度

        // 创建和设置材质
        lineRenderer.material = new Material(Shader.Find("Unlit/Color"));
        lineRenderer.material.color = Color.red;  // 设置材质颜色为红色
    }

    // Update is called once per frame
    void Update()
    {
        // 定义射线的起点和方向
        Vector3 start = WeldToolPoint.transform.position;
        Vector3 direction = -transform.up; // y轴的负方向
                                           
        float rayLength = 1.0f;
        Vector3 end = start + direction * rayLength; // 射线的长度

        // 进行射线检测
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
