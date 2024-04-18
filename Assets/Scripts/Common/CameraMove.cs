using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class CameraMove : MonoBehaviour
{
    public float MouseWheelSensitivity = 100; //滚轮灵敏度设置

    private float xSpeed = 4.0f; //旋转视角时相机x轴转速
    private float ySpeed = 2.0f; //旋转视角时相机y轴转速
    private float x = 0.0f; //存储相机的euler角
    private float y = 0.0f; //存储相机的euler角


    private Quaternion storeRotation; //存储相机的姿态四元数

    private Vector3 initScreenPos; //中键刚按下时鼠标的屏幕坐标（第三个值其实没什么用）
    private Vector3 curScreenPos; //当前鼠标的屏幕坐标（第三个值其实没什么用）
    public float move_scala = 0.001f;
    void Start()
    {
        //这里就是设置一下初始的相机视角以及一些其他变量，这里的x和y。。。是和下面getAxis的mouse x与mouse y对应
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;
        storeRotation = Quaternion.Euler(y, x, 0);
        transform.rotation = storeRotation; //设置相机姿态
    }

    void Update()
    {
        // 判断是否在UI上
        if (EventSystem.current.IsPointerOverGameObject())
        {
            return;
        }

        //鼠标右键旋转功能
        if (Input.GetMouseButton(1))
        {
            x += Input.GetAxis("Mouse X") * xSpeed;
            y -= Input.GetAxis("Mouse Y") * ySpeed;
            transform.rotation = Quaternion.Euler(y, x, 0);
        }
        else if (Input.GetAxis("Mouse ScrollWheel") != 0) //鼠标滚轮缩放功能
        {
            transform.position += transform.forward * Input.GetAxis("Mouse ScrollWheel") * MouseWheelSensitivity;
        }

        //鼠标中键平移
        if (Input.GetMouseButtonDown(2))
        {
            initScreenPos = new Vector3(Input.mousePosition.x, Input.mousePosition.y, 0);
        }

        if (Input.GetMouseButton(2))
        {
            curScreenPos = new Vector3(Input.mousePosition.x, Input.mousePosition.y, 0);
            Vector3 move_right = transform.right * (initScreenPos.x - curScreenPos.x) * move_scala;
            Vector3 move_up = transform.up * (initScreenPos.y - curScreenPos.y) * move_scala;
            transform.localPosition += (move_right + move_up);
            initScreenPos = curScreenPos;
        }
    }
}
