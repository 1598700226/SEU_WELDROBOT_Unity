using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class CameraMove : MonoBehaviour
{
    public float MouseWheelSensitivity = 100; //��������������

    private float xSpeed = 4.0f; //��ת�ӽ�ʱ���x��ת��
    private float ySpeed = 2.0f; //��ת�ӽ�ʱ���y��ת��
    private float x = 0.0f; //�洢�����euler��
    private float y = 0.0f; //�洢�����euler��


    private Quaternion storeRotation; //�洢�������̬��Ԫ��

    private Vector3 initScreenPos; //�м��հ���ʱ������Ļ���꣨������ֵ��ʵûʲô�ã�
    private Vector3 curScreenPos; //��ǰ������Ļ���꣨������ֵ��ʵûʲô�ã�
    public float move_scala = 0.001f;
    void Start()
    {
        //�����������һ�³�ʼ������ӽ��Լ�һЩ���������������x��y�������Ǻ�����getAxis��mouse x��mouse y��Ӧ
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;
        storeRotation = Quaternion.Euler(y, x, 0);
        transform.rotation = storeRotation; //���������̬
    }

    void Update()
    {
        // �ж��Ƿ���UI��
        if (EventSystem.current.IsPointerOverGameObject())
        {
            return;
        }

        //����Ҽ���ת����
        if (Input.GetMouseButton(1))
        {
            x += Input.GetAxis("Mouse X") * xSpeed;
            y -= Input.GetAxis("Mouse Y") * ySpeed;
            transform.rotation = Quaternion.Euler(y, x, 0);
        }
        else if (Input.GetAxis("Mouse ScrollWheel") != 0) //���������Ź���
        {
            transform.position += transform.forward * Input.GetAxis("Mouse ScrollWheel") * MouseWheelSensitivity;
        }

        //����м�ƽ��
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
