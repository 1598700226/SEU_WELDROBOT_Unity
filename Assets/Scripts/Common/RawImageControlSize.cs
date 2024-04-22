using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class RawImageControlSize : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    public RawImage rawImage;   // ������ʾͼ���UI RawImage
    public float zoomSpeed = 50.0f; 
    private bool isMouseEnter = false;

    void Update()
    {
        if (isMouseEnter) {
            // ��ȡ�����ֵ�����
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (scroll != 0)
            {
                // ���ݹ����������RawImage��sizeDelta
                rawImage.rectTransform.sizeDelta += new Vector2(scroll, scroll) * zoomSpeed;
            }
        }
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        // ����������Ϸ����ʱ����
        isMouseEnter = true;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        // ������뿪��Ϸ����ʱ����
        isMouseEnter = false;
    }

}
