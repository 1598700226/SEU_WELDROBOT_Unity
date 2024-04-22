using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class RawImageControlSize : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    public RawImage rawImage;   // 用于显示图像的UI RawImage
    public float zoomSpeed = 50.0f; 
    private bool isMouseEnter = false;

    void Update()
    {
        if (isMouseEnter) {
            // 获取鼠标滚轮的输入
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (scroll != 0)
            {
                // 根据滚轮输入调整RawImage的sizeDelta
                rawImage.rectTransform.sizeDelta += new Vector2(scroll, scroll) * zoomSpeed;
            }
        }
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        // 当鼠标进入游戏对象时触发
        isMouseEnter = true;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        // 当鼠标离开游戏对象时触发
        isMouseEnter = false;
    }

}
