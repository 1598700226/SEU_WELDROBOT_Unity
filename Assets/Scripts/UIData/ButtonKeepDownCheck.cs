using UnityEngine;
using UnityEngine.EventSystems;

public class ButtonKeepDownCheck : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    public bool isButtonPressed = false;

    public void OnPointerDown(PointerEventData eventData)
    {
        isButtonPressed = true;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        isButtonPressed = false;
    }
}
