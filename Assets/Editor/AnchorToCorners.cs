using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class AnchorToCorners : MonoBehaviour
{
    [MenuItem("Tools/Anchor To Corners %#t")] // Shortcut key Ctrl+Shift+T
    public static void AnchorSelectedToCorners()
    {
        foreach (GameObject selectedObject in Selection.gameObjects)
        {
            RectTransform rectTransform = selectedObject.GetComponent<RectTransform>();
            if (rectTransform != null && rectTransform.parent != null)
            {
                RectTransform parentRect = rectTransform.parent.GetComponent<RectTransform>();

                if (parentRect != null)
                {
                    Undo.RecordObject(rectTransform, "Anchor to Corners");

                    // Calculate the ratio of the position of the edges relative to the parent
                    rectTransform.anchorMin = new Vector2(rectTransform.anchorMin.x + rectTransform.offsetMin.x / parentRect.rect.width,
                                                          rectTransform.anchorMin.y + rectTransform.offsetMin.y / parentRect.rect.height);
                    rectTransform.anchorMax = new Vector2(rectTransform.anchorMax.x + rectTransform.offsetMax.x / parentRect.rect.width,
                                                          rectTransform.anchorMax.y + rectTransform.offsetMax.y / parentRect.rect.height);

                    // Set the offsets to zero after adjusting the anchors
                    rectTransform.offsetMin = rectTransform.offsetMax = Vector2.zero;
                }
            }
        }
    }
}
