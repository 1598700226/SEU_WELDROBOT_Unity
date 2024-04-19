using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEditor;
using UnityEngine;

public class AutoSizeTextMeshPro : MonoBehaviour
{
    [MenuItem("Tools/Auto-Size TextMeshPro in Selection")]
    public static void AutoSizeTextMeshProSelection()
    {
        foreach (GameObject selectedObject in Selection.gameObjects)
        {
            TextMeshProUGUI tmp = selectedObject.GetComponent<TextMeshProUGUI>();
            if (tmp != null)
            {
                Undo.RecordObject(tmp, "Set Auto-Size for TextMeshPro");

                // Enable auto-size
                tmp.enableAutoSizing = true;

                // Optional: Set the min and max font sizes
                tmp.fontSizeMin = 10; // Minimum font size
                tmp.fontSizeMax = 50; // Maximum font size

                Debug.Log($"Auto-size enabled for {selectedObject.name} with TextMeshPro.");
            }
        }
    }
}
