using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class SliderValueUpdate : MonoBehaviour
{
    public Slider slider;
    public TMP_Text valueText;

    void Start()
    {
        slider.onValueChanged.AddListener(delegate { UpdateValueText(); });
    }

    void UpdateValueText()
    {
        valueText.text = slider.value.ToString("F8");
    }
}
