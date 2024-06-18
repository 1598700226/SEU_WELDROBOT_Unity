using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class OperateModeDropDownChange : MonoBehaviour
{
    public TMP_Dropdown m_dropdown_list;
    public TMP_Text TMP_Text;

    // Update is called once per frame
    void Update()
    {
        if (m_dropdown_list != null) 
        {
            switch (m_dropdown_list.value)
            {
                case 0:
                    TMP_Text.text = "内置参数(m)";
                    break;
                case 1:
                    TMP_Text.text = "半径(m)";
                    break;
                case 2:
                    TMP_Text.text = "搭接率(m)";
                    break;
                case 3:
                    TMP_Text.text = "内置参数(m)";
                    break;
            }   
        }
    }
}
