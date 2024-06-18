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
                    TMP_Text.text = "���ò���(m)";
                    break;
                case 1:
                    TMP_Text.text = "�뾶(m)";
                    break;
                case 2:
                    TMP_Text.text = "�����(m)";
                    break;
                case 3:
                    TMP_Text.text = "���ò���(m)";
                    break;
            }   
        }
    }
}
