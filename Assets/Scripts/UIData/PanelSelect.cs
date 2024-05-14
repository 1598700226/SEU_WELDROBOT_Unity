using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEngine.UI;

public class PanelSelect : MonoBehaviour
{
    // Start is called before the first frame update
    public List<GameObject> panels;
    public Button leftButton;
    public Button rightButton;
    public Button showPanelButton;
    public Button closeButton;

    private int index = 0;

    void Start()
    {
        // ��ȡ��ť���, ����¼�
        leftButton.onClick.AddListener(SwitchLeftPanel);
        rightButton.onClick.AddListener(SwitchRightPanel);
        showPanelButton.onClick.AddListener(FoldNowPanel);
        closeButton.onClick.AddListener(CloseApplication);
        SetActivePanel(ref index);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void SwitchLeftPanel() 
    {
        index--;
        SetActivePanel(ref index);
    }

    void SwitchRightPanel()
    {
        index++;
        SetActivePanel(ref index);
    }

    void SetActivePanel(ref int index)
    {
        if (index < 0) { 
            index = 0;
        }

        if (index >= panels.Count) { 
            index = panels.Count - 1;
        }

        for (int i = 0; i < panels.Count; i++)
        {
            if (i == index)
            {
                panels[i].SetActive(true);
            }
            else 
            {
                panels[i].SetActive(false);
            }
        }
    }

    void FoldNowPanel() 
    {
        panels[index].SetActive(!panels[index].activeSelf);
        GameObject laserPanal = GameObject.Find("LaserPanel");
        laserPanal.SetActive(!panels[index].activeSelf);
    }

    void CloseApplication()
    {
        // ����Ƿ���Unity�༭��������
        #if UNITY_EDITOR
        // ����ǣ��ڱ༭����ֹͣ����ģʽ
        UnityEditor.EditorApplication.isPlaying = false;
        #else
        // �����exe����йر�
        Application.Quit();
        #endif
    }
}
