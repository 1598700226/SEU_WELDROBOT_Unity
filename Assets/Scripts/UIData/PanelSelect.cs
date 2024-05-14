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
        // 获取按钮组件, 添加事件
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
        // 检测是否在Unity编辑器中运行
        #if UNITY_EDITOR
        // 如果是，在编辑器中停止播放模式
        UnityEditor.EditorApplication.isPlaying = false;
        #else
        // 如果在exe打包中关闭
        Application.Quit();
        #endif
    }
}
