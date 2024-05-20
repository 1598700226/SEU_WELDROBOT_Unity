using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class DebugLogGUI : MonoBehaviour
{
    public bool isShowGUI = true;

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.Space) && Input.GetKeyDown(KeyCode.O))
        {
            DebugGUI.LogString("��keyboard Listener����DebugGUI��ʾ");
            isShowGUI = true;
            DebugGUI.Settings.enableLogs = true;
        }
        if (Input.GetKey(KeyCode.Space) && Input.GetKeyDown(KeyCode.C))
        {
            DebugGUI.LogString("��keyboard Listener���ر�DebugGUI��ʾ");
            isShowGUI = false;
            DebugGUI.ClearPersistent();
            DebugGUI.Settings.enableLogs = false;
        }

        if (isShowGUI)
        {
            DebugGUI.LogPersistent("frameTitle", $"*** ���Դ��ڣ��ո�+'c'���йر� ***\n");
            DebugGUI.LogPersistent("framePerPath", $"*** PersistentPath: {Application.persistentDataPath} ***");
            DebugGUI.LogPersistent("frameRate", $"*** FPS: {(1 / Time.deltaTime).ToString("F3")} ***");
        }
    }
}
