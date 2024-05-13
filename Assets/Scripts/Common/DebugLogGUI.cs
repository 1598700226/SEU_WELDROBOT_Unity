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
        if (Input.GetKey(KeyCode.Space) && Input.GetKey(KeyCode.O))
        {
            DebugGUI.Log("【keyboard Listener】打开DebugGUI显示");
            isShowGUI = true;
        }
        if (Input.GetKey(KeyCode.Space) && Input.GetKey(KeyCode.C))
        {
            DebugGUI.Log("【keyboard Listener】关闭DebugGUI显示");
            isShowGUI = false;
            DebugGUI.ClearPersistent();
        }

        if (isShowGUI)
        {
            DebugGUI.LogPersistent("frameTitle", $"*** 调试窗口，空格+'c'进行关闭 ***\n");
            DebugGUI.LogPersistent("framePerPath", $"*** PersistentPath: {Application.persistentDataPath} ***");
            DebugGUI.LogPersistent("frameRate", $"*** FPS: {(1 / Time.deltaTime).ToString("F3")} ***");
        }
    }
}
