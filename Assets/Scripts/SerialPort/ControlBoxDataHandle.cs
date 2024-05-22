using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using TMPro;
using UnityEngine;
using UnityEngine.UIElements;

public class ControlBoxDataHandle : MonoBehaviour
{
    public TMP_Text receiveTextShow;
    [Header("左摇杆位移距离参数")]
    public float positonDisPerFrame = 0.1f;

    /************接收数据格式**********/
    SerialPortControl serialPortControl;
    int requireDataLength = 30;     // 要求的格式长度 + "\n"
    int buttonDataStart = 0;        // 按钮起始位
    int buttonDataLength = 5;       // 按钮数据长度
    int adDataStart = 5;            // AD起始位
    int adDataLength = 24;          // AD数据长度
    int adDataEachDataSize = 3;     // AD数据占位

    char[] oldButtonData = null;
    int[] oldADData = null;

    /************控制箱按键对应的数据**********/
    /*
    车体电源：第三位
    机械臂电源：第二位
    通信连接：第九位
    激光电源：第四位
    启动程序：第五位
    结束程序：第六位
    机械臂归位：第七位
    空按钮： 第八位

    机械臂急停：倒数第四位
    机器人急停：倒数第五位

    左摇杆(暂时保留)： AD2（上0 下4096) AD3(左0 右4096)
    机械臂速度：AD7(min0 max4096)
    行驶速度:AD6(min0 max 4096)
    右摇杆(小车移动) : AD4(上0 下4096) AD5(左0 右4096)
    */

    // Start is called before the first frame update
    void Start()
    {
        serialPortControl = GameObject.Find("ControlBoxControl").GetComponent<SerialPortControl>();
        oldButtonData = new char[buttonDataLength * 4];
        oldADData = new int[adDataLength / adDataEachDataSize];
        for (int i = 0; i < oldButtonData.Length; i++)
        {
            oldButtonData[i] = '1';
        }
    }

    // Update is called once per frame
    void Update()
    {
        string receivedData = serialPortControl.receivedData;
        if (!string.IsNullOrEmpty(receivedData))
        {
            if (receivedData.Length == requireDataLength)
            {
                char[] receivedCharData = receivedData.ToCharArray();
                char[] oriButtonData = new char[buttonDataLength];
                Array.Copy(receivedCharData, buttonDataStart, oriButtonData, 0, buttonDataLength);
                char[] oriAdData = new char[adDataLength];
                Array.Copy(receivedCharData, adDataStart, oriAdData, 0, adDataLength);
                // Data Format
                char[] buttonData = ButtonCharArray(oriButtonData);
                int[] adData = ADIntArray(oriAdData);
                string receiveDataFormat = ReceivedDataFormat(receivedData, buttonData, adData);
                if (receiveTextShow != null)
                {
                    receiveTextShow.text = receiveDataFormat;
                }
                // Data Control
                DataControl(buttonData, adData, oldButtonData, oldADData);

                // record data
                Array.Copy(buttonData, 0, oldButtonData, 0, buttonData.Length);
                Array.Copy(adData, 0, oldADData, 0, adData.Length);
                serialPortControl.receivedData = null;
            }
        }
    }

    /********************************************数据处理***************************************************/
    private char[] ButtonCharArray(char[] chars)
    {
        char[] buttonData = new char[chars.Length * 4];
        for (int i = 0; i < chars.Length; i++)
        {
            int decimalValue = Convert.ToInt32(chars[i].ToString(), 16);  // 将十六进制字符转换为十进制数值
            string binaryString = Convert.ToString(decimalValue, 2);  // 将十进制数值转换为二进制字符串
            binaryString = binaryString.PadLeft(4, '0');
            char[] binaryChars = binaryString.ToCharArray();
            for (int j = 0; j < 4; j++)
            {
                buttonData[i * 4 + j] = binaryChars[j];
            }
        }
        return buttonData;
    }

    private int[] ADIntArray(char[] chars)
    {
        List<int> adData = new List<int>();
        for (int i = 0; i < chars.Length; i += adDataEachDataSize)
        {
            string hexString = new string(chars, i, adDataEachDataSize);
            int number = Convert.ToInt32(hexString, 16);
            adData.Add(number);
        }
        return adData.ToArray();
    }

    private string ReceivedDataFormat(string oridata, char[] buttonData, int[] adData)
    {
        StringBuilder sb = new StringBuilder();

        // ReceiveTime
        DateTime now = DateTime.Now;
        string timeString = now.ToString("yyyy-MM-dd HH:mm:ss");
        sb.Append("Time:" + timeString + "\n");
        sb.Append("oriData:\n" + oridata + "\n");

        // Button
        string buttonStr = new string(buttonData);
        sb.Append("button:" + buttonStr + "\n");

        // AD
        for (int i = 0; i < adData.Length; i++)
        {
            sb.Append($"AD{i}:" + adData[i] + "  ");
            if ((i + 1) % 3 == 0)
                sb.Append("\n");
        }

        return sb.ToString();
    }

    private void DataControl(char[] buttonData, int[] adData, char[] oldButtonData, int[] oldADData)
    {
        if (buttonData[8] != '0')
        {
            return;
        }

        DataCommon dataCommon = GameObject.Find("DataManager").GetComponent<DataCommon>();
        UnityPublish_MoveCommand unityPublish_MoveCommand = GameObject.Find("RosCarMove").GetComponent<UnityPublish_MoveCommand>();
        // 按钮
        #region 机械臂虚拟归位
        if (buttonData[6] != oldButtonData[6])
        {
            DebugGUI.LogString($"【控制箱】虚拟机械臂归位 btn_6:{buttonData[6]}");
            dataCommon.AuboJointHome();
        }
        #endregion
        #region  机械臂急停
        if (buttonData[16] != oldButtonData[16])
        {
            DebugGUI.LogString($"【控制箱】机械臂急停 btn_16:{buttonData[16]}");
            dataCommon.RobotStop();
        }
        #endregion
        #region 机器人急停
        if (buttonData[15] != oldButtonData[15])
        {
            DebugGUI.LogString($"【控制箱】机器人急停 btn_15:{buttonData[15]}");
            dataCommon.CarStop();
        }
        #endregion
        // AD
        #region 行驶速度
        if (adData[6] != oldADData[6])
        {
            float carSpeed = (adData[6] - 0.0f) / 4096f * (unityPublish_MoveCommand.max_speed - unityPublish_MoveCommand.min_speed) + unityPublish_MoveCommand.min_speed;
            unityPublish_MoveCommand.SetSpeed(carSpeed);
            dataCommon.input_carSpeedSetting.text = carSpeed.ToString("F2");
        }
        #endregion
        #region 机械臂速度
        if (adData[7] != oldADData[7])
        {
            float auboSpeed = (adData[7] - 0.0f) / 4096f * 0.1f + 0.05f;
            dataCommon.input_aubo_speed.text = auboSpeed.ToString("F2");
        }
        #endregion
        #region 右摇杆控制车移动
        // 前进
        if (adData[4] < 1200)
        {
            DebugGUI.LogString($"【控制箱】右摇杆控制<前进> ad_4:{adData[4]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(1, 0, 0, 0);
        }
        else if (adData[4] > 2300)
        {
            DebugGUI.LogString($"【控制箱】右摇杆控制<后退> ad_4:{adData[4]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(-1, 0, 0, 0);
        }
        // 转向
        if (adData[5] < 1200)
        {
            DebugGUI.LogString($"【控制箱】右摇杆控制<左转> ad_5:{adData[5]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(0, 0, 0, 1);
        }
        else if (adData[5] > 2300)
        {
            DebugGUI.LogString($"【控制箱】右摇杆控制<右转> ad_5:{adData[5]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(0, 0, 0, -1);
        }
        #endregion

        #region 左摇杆控制机械臂移动
        // 获取速度
        float armSpeed = float.Parse(dataCommon.input_aubo_speed.text);
        Vector3 pos_incre = Vector3.zero;
        // 前进z front+ back-
        if (adData[2] < 1200)
        {
            DebugGUI.LogString($"【控制箱】左摇杆控制<前进> ad_2:{adData[2]}");
            pos_incre.z += armSpeed * positonDisPerFrame;
        }
        else if (adData[2] > 2300)
        {
            DebugGUI.LogString($"【控制箱】左摇杆控制<后退> ad_2:{adData[2]}");
            pos_incre.z -= armSpeed * positonDisPerFrame;
        }
        // 水平x left- right+
        if (adData[3] < 1200)
        {
            DebugGUI.LogString($"【控制箱】左摇杆控制<水平左> ad_3:{adData[3]}");
            pos_incre.x -= armSpeed * positonDisPerFrame;
        }
        else if (adData[3] > 2300)
        {
            DebugGUI.LogString($"【控制箱】左摇杆控制<水平右> ad_3:{adData[3]}");
            pos_incre.x += armSpeed * positonDisPerFrame;
        }
        #endregion

    }
}
