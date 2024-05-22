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
    [Header("��ҡ��λ�ƾ������")]
    public float positonDisPerFrame = 0.1f;

    /************�������ݸ�ʽ**********/
    SerialPortControl serialPortControl;
    int requireDataLength = 30;     // Ҫ��ĸ�ʽ���� + "\n"
    int buttonDataStart = 0;        // ��ť��ʼλ
    int buttonDataLength = 5;       // ��ť���ݳ���
    int adDataStart = 5;            // AD��ʼλ
    int adDataLength = 24;          // AD���ݳ���
    int adDataEachDataSize = 3;     // AD����ռλ

    char[] oldButtonData = null;
    int[] oldADData = null;

    /************�����䰴����Ӧ������**********/
    /*
    �����Դ������λ
    ��е�۵�Դ���ڶ�λ
    ͨ�����ӣ��ھ�λ
    �����Դ������λ
    �������򣺵���λ
    �������򣺵���λ
    ��е�۹�λ������λ
    �հ�ť�� �ڰ�λ

    ��е�ۼ�ͣ����������λ
    �����˼�ͣ����������λ

    ��ҡ��(��ʱ����)�� AD2����0 ��4096) AD3(��0 ��4096)
    ��е���ٶȣ�AD7(min0 max4096)
    ��ʻ�ٶ�:AD6(min0 max 4096)
    ��ҡ��(С���ƶ�) : AD4(��0 ��4096) AD5(��0 ��4096)
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

    /********************************************���ݴ���***************************************************/
    private char[] ButtonCharArray(char[] chars)
    {
        char[] buttonData = new char[chars.Length * 4];
        for (int i = 0; i < chars.Length; i++)
        {
            int decimalValue = Convert.ToInt32(chars[i].ToString(), 16);  // ��ʮ�������ַ�ת��Ϊʮ������ֵ
            string binaryString = Convert.ToString(decimalValue, 2);  // ��ʮ������ֵת��Ϊ�������ַ���
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
        // ��ť
        #region ��е�������λ
        if (buttonData[6] != oldButtonData[6])
        {
            DebugGUI.LogString($"�������䡿�����е�۹�λ btn_6:{buttonData[6]}");
            dataCommon.AuboJointHome();
        }
        #endregion
        #region  ��е�ۼ�ͣ
        if (buttonData[16] != oldButtonData[16])
        {
            DebugGUI.LogString($"�������䡿��е�ۼ�ͣ btn_16:{buttonData[16]}");
            dataCommon.RobotStop();
        }
        #endregion
        #region �����˼�ͣ
        if (buttonData[15] != oldButtonData[15])
        {
            DebugGUI.LogString($"�������䡿�����˼�ͣ btn_15:{buttonData[15]}");
            dataCommon.CarStop();
        }
        #endregion
        // AD
        #region ��ʻ�ٶ�
        if (adData[6] != oldADData[6])
        {
            float carSpeed = (adData[6] - 0.0f) / 4096f * (unityPublish_MoveCommand.max_speed - unityPublish_MoveCommand.min_speed) + unityPublish_MoveCommand.min_speed;
            unityPublish_MoveCommand.SetSpeed(carSpeed);
            dataCommon.input_carSpeedSetting.text = carSpeed.ToString("F2");
        }
        #endregion
        #region ��е���ٶ�
        if (adData[7] != oldADData[7])
        {
            float auboSpeed = (adData[7] - 0.0f) / 4096f * 0.1f + 0.05f;
            dataCommon.input_aubo_speed.text = auboSpeed.ToString("F2");
        }
        #endregion
        #region ��ҡ�˿��Ƴ��ƶ�
        // ǰ��
        if (adData[4] < 1200)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<ǰ��> ad_4:{adData[4]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(1, 0, 0, 0);
        }
        else if (adData[4] > 2300)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<����> ad_4:{adData[4]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(-1, 0, 0, 0);
        }
        // ת��
        if (adData[5] < 1200)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<��ת> ad_5:{adData[5]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(0, 0, 0, 1);
        }
        else if (adData[5] > 2300)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<��ת> ad_5:{adData[5]}");
            unityPublish_MoveCommand.SendMoveCommandtoTopic(0, 0, 0, -1);
        }
        #endregion

        #region ��ҡ�˿��ƻ�е���ƶ�
        // ��ȡ�ٶ�
        float armSpeed = float.Parse(dataCommon.input_aubo_speed.text);
        Vector3 pos_incre = Vector3.zero;
        // ǰ��z front+ back-
        if (adData[2] < 1200)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<ǰ��> ad_2:{adData[2]}");
            pos_incre.z += armSpeed * positonDisPerFrame;
        }
        else if (adData[2] > 2300)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<����> ad_2:{adData[2]}");
            pos_incre.z -= armSpeed * positonDisPerFrame;
        }
        // ˮƽx left- right+
        if (adData[3] < 1200)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<ˮƽ��> ad_3:{adData[3]}");
            pos_incre.x -= armSpeed * positonDisPerFrame;
        }
        else if (adData[3] > 2300)
        {
            DebugGUI.LogString($"�������䡿��ҡ�˿���<ˮƽ��> ad_3:{adData[3]}");
            pos_incre.x += armSpeed * positonDisPerFrame;
        }
        #endregion

    }
}
