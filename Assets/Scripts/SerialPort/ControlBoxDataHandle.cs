using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using TMPro;
using UnityEngine;

public class ControlBoxDataHandle : MonoBehaviour
{
    public TMP_Text receiveTextShow;


    /************接收数据格式**********/
    SerialPortControl serialPortControl;
    int requireDataLength = 23;     // 要求的格式长度 + "\n"
    int buttonDataStart = 0;        // 按钮起始位
    int buttonDataLength = 5;       // 按钮数据长度
    int adDataStart = 5;            // AD起始位
    int adDataLength = 16;          // AD数据长度

    // Start is called before the first frame update
    void Start()
    {
        serialPortControl = GameObject.Find("ControlBoxControl").GetComponent<SerialPortControl>();
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
        for (int i = 0; i < chars.Length; i += 2)
        {
            string hexString = new string(new char[] { chars[i], chars[i + 1] });
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
        sb.Append("oriData:" + oridata + "\n");

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
}
