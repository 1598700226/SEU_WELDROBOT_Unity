using UnityEngine;
using System.IO.Ports;
using System.Threading;

public class SerialPortControl : MonoBehaviour
{

    public string receivedData;
    public string sendData;

    private SerialPort serialPort;
    private Thread threadReceive;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    void Update()
    {
        if (!string.IsNullOrEmpty(sendData))
        {
            SendData(sendData);
            sendData = null;
        }
    }

    public string[] GetAvailablePorts()
    {
        return SerialPort.GetPortNames();
    }

    public bool OpenSerialPorts(string portName, int baudRate)
    {
        try
        {
            // ȷ�����ڲ����Ѿ��򿪵�״̬
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
            // �����µĴ�������
            serialPort = new SerialPort(portName, baudRate);
            serialPort.Open();
            threadReceive = new Thread(ReceiveData);
            threadReceive.IsBackground = true;
            threadReceive.Start();
            Debug.Log("��SerialPortControl��Serial port opened successfully.");
            return true;
        }
        catch (System.Exception ex)
        {
            Debug.LogError("��SerialPortControl��Error opening serial port: " + ex.Message);
            return false;
        }
    }

    void OnDisable()
    {
        CloseSerialPort();
    }

    public void CloseSerialPort()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
            Debug.Log("��SerialPortControl��Serial port closed.");
        }

        if (threadReceive != null)
        {
            threadReceive.Abort();
            threadReceive.Interrupt();
            threadReceive = null;
        }
    }

    public void SendData(string data)
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.WriteLine(data); // ��������
            Debug.Log("��SerialPortControl��Sent to serial port: " + data);
        }
    }

    private void ReceiveData()
    {
        while (true)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                string data = serialPort.ReadExisting(); // Read the data in the buffer
                if (string.IsNullOrEmpty(data))
                {
                    continue;
                }
                receivedData = data;
                Debug.Log($"��SerialPortControl��Received from serial port: length:{receivedData.Length} data:{receivedData}");
            }
        }
    }
}
