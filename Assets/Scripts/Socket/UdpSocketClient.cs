using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using UnityEngine;
using System.Text;

public class UdpSocketClient : MonoBehaviour
{
    public int port = 5000;
    public string ip = "127.0.0.1";
    public bool isConnect = false;

    private UdpClient udpServer;
    private IPEndPoint endPoint;
    private Thread threadReceive;
    private Thread threadSend;

    public string sendData = null;
    public string receiveData = null;
    public byte[] sendBytes = null;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    public void connectUdp() {
        isConnect = true;

        udpServer = new UdpClient();
        endPoint = new IPEndPoint(IPAddress.Parse(ip), port);

        threadReceive = new Thread(ReceiveData);
        threadReceive.IsBackground = true;
        threadReceive.Start();

        threadSend = new Thread(SendData);
        threadSend.IsBackground = true;
        threadSend.Start();
    }

    public void closeUdp() {
        isConnect = false;

        udpServer.Close();
        threadReceive.Join();
        threadSend.Join();
    }

    private void ReceiveData()
    {
        while (true)
        {
            if (!isConnect)
            {
                Debug.Log("【UdpSocketClient】连接关闭 断开接收线程");
                break;
            }

            // 判断缓冲区是否有数据
            if (udpServer.Available > 0)
            {
                try
                {
                    //  Grab the data.
                    byte[] data = udpServer.Receive(ref endPoint);
                    receiveData = Encoding.UTF8.GetString(data);
                    Debug.Log("【UdpSocketClient】ReceiveData message：" + receiveData);
                }
                catch (System.Exception e)
                {
                    Debug.Log("【UdpSocketClient】ReceiveData ex：" + e.ToString());
                }
            }
        }
    }

    private void SendData()
    {
        while (true)
        {
            if (!isConnect)
            {
                Debug.Log("【UdpSocketClient】连接关闭 断开发送线程");
                break;
            }

            // 判断缓冲区是否有数据
            if (!string.IsNullOrEmpty(sendData))
            {
                try
                {
                    byte[] data = Encoding.UTF8.GetBytes(sendData);
                    udpServer.Send(data, data.Length, endPoint);
                    sendData = null;
                }
                catch (System.Exception e)
                {
                    Debug.Log("【UdpSocketClient】SendData ex：" + e.ToString());
                }
            }

            if (sendBytes != null && sendBytes.Length > 0) {
                try
                {
                    udpServer.Send(sendBytes, sendBytes.Length, endPoint);
                    sendBytes = null;
                }
                catch (System.Exception e)
                {
                    Debug.Log("【UdpSocketClient】SendData ex：" + e.ToString());
                }
            }
        }
    }

    private void OnApplicationQuit()
    {
        SocketQuit();
    }

    void OnDestroy()
    {
        SocketQuit();
    }

    private void SocketQuit()
    {
        if (threadReceive != null) {
            threadReceive.Abort();
            threadReceive.Interrupt();
        }
        if (threadSend != null) {
            threadSend.Abort();
            threadSend.Interrupt();
        }
        if (udpServer != null)
        {
            udpServer.Close();
        }
    }
}
