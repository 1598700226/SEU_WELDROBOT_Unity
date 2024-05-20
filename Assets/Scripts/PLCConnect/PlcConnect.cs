using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.PlcCommunicate;
using RosMessageTypes.Std;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class PlcConnect : MonoBehaviour
{
    const string m_PlcDataTopicName = "/plc_data";
    const string m_WriteDataServiceName = "/write_plc_data";
    const string m_SetPlcModeServiceName = "/set_plc_mode";
    const string m_HeartbeatTopicName = "/heartbeat_signal";

    ROSConnection m_Ros;

    //public PlcReadDataMsg m_PlcData;
    //public WritePlcDataRequest m_WritePlcData;

    public float k_PublishHeartbeatFrequecy;
    public bool heartbeat_flag;

    //Read Data
    public bool read_plc_pulse = false;
    public bool read_plc_ready = false;
    public bool read_plc_automatic = false;
    public bool read_plc_manual = false;
    public bool read_plc_reset = false;
    public bool read_laser_enable = false;
    public bool read_laser_error = false;
    public bool read_laser_working = false;
    public bool read_open_instruction = false;
    public bool read_open_gas = false;
    public bool read_open_powder = false;
    public ushort read_current_power = 0;
    public ushort read_current_speed = 0;
    public ushort read_current_product = 0;
    public ushort read_current_technology = 0;
    public float read_env_temp = 0.0f;
    public float read_env_humi = 0.0f;

    //Write Data
    public bool write_plc_pulse = false;
    public bool write_plc_ready = false;
    public bool write_plc_automatic = false;
    public bool write_powdering = false;
    public bool write_laser_enable = false;
    public bool write_laser_reset = false;
    public bool write_laser_output = false;
    public bool write_open_instruction = false;
    public bool write_open_gas = false;
    public bool write_open_powder = false;
    public ushort write_current_power = 0;
    public ushort write_current_speed = 0;
    public ushort write_current_product = 0;
    public ushort write_current_technology = 0;

    // Start is called before the first frame update
    void Start()
    {
        k_PublishHeartbeatFrequecy = 0.5f;
        heartbeat_flag = false;

        m_Ros = GetComponent<ROSConnection>();
        m_Ros.Subscribe<PlcReadDataMsg>(m_PlcDataTopicName, SubPlcData);
        m_Ros.RegisterRosService<WritePlcDataRequest, WritePlcDataResponse>(m_WriteDataServiceName);
        m_Ros.RegisterPublisher<BoolMsg>(m_HeartbeatTopicName);
        //m_Ros.RegisterRosService<SetPlcModeRequest, SetPlcModeResponse>(m_SetPlcModeServiceName);

        StartCoroutine(SendHeartbeat(k_PublishHeartbeatFrequecy)); // 开始协程，间隔发送消息

    }

    // Update is called once per frame
    void Update()
    {
        
    }

    IEnumerator SendHeartbeat(float interval)
    {
        while (true) 
        { 
            heartbeat_flag = !heartbeat_flag;
            BoolMsg msg = new BoolMsg(heartbeat_flag);
            m_Ros.Publish(m_HeartbeatTopicName, msg);

            yield return new WaitForSeconds(interval); // 等待指定的时间间隔
        }

    }

    public void SubPlcData(PlcReadDataMsg plc_data)
    {
        read_plc_pulse = plc_data.plc_pulse;
        read_plc_ready = plc_data.plc_ready;
        read_plc_automatic = plc_data.plc_automatic;
        read_plc_manual = plc_data.plc_manual;
        read_plc_reset = plc_data.plc_reset;
        read_laser_enable = plc_data.laser_enable;
        read_laser_error = plc_data.laser_error;   
        read_laser_working = plc_data.laser_working;
        read_open_instruction = plc_data.open_instruction;
        read_open_gas = plc_data.open_gas;
        read_open_powder = plc_data.open_powder;
        read_current_power = plc_data.current_power;
        read_current_speed = plc_data.current_speed;
        read_current_product = plc_data.current_product;
        read_current_technology = plc_data.current_technology;
        read_env_temp = plc_data.env_temp;
        read_env_humi = plc_data.env_humi;

    }
    // mode_choose:用于选择是否仅开启激光器，不再改变其他参数
    // 大于2时：仅开启或关闭激光器  偶数：开启激光器  奇数：关闭激光器
    // 小于等于2时 设置相应参数
    public void WriteToPlcData(int mode_choose) 
    {
        var request = new WritePlcDataRequest();

        if (mode_choose > 2)  //仅开启或关闭激光器，不设置参数
        {
            request.mode = mode_choose;
            //request.mode ：大于2 的偶数开启激光器，大于2的奇数关闭激光器
        }
        else if (mode_choose <= 2) //设置激光器参数
        {
            request.mode = mode_choose;
            request.write_data.plc_pulse = write_plc_pulse;
            request.write_data.plc_ready = write_plc_ready;
            request.write_data.plc_automatic = write_plc_automatic;
            request.write_data.powdering = write_powdering;
            request.write_data.laser_enable = write_laser_enable;
            request.write_data.laser_reset = write_laser_reset;
            request.write_data.laser_output = write_laser_output;
            request.write_data.open_instruction = write_open_instruction;
            request.write_data.open_gas = write_open_gas;
            request.write_data.open_powder = write_open_powder;
            request.write_data.current_power = write_current_power;
            request.write_data.current_speed = write_current_speed;
            request.write_data.current_product = write_current_product;
            request.write_data.current_technology = write_current_technology;
        }

        m_Ros.SendServiceMessage<WritePlcDataResponse>(m_WriteDataServiceName, request, WriteToPlcData);
    }

    public void WriteToPlcData(WritePlcDataResponse response)
    {
        if (response.success == true)
        {
            Debug.Log("Write Plc Data Successful!");
        }
        else
        {
            if (response.error_mode == 1)
            {
                Debug.Log("Plc Conneting is failed!");
            }
            else if (response.error_mode == 2)
            {
                Debug.Log("The Plc Mode: enable_write is false!");
            }
        }
    }


}
