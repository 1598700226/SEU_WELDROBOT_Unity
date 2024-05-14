using RosMessageTypes.PlcCommunicate;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class LaserStatusData : MonoBehaviour
{
    #region PLC控制
    /***************************************激光器状态更新*************************************/
    private Color CloseColor = new Color(0.7f, 0f, 0f);
    private Color OpenColor = new Color(0f, 0.65f, 0.25f);

    [Header("激光器使能")]
    public Button btnLaserEnable;
    [Header("指导光")]
    public Button btnOpenInstruction;
    [Header("出光中")]
    public Button btnLaserOutput;
    [Header("激光器报错")]
    public Button btnLaserError;
    [Header("激光器复位")]
    public Button btnLaserReset;
    [Header("激光功率")]
    public TMP_Text txtLaserPower;

    [Header("气体使能")]
    public Button btnOpenGas;
    [Header("送粉使能")]
    public Button btnOpenPowder;
    [Header("送粉器转速")]
    public TMP_Text txtPowderSpeed;
    [Header("产品")]
    public TMP_Text txtProduct;
    [Header("工艺")]
    public TMP_Text txtTechnology;

    [Header("脉冲信号")]
    public Button btnPLCPulse;
    [Header("就绪信号")]
    public Button btnPLCReady;
    [Header("自动模式")]
    public Button btnPLCAutomatic;
    [Header("手动模式")]
    public Button btnPLCManual;
    [Header("温度")]
    public TMP_Text txtTemperature;
    [Header("湿度")]
    public TMP_Text txtHumidity;
    #endregion

    // Update is called once per frame
    void Update()
    {
        PlcConnect plcConnect = GameObject.Find("PlcCommunication").GetComponent<PlcConnect>();

        BtnStatusImgUpdate(btnLaserEnable, plcConnect.read_laser_enable);
        BtnStatusImgUpdate(btnOpenInstruction, plcConnect.read_open_instruction);
        BtnStatusImgUpdate(btnLaserOutput, plcConnect.read_laser_working);
        BtnStatusImgUpdate(btnLaserError, plcConnect.read_laser_error);
        BtnStatusImgUpdate(btnLaserReset, plcConnect.read_plc_reset);
        txtLaserPower.text = plcConnect.read_current_power.ToString();

        BtnStatusImgUpdate(btnOpenGas, plcConnect.read_open_gas);
        BtnStatusImgUpdate(btnOpenPowder, plcConnect.read_open_powder);
        txtPowderSpeed.text = plcConnect.read_current_speed.ToString();
        txtProduct.text = plcConnect.read_current_product.ToString();
        txtTechnology.text = plcConnect.read_current_technology.ToString();

        BtnStatusImgUpdate(btnPLCPulse, plcConnect.read_plc_pulse);
        BtnStatusImgUpdate(btnPLCReady, plcConnect.read_plc_ready);
        BtnStatusImgUpdate(btnPLCAutomatic, plcConnect.read_plc_automatic);
        BtnStatusImgUpdate(btnPLCManual, plcConnect.read_plc_manual);
        txtTemperature.text = plcConnect.read_env_temp.ToString();
        txtHumidity.text = plcConnect.read_env_humi.ToString();
    }

    private void BtnStatusImgUpdate(Button button, bool isOpen)
    {
        if (isOpen)
        {
            button.image.color = OpenColor;
        }
        else 
        {
            button.image.color = CloseColor;
        }
    }
}
