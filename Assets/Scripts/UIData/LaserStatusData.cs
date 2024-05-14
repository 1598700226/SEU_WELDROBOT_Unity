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
        // todo
        PlcReadDataMsg plcReadDataMsg = new PlcReadDataMsg();
        BtnStatusImgUpdate(btnLaserEnable, plcReadDataMsg.laser_enable);
        BtnStatusImgUpdate(btnOpenInstruction, plcReadDataMsg.open_instruction);
        BtnStatusImgUpdate(btnLaserOutput, plcReadDataMsg.laser_working);
        BtnStatusImgUpdate(btnLaserError, plcReadDataMsg.laser_error);
        BtnStatusImgUpdate(btnLaserReset, plcReadDataMsg.plc_reset);
        txtLaserPower.text = plcReadDataMsg.current_power.ToString();

        BtnStatusImgUpdate(btnOpenGas, plcReadDataMsg.open_gas);
        BtnStatusImgUpdate(btnOpenPowder, plcReadDataMsg.open_powder);
        txtPowderSpeed.text = plcReadDataMsg.current_speed.ToString();
        txtProduct.text = plcReadDataMsg.current_product.ToString();
        txtTechnology.text = plcReadDataMsg.current_technology.ToString();

        BtnStatusImgUpdate(btnPLCPulse, plcReadDataMsg.plc_pulse);
        BtnStatusImgUpdate(btnPLCReady, plcReadDataMsg.plc_ready);
        BtnStatusImgUpdate(btnPLCAutomatic, plcReadDataMsg.plc_automatic);
        BtnStatusImgUpdate(btnPLCManual, plcReadDataMsg.plc_manual);
        txtTemperature.text = plcReadDataMsg.env_temp.ToString();
        txtHumidity.text = plcReadDataMsg.env_humi.ToString();
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
