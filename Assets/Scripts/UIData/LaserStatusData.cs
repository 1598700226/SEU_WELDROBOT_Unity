using RosMessageTypes.PlcCommunicate;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class LaserStatusData : MonoBehaviour
{
    #region PLC����
    /***************************************������״̬����*************************************/
    private Color CloseColor = new Color(0.7f, 0f, 0f);
    private Color OpenColor = new Color(0f, 0.65f, 0.25f);

    [Header("������ʹ��")]
    public Button btnLaserEnable;
    [Header("ָ����")]
    public Button btnOpenInstruction;
    [Header("������")]
    public Button btnLaserOutput;
    [Header("����������")]
    public Button btnLaserError;
    [Header("��������λ")]
    public Button btnLaserReset;
    [Header("���⹦��")]
    public TMP_Text txtLaserPower;

    [Header("����ʹ��")]
    public Button btnOpenGas;
    [Header("�ͷ�ʹ��")]
    public Button btnOpenPowder;
    [Header("�ͷ���ת��")]
    public TMP_Text txtPowderSpeed;
    [Header("��Ʒ")]
    public TMP_Text txtProduct;
    [Header("����")]
    public TMP_Text txtTechnology;

    [Header("�����ź�")]
    public Button btnPLCPulse;
    [Header("�����ź�")]
    public Button btnPLCReady;
    [Header("�Զ�ģʽ")]
    public Button btnPLCAutomatic;
    [Header("�ֶ�ģʽ")]
    public Button btnPLCManual;
    [Header("�¶�")]
    public TMP_Text txtTemperature;
    [Header("ʪ��")]
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
