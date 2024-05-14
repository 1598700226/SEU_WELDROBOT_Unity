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
