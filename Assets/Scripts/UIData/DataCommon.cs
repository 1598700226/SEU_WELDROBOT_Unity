using RosMessageTypes.Geometry;
using RosMessageTypes.PlcCommunicate;
using System;
using System.Collections.Generic;
using TMPro;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UI;

public class DataCommon : MonoBehaviour
{
    #region ������ذ�ť������
    /***************************************�������*************************************/
    [Header("********�������********")]
    [Header("�������ڵ��ļ���")]
    public TMP_InputField input_pointCloudFilePath; //�������ڵ��ļ���
    [Header("���Ƹ����ӳ�ʱ��")]
    public TMP_InputField input_pointCloudUpdateSecond; // ���Ƹ����ӳ�ʱ��
    [Header("��������ϵ��")]
    public TMP_InputField input_pointCloudScala; // ��������ϵ��
    [Header("�Ƿ�������ǻ�")]
    public Toggle toggle_pointTriangulation; // �Ƿ�������ǻ�
    [Header("�Ƿ����YZ����")]
    public Toggle toggle_inverseYZ; // �Ƿ����YZ����
    [Header("�Ƿ���ROS���topic")]
    public Toggle toggle_rosCameraSubscription; // �Ƿ���ROS���topic
    [Header("�Ƿ���ROS���topic")]
    public TMP_Dropdown dropdown_CameraShow; // �Ƿ���ROS���topic
    [Header("�Ƿ񱣴���ӽǵ���")]
    public Toggle toggle_mulViewPointCloud; // �Ƿ񱣴���ӽǵ���
    [Header("�Ƿ�ʵʱˢ��")]
    public Toggle toggle_pointCloudRealRefesh; // �Ƿ�ʵʱˢ��
    [Header("��������")]
    public Button btn_clear_pointcloud;    // ��������
    [Header("����һ��")]
    public Button btn_update_once;    // ����һ��
    [Header("�������۱궨")]
    public Toggle toggle_eyeOnHandCalibration; // �Ƿ������۱궨
    [Header("���۱궨���ȷ��")]
    public Button btn_eyeOnHandCalibrationStart; // ���۱궨������㿪ʼ
    [Header("��������")]
    public Button btn_update_setting; // ��������

    public static string PointCloudFilePath = "";       //�����ļ�·��
    public static int PointCloudUpdateSecond = 5;       // ����ʱ��
    public static float PointCloudScala = 0.001f;       // �������ű���
    public static bool isPointTriangulation = true;     // �Ƿ�������ǻ�
    public static bool isinverseYZ = true;              // �Ƿ����YZ
    public static bool isRosCameraSubscription = false; // �Ƿ���ROS���topic
    public static bool isMulViewPointCloud = false;     // �Ƿ���ӽǱ������
    public static bool isRealRefeshPointCloud = false;  // �Ƿ�ʵʱ���µ���
    public static bool isPointCloudShowOnce = false;    // ���θ���
    #endregion

    #region Socket��ذ�ť������
    /***************************************Socket���*************************************/
    [Space(10)]
    [Header("********Socket���********")]
    [Header("ʾ�̵�ip��ַ")]
    public TMP_InputField input_socket_ip;
    [Header("ʾ�̵�˿ں�")]
    public TMP_InputField input_socket_host;
    [Header("socket���Ӱ�ť")]
    public Button btn_socket_connect;    // socket ���� �������ݴ���
    [Header("Mapip��ַ")]
    public TMP_InputField input_socket_ip2;
    [Header("Map�˿ں�")]
    public TMP_InputField input_socket_host2;
    [Header("socket���Ӱ�ť")]
    public Button btn_socket_map_connect;    // socket ���� map�����ݴ���

    public static string socket_ip = "";
    public static string socket_host = "";
    #endregion

    #region ʾ����ذ�ť������
    /***************************************ʾ�����*************************************/
    [Space(10)]
    [Header("********ʾ�����********")]
    [Header("�ֶ���קʾ�̿�ʼ")]
    public Button btn_manualoperate_start; // �Ƿ���ʾ���������
    [Header("�ֶ���קʾ�����ݴ洢")]
    public Button btn_manualoperate_save; // �Ƿ���ʾ���������
    [Header("�Ƿ���ʾ���������")]
    public Toggle toggle_pointCloudMouseShow; // �Ƿ���ʾ���������
    [Header("�������ǰ������")]
    public TMP_InputField input_mousePointLoction; // �������ǰ������
    [Header("ʾ��ģʽ")]
    public TMP_Dropdown dropdown_teachingOperateMode; // ֱ��0 ��Բ1 ����2
    [Header("ʾ��ģʽ��Ӧ�Ĳ���")]
    public TMP_InputField input_teachingOperateParam; // ��Ӧ������Բ���ǰ뾶 �������ʾ����֮��Ŀ��
    [Header("ʾ�̲���")]
    public Button btn_teaching_operate;
    [Header("ɾ����һ����")]
    public Button btn_delete_lastPoint;
    [Header("��ֵ���")]
    public TMP_InputField input_point_interpolated_distance;
    [Header("ѡ��ִ��λ��")]
    public TMP_Dropdown dropdown_selectRobotPlanPose;
    [Header("ĩ�˹���ƫ��")]
    public TMP_InputField input_toolEndPoint_offset;
    [Header("�滮")]
    public Button btn_robot_plan;
    [Header("���¹滮")]
    public Button btn_robot_replan;
    [Header("ִ��")]
    public Button btn_robot_exc;

    public static bool isShowMousePoint = true;         // ��ʾ���������
    public static string MousePointLocation = "";       // �����������ʽ��
    #endregion

    #region PGM��ͼ��ش��롢����
    /***************************************PGM��ͼ���*************************************/
    [Space(10)]
    [Header("********PGM��ͼ���********")]
    [Header("pgm���ڵ��ļ���")]
    public TMP_InputField input_pgmFileDir;  // pgm�ļ����ڵ��ļ���
    [Header("pgm����·��")]
    public TMP_InputField input_pgmFilePath; // pgm�ļ����ڵľ���·��
    [Header("�Ƿ�رյ�ͼ��ʾ")]
    public Toggle toggle_closeMapViewSelect;
    [Header("�Ƿ���ros��ͼ����")]
    public Toggle toggle_rosMapSubscriptionSelect;
    [Header("�Ƿ���ͼ�ֲ��Ŵ�")]
    public Toggle toggle_LocalMagnificationSelect;
    [Header("�ٶ���ʾ������")]
    public TMP_InputField input_carSpeedSetting;
    [Header("����Map����")]
    public Button btn_map_settingUpdate;
    [Header("A���㷨�滮")]
    public Button btn_AstarPlanPath;
    [Header("��ͼ�ϵ�����")]
    public TMP_InputField input_pgmMousePosition;
    [Header("socket����PGM��ͼ���λ��")]
    public Button btn_sendPGMMousePosition;
    #endregion

    #region Aubo��е�����
    /***************************************Aubo��е�����*************************************/
    [Space(10)]
    [Header("********Aubo��е�����********")]
    public TMP_Text laber_real_joint_1_angle;
    public TMP_Text laber_real_joint_2_angle;
    public TMP_Text laber_real_joint_3_angle;
    public TMP_Text laber_real_joint_4_angle;
    public TMP_Text laber_real_joint_5_angle;
    public TMP_Text laber_real_joint_6_angle;
    public TMP_Text laber_real_pose_x;
    public TMP_Text laber_real_pose_y;
    public TMP_Text laber_real_pose_z;
    public TMP_Text laber_real_pose_rx;
    public TMP_Text laber_real_pose_ry;
    public TMP_Text laber_real_pose_rz;
    public TMP_Text laber_real_pose_rw;

    public TMP_Text laber_virtual_joint_1_angle;
    public TMP_Text laber_virtual_joint_2_angle;
    public TMP_Text laber_virtual_joint_3_angle;
    public TMP_Text laber_virtual_joint_4_angle;
    public TMP_Text laber_virtual_joint_5_angle;
    public TMP_Text laber_virtual_joint_6_angle;

    public TMP_InputField input_aubo_joint_1_angle;
    public TMP_InputField input_aubo_joint_2_angle;
    public TMP_InputField input_aubo_joint_3_angle;
    public TMP_InputField input_aubo_joint_4_angle;
    public TMP_InputField input_aubo_joint_5_angle;
    public TMP_InputField input_aubo_joint_6_angle;

    public TMP_InputField input_aubo_endPoint_x;
    public TMP_InputField input_aubo_endPoint_y;
    public TMP_InputField input_aubo_endPoint_z;

    public TMP_InputField input_aubo_endPoint_rx;
    public TMP_InputField input_aubo_endPoint_ry;
    public TMP_InputField input_aubo_endPoint_rz;
    public TMP_InputField input_aubo_endPoint_rw;
    public Toggle toggle_aubo_send_isJoint;
    public TMP_InputField input_aubo_speed;

    public TMP_Dropdown dropDown_aubo_teleOperateMode;
    public Button btn_aubo_teleOperate;
    public Button btn_aubo_home;
    public Button btn_aubo_sync_virtual2real;
    public Button btn_aubo_sync_real2virtual;
    #endregion

    #region �ߵµ�ͼ���
    /***************************************�ߵµ�ͼ���*************************************/
    [Space(10)]
    [Header("********�ߵµ�ͼ���********")]
    public TMP_InputField input_gaode_longitude;
    public TMP_InputField input_gaode_latitude;
    public TMP_InputField input_gaode_ip;
    public Slider slider_gaode_mapsize;
    public Button btn_gaode_refresh;
    #endregion

    #region ��ͣ���
    /***************************************��ͣ���*************************************/
    [Space(10)]
    [Header("********��ͣ������********")]
    public Button btn_robot_stop;
    public Button btn_car_stop;
    public Button btn_everything_stop;

    private bool isBtnRobotStopDown = false;
    private bool isBtnCarStopDown = false;
    private bool isBtnEverythingStopDown = false;
    private Color StopDownColor = new Color(0.6f, 0.2f, 0.2f);
    private Color StopUpColor = new Color(0.2f, 0.6f, 0.2f);
    #endregion

    #region ���������
    /***************************************���������*************************************/
    [Space(10)]
    [Header("********���������********")]
    public Button btn_serialport_search;
    public TMP_Dropdown dropdowm_portname;
    public TMP_Dropdown dropdown_baudrate;
    public Button btn_serialport_open;
    public Button btn_serialport_close;
    #endregion

    #region PLC����
    /***************************************PLC����*************************************/
    [Space(10)]
    [Header("********PLC����********")]
    [Header("��ȡ��ť")]
    public Button btn_read_plc;
    [Header("UISwitcher������ʹ��")]
    public UISwitcher.UISwitcher uISwitcherLaserEnable;
    [Header("UISwitcherָ����ʹ��")]
    public UISwitcher.UISwitcher uISwitcherOpenInstruction;
    [Header("UISwitcher����������")]
    public UISwitcher.UISwitcher uISwitcherLaserOutput;
    [Header("UISwitcher��������λ")]
    public UISwitcher.UISwitcher uISwitcherLaserReset;
    [Header("input����������")]
    public TMP_InputField input_laserPower;
    [Header("UISwitcher����ʹ��")]
    public UISwitcher.UISwitcher uISwitcherOpenGas;
    [Header("UISwitcher�ͷ�ʹ��")]
    public UISwitcher.UISwitcher uISwitcherOpenPowder;
    [Header("UISwitcher�ͷ�")]
    public UISwitcher.UISwitcher uISwitcherPowdering;
    [Header("input�ͷ���ת��")]
    public TMP_InputField input_powderSpeed;
    [Header("input�ӹ���Ʒ")]
    public TMP_InputField input_product;
    [Header("input�ӹ�����")]
    public TMP_InputField input_technology;
    [Header("UISwitcher�����ź�")]
    public UISwitcher.UISwitcher uISwitcherPLCPulse;
    [Header("UISwitcher�����ź�")]
    public UISwitcher.UISwitcher uISwitcherPLCReady;
    [Header("UISwitcher�Զ�ģʽ")]
    public UISwitcher.UISwitcher uISwitcherPLCAutomatic;
    [Header("���°�ť")]
    public Button btn_update_plc;
    [Header("�򿪼�����")]
    public Button btn_open_laser;

    // ����2ʱ����������رռ�����  ż��������������  �������رռ�����
    // С�ڵ���2ʱ ������Ӧ����
    private LaserModeChoose laserModeChoose = LaserModeChoose.Close;
    private enum LaserModeChoose
    {
        Setting = 1,
        Close = 3,
        Open = 4,
    }
    #endregion

    // Start is called before the first frame update
    void Start()
    {
        input_pointCloudFilePath.text = PointCloudFilePath;
        input_pointCloudUpdateSecond.text = PointCloudUpdateSecond.ToString();
        input_pointCloudScala.text = PointCloudScala.ToString();
        toggle_pointTriangulation.isOn = isPointTriangulation;
        toggle_inverseYZ.isOn = isinverseYZ;
        toggle_pointCloudRealRefesh.isOn = isRealRefeshPointCloud;

        toggle_pointCloudMouseShow.isOn = isShowMousePoint;
        input_mousePointLoction.text = MousePointLocation;


        btn_update_setting.onClick.AddListener(PointCloudUpdateSetting);
        btn_update_once.onClick.AddListener(PointCloudUpdateOnce);
        btn_clear_pointcloud.onClick.AddListener(PointCloudClear);
        btn_eyeOnHandCalibrationStart.onClick.AddListener(EyeOnHandCalibrationStart);

        btn_manualoperate_start.onClick.AddListener(ManualOperateStart);
        btn_manualoperate_save.onClick.AddListener(ManualOperateSavePostionAndOrientation);
        btn_teaching_operate.onClick.AddListener(TeachingOperateBegin);
        btn_delete_lastPoint.onClick.AddListener(DeleteLastTeachingPoint);
        btn_robot_plan.onClick.AddListener(RobotPointPlan);
        btn_robot_replan.onClick.AddListener(RobotPointRePlan);
        btn_robot_exc.onClick.AddListener(RobotPointExc);

        btn_socket_connect.onClick.AddListener(SocketConnect);
        btn_socket_map_connect.onClick.AddListener(SocketMapConnect);

        btn_map_settingUpdate.onClick.AddListener(PGMUpdateSetting);
        btn_AstarPlanPath.onClick.AddListener(AstarPlanPath);
        input_pgmMousePosition.readOnly = true;
        btn_sendPGMMousePosition.onClick.AddListener(SendPGMMousePosition);

        btn_aubo_teleOperate.onClick.AddListener(AuboTeleOperateMode);
        btn_aubo_home.onClick.AddListener(AuboJointHome);
        btn_aubo_sync_virtual2real.onClick.AddListener(AuboSettingSyncV2R);
        btn_aubo_sync_real2virtual.onClick.AddListener(AuboSettingSyncR2V);

        btn_gaode_refresh.onClick.AddListener(GaodeRefresh);

        btn_robot_stop.onClick.AddListener(RobotStop);
        btn_car_stop.onClick.AddListener(CarStop);
        btn_everything_stop.onClick.AddListener(EveryThingStop);

        btn_serialport_search.onClick.AddListener(ControlBoxSerialportSearch);
        btn_serialport_open.onClick.AddListener(ControlBoxSerialportOpen);
        btn_serialport_close.onClick.AddListener(ControlBoxSerialportClose);

        btn_read_plc.onClick.AddListener(ReadPLCData);
        btn_update_plc.onClick.AddListener(UpdatePLCData);
        btn_open_laser.onClick.AddListener(OpenLaser);
    }

    // Update is called once per frame
    void Update()
    {
        if (toggle_pointCloudMouseShow.isOn) {
            Vector3 hitPoint = RayTest.hitPoint;
            input_mousePointLoction.text = String.Format("({0:F3},{1:F3},{2:F3})", hitPoint.x, hitPoint.y, hitPoint.z);
        }

        PGMFileReader pgmFileReader = GameObject.Find("RawImageNavigationMap").GetComponent<PGMFileReader>();
        Vector2 pgmPoint = pgmFileReader.mouseDownPositon_actual;
        input_pgmMousePosition.text = String.Format("({0:F3},{1:F3} )", pgmPoint.x, pgmPoint.y);

        AuboJointAndPoseRefresh();

        // keyboard Listener
        // ���̰�ť�������
        if (Input.GetKey(KeyCode.Space) && Input.GetKeyDown(KeyCode.P))
        {
            DebugGUI.LogString("��keyboard Listener�������������");
            UnitySubscription_PointCloud unitySubscription_PointCloud = GameObject.Find("RosColorDepthData").GetComponent<UnitySubscription_PointCloud>();
            unitySubscription_PointCloud.IsSavePointCloud = true;
        }
    }

    public void RefreshPointCloudUIData() {
        try {
            PointCloudFilePath = input_pointCloudFilePath.text;
            PointCloudUpdateSecond = int.Parse(input_pointCloudUpdateSecond.text);
            PointCloudScala = float.Parse(input_pointCloudScala.text);
            isPointTriangulation = toggle_pointTriangulation.isOn;
            isinverseYZ = toggle_inverseYZ.isOn;
            isRealRefeshPointCloud = toggle_pointCloudRealRefesh.isOn;
            isShowMousePoint = toggle_pointCloudMouseShow.isOn;
            MousePointLocation = input_mousePointLoction.text;
            isRosCameraSubscription = toggle_rosCameraSubscription.isOn;
            isMulViewPointCloud = toggle_mulViewPointCloud.isOn;

        } catch (Exception ex) {

            Debug.Log("��DataCommon�� RefreshUIData ex:" + ex.Message);
            DebugGUI.Log("��DataCommon�� RefreshUIData ex:" + ex.Message);
        }
    }

    /**********************************************��������¼�******************************************/
    void PointCloudUpdateSetting() {
        RefreshPointCloudUIData();
        PointCloudShow.realRefreshPotinCloud = isRealRefeshPointCloud;
        PointCloudShow.pointCloudDelayShow = PointCloudUpdateSecond;
        PointCloudShow.pointCloudDirPath = PointCloudFilePath;
        PointCloudShow.scale = PointCloudScala;
        PointCloudShow.invertYZ = isinverseYZ;
        PointCloudShow.isPointTriangle = isPointTriangulation;

        // ����ros������ʾ������
        UnitySubscription_PointCloud unitySubscription_PointCloud = GameObject.Find("RosColorDepthData").GetComponent<UnitySubscription_PointCloud>();
        unitySubscription_PointCloud.showType = dropdown_CameraShow.value;
        unitySubscription_PointCloud.pointCloudUpdateTime = PointCloudUpdateSecond;
        unitySubscription_PointCloud.invertYZ = isinverseYZ;
        unitySubscription_PointCloud.isPointTriangle = isPointTriangulation;
        unitySubscription_PointCloud.scala = PointCloudScala;
        unitySubscription_PointCloud.IsMultiViewPointCloudShow = toggle_mulViewPointCloud.isOn;
        unitySubscription_PointCloud.IsCalibration = toggle_eyeOnHandCalibration.isOn;
        UnitySubscription_AvoidanceCamrea unitySubscription_AvoidanceCamrea = GameObject.Find("RosAvoidanceColorDepthData").GetComponent<UnitySubscription_AvoidanceCamrea>();
        unitySubscription_AvoidanceCamrea.showType = dropdown_CameraShow.value;
        if (toggle_rosCameraSubscription.isOn)
        {
            if (dropdown_CameraShow.value <= 2)
            {
                unitySubscription_PointCloud.realRefreshPotinCloud = isRealRefeshPointCloud;
                unitySubscription_PointCloud.SubscribeTopics(true, false); // ���Ĳ�ɫͼ�����������ͼ
                unitySubscription_PointCloud.DepthImageRegisterServiceCall(); // �������ͼ�ķ���
                unitySubscription_AvoidanceCamrea.UnSubscribeTopics(); // �رյ����Ķ���
            }
            else if (dropdown_CameraShow.value > 2)
            {
                unitySubscription_AvoidanceCamrea.SubscribeTopics(true, true); // ���ĵ�����ɫͼ���������ͼ
                unitySubscription_PointCloud.UnSubscribeTopics(); // �رյ��ƵĶ���
            }
        }
        else
        {
            unitySubscription_PointCloud.realRefreshPotinCloud = false;
            unitySubscription_PointCloud.UnSubscribeTopics();
            unitySubscription_AvoidanceCamrea.UnSubscribeTopics();
        }
    }

    void PointCloudUpdateOnce() {
        if (toggle_rosCameraSubscription.isOn)
        {
            UnitySubscription_PointCloud unitySubscription_PointCloud = GameObject.Find("RosColorDepthData").GetComponent<UnitySubscription_PointCloud>();
            unitySubscription_PointCloud.DepthServiceCall();
        }
        else {
            if (!toggle_pointCloudRealRefesh.isOn)
            {
                PointCloudShow.realRefreshPotinCloud = false;
                PointCloudShow.pointCloudShowOnce = true;
            }
        }
    }

    void PointCloudClear()
    {
        UnitySubscription_PointCloud unitySubscription_PointCloud = GameObject.Find("RosColorDepthData").GetComponent<UnitySubscription_PointCloud>();
        unitySubscription_PointCloud.IsClearPointCloud = true;
        PointCloudShow pointCloudShow = GameObject.Find("PointCloudShow").GetComponent<PointCloudShow>();
        pointCloudShow.ClearPointCloud();
    }

    void EyeOnHandCalibrationStart()
    {
        UnitySubscription_PointCloud unitySubscription_PointCloud = GameObject.Find("RosColorDepthData").GetComponent<UnitySubscription_PointCloud>();
        unitySubscription_PointCloud.EyeOnHandCalibrationConfrim();
    }

    /**********************************************ʾ������¼�******************************************/
    void ManualOperateStart()
    {
        AuboMaunalOperatePlan auboMaunalOperatePlan = GameObject.Find("TeachingOperate").GetComponent<AuboMaunalOperatePlan>();
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        auboMaunalOperatePlan.isMaunalOperateMode = !auboMaunalOperatePlan.isMaunalOperateMode;
        if (auboMaunalOperatePlan.isMaunalOperateMode)
        {
            auboMaunalOperatePlan.ClearData();
            btn_manualoperate_start.GetComponentInChildren<TMP_Text>().text = "ʾ����ģʽ����";
            auboControl.RobotControllerSwitch(0);   // 0 controller - AuboApi
            // add user prompt dialog
            ModelDialogControl modelDialogControl = this.GetComponent<ModelDialogControl>();
            modelDialogControl.ShowDialog("ע�⣡��Ҫ��", "������קʾ��ģʽ����ʹ������ģʽ��");
        }
        else
        {
            btn_manualoperate_start.GetComponentInChildren<TMP_Text>().text = "ʾ����ģʽ����";
            auboControl.RobotControllerSwitch(1);   // 1 controller - Ros Controller
        }
    }

    void ManualOperateSavePostionAndOrientation()
    {
        AuboMaunalOperatePlan auboMaunalOperatePlan = GameObject.Find("TeachingOperate").GetComponent<AuboMaunalOperatePlan>();
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        if (auboMaunalOperatePlan.isMaunalOperateMode)
        {
            //PoseMsg realPose = auboControl.m_RealPose;
            PoseMsg realPose = auboControl.m_Transform;
            auboMaunalOperatePlan.AddEndPointPositionAndOrientation(realPose.position, realPose.orientation);
        }
        else if (auboControl.is_TeleOperation)
        {
            // todo ����λ��
            Kinematics kinematics = GameObject.Find("HapticActor_DefaultDevice").GetComponent<Kinematics>();
            double[] pose = kinematics.slaver_current_position;
            double[] quat = kinematics.slaver_current_quat;
            Vector3<FLU> pose_ros = new Vector3<FLU>((float)pose[0], (float)pose[1], (float)pose[2]);
            Quaternion<FLU> quat_ros = new Quaternion<FLU>((float)quat[0], (float)quat[1], (float)quat[2], (float)quat[3]);
            /*            Vector3<FLU> pose_ros = new Vector3((float)pose[0], (float)pose[1], (float)pose[2]).To<FLU>();
                        Quaternion<FLU> quat_ros = new Quaternion((float)quat[0], (float)quat[1], (float)quat[2], (float)quat[3]).To<FLU>();
            */
            auboMaunalOperatePlan.AddEndPointPositionAndOrientation(pose_ros, quat_ros);
        }
    }

    void TeachingOperateBegin()
    {
        int modeIndex = dropdown_teachingOperateMode.value;
        int poseModeIndex = dropdown_selectRobotPlanPose.value;

        RayTest rayTest = GameObject.Find("Main Camera").GetComponent<RayTest>();
        float interpolation_distance = float.Parse(input_point_interpolated_distance.text);
        rayTest.isTeachingOperateOpen = true;
        rayTest.interpolationDistance = interpolation_distance;

        switch (modeIndex)
        {
            case 0:
                rayTest.teachingOperateMode = RayTest.TeachingOperateMode.Line;
                break;
            case 1:
                rayTest.teachingOperateMode = RayTest.TeachingOperateMode.Circle;
                rayTest.teachingOperateParam = float.Parse(input_teachingOperateParam.text);
                break;
            case 2:
                rayTest.teachingOperateMode = RayTest.TeachingOperateMode.Curve;
                rayTest.teachingOperateParam = float.Parse(input_teachingOperateParam.text);
                break;
            case 3:
                rayTest.teachingOperateMode = RayTest.TeachingOperateMode.Point;
                rayTest.teachingOperateParam = float.Parse(input_teachingOperateParam.text);
                break;
        }
        DrawPoint drawPoint = GameObject.Find("TeachingOperate").GetComponent<DrawPoint>();
        drawPoint.deleteAllDrawPoint();
        AuboMaunalOperatePlan auboMaunalOperatePlan = GameObject.Find("TeachingOperate").GetComponent<AuboMaunalOperatePlan>();
        auboMaunalOperatePlan.ClearData();

        switch (poseModeIndex)
        {
            case 0:
                drawPoint.poseMode = DrawPoint.PoseMode.Vertical;
                break;
            case 1:
                drawPoint.poseMode = DrawPoint.PoseMode.Parallel;
                break;
            case 2:
                drawPoint.poseMode = DrawPoint.PoseMode.Normal;
                break;
        }
    }

    void DeleteLastTeachingPoint() {
        DrawPoint drawPoint = GameObject.Find("TeachingOperate").GetComponent<DrawPoint>();
        drawPoint.deleteDrawPoint(drawPoint.drawPointList.Count);
    }

    void RobotPointPlan()
    {
        AuboTrajectoryRequest auboTrajectoryRequest = GameObject.Find("aubo_i5_publish").GetComponent<AuboTrajectoryRequest>();
        // ����ĩ��ƫ��
        try
        {
            float offset = float.Parse(input_toolEndPoint_offset.text.Trim());
            auboTrajectoryRequest.ToolEndSettingOffset(offset);
        }
        catch (Exception e)
        {
            Debug.Log("��RobotPointPlan������ƫ��ʧ��, ex:" + e.Message);
            auboTrajectoryRequest.ToolEndSettingOffset(0.1f);
        }

        // �жϵ�ǰģʽ�Ƿ����ֶ���קʾ��ģʽ
        AuboMaunalOperatePlan auboMaunalOperatePlan = GameObject.Find("TeachingOperate").GetComponent<AuboMaunalOperatePlan>();
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        if (auboMaunalOperatePlan.positon.Count > 0 && auboMaunalOperatePlan.orientation.Count > 0)
        {
            auboMaunalOperatePlan.GetPointPositonAndOrientation(out List<double[]> position, out List<Quaternion<FLU>> orientation);
            List<List<double[]>> request = new List<List<double[]>> {
                position
            };
            List<List<Quaternion<FLU>>> trailOrientationFLU = new List<List<Quaternion<FLU>>> {
                orientation
            };

            auboTrajectoryRequest.PublishMultiRequestEndPoint(request, trailOrientationFLU, 2);
        }
        else
        {
            float interpolation_distance = float.Parse(input_point_interpolated_distance.text);
            DrawPoint drawPoint = GameObject.Find("TeachingOperate").GetComponent<DrawPoint>();
            List<Vector3> points = drawPoint.GetDrawPoints();
            if (points.Count == 0)
            {
                Debug.Log("��DataCommon�����͵ĵ�����Ϊ0");
                DebugGUI.Log("��DataCommon�����͵ĵ�����Ϊ0");
                return;
            }

            // ת����ros����ϵ
            List<double[]> rosPosition = new List<double[]>();
            points.ForEach(item => {
                Vector3<FLU> point = item.To<FLU>();
                rosPosition.Add(new double[] { point.x, point.y, point.z });
            });

            List<List<double[]>> request = new List<List<double[]>> {
                rosPosition
            };
            int type = dropdown_selectRobotPlanPose.value;
            Debug.Log($"���͵��������{rosPosition.Count} ���ͣ�{type}");
            DebugGUI.Log($"���͵��������{rosPosition.Count} ���ͣ�{type}");

            // todo ���������̬
            if (type == 2)
            {
                Vector3[] nors_unity = drawPoint.getPointsNormals(points, 25, Camera.main.transform.position);
                if (nors_unity == null)
                {
                    Debug.Log("��DataCommon�� RobotPointPlan fail: ��ȡ�ķ�����Ϊnull");
                    DebugGUI.Log("��DataCommon�� RobotPointPlan fail: ��ȡ�ķ�����Ϊnull");
                    return;
                }

                Vector3 init_nors = new Vector3(0, -1, 0);  // ��ʼ������
                List<Quaternion<FLU>> trailOrientation = new List<Quaternion<FLU>>();
                for (int i = 0; i < nors_unity.Length; i++)
                {
                    // �����Vector3.forward��targetNormal����ת
                    Quaternion targetRotation = Quaternion.FromToRotation(init_nors, nors_unity[i]);
                    Quaternion initialRotation = Quaternion.Euler(0, 0, -180);
                    // Ӧ��ת����ת����ʼλ��
                    Quaternion finalRotation = targetRotation * initialRotation;
                    Quaternion<FLU> finalRotation_ros = finalRotation.To<FLU>();
                    trailOrientation.Add(finalRotation_ros);
                }
                List<List<Quaternion<FLU>>> trailOrientationFLU = new List<List<Quaternion<FLU>>> {
                    trailOrientation
                };
                auboTrajectoryRequest.PublishMultiRequest(request, trailOrientationFLU, type);

            }
            else
            {
                auboTrajectoryRequest.PublishMultiRequest(request, null, type);
            }
        }
    }

    void RobotPointRePlan()
    {
        AuboTrajectoryRequest auboTrajectoryRequest = GameObject.Find("aubo_i5_publish").GetComponent<AuboTrajectoryRequest>();
        auboTrajectoryRequest.ReStartPlan();
    }

    void RobotPointExc()
    {
        AuboTrajectoryRequest auboTrajectoryRequest = GameObject.Find("aubo_i5_publish").GetComponent<AuboTrajectoryRequest>();
        auboTrajectoryRequest.PublishExecuteRequest();
    }

    /**********************************************Socket����¼�******************************************/
    void SocketConnect() {
        /*        UdpSocketClient udpSocket = GameObject.Find("TeachingOperate").GetComponent<UdpSocketClient>();

                if (udpSocket.isConnect)
                {
                    udpSocket.closeUdp();
                    udpSocket.ip = input_socket_ip.text;
                    udpSocket.port = int.Parse(input_socket_host.text);
                    udpSocket.connectUdp();
                }
                else
                {
                    udpSocket.ip = input_socket_ip.text;
                    udpSocket.port = int.Parse(input_socket_host.text);
                    udpSocket.connectUdp();
                }*/
    }

    void SocketMapConnect() {
        /*        UdpSocketClient udpSocket = GameObject.Find("RawImageNavigationMap").GetComponent<UdpSocketClient>();

                if (udpSocket.isConnect)
                {
                    udpSocket.closeUdp();
                    udpSocket.ip = input_socket_ip2.text;
                    udpSocket.port = int.Parse(input_socket_host2.text);
                    udpSocket.connectUdp();
                }
                else
                {
                    udpSocket.ip = input_socket_ip2.text;
                    udpSocket.port = int.Parse(input_socket_host2.text);
                    udpSocket.connectUdp();
                }*/
    }

    /************************************************PGM����¼�********************************************/
    void PGMUpdateSetting() {
        PGMFileReader pgmFileReader = GameObject.Find("RawImageNavigationMap").GetComponent<PGMFileReader>();
        UnitySubscription_Map unitySubscription_Map = GameObject.Find("RosMapData").GetComponent<UnitySubscription_Map>();
        UnityPublish_MoveCommand unityPublish_MoveCommand = GameObject.Find("RosCarMove").GetComponent<UnityPublish_MoveCommand>();
        pgmFileReader.pgmFileDir = input_pgmFileDir.text;
        pgmFileReader.pgmFilePath = input_pgmFilePath.text;
        pgmFileReader.isUpdatePGMMapOnce = true;
        pgmFileReader.rawImage.enabled = !toggle_closeMapViewSelect.isOn;
        pgmFileReader.lableTitle.enabled = !toggle_closeMapViewSelect.isOn;
        pgmFileReader.isReceviceRosMapTopic = toggle_rosMapSubscriptionSelect.isOn;
        pgmFileReader.isLocalMagnification = toggle_LocalMagnificationSelect.isOn;
        if (toggle_rosMapSubscriptionSelect.isOn)
        {
            unitySubscription_Map.SubscribeTopics();
        }
        else {
            unitySubscription_Map.UnSubscribeTopics();
        }

        try
        {
            float speed = float.Parse(input_carSpeedSetting.text);
            unityPublish_MoveCommand.SetSpeed(speed);
        }
        catch (Exception ex)
        {
            Debug.Log("��PGMUpdateSetting���ٶ�����ʧ�� ex:" + ex.Message);
            DebugGUI.Log("��PGMUpdateSetting���ٶ�����ʧ�� ex:" + ex.Message);
            unityPublish_MoveCommand.SetSpeed(unityPublish_MoveCommand.min_speed);
        }
    }

    void AstarPlanPath()
    {
        PGMFileReader pgmFileReader = GameObject.Find("RawImageNavigationMap").GetComponent<PGMFileReader>();
        pgmFileReader.AstarPlan();
    }

    void SendPGMMousePosition() {
        PGMFileReader pgmFileReader = GameObject.Find("RawImageNavigationMap").GetComponent<PGMFileReader>();
        Vector2 target = pgmFileReader.mouseDownPositon_actual;
        UnitySubscription_Map naviMap = GameObject.Find("RosMapData").GetComponent<UnitySubscription_Map>();
        naviMap.SendTargetPosition(target);
    }

    /**********************************************Aubo-i5����¼�******************************************/
    void AuboJointAndPoseRefresh() {
        GameObject aubo_i5_publish = GameObject.Find("aubo_i5_publish");
        if (aubo_i5_publish == null)
        {
            return;
        }
        AuboControl auboControl = aubo_i5_publish.GetComponent<AuboControl>();
        if (auboControl == null) {
            return;
        }

        double[] realJoints = auboControl.m_RealJointsState;
        PoseMsg realPose = auboControl.m_RealPose;
        double[] virtualJoint = auboControl.m_VirtualJointsState;

        if (realJoints != null && realJoints.Length == 6) {
            laber_real_joint_1_angle.text = String.Format("{0:F3}��", realJoints[0] * Mathf.Rad2Deg);
            laber_real_joint_2_angle.text = String.Format("{0:F3}��", realJoints[1] * Mathf.Rad2Deg);
            laber_real_joint_3_angle.text = String.Format("{0:F3}��", realJoints[2] * Mathf.Rad2Deg);
            laber_real_joint_4_angle.text = String.Format("{0:F3}��", realJoints[3] * Mathf.Rad2Deg);
            laber_real_joint_5_angle.text = String.Format("{0:F3}��", realJoints[4] * Mathf.Rad2Deg);
            laber_real_joint_6_angle.text = String.Format("{0:F3}��", realJoints[5] * Mathf.Rad2Deg);
        }

        if (realPose != null) {
            laber_real_pose_x.text = String.Format("{0:F2}", realPose.position.x);
            laber_real_pose_y.text = String.Format("{0:F2}", realPose.position.y);
            laber_real_pose_z.text = String.Format("{0:F2}", realPose.position.z);
            laber_real_pose_rx.text = String.Format("{0:F6}", realPose.orientation.x);
            laber_real_pose_ry.text = String.Format("{0:F6}", realPose.orientation.y);
            laber_real_pose_rz.text = String.Format("{0:F6}", realPose.orientation.z);
            laber_real_pose_rw.text = String.Format("{0:F6}", realPose.orientation.w);
        }

        if (virtualJoint != null && virtualJoint.Length == 6) {
            laber_virtual_joint_1_angle.text = String.Format("{0:F3}��", virtualJoint[0] * Mathf.Rad2Deg);
            laber_virtual_joint_2_angle.text = String.Format("{0:F3}��", virtualJoint[1] * Mathf.Rad2Deg);
            laber_virtual_joint_3_angle.text = String.Format("{0:F3}��", virtualJoint[2] * Mathf.Rad2Deg);
            laber_virtual_joint_4_angle.text = String.Format("{0:F3}��", virtualJoint[3] * Mathf.Rad2Deg);
            laber_virtual_joint_5_angle.text = String.Format("{0:F3}��", virtualJoint[4] * Mathf.Rad2Deg);
            laber_virtual_joint_6_angle.text = String.Format("{0:F3}��", virtualJoint[5] * Mathf.Rad2Deg);
        }

        if (!string.IsNullOrEmpty(input_aubo_speed.text))
        {
            auboControl.m_JointVelocity = double.Parse(input_aubo_speed.text);
        }
    }

    void AuboTeleOperateMode()
    {
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        auboControl.is_TeleOperation = !auboControl.is_TeleOperation;
        AuboMaunalOperatePlan auboMaunalOperatePlan = GameObject.Find("TeachingOperate").GetComponent<AuboMaunalOperatePlan>();
        auboMaunalOperatePlan.isTeleOperateMode = auboControl.is_TeleOperation;
        if (auboControl.is_TeleOperation)
        {
            auboMaunalOperatePlan.ClearData();
            btn_aubo_teleOperate.GetComponentInChildren<TMP_Text>().text = "ң����ģʽ����";

            Kinematics kinematics = GameObject.Find("HapticActor_DefaultDevice").GetComponent<Kinematics>();
            switch (dropDown_aubo_teleOperateMode.value)
            {
                case 0:
                    kinematics.is_angular_only = false;
                    kinematics.is_linear_only = false;
                    break;
                case 1:
                    kinematics.is_angular_only = false;
                    kinematics.is_linear_only = true;
                    break;
                case 2:
                    kinematics.is_angular_only = true;
                    kinematics.is_linear_only = false;
                    break;
            }

            // add user prompt dialog
            ModelDialogControl modelDialogControl = this.GetComponent<ModelDialogControl>();
            modelDialogControl.ShowDialog("ע�⣡��Ҫ��", "����ң����ģʽ����ʹ������ģʽ��");
        }
        else
        {
            btn_aubo_teleOperate.GetComponentInChildren<TMP_Text>().text = "ң����ģʽ����";
        }
    }

    public void AuboJointHome() {
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        auboControl.AuboToHome();
    }

    void AuboSettingSyncV2R()
    {
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        if (!string.IsNullOrEmpty(input_aubo_speed.text))
        {
            auboControl.m_JointVelocity = double.Parse(input_aubo_speed.text);
        }

        if (toggle_aubo_send_isJoint.isOn) {

            double[] virtual_joints = auboControl.m_VirtualJointsState;
            double joint_1 = virtual_joints[0];
            if (!string.IsNullOrEmpty(input_aubo_joint_1_angle.text))
                joint_1 = double.Parse(input_aubo_joint_1_angle.text) * Mathf.Deg2Rad;

            double joint_2 = virtual_joints[1];
            if (!string.IsNullOrEmpty(input_aubo_joint_2_angle.text))
                joint_2 = double.Parse(input_aubo_joint_2_angle.text) * Mathf.Deg2Rad;

            double joint_3 = virtual_joints[2];
            if (!string.IsNullOrEmpty(input_aubo_joint_3_angle.text))
                joint_3 = double.Parse(input_aubo_joint_3_angle.text) * Mathf.Deg2Rad;

            double joint_4 = virtual_joints[3];
            if (!string.IsNullOrEmpty(input_aubo_joint_4_angle.text))
                joint_4 = double.Parse(input_aubo_joint_4_angle.text) * Mathf.Deg2Rad;

            double joint_5 = virtual_joints[4];
            if (!string.IsNullOrEmpty(input_aubo_joint_5_angle.text))
                joint_5 = double.Parse(input_aubo_joint_5_angle.text) * Mathf.Deg2Rad;

            double joint_6 = virtual_joints[5];
            if (!string.IsNullOrEmpty(input_aubo_joint_6_angle.text))
                joint_6 = double.Parse(input_aubo_joint_6_angle.text) * Mathf.Deg2Rad;

            double[] joints = new double[] { joint_1, joint_2, joint_3, joint_4, joint_5, joint_6 };
            auboControl.SycRequest(joints, new PoseMsg(), true);
        }
        else
        {
            /*            if (string.IsNullOrEmpty(input_aubo_endPoint_x.text) ||
                            string.IsNullOrEmpty(input_aubo_endPoint_y.text) ||
                            string.IsNullOrEmpty(input_aubo_endPoint_z.text) ||
                            string.IsNullOrEmpty(input_aubo_endPoint_rx.text) ||
                            string.IsNullOrEmpty(input_aubo_endPoint_ry.text) ||
                            string.IsNullOrEmpty(input_aubo_endPoint_rz.text) ||
                            string.IsNullOrEmpty(input_aubo_endPoint_rw.text))
                        {
                            input_aubo_endPoint_x.text = string.Empty;
                            input_aubo_endPoint_y.text = string.Empty;
                            input_aubo_endPoint_z.text = string.Empty;
                            input_aubo_endPoint_rx.text = string.Empty;
                            input_aubo_endPoint_ry.text = string.Empty;
                            input_aubo_endPoint_rz.text = string.Empty;
                            input_aubo_endPoint_rw.text = string.Empty;

                            double joint_1 = double.Parse(laber_virtual_joint_1_angle.text);
                            double joint_2 = double.Parse(laber_virtual_joint_2_angle.text);
                            double joint_3 = double.Parse(laber_virtual_joint_3_angle.text);
                            double joint_4 = double.Parse(laber_virtual_joint_4_angle.text);
                            double joint_5 = double.Parse(laber_virtual_joint_5_angle.text);
                            double joint_6 = double.Parse(laber_virtual_joint_6_angle.text);
                            double[] joints = new double[] { joint_1, joint_2, joint_3, joint_4, joint_5, joint_6 };
                            auboControl.SycRequest(joints, new PoseMsg(), true);
                            return;
                        }
                        else
                        {
                            double x = double.Parse(input_aubo_endPoint_x.text);
                            double y = double.Parse(input_aubo_endPoint_y.text);
                            double z = double.Parse(input_aubo_endPoint_z.text);
                            double rx = double.Parse(input_aubo_endPoint_rx.text);
                            double ry = double.Parse(input_aubo_endPoint_ry.text);
                            double rz = double.Parse(input_aubo_endPoint_rz.text);
                            double rw = double.Parse(input_aubo_endPoint_rw.text);
                            PoseMsg poseMsg = new PoseMsg(new PointMsg(x, y, z), new QuaternionMsg(rx, ry, rz, rw));
                            auboControl.SycRequest(new double[6], poseMsg, false);
                        }*/
            return;
        }
    }

    void AuboSettingSyncR2V()
    {
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        double[] realjoints = auboControl.m_RealJointsState;
        auboControl.SetJointState(realjoints);
    }

    /**********************************************�ߵµ�ͼ����¼�******************************************/
    void GaodeRefresh()
    {
        string longitude = input_gaode_longitude.text;
        string latitude = input_gaode_latitude.text;
        string ip = input_gaode_ip.text;
        GaoDeApi gaoDeApi = GetComponent<GaoDeApi>();
        gaoDeApi.ZOOM = slider_gaode_mapsize.value.ToString();
        gaoDeApi.RefreshRequest(ip, longitude, latitude);
    }

    /**********************************************��ͣ����¼�******************************************/
    public void RobotStop()
    {
        isBtnRobotStopDown = !isBtnRobotStopDown | isBtnEverythingStopDown;
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        if (isBtnRobotStopDown)
        {
            btn_robot_stop.image.color = StopDownColor;
            auboControl.ArmEmergencyAction(true);
        }
        else
        {
            btn_robot_stop.image.color = StopUpColor;
            auboControl.ArmEmergencyAction(false);
        }

    }

    public void CarStop()
    {
        isBtnCarStopDown = !isBtnCarStopDown | isBtnEverythingStopDown;
        if (isBtnCarStopDown)
        {
            UnitySubscription_Map unitySubscription_Map = GameObject.Find("RosMapData").GetComponent<UnitySubscription_Map>();
            unitySubscription_Map.NaviCancel();
            btn_car_stop.image.color = StopDownColor;
        }
        else
        {
            btn_car_stop.image.color = StopUpColor;
        }
    }

    public void EveryThingStop()
    {
        isBtnEverythingStopDown = !isBtnEverythingStopDown;

        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        UnitySubscription_Map unitySubscription_Map = GameObject.Find("RosMapData").GetComponent<UnitySubscription_Map>();

        if (isBtnEverythingStopDown)
        {
            // robot stop down
            auboControl.ArmEmergencyAction(true);
            isBtnRobotStopDown = true;
            btn_robot_stop.image.color = StopDownColor;
            // car stop down
            unitySubscription_Map.NaviCancel();
            isBtnCarStopDown = true;
            btn_car_stop.image.color = StopDownColor;
            btn_everything_stop.image.color = StopDownColor;
        }
        else
        {
            // robot stop up
            btn_everything_stop.image.color = StopUpColor;
            auboControl.ArmEmergencyAction(false);
            isBtnRobotStopDown = false;
            // car stop up
            isBtnCarStopDown = false;
            btn_robot_stop.image.color = StopUpColor;
            btn_car_stop.image.color = StopUpColor;
        }
    }

    /**********************************************�������¼�******************************************/
    void ControlBoxSerialportSearch()
    {
        SerialPortControl serialPortControl = GameObject.Find("ControlBoxControl").GetComponent<SerialPortControl>();
        string[] availablePorts = serialPortControl.GetAvailablePorts();
        dropdowm_portname.ClearOptions();
        dropdowm_portname.AddOptions(new List<string>(availablePorts));
        dropdowm_portname.value = 0;
        dropdowm_portname.RefreshShownValue();
    }

    void ControlBoxSerialportOpen()
    {
        try
        {
            string portname = dropdowm_portname.options[dropdowm_portname.value].text;
            int baudrate = int.Parse(dropdown_baudrate.options[dropdown_baudrate.value].text);
            SerialPortControl serialPortControl = GameObject.Find("ControlBoxControl").GetComponent<SerialPortControl>();
            serialPortControl.OpenSerialPorts(portname, baudrate);
        }
        catch (Exception ex)
        {
            Debug.Log("��DataCommon��ControlBoxSerialportOpen error\n" + ex);
            DebugGUI.Log("��DataCommon��ControlBoxSerialportOpen error\n" + ex);
        }
    }

    void ControlBoxSerialportClose()
    {
        SerialPortControl serialPortControl = GameObject.Find("ControlBoxControl").GetComponent<SerialPortControl>();
        serialPortControl.CloseSerialPort();
    }

    /**********************************************PLCϵͳ******************************************/
    void ReadPLCData()
    {
        // todo
        PlcConnect plcConnect = GameObject.Find("PlcCommunication").GetComponent<PlcConnect>();

        uISwitcherLaserEnable.isOn = plcConnect.read_laser_enable;
        uISwitcherOpenInstruction.isOn = plcConnect.read_open_instruction;
        uISwitcherLaserOutput.isOn = plcConnect.read_laser_working;
        uISwitcherLaserReset.isOn = plcConnect.read_plc_reset;
        input_laserPower.text = plcConnect.read_current_power.ToString();
        DebugGUI.LogString(($"��PLCRead Laser��ʹ��:{plcConnect.read_laser_enable} ָ����:{plcConnect.read_open_instruction} ����:{plcConnect.read_laser_working}" +
            $"��λ:{plcConnect.read_plc_reset} ����:{plcConnect.read_current_power}"));
        Debug.Log($"��PLCRead Laser��ʹ��:{plcConnect.read_laser_enable} ָ����:{plcConnect.read_open_instruction} ����:{plcConnect.read_laser_working}" +
            $"��λ:{plcConnect.read_plc_reset} ����:{plcConnect.read_current_power}");

        uISwitcherOpenGas.isOn = plcConnect.read_open_gas;
        uISwitcherOpenPowder.isOn = plcConnect.read_open_powder;
        input_powderSpeed.text = plcConnect.read_current_speed.ToString();
        input_product.text = plcConnect.read_current_product.ToString();
        input_technology.text = plcConnect.read_current_technology.ToString();
        DebugGUI.LogString(($"��PLCRead GasPowder������:{plcConnect.read_open_gas} �ͷ�:{plcConnect.read_open_powder} �ͷ�ת��:{plcConnect.read_current_speed}" +
            $"��Ʒ:{plcConnect.read_current_product} ����:{plcConnect.read_current_technology}"));
        Debug.Log($"��PLCRead GasPowder������:{plcConnect.read_open_gas} �ͷ�:{plcConnect.read_open_powder} �ͷ�ת��:{plcConnect.read_current_speed}" +
            $"��Ʒ:{plcConnect.read_current_product} ����:{plcConnect.read_current_technology}");

        uISwitcherPLCPulse.isOn = plcConnect.read_plc_pulse;
        uISwitcherPLCReady.isOn = plcConnect.read_plc_ready;
        uISwitcherPLCAutomatic.isOn = plcConnect.read_plc_automatic;

        DebugGUI.LogString($"��PLCRead�������ź�:{plcConnect.read_plc_pulse} �����ź�:{plcConnect.read_plc_ready} �Զ�ģʽ:{plcConnect.read_plc_automatic}");
        Debug.Log($"��PLCRead�������ź�:{plcConnect.read_plc_pulse} �����ź�:{plcConnect.read_plc_ready} �Զ�ģʽ:{plcConnect.read_plc_automatic}");

        DebugGUI.LogString($"��PLCRead���¶�:{plcConnect.read_env_temp} ʪ��:{plcConnect.read_env_humi}");
        Debug.Log($"��PLCRead���¶�:{plcConnect.read_env_temp} ʪ��:{plcConnect.read_env_humi}");
    }

    void UpdatePLCData()
    {
        PlcConnect plcConnect = GameObject.Find("PlcCommunication").GetComponent<PlcConnect>();

        plcConnect.write_laser_enable = uISwitcherLaserEnable.isOn;
        plcConnect.write_open_instruction = uISwitcherOpenInstruction.isOn;
        plcConnect.write_laser_output = uISwitcherLaserOutput.isOn;
        plcConnect.write_laser_reset = uISwitcherLaserReset.isOn;
        plcConnect.write_current_power = ConvertUshortByString(input_laserPower.text, plcConnect.read_current_power);
        DebugGUI.LogString(($"��PLCWrite Laser��ʹ��:{plcConnect.write_laser_enable} ָ����:{plcConnect.write_open_instruction} ����:{plcConnect.write_laser_output}" +
            $"��λ:{plcConnect.write_laser_reset} ����:{plcConnect.write_current_power}"));
        Debug.Log($"��PLCWrite Laser��ʹ��:{plcConnect.write_laser_enable} ָ����:{plcConnect.write_open_instruction} ����:{plcConnect.write_laser_output}" +
            $"��λ:{plcConnect.write_laser_reset} ����:{plcConnect.write_current_power}");

        plcConnect.write_open_gas = uISwitcherOpenGas.isOn;
        plcConnect.write_open_powder = uISwitcherOpenPowder.isOn;
        plcConnect.write_powdering = uISwitcherPowdering.isOn;
        plcConnect.write_current_speed = ConvertUshortByString(input_powderSpeed.text, plcConnect.read_current_speed);
        plcConnect.write_current_product = ConvertUshortByString(input_product.text, plcConnect.read_current_product);
        plcConnect.write_current_technology = ConvertUshortByString(input_technology.text, plcConnect.read_current_technology);
        DebugGUI.LogString(($"��PLCWrite GasPowder������:{plcConnect.write_open_gas} �ͷ�ʹ��:{plcConnect.write_open_powder} �ͷ�:{plcConnect.write_powdering}" +
            $"�ͷ�ת��:{plcConnect.write_current_speed} ��Ʒ:{plcConnect.write_current_product} ����:{plcConnect.write_current_technology}"));
        Debug.Log($"��PLCWrite GasPowder������:{plcConnect.write_open_gas} �ͷ�ʹ��:{plcConnect.write_open_powder} �ͷ�:{plcConnect.write_powdering}" +
            $"�ͷ�ת��:{plcConnect.write_current_speed} ��Ʒ:{plcConnect.write_current_product} ����:{plcConnect.write_current_technology}");

        plcConnect.write_plc_pulse = uISwitcherPLCPulse.isOn;
        plcConnect.write_plc_ready = uISwitcherPLCReady.isOn;
        plcConnect.write_plc_automatic = uISwitcherPLCAutomatic.isOn;
        plcConnect.WriteToPlcData((int)LaserModeChoose.Setting);
        DebugGUI.LogString($"��PLCWrite��ģʽ��{LaserModeChoose.Setting} �����ź�:{plcConnect.write_plc_pulse} �����ź�:{plcConnect.write_plc_ready} �Զ�ģʽ:{plcConnect.write_plc_automatic}");
        Debug.Log($"��PLCWrite��ģʽ��{LaserModeChoose.Setting} �����ź�:{plcConnect.write_plc_pulse} �����ź�:{plcConnect.write_plc_ready} �Զ�ģʽ:{plcConnect.write_plc_automatic}");
    }

    ushort ConvertUshortByString(string numberStr, ushort oldData)
    {
        if (string.IsNullOrEmpty(numberStr))
        {
            Debug.Log($"��convertUshortByString error����������Ϊ��;oldData:{oldData}");
            DebugGUI.LogString($"��convertUshortByString error����������Ϊ��;oldData:{oldData}");
            return oldData;
        }

        try
        {
            ushort number = UInt16.Parse(numberStr);
            return number;
        }
        catch (Exception)
        {
            Debug.Log($"��convertUshortByString error��numberStr:{numberStr} ת��ʧ��;oldData:{oldData}");
            DebugGUI.LogString($"��convertUshortByString error��numberStr:{numberStr} ת��ʧ��;oldData:{oldData}");
            return oldData;
        }
    }

    void OpenLaser()
    {
        PlcConnect plcConnect = GameObject.Find("PlcCommunication").GetComponent<PlcConnect>();
        if (laserModeChoose.Equals(LaserModeChoose.Close))
        {
            laserModeChoose = LaserModeChoose.Open;
            plcConnect.WriteToPlcData((int)laserModeChoose);
            btn_open_laser.GetComponentInChildren<TMP_Text>().text = "�رռ�����";
        }
        else if (laserModeChoose.Equals(LaserModeChoose.Open))
        {
            laserModeChoose = LaserModeChoose.Close;
            plcConnect.WriteToPlcData((int)laserModeChoose);
            btn_open_laser.GetComponentInChildren<TMP_Text>().text = "�򿪼�����";
        }

        DebugGUI.LogString($"��PLCWrite����ǰģʽ��{laserModeChoose}");
        Debug.Log($"��PLCWrite����ǰģʽ��{laserModeChoose}");
    }
}