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
    #region 点云相关按钮、变量
    /***************************************点云相关*************************************/
    [Header("********点云相关********")]
    [Header("点云所在的文件夹")]
    public TMP_InputField input_pointCloudFilePath; //点云所在的文件夹
    [Header("点云更新延迟时间")]
    public TMP_InputField input_pointCloudUpdateSecond; // 点云更新延迟时间
    [Header("点云缩放系数")]
    public TMP_InputField input_pointCloudScala; // 点云缩放系数
    [Header("是否点云三角化")]
    public Toggle toggle_pointTriangulation; // 是否点云三角化
    [Header("是否调换YZ数据")]
    public Toggle toggle_inverseYZ; // 是否调换YZ数据
    [Header("是否订阅ROS相机topic")]
    public Toggle toggle_rosCameraSubscription; // 是否订阅ROS相机topic
    [Header("是否订阅ROS相机topic")]
    public TMP_Dropdown dropdown_CameraShow; // 是否订阅ROS相机topic
    [Header("是否保存多视角点云")]
    public Toggle toggle_mulViewPointCloud; // 是否保存多视角点云
    [Header("是否实时刷新")]
    public Toggle toggle_pointCloudRealRefesh; // 是否实时刷新
    [Header("擦除点云")]
    public Button btn_clear_pointcloud;    // 擦除点云
    [Header("更新一次")]
    public Button btn_update_once;    // 更新一次
    [Header("开启手眼标定")]
    public Toggle toggle_eyeOnHandCalibration; // 是否开启手眼标定
    [Header("手眼标定结果确认")]
    public Button btn_eyeOnHandCalibrationStart; // 手眼标定结果计算开始
    [Header("更新设置")]
    public Button btn_update_setting; // 更新设置

    public static string PointCloudFilePath = "";       //点云文件路径
    public static int PointCloudUpdateSecond = 5;       // 更新时间
    public static float PointCloudScala = 0.001f;       // 点云缩放比例
    public static bool isPointTriangulation = true;     // 是否点云三角化
    public static bool isinverseYZ = true;              // 是否调换YZ
    public static bool isRosCameraSubscription = false; // 是否订阅ROS相机topic
    public static bool isMulViewPointCloud = false;     // 是否多视角保存点云
    public static bool isRealRefeshPointCloud = false;  // 是否实时更新点云
    public static bool isPointCloudShowOnce = false;    // 单次更新
    #endregion

    #region Socket相关按钮、变量
    /***************************************Socket相关*************************************/
    [Space(10)]
    [Header("********Socket相关********")]
    [Header("示教点ip地址")]
    public TMP_InputField input_socket_ip;
    [Header("示教点端口号")]
    public TMP_InputField input_socket_host;
    [Header("socket连接按钮")]
    public Button btn_socket_connect;    // socket 连接 点云数据传输
    [Header("Mapip地址")]
    public TMP_InputField input_socket_ip2;
    [Header("Map端口号")]
    public TMP_InputField input_socket_host2;
    [Header("socket连接按钮")]
    public Button btn_socket_map_connect;    // socket 连接 map点数据传输

    public static string socket_ip = "";
    public static string socket_host = "";
    #endregion

    #region 示教相关按钮、变量
    /***************************************示教相关*************************************/
    [Space(10)]
    [Header("********示教相关********")]
    [Header("手动拖拽示教开始")]
    public Button btn_manualoperate_start; // 是否显示鼠标点击坐标
    [Header("手动拖拽示教数据存储")]
    public Button btn_manualoperate_save; // 是否显示鼠标点击坐标
    [Header("是否显示鼠标点击坐标")]
    public Toggle toggle_pointCloudMouseShow; // 是否显示鼠标点击坐标
    [Header("鼠标点击当前的坐标")]
    public TMP_InputField input_mousePointLoction; // 鼠标点击当前的坐标
    [Header("示教模式")]
    public TMP_Dropdown dropdown_teachingOperateMode; // 直线0 画圆1 回字2
    [Header("示教模式对应的参数")]
    public TMP_InputField input_teachingOperateParam; // 对应参数，圆则是半径 回字则表示回线之间的宽度
    [Header("示教操作")]
    public Button btn_teaching_operate;
    [Header("删除上一个点")]
    public Button btn_delete_lastPoint;
    [Header("插值间隔")]
    public TMP_InputField input_point_interpolated_distance;
    [Header("选择执行位姿")]
    public TMP_Dropdown dropdown_selectRobotPlanPose;
    [Header("末端工具偏移")]
    public TMP_InputField input_toolEndPoint_offset;
    [Header("规划")]
    public Button btn_robot_plan;
    [Header("重新规划")]
    public Button btn_robot_replan;
    [Header("执行")]
    public Button btn_robot_exc;

    public static bool isShowMousePoint = true;         // 显示鼠标点的坐标
    public static string MousePointLocation = "";       // 鼠标点的坐标点格式化
    #endregion

    #region PGM地图相关代码、变量
    /***************************************PGM地图相关*************************************/
    [Space(10)]
    [Header("********PGM地图相关********")]
    [Header("pgm所在的文件夹")]
    public TMP_InputField input_pgmFileDir;  // pgm文件所在的文件夹
    [Header("pgm绝对路径")]
    public TMP_InputField input_pgmFilePath; // pgm文件所在的绝对路径
    [Header("是否关闭地图显示")]
    public Toggle toggle_closeMapViewSelect;
    [Header("是否订阅ros地图话题")]
    public Toggle toggle_rosMapSubscriptionSelect;
    [Header("是否视图局部放大")]
    public Toggle toggle_LocalMagnificationSelect;
    [Header("速度显示与设置")]
    public TMP_InputField input_carSpeedSetting;
    [Header("更新Map设置")]
    public Button btn_map_settingUpdate;
    [Header("A星算法规划")]
    public Button btn_AstarPlanPath;
    [Header("地图上的坐标")]
    public TMP_InputField input_pgmMousePosition;
    [Header("socket发送PGM地图点的位置")]
    public Button btn_sendPGMMousePosition;
    #endregion

    #region Aubo机械臂相关
    /***************************************Aubo机械臂相关*************************************/
    [Space(10)]
    [Header("********Aubo机械臂相关********")]
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

    #region 高德地图相关
    /***************************************高德地图相关*************************************/
    [Space(10)]
    [Header("********高德地图相关********")]
    public TMP_InputField input_gaode_longitude;
    public TMP_InputField input_gaode_latitude;
    public TMP_InputField input_gaode_ip;
    public Slider slider_gaode_mapsize;
    public Button btn_gaode_refresh;
    #endregion

    #region 急停面板
    /***************************************急停面板*************************************/
    [Space(10)]
    [Header("********急停面板相关********")]
    public Button btn_robot_stop;
    public Button btn_car_stop;
    public Button btn_everything_stop;

    private bool isBtnRobotStopDown = false;
    private bool isBtnCarStopDown = false;
    private bool isBtnEverythingStopDown = false;
    private Color StopDownColor = new Color(0.6f, 0.2f, 0.2f);
    private Color StopUpColor = new Color(0.2f, 0.6f, 0.2f);
    #endregion

    #region 控制箱面板
    /***************************************控制箱面板*************************************/
    [Space(10)]
    [Header("********控制箱面板********")]
    public Button btn_serialport_search;
    public TMP_Dropdown dropdowm_portname;
    public TMP_Dropdown dropdown_baudrate;
    public Button btn_serialport_open;
    public Button btn_serialport_close;
    #endregion

    #region PLC控制
    /***************************************PLC控制*************************************/
    [Space(10)]
    [Header("********PLC控制********")]
    [Header("读取按钮")]
    public Button btn_read_plc;
    [Header("UISwitcher激光器使能")]
    public UISwitcher.UISwitcher uISwitcherLaserEnable;
    [Header("UISwitcher指导光使能")]
    public UISwitcher.UISwitcher uISwitcherOpenInstruction;
    [Header("UISwitcher激光器出光")]
    public UISwitcher.UISwitcher uISwitcherLaserOutput;
    [Header("UISwitcher激光器复位")]
    public UISwitcher.UISwitcher uISwitcherLaserReset;
    [Header("input激光器功率")]
    public TMP_InputField input_laserPower;
    [Header("UISwitcher气体使能")]
    public UISwitcher.UISwitcher uISwitcherOpenGas;
    [Header("UISwitcher送粉使能")]
    public UISwitcher.UISwitcher uISwitcherOpenPowder;
    [Header("UISwitcher送粉")]
    public UISwitcher.UISwitcher uISwitcherPowdering;
    [Header("input送粉器转速")]
    public TMP_InputField input_powderSpeed;
    [Header("input加工产品")]
    public TMP_InputField input_product;
    [Header("input加工工艺")]
    public TMP_InputField input_technology;
    [Header("UISwitcher脉冲信号")]
    public UISwitcher.UISwitcher uISwitcherPLCPulse;
    [Header("UISwitcher就绪信号")]
    public UISwitcher.UISwitcher uISwitcherPLCReady;
    [Header("UISwitcher自动模式")]
    public UISwitcher.UISwitcher uISwitcherPLCAutomatic;
    [Header("更新按钮")]
    public Button btn_update_plc;
    [Header("打开激光器")]
    public Button btn_open_laser;

    // 大于2时：仅开启或关闭激光器  偶数：开启激光器  奇数：关闭激光器
    // 小于等于2时 设置相应参数
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
        // 键盘按钮保存点云
        if (Input.GetKey(KeyCode.Space) && Input.GetKeyDown(KeyCode.P))
        {
            DebugGUI.LogString("【keyboard Listener】保存点云数据");
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

            Debug.Log("【DataCommon】 RefreshUIData ex:" + ex.Message);
            DebugGUI.Log("【DataCommon】 RefreshUIData ex:" + ex.Message);
        }
    }

    /**********************************************点云相关事件******************************************/
    void PointCloudUpdateSetting() {
        RefreshPointCloudUIData();
        PointCloudShow.realRefreshPotinCloud = isRealRefeshPointCloud;
        PointCloudShow.pointCloudDelayShow = PointCloudUpdateSecond;
        PointCloudShow.pointCloudDirPath = PointCloudFilePath;
        PointCloudShow.scale = PointCloudScala;
        PointCloudShow.invertYZ = isinverseYZ;
        PointCloudShow.isPointTriangle = isPointTriangulation;

        // 更新ros订阅显示的配置
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
                unitySubscription_PointCloud.SubscribeTopics(true, false); // 订阅彩色图，不订阅深度图
                unitySubscription_PointCloud.DepthImageRegisterServiceCall(); // 订阅深度图的服务
                unitySubscription_AvoidanceCamrea.UnSubscribeTopics(); // 关闭导航的订阅
            }
            else if (dropdown_CameraShow.value > 2)
            {
                unitySubscription_AvoidanceCamrea.SubscribeTopics(true, true); // 订阅导航彩色图，订阅深度图
                unitySubscription_PointCloud.UnSubscribeTopics(); // 关闭点云的订阅
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

    /**********************************************示教相关事件******************************************/
    void ManualOperateStart()
    {
        AuboMaunalOperatePlan auboMaunalOperatePlan = GameObject.Find("TeachingOperate").GetComponent<AuboMaunalOperatePlan>();
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        auboMaunalOperatePlan.isMaunalOperateMode = !auboMaunalOperatePlan.isMaunalOperateMode;
        if (auboMaunalOperatePlan.isMaunalOperateMode)
        {
            auboMaunalOperatePlan.ClearData();
            btn_manualoperate_start.GetComponentInChildren<TMP_Text>().text = "示教器模式：开";
            auboControl.RobotControllerSwitch(0);   // 0 controller - AuboApi
            // add user prompt dialog
            ModelDialogControl modelDialogControl = this.GetComponent<ModelDialogControl>();
            modelDialogControl.ShowDialog("注意！重要！", "开启拖拽示教模式后不能使用其他模式！");
        }
        else
        {
            btn_manualoperate_start.GetComponentInChildren<TMP_Text>().text = "示教器模式：关";
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
            // todo 虚拟位姿
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
        // 更新末端偏移
        try
        {
            float offset = float.Parse(input_toolEndPoint_offset.text.Trim());
            auboTrajectoryRequest.ToolEndSettingOffset(offset);
        }
        catch (Exception e)
        {
            Debug.Log("【RobotPointPlan】设置偏移失败, ex:" + e.Message);
            auboTrajectoryRequest.ToolEndSettingOffset(0.1f);
        }

        // 判断当前模式是否是手动拖拽示教模式
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
                Debug.Log("【DataCommon】发送的点数量为0");
                DebugGUI.Log("【DataCommon】发送的点数量为0");
                return;
            }

            // 转换到ros坐标系
            List<double[]> rosPosition = new List<double[]>();
            points.ForEach(item => {
                Vector3<FLU> point = item.To<FLU>();
                rosPosition.Add(new double[] { point.x, point.y, point.z });
            });

            List<List<double[]>> request = new List<List<double[]>> {
                rosPosition
            };
            int type = dropdown_selectRobotPlanPose.value;
            Debug.Log($"发送点的数量：{rosPosition.Count} 类型：{type}");
            DebugGUI.Log($"发送点的数量：{rosPosition.Count} 类型：{type}");

            // todo 传入相对姿态
            if (type == 2)
            {
                Vector3[] nors_unity = drawPoint.getPointsNormals(points, 25, Camera.main.transform.position);
                if (nors_unity == null)
                {
                    Debug.Log("【DataCommon】 RobotPointPlan fail: 获取的法向量为null");
                    DebugGUI.Log("【DataCommon】 RobotPointPlan fail: 获取的法向量为null");
                    return;
                }

                Vector3 init_nors = new Vector3(0, -1, 0);  // 初始法向量
                List<Quaternion<FLU>> trailOrientation = new List<Quaternion<FLU>>();
                for (int i = 0; i < nors_unity.Length; i++)
                {
                    // 计算从Vector3.forward到targetNormal的旋转
                    Quaternion targetRotation = Quaternion.FromToRotation(init_nors, nors_unity[i]);
                    Quaternion initialRotation = Quaternion.Euler(0, 0, -180);
                    // 应用转换旋转到初始位姿
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

    /**********************************************Socket相关事件******************************************/
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

    /************************************************PGM相关事件********************************************/
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
            Debug.Log("【PGMUpdateSetting】速度设置失败 ex:" + ex.Message);
            DebugGUI.Log("【PGMUpdateSetting】速度设置失败 ex:" + ex.Message);
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

    /**********************************************Aubo-i5相关事件******************************************/
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
            laber_real_joint_1_angle.text = String.Format("{0:F3}°", realJoints[0] * Mathf.Rad2Deg);
            laber_real_joint_2_angle.text = String.Format("{0:F3}°", realJoints[1] * Mathf.Rad2Deg);
            laber_real_joint_3_angle.text = String.Format("{0:F3}°", realJoints[2] * Mathf.Rad2Deg);
            laber_real_joint_4_angle.text = String.Format("{0:F3}°", realJoints[3] * Mathf.Rad2Deg);
            laber_real_joint_5_angle.text = String.Format("{0:F3}°", realJoints[4] * Mathf.Rad2Deg);
            laber_real_joint_6_angle.text = String.Format("{0:F3}°", realJoints[5] * Mathf.Rad2Deg);
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
            laber_virtual_joint_1_angle.text = String.Format("{0:F3}°", virtualJoint[0] * Mathf.Rad2Deg);
            laber_virtual_joint_2_angle.text = String.Format("{0:F3}°", virtualJoint[1] * Mathf.Rad2Deg);
            laber_virtual_joint_3_angle.text = String.Format("{0:F3}°", virtualJoint[2] * Mathf.Rad2Deg);
            laber_virtual_joint_4_angle.text = String.Format("{0:F3}°", virtualJoint[3] * Mathf.Rad2Deg);
            laber_virtual_joint_5_angle.text = String.Format("{0:F3}°", virtualJoint[4] * Mathf.Rad2Deg);
            laber_virtual_joint_6_angle.text = String.Format("{0:F3}°", virtualJoint[5] * Mathf.Rad2Deg);
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
            btn_aubo_teleOperate.GetComponentInChildren<TMP_Text>().text = "遥操作模式：开";

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
            modelDialogControl.ShowDialog("注意！重要！", "开启遥操作模式后不能使用其他模式！");
        }
        else
        {
            btn_aubo_teleOperate.GetComponentInChildren<TMP_Text>().text = "遥操作模式：关";
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

    /**********************************************高德地图相关事件******************************************/
    void GaodeRefresh()
    {
        string longitude = input_gaode_longitude.text;
        string latitude = input_gaode_latitude.text;
        string ip = input_gaode_ip.text;
        GaoDeApi gaoDeApi = GetComponent<GaoDeApi>();
        gaoDeApi.ZOOM = slider_gaode_mapsize.value.ToString();
        gaoDeApi.RefreshRequest(ip, longitude, latitude);
    }

    /**********************************************急停相关事件******************************************/
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

    /**********************************************控制箱事件******************************************/
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
            Debug.Log("【DataCommon】ControlBoxSerialportOpen error\n" + ex);
            DebugGUI.Log("【DataCommon】ControlBoxSerialportOpen error\n" + ex);
        }
    }

    void ControlBoxSerialportClose()
    {
        SerialPortControl serialPortControl = GameObject.Find("ControlBoxControl").GetComponent<SerialPortControl>();
        serialPortControl.CloseSerialPort();
    }

    /**********************************************PLC系统******************************************/
    void ReadPLCData()
    {
        // todo
        PlcConnect plcConnect = GameObject.Find("PlcCommunication").GetComponent<PlcConnect>();

        uISwitcherLaserEnable.isOn = plcConnect.read_laser_enable;
        uISwitcherOpenInstruction.isOn = plcConnect.read_open_instruction;
        uISwitcherLaserOutput.isOn = plcConnect.read_laser_working;
        uISwitcherLaserReset.isOn = plcConnect.read_plc_reset;
        input_laserPower.text = plcConnect.read_current_power.ToString();
        DebugGUI.LogString(($"【PLCRead Laser】使能:{plcConnect.read_laser_enable} 指导光:{plcConnect.read_open_instruction} 出光:{plcConnect.read_laser_working}" +
            $"复位:{plcConnect.read_plc_reset} 功率:{plcConnect.read_current_power}"));
        Debug.Log($"【PLCRead Laser】使能:{plcConnect.read_laser_enable} 指导光:{plcConnect.read_open_instruction} 出光:{plcConnect.read_laser_working}" +
            $"复位:{plcConnect.read_plc_reset} 功率:{plcConnect.read_current_power}");

        uISwitcherOpenGas.isOn = plcConnect.read_open_gas;
        uISwitcherOpenPowder.isOn = plcConnect.read_open_powder;
        input_powderSpeed.text = plcConnect.read_current_speed.ToString();
        input_product.text = plcConnect.read_current_product.ToString();
        input_technology.text = plcConnect.read_current_technology.ToString();
        DebugGUI.LogString(($"【PLCRead GasPowder】气体:{plcConnect.read_open_gas} 送粉:{plcConnect.read_open_powder} 送粉转速:{plcConnect.read_current_speed}" +
            $"产品:{plcConnect.read_current_product} 工艺:{plcConnect.read_current_technology}"));
        Debug.Log($"【PLCRead GasPowder】气体:{plcConnect.read_open_gas} 送粉:{plcConnect.read_open_powder} 送粉转速:{plcConnect.read_current_speed}" +
            $"产品:{plcConnect.read_current_product} 工艺:{plcConnect.read_current_technology}");

        uISwitcherPLCPulse.isOn = plcConnect.read_plc_pulse;
        uISwitcherPLCReady.isOn = plcConnect.read_plc_ready;
        uISwitcherPLCAutomatic.isOn = plcConnect.read_plc_automatic;

        DebugGUI.LogString($"【PLCRead】脉冲信号:{plcConnect.read_plc_pulse} 就绪信号:{plcConnect.read_plc_ready} 自动模式:{plcConnect.read_plc_automatic}");
        Debug.Log($"【PLCRead】脉冲信号:{plcConnect.read_plc_pulse} 就绪信号:{plcConnect.read_plc_ready} 自动模式:{plcConnect.read_plc_automatic}");

        DebugGUI.LogString($"【PLCRead】温度:{plcConnect.read_env_temp} 湿度:{plcConnect.read_env_humi}");
        Debug.Log($"【PLCRead】温度:{plcConnect.read_env_temp} 湿度:{plcConnect.read_env_humi}");
    }

    void UpdatePLCData()
    {
        PlcConnect plcConnect = GameObject.Find("PlcCommunication").GetComponent<PlcConnect>();

        plcConnect.write_laser_enable = uISwitcherLaserEnable.isOn;
        plcConnect.write_open_instruction = uISwitcherOpenInstruction.isOn;
        plcConnect.write_laser_output = uISwitcherLaserOutput.isOn;
        plcConnect.write_laser_reset = uISwitcherLaserReset.isOn;
        plcConnect.write_current_power = ConvertUshortByString(input_laserPower.text, plcConnect.read_current_power);
        DebugGUI.LogString(($"【PLCWrite Laser】使能:{plcConnect.write_laser_enable} 指导光:{plcConnect.write_open_instruction} 出光:{plcConnect.write_laser_output}" +
            $"复位:{plcConnect.write_laser_reset} 功率:{plcConnect.write_current_power}"));
        Debug.Log($"【PLCWrite Laser】使能:{plcConnect.write_laser_enable} 指导光:{plcConnect.write_open_instruction} 出光:{plcConnect.write_laser_output}" +
            $"复位:{plcConnect.write_laser_reset} 功率:{plcConnect.write_current_power}");

        plcConnect.write_open_gas = uISwitcherOpenGas.isOn;
        plcConnect.write_open_powder = uISwitcherOpenPowder.isOn;
        plcConnect.write_powdering = uISwitcherPowdering.isOn;
        plcConnect.write_current_speed = ConvertUshortByString(input_powderSpeed.text, plcConnect.read_current_speed);
        plcConnect.write_current_product = ConvertUshortByString(input_product.text, plcConnect.read_current_product);
        plcConnect.write_current_technology = ConvertUshortByString(input_technology.text, plcConnect.read_current_technology);
        DebugGUI.LogString(($"【PLCWrite GasPowder】气体:{plcConnect.write_open_gas} 送粉使能:{plcConnect.write_open_powder} 送粉:{plcConnect.write_powdering}" +
            $"送粉转速:{plcConnect.write_current_speed} 产品:{plcConnect.write_current_product} 工艺:{plcConnect.write_current_technology}"));
        Debug.Log($"【PLCWrite GasPowder】气体:{plcConnect.write_open_gas} 送粉使能:{plcConnect.write_open_powder} 送粉:{plcConnect.write_powdering}" +
            $"送粉转速:{plcConnect.write_current_speed} 产品:{plcConnect.write_current_product} 工艺:{plcConnect.write_current_technology}");

        plcConnect.write_plc_pulse = uISwitcherPLCPulse.isOn;
        plcConnect.write_plc_ready = uISwitcherPLCReady.isOn;
        plcConnect.write_plc_automatic = uISwitcherPLCAutomatic.isOn;
        plcConnect.WriteToPlcData((int)LaserModeChoose.Setting);
        DebugGUI.LogString($"【PLCWrite】模式：{LaserModeChoose.Setting} 脉冲信号:{plcConnect.write_plc_pulse} 就绪信号:{plcConnect.write_plc_ready} 自动模式:{plcConnect.write_plc_automatic}");
        Debug.Log($"【PLCWrite】模式：{LaserModeChoose.Setting} 脉冲信号:{plcConnect.write_plc_pulse} 就绪信号:{plcConnect.write_plc_ready} 自动模式:{plcConnect.write_plc_automatic}");
    }

    ushort ConvertUshortByString(string numberStr, ushort oldData)
    {
        if (string.IsNullOrEmpty(numberStr))
        {
            Debug.Log($"【convertUshortByString error】传入数据为空;oldData:{oldData}");
            DebugGUI.LogString($"【convertUshortByString error】传入数据为空;oldData:{oldData}");
            return oldData;
        }

        try
        {
            ushort number = UInt16.Parse(numberStr);
            return number;
        }
        catch (Exception)
        {
            Debug.Log($"【convertUshortByString error】numberStr:{numberStr} 转换失败;oldData:{oldData}");
            DebugGUI.LogString($"【convertUshortByString error】numberStr:{numberStr} 转换失败;oldData:{oldData}");
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
            btn_open_laser.GetComponentInChildren<TMP_Text>().text = "关闭激光器";
        }
        else if (laserModeChoose.Equals(LaserModeChoose.Open))
        {
            laserModeChoose = LaserModeChoose.Close;
            plcConnect.WriteToPlcData((int)laserModeChoose);
            btn_open_laser.GetComponentInChildren<TMP_Text>().text = "打开激光器";
        }

        DebugGUI.LogString($"【PLCWrite】当前模式：{laserModeChoose}");
        Debug.Log($"【PLCWrite】当前模式：{laserModeChoose}");
    }
}