using JetBrains.Annotations;
using RosMessageTypes.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.EventSystems;
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
        auboMaunalOperatePlan.isMaunalOperateMode = !auboMaunalOperatePlan.isMaunalOperateMode;
        auboMaunalOperatePlan.ClearData();
        if (auboMaunalOperatePlan.isMaunalOperateMode)
        {
            btn_manualoperate_start.GetComponentInChildren<TMP_Text>().text = "��קʾ��ģʽ����";
            // add user prompt dialog
            ModelDialogControl modelDialogControl = this.GetComponent<ModelDialogControl>();
            modelDialogControl.ShowDialog("ע�⣡��Ҫ��", "������קʾ��ģʽ����ʹ������ģʽ��");
        }
        else
        {
            btn_manualoperate_start.GetComponentInChildren<TMP_Text>().text = "��קʾ��ģʽ����";
        }
    }

    void ManualOperateSavePostionAndOrientation()
    {
        AuboMaunalOperatePlan auboMaunalOperatePlan = GameObject.Find("TeachingOperate").GetComponent<AuboMaunalOperatePlan>();
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        if (auboMaunalOperatePlan.isMaunalOperateMode)
        {
            PoseMsg realPose = auboControl.m_RealPose;
            auboMaunalOperatePlan.AddEndPointPositionAndOrientation(realPose.position, realPose.orientation);
        }
        else if (auboControl.is_TeleOperation)
        { 
            // todo ����λ��
        }
    }

    void TeachingOperateBegin() {
        int modeIndex = dropdown_teachingOperateMode.value;

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
        if (auboMaunalOperatePlan.isMaunalOperateMode)
        {
            auboMaunalOperatePlan.GetPointPositonAndOrientation(out List<double[]> position, out List<Quaternion<FLU>> orientation);
            List<List<double[]>> request = new List<List<double[]>> {
                position
            };
            List<List<Quaternion<FLU>>> trailOrientationFLU = new List<List<Quaternion<FLU>>> {
                orientation
            };

            auboTrajectoryRequest.PublishMultiRequest(request, trailOrientationFLU, 2);
        }
        else
        {
            float interpolation_distance = float.Parse(input_point_interpolated_distance.text);
            DrawPoint drawPoint = GameObject.Find("TeachingOperate").GetComponent<DrawPoint>();
            List<Vector3> points = drawPoint.GetDrawPoints();

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

            // todo ���������̬
            if (type == 2)
            {
                Vector3[] nors_unity = drawPoint.getPointsNormals(points, 25, Camera.main.transform.position);
                if (nors_unity == null)
                {
                    Debug.Log("��DataCommon�� RobotPointPlan fail: ��ȡ�ķ�����Ϊnull");
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
        if (auboControl.is_TeleOperation)
        {
            btn_aubo_teleOperate.GetComponentInChildren<TMP_Text>().text = "ң����ģʽ����";

            // add user prompt dialog
            ModelDialogControl modelDialogControl = this.GetComponent<ModelDialogControl>();
            modelDialogControl.ShowDialog("ע�⣡��Ҫ��", "����ң����ģʽ����ʹ������ģʽ��");
        }
        else 
        {
            btn_aubo_teleOperate.GetComponentInChildren<TMP_Text>().text = "ң����ģʽ����";
        }
    }

    void AuboJointHome() {
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
            if(!string.IsNullOrEmpty(input_aubo_joint_1_angle.text))
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
    void RobotStop()
    {
        AuboControl auboControl = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        auboControl.CancelArmAction();
    }

    void CarStop()
    { 
        UnitySubscription_Map unitySubscription_Map = GameObject.Find("RosMapData").GetComponent<UnitySubscription_Map>();
        unitySubscription_Map.NaviCancel();
    }

    void EveryThingStop()
    { 
    
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
        }
    }

    void ControlBoxSerialportClose()
    {
        SerialPortControl serialPortControl = GameObject.Find("ControlBoxControl").GetComponent<SerialPortControl>();
        serialPortControl.CloseSerialPort();
    }
}