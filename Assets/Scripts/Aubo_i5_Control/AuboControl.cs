using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.VirtualRobotControl;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEditor;
using UnityEngine;

// 01.Robot parameter
// 02.Robot initialize
// 03.Parameter of robot in unity acquring
// 04.Basic control of robot in unity

public class AuboControl : MonoBehaviour
{

    public enum RobotType
    {
        aubo_i5,
        aubo_i10,
    }

    public RobotType robotType;

    private static readonly Dictionary<RobotType, string[]> AuboLinkNames = new Dictionary<RobotType, string[]>
    {
        { RobotType.aubo_i5, new string[6] { "world/base_link/shoulder_Link", "/upperArm_Link", "/foreArm_Link", "/wrist1_Link", "/wrist2_Link", "/wrist3_Link" }},
        { RobotType.aubo_i10, new string[6] { "base_link/shoulder_Link", "/upperArm_Link", "/foreArm_Link", "/wrist1_Link", "/wrist2_Link", "/wrist3_Link" }}
    };

    private static Dictionary<RobotType, double[]> AuboHomeJoints = new Dictionary<RobotType, double[]>
    {
        { RobotType.aubo_i5, new double[6] { 0, 0, -1.54, 0, -1.52, 0 }},
        { RobotType.aubo_i10, new double[6] { 0, -0.087, 1.829596, 0.43633, 1.621238, 0 }}
        //{ RobotType.aubo_i10, new double[6] { 0.0214, 0.10886, 1.5332, -0.14046, 1.5664, -0.0146 }}
    };

    private static readonly Dictionary<RobotType, double[]> AuboStartJoints = new Dictionary<RobotType, double[]>
    {
        { RobotType.aubo_i5, new double[6] { 0, 0, -1.54, 0, -1.52, 0 }},
        { RobotType.aubo_i10, new double[6] { 0, -0.087, 1.829596, 0.43633, 1.621238, 0 }}
        //{ RobotType.aubo_i10, new double[6] { 0.0214, 0.10886, 1.5332, -0.14046, 1.5664, -0.0146 }}
    };


    // Robot joint
    public static readonly int k_NumRobotJoints = 6;

    public double[] m_RealJointsState;
    public PoseMsg m_RealPose;
    public PoseMsg m_Transform;
    public double[] m_VirtualJointsState;

    // Interval of simulate robot executing
    public float k_JointAssignmentWait = 0.1f;
    public float k_PoseAssignmentWait = 0.5f;
    public float k_PublishMsgFrequency = 0.1f;  //ROS消息发送频率

    // 是否进入遥操作模式      ---------*******记得做互斥的修改********-------------
    public bool is_TeleOperation = false;
    public bool is_AuboApi = false;           // false -- 控制器是ros-controller     true -- 控制器是aubo-api


    // Velocity of real robot
    public double m_JointVelocity = 0.1;

    // The position of welding( vertical or parallel)
    // Vertical
    public readonly Quaternion m_VerticalOrientation = new Quaternion(0.70684f, -0.01705f, -0.70692f, -0.01861f);//Quaternion.Euler(0, 0, -180); //0, 0, -180
    // Parallel
    public readonly Quaternion m_ParallelOrientation = Quaternion.Euler(90, 90, 0);
    // Welding offset
    public Vector3 m_ParalleleOffset = Vector3.up * 0.1f;

    //Ros topic or service for inilialize and control
    const string m_JointTopicName = "/aubo_joints_state";
    const string m_SycServiceName = "/aubo_unity_syc";
    const string m_JointPubName = "/aubo_target_joints";
    const string m_EmergencyStopName = "/aubo_driver/tele_stop_action";
    const string m_ControllerSwitchName = "/aubo_driver/controller_switch";
    

    // Gameobject of robot
    GameObject m_Aubo;

    // Ros Connector
    ROSConnection m_Ros;

    // Ariticulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    // Start is called before the first frame update
    void Start()
    {
        Quaternion<FLU> ros = new Quaternion<FLU>(-0.706923f, -0.706840f, -0.017048f, 0.018613f);
        Quaternion<RUF> unity = ros.To<RUF>();
        Debug.Log($"ROS quaternion: {ros} Unity quaternion:{unity}");

        // Find the robot
        m_Aubo = GameObject.Find(robotType.ToString());

        m_RealPose = new PoseMsg();
        m_Transform = new PoseMsg();
        m_RealJointsState = new double[k_NumRobotJoints];
        m_VirtualJointsState = new double[k_NumRobotJoints];


        // Initialize Robot Joints
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += AuboLinkNames[robotType][i];
            m_JointArticulationBodies[i] = m_Aubo.transform.Find(linkName).GetComponent<ArticulationBody>();

        }

        // Get Ros connetion static instace
        m_Ros = ROSConnection.GetOrCreateInstance();
        // 同步的问题： 发送服务:untiy->ros   订阅话题:ros->unity
        // Topic of robot joint states
        m_Ros.Subscribe<AuboJointsStateMsg>(m_JointTopicName, SubJointState);
        // 实时发布虚拟关节角的话题，然而现在底层driver并没有改好，暂时不能用
        m_Ros.RegisterPublisher<AuboJointsMsg>(m_JointPubName);
        // 同步虚-实机械臂的服务
        m_Ros.RegisterRosService<AuboSycServiceRequest, AuboSycServiceResponse>(m_SycServiceName);
        // 急停的话题 0-非急停  1-急停
        m_Ros.RegisterPublisher<UInt8Msg>(m_EmergencyStopName);
        // 控制器选择的话题 0-Aubo API  1-Ros Contr0ller
        m_Ros.RegisterPublisher<Int32Msg>(m_ControllerSwitchName);

        // init
        AuboToStart();

        StartCoroutine(SendMessageRepeatedly(k_PublishMsgFrequency)); // 开始协程，间隔发送消息
    }

    // Update is called once per frame
    void Update()
    {
        // update the virtual joints states in time
        m_VirtualJointsState = GetCurrenJoints().joints;
        if (is_AuboApi)
        {
            SetJointState(m_RealJointsState);
        }


    }

    // Initailize and Home 

    public void AuboToStart()
    {
        SetJointState(AuboStartJoints[robotType]);

        AuboConfigData auboConfigData = AuboConfig.ReadJsonData();
        AuboHomeJoints[robotType][0] = auboConfigData.initJoint1;
        AuboHomeJoints[robotType][1] = auboConfigData.initJoint2;
        AuboHomeJoints[robotType][2] = auboConfigData.initJoint3;
        AuboHomeJoints[robotType][3] = auboConfigData.initJoint4;
        AuboHomeJoints[robotType][4] = auboConfigData.initJoint5;
        AuboHomeJoints[robotType][5] = auboConfigData.initJoint6;
    }

    public void AuboToHome()
    {
        SetJointState(AuboHomeJoints[robotType]);
    }

    public void SaveCurrenJointsHome()
    {
        AuboJointsMsg auboJoints = GetCurrenJoints();
        AuboConfigData auboConfigData = new AuboConfigData(auboJoints.joints[0], auboJoints.joints[1], auboJoints.joints[2],
            auboJoints.joints[3], auboJoints.joints[4], auboJoints.joints[5]);
        AuboConfig.SaveJsonData(auboConfigData);
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            AuboHomeJoints[robotType][i] = auboJoints.joints[i];
        }
    }

    // Simulate robot pose get(Fk  joint state and  pose)
    public AuboJointsMsg GetCurrenJoints()
    {
        var joints = new AuboJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];

        }

        return joints;
    }

    // Basic control of robot
    public void SetJointState(double[] target_joints)
    {
        // double -> float
        var result = target_joints.Select(r => (float)r * Mathf.Rad2Deg).ToArray();
        for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
        {
            var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
            joint1XDrive.target = result[joint];
            m_JointArticulationBodies[joint].xDrive = joint1XDrive;

            //m_JointArticulationBodies[joint].xDrive.target = result[joint];

        }

    }

    IEnumerator SendMessageRepeatedly(float interval)
    {
        while (true) // 遥操作时持续发送消息
        {
            if (is_TeleOperation)
            {
                UnityEngine.Debug.Log("开始协程");
                AuboJointsMsg pub_joints = new AuboJointsMsg(m_VirtualJointsState);
                m_Ros.Publish(m_JointPubName, pub_joints);
            }


            yield return new WaitForSeconds(interval); // 等待指定的时间间隔
        }

    }

    //机械臂急停
    public void ArmEmergencyAction(bool stop_enable)
    {
        UInt8Msg pub_data = new UInt8Msg();
        if (stop_enable)
        {
            pub_data.data = 1;    //急停
        }
        else
        {
            pub_data.data = 0;   //取消急停
        }
        m_Ros.Publish(m_EmergencyStopName, pub_data);
    }

    //机械臂控制模式选择
    public void RobotControllerSwitch(int flag)
    {
        Int32Msg pub_msg = new Int32Msg();
        if (flag == 0)
        {
            pub_msg.data = 0;  // 0 - Aubo Api/示教器控制
            is_AuboApi = true;  
        }
        else if (flag == 1)
        {
            pub_msg.data = 1;  // 1 - Ros控制
            is_AuboApi = false; 
        }
        m_Ros.Publish(m_ControllerSwitchName, pub_msg);
    }

    /*
    public void PubJoints(double[] target_joints)
    {
        AuboJointsMsg pub_joints = new AuboJointsMsg(target_joints);
        m_Ros.Publish(m_JointPubName, pub_joints);
        SetJointState(target_joints);
    }
    */

    // Subscribe the joints state and print them
    void SubJointState(AuboJointsStateMsg jointsstate)
    {
        m_RealJointsState = jointsstate.joints;
        m_RealPose = jointsstate.pose;
        m_Transform = jointsstate.transform;
        // Show the data to screen
        // -----------need to change-----------
        // ------------------------------------
        double joint1 = m_RealJointsState[0];
        var pose_x = m_RealPose.position.x;
        //Debug.LogString(joint1.ToString());
        //Debug.LogString(pose_x);
        // -----------------------------------
        // -----------------------------------
    }

    // Service request for robot synch
    public void SycRequest(double[] joints, PoseMsg pose, bool isJoint)
    {
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // ros msg : float64  ------>   unity msg : double
        // use float664
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        var request = new AuboSycServiceRequest();
        var target_joints = new double[6];
        var target_pose = new PoseMsg();

        target_joints = joints;
        target_pose = pose;

        request.current_joints = GetCurrenJoints();

        request.target_joints = target_joints;
        request.target_pose = target_pose;

        if (isJoint == true)
        {
            // joints control
            request.is_joints = true;
            request.is_pose = false;
        }
        else
        {
            // pose control
            request.is_joints = false;
            request.is_pose = true;
        }
        Debug.Log($"【SycRequest】joints: {joints[0]}, {joints[1]}, {joints[2]}," +
    $"{joints[3]}, {joints[4]}, {joints[5]}");
        DebugGUI.Log($"【SycRequest】joints: {joints[0]}, {joints[1]}, {joints[2]}," +
    $"{joints[3]}, {joints[4]}, {joints[5]}");
        m_Ros.SendServiceMessage<AuboSycServiceResponse>(m_SycServiceName, request, SycResponse);

    }

    public void SycResponse(AuboSycServiceResponse response)
    {
        if (response.aim_get == true)
        {
            Debug.Log($"response: {response.aim_joints[0]}, {response.aim_joints[1]}, {response.aim_joints[2]}," +
                $"{response.aim_joints[3]}, {response.aim_joints[4]}, {response.aim_joints[5]}");
            DebugGUI.Log($"response: {response.aim_joints[0]}, {response.aim_joints[1]}, {response.aim_joints[2]}," +
    $"{response.aim_joints[3]}, {response.aim_joints[4]}, {response.aim_joints[5]}");
            SetJointState(response.aim_joints);
        }
        else
        {
            Debug.LogError("Synch Failed!");
            DebugGUI.Log("Synch Failed!");
        }


    }





}
