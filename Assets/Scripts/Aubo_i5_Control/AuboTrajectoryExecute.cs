using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.Geometry;
using RosMessageTypes.VirtualRobotControl;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityEngine;

public class AuboTrajectoryExecute : MonoBehaviour
{
    //var
    const int k_NumRobotJoints = 6;
    public static readonly string[] AuboLinkNames =
        { "world/base_link/shoulder_Link", "/upperArm_Link", "/foreArm_Link", "/wrist1_Link", "/wrist2_Link", "/wrist3_Link" };
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;
    
    public float m_JointVelocity = 0.1f;

    public bool plan = false;
    public bool exc = false;

    // Ros Service Name
    [SerializeField]
    string m_PlanServiceName = "aubo_move_plan";

    [SerializeField]
    string m_ExecuteServiceName = "aubo_move_execute";

    // Robot
    // [SerializeField]
    // GameObject m_Aubo;
    GameObject m_Aubo;

    // The points of trajectory to plan
    [SerializeField]
    GameObject m_Target;

    [SerializeField]
    GameObject m_TargetPlacement;

    // Ariticulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    // Ros Connector
    ROSConnection m_Ros;

    // The position of welding( vertical or parallel)
    // Vertical
    readonly Quaternion m_VerticalOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_VerticalOffset = Vector3.up * 0.1f;
    // Parallel
    readonly Quaternion m_ParallelOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_ParalleleOffset = Vector3.up * 0.1f;

    // The request of aubo execute
    
    //public var m_Executerequest = new AuboExecuteServiceRequest();
    AuboExecuteServiceRequest m_Executerequest;


    // Find robot and initialization
    // Add joints to ArticulationBodise Array
    void Start()
    {
        // Get Ros connetion static instace
        m_Ros = ROSConnection.GetOrCreateInstance();
        // Service of Plan
        m_Ros.RegisterRosService<AuboPlanServiceRequest, AuboPlanServiceResponse>(m_PlanServiceName);
        // Service of Executr
        m_Ros.RegisterRosService<AuboExecuteServiceRequest, AuboExecuteServiceResponse>(m_ExecuteServiceName);

        // Find the robot
        m_Aubo = GameObject.Find("aubo_i5");

        // Initialize the Executerequest
        m_Executerequest = new AuboExecuteServiceRequest();
        m_JointVelocity = 0.1f;

        // Initialize Robot Joints
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += AuboLinkNames[i];
            m_JointArticulationBodies[i] = m_Aubo.transform.Find(linkName).GetComponent<ArticulationBody>();

        }

    }

    private void Update()
    {
        if (plan) { 
            plan = false;
            PublishRequest();
        }

        if (exc) {
            exc = false;
            PublishExecuteRequest();
        }
          
    }


    // Get Current Joints
    AuboJointsMsg GetCurrenJoints()
    {
        var joints = new AuboJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];

        }

        return joints;

    }

    // Ros Service Request
    // Publish the points trying to plan
    public void PublishRequest()
    {
        var request = new AuboPlanServiceRequest();

        request.current_joints = GetCurrenJoints();

        request.start_pose = new PoseMsg
        {
            position = (m_Target.transform.position).To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        request.end_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position).To<FLU>(),
            orientation = Quaternion.Euler(90, 90, 0).To<FLU>()
        };

        m_Ros.SendServiceMessage<AuboPlanServiceResponse>(m_PlanServiceName, request, PlanResponse);

    }

    // Ros Service Response
    // Show the Response Trajectory
    void PlanResponse(AuboPlanServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            // need copy? or 
            m_Executerequest.trajectories = response.trajectories;
            StartCoroutine(ExecutePlanTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned.");
        }
    }

    // Excute the plan trajectories
    IEnumerator ExecutePlanTrajectories(AuboPlanServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // for every trajectory plan returned
            for (var trajectoryIndex = 0; trajectoryIndex < response.trajectories.Length; trajectoryIndex++)
            {
                //for every robot pose in trajectory
                foreach (var p in response.trajectories[trajectoryIndex].joint_trajectory.points)
                {
                    var jointPositions = p.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    //Set the joint values for every joint 
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;

                        //m_JointArticulationBodies[joint].xDrive.target = result[joint];

                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                if (trajectoryIndex == (int)Trajectory.Prepare)
                {
                    Debug.Log("Start to weld, Turn on the laser!");
                }

                if (trajectoryIndex == (response.trajectories.Length -2))
                {
                    Debug.Log("Welding end, Turn off the laser!");
                }
                
                /*
                if (trajectoryIndex == (int)Trajectory.Execute)
                {
                    Debug.LogString("Welding end, Turn off the laser!")
                }
                */

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);



            }
        }
    }

    // Ros Service Request
    // Publish the points trying to execute
    public void PublishExecuteRequest()
    {

        //m_Executerequest.velocity = m_JointVelocity;

        m_Ros.SendServiceMessage<AuboExecuteServiceResponse>(m_ExecuteServiceName, m_Executerequest, ExecuteResponse);

    }

    // Ros Service Response
    // Show whether execute success
    void ExecuteResponse(AuboExecuteServiceResponse response)
    {
        bool success_flag = response.execute;
        if (success_flag == true)
        {
            Debug.Log("Robot execute the plan success!");
        }
        else
        {
            Debug.Log("Robot execute the plan failed!");
        }

    }


    enum Trajectory
    {
        Prepare,
        Execute,
        Home
    }



}
