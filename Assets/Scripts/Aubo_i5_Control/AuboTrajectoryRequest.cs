using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.Geometry;
using RosMessageTypes.VirtualRobotControl;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityEngine;

public class AuboTrajectoryRequest : MonoBehaviour
{
    // controller instance
    public AuboControl i_controller;




    // Ros Service Name
    string m_PlanServiceName = "aubo_move_plan";
    string m_MultiPlanServiceName = "aubo_multi_plan";
    string m_ExecuteServiceName = "aubo_move_execute";



    // The points of trajectory to plan
    [SerializeField]
    GameObject m_Target;

    [SerializeField]
    GameObject m_TargetPlacement;


    // Ros Connector
    ROSConnection m_Ros;


    // The request of aubo execute  
    AuboExecuteServiceRequest m_Executerequest;
    AuboJointsMsg m_PlanCurrentJoints;
    bool is_Replan;


    // 焊枪末端到EndPoint的位姿变换关系
    Matrix4x4 matrix4X4_weld2EndPoints_Ros = new Matrix4x4(
        new Vector4(1, 0, 0, 0),
        new Vector4(0, 1, 0, 0),
        new Vector4(0, 0, 1, 0),
        new Vector4(-0.1342f, 0.0049f, 0.16771f, 1)
        );

    void Start()
    {
        Quaternion<FLU>  pose_orientation = new Quaternion<FLU>(0.698678f, 0.710840f, 0.066345f, 0.046496f);//i_controller.m_VerticalOrientation.To<FLU>();
        Debug.Log( $"【pose_orientation.To<RUF>】{pose_orientation.To<RUF>()}");
        // Initialize the control instance
        i_controller = GetComponent<AuboControl>();


        // Get Ros connetion static instace
        m_Ros = ROSConnection.GetOrCreateInstance();
        // Service of Plan
        m_Ros.RegisterRosService<AuboPlanServiceRequest, AuboPlanServiceResponse>(m_PlanServiceName);
        // Service of MultiPlan
        m_Ros.RegisterRosService<AuboMultiPlanServiceRequest, AuboMultiPlanServiceResponse>(m_MultiPlanServiceName);
        // Service of Executr
        m_Ros.RegisterRosService<AuboExecuteServiceRequest, AuboExecuteServiceResponse>(m_ExecuteServiceName);

        
        // Initialize the Executerequest
        m_Executerequest = new AuboExecuteServiceRequest();
       

        // Initialzie the current joints for plan(if last plan failed)
        m_PlanCurrentJoints = new AuboJointsMsg();
        is_Replan = false;

        

    }


    
    /// <summary>
    /// Robot plan for two points: current -> start -> end
    /// </summary>
    // Ros Service Request
    // Publish the points trying to plan
    public void PublishRequest()
    {
        var request = new AuboPlanServiceRequest();

        // when plan execute, the is_replan -> false
        if (is_Replan == false)
        {
            request.current_joints = i_controller.GetCurrenJoints();
            m_PlanCurrentJoints = i_controller.GetCurrenJoints();
            is_Replan = true;
        }
        else if (is_Replan == true)
        {
            request.current_joints = m_PlanCurrentJoints;
        }

        //request.current_joints = GetCurrenJoints();

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

    /// <summary>
    /// Robot plan for multi points: a series of points
    /// AuboJoints current_joints
    /// AuboPoseList[] pose_list
    /// ---
    /// moveit_msgs/RobotTrajectory[] trajectories
    /// </summary>
    // Ros Service Request
    // Publish the points trying to plan
    public void PublishMultiRequest(List<List<double[]>> trails, List<List<Quaternion<FLU>>> trailsOrientation, int type)
    {
        var request = new AuboMultiPlanServiceRequest();

        request.velocity = i_controller.m_JointVelocity;

        // when plan execute, the is_replan -> false
        if (is_Replan == false)
        {
            request.current_joints = i_controller.GetCurrenJoints();
            m_PlanCurrentJoints = i_controller.GetCurrenJoints();
            is_Replan = true;
        }
        else if (is_Replan == true)
        {
            request.current_joints = m_PlanCurrentJoints;
        }

        //todo 检查当前真实机械臂关节角和当前虚拟的关节角是否一致，不一致需要同步
        double sum = 0;
        for (int i = 0; i < 6; i++)
        {
            sum += Math.Abs(request.current_joints.joints[i] - i_controller.m_RealJointsState[i]);
        }
        if (sum >= 0.02)   //1度左右
        {
            Debug.Log("请同步真实与虚拟机械臂！");
            return;
        }

        var pose_orientation = i_controller.m_VerticalOrientation.To<FLU>();
        
        var traillists = new AuboPoseListMsg[trails.Count];

        for (int i = 0; i < trails.Count; i++)
        {
            var pointslists = new AuboPoseListMsg();
            pointslists.waypoints = new PoseMsg[trails[i].Count];
            for (int j = 0; j < trails[i].Count; j++)
            {
                var pose = new PoseMsg();
                // point xyz in unity or ros

                if (type == 0)
                {
                    pose_orientation = i_controller.m_VerticalOrientation.To<FLU>();
                }
                else if (type == 1)
                {
                    pose_orientation = i_controller.m_ParallelOrientation.To<FLU>();
                }
                else if (type == 2)
                {
                    if (trailsOrientation != null)
                    {
                        pose_orientation = trailsOrientation[i][j];
                    }
                    else 
                    {
                        pose_orientation = i_controller.m_VerticalOrientation.To<FLU>();
                    }
                }
                pose_orientation.Normalize();

                PointMsg positon = new PointMsg(trails[i][j][0], trails[i][j][1], trails[i][j][2]);
                Matrix4x4 weld2BaseInRos = Matrix4x4.identity;
                weld2BaseInRos.SetTRS(new Vector3((float)positon.x, (float)positon.y, (float)positon.z),
                    new Quaternion(pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w),
                    Vector3.one);
                Matrix4x4 endpoint2Base = weld2BaseInRos * matrix4X4_weld2EndPoints_Ros.inverse;

                Vector3 posePosition = endpoint2Base.GetPosition();
                Quaternion poseOrientation = endpoint2Base.rotation;

                pose.position = new PointMsg(posePosition.x, posePosition.y, posePosition.z);
                pose.orientation = new QuaternionMsg(poseOrientation.x, poseOrientation.y, poseOrientation.z, poseOrientation.w);
                Debug.Log($"【pose.orientation】 \n {pose.orientation}");

/*                pose.position = new PointMsg(trails[i][j][0], trails[i][j][1], trails[i][j][2]);
                pose.orientation = pose_orientation;*/

                pointslists.waypoints[j] = pose;
            }

            traillists[i] = pointslists;
        }


        request.trail_list = traillists;




        m_Ros.SendServiceMessage<AuboMultiPlanServiceResponse>(m_MultiPlanServiceName, request, MultiPlanResponse);

    }

    // Ros Service Response
    // Show the Response Trajectory
    void MultiPlanResponse(AuboMultiPlanServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            // need copy? or 
            m_Executerequest.trajectories = response.trajectories;
            StartCoroutine(ExecuteMultiPlanTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned.");
        }
    }
    
    /// <summary>
    /// Executing the plan in unity
    /// </summary>
    /// <param name="response"></param>
    /// <returns></returns>
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
                    i_controller.SetJointState(jointPositions);                   

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(i_controller.k_JointAssignmentWait);
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
                    Debug.Log("Welding end, Turn off the laser!")
                }
                */

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(i_controller.k_PoseAssignmentWait);



            }
        }
    }

    IEnumerator ExecuteMultiPlanTrajectories(AuboMultiPlanServiceResponse response)
    {
        if (response.trajectories != null)
        {
            Debug.Log($"【ExecuteMultiPlanTrajectories】 response.trajectories.Length : {response.trajectories.Length}");
            // for every trajectory plan returned
            for (var trajectoryIndex = 0; trajectoryIndex < response.trajectories.Length; trajectoryIndex++)
            {
                //for every robot pose in trajectory
                float joint_update_time = response.total_time[trajectoryIndex] / response.trajectories[trajectoryIndex].joint_trajectory.points.Length;
                foreach (var p in response.trajectories[trajectoryIndex].joint_trajectory.points)
                {
                    var jointPositions = p.positions;
                    i_controller.SetJointState(jointPositions);                   

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(joint_update_time);
                }

                if (trajectoryIndex == (int)Trajectory.Prepare)
                {
                    Debug.Log("Start to weld, Turn on the laser!");
                }

                if (trajectoryIndex == (response.trajectories.Length - 1))
                {
                    Debug.Log("Welding end, Turn off the laser!");
                }
                
                /*
                if (trajectoryIndex == (int)Trajectory.Execute)
                {
                    Debug.Log("Welding end, Turn off the laser!")
                }
                */

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(i_controller.k_PoseAssignmentWait);



            }
        }
    }  

    /// <summary>
    /// Robot Service for executing the successfull planning in real robot
    /// </summary>
    // Ros Service Request
    // Publish the points trying to execute
    public void PublishExecuteRequest()
    {
        if (m_Executerequest.trajectories.Length > 0)
        {
            //m_Executerequest.velocity = i_controller.m_JointVelocity;

            m_Ros.SendServiceMessage<AuboExecuteServiceResponse>(m_ExecuteServiceName, m_Executerequest, ExecuteResponse);

            // plan published, need new plan;
            is_Replan = false;
            m_Executerequest = new AuboExecuteServiceRequest();

        }
        else
        {
            Debug.Log("No Trajetories to Execute!");
        }

        
    }

    // Ros Service Response
    // Show whether execute success
    void ExecuteResponse(AuboExecuteServiceResponse response)
    {
        bool success_flag = response.execute;
        if (success_flag == true)
        {
            Debug.Log("Robot execute the plan success!");
            //clear origin data
        }
        else
        {
            Debug.Log("Robot execute the plan failed!");
        }

    }
    
    
    // Give up last plan, chose new points or new start position
    public void ReStartPlan()
    {
        is_Replan = false;
        m_Executerequest = new AuboExecuteServiceRequest();
    }


    enum Trajectory
    {
        Prepare,
        Execute,
        Home
    }

}
