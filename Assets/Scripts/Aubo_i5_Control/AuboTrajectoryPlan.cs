using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.Geometry;
using RosMessageTypes.VirtualRobotControl;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using UnityEngine;

public class AuboTrajectoryPlan : MonoBehaviour
{

    //var
    const int k_NumRobotJoints = 6;
    public static readonly string[] AuboLinkNames =
        { "world/base_link/shoulder_Link", "/upperArm_Link", "/foreArm_Link", "/wrist1_Link", "/wrist2_Link", "/wrist3_Link" };
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Ros Service Name
    [SerializeField]
    string m_RosServiceName = "aubo_move_plan";

    // Robot
    [SerializeField]
    GameObject m_Aubo;
    //GameObject m_Aubo_const = GameObject.Find("aubo_i5");

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


    // Find robot and initialization
    // Add joints to ArticulationBodise Array
    void Start()
    {
        // Get Ros connetion static instace
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<AuboPlanServiceRequest, AuboPlanServiceResponse>(m_RosServiceName);

        // Initialize Robot Joints
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += AuboLinkNames[i];
            m_JointArticulationBodies[i] = m_Aubo.transform.Find(linkName).GetComponent<ArticulationBody>();

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

        m_Ros.SendServiceMessage<AuboPlanServiceResponse>(m_RosServiceName, request, PlanResponse);

    }

    // Ros Service Response
    // Show the Response Trajectory
    void PlanResponse(AuboPlanServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
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
                    Debug.Log("Welding end, Turn off the laser!")
                }
                */

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);



            }
        }
    }

    enum Trajectory
    {
        Prepare,
        Execute,
        Home
    }

}
