//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.VirtualRobotControl
{
    [Serializable]
    public class AuboSycServiceRequest : Message
    {
        public const string k_RosMessageName = "virtual_robot_control/AuboSycService";
        public override string RosMessageName => k_RosMessageName;

        public AuboJointsMsg current_joints;
        public bool is_joints;
        public double[] target_joints;
        public bool is_pose;
        public Geometry.PoseMsg target_pose;

        public AuboSycServiceRequest()
        {
            this.current_joints = new AuboJointsMsg();
            this.is_joints = false;
            this.target_joints = new double[6];
            this.is_pose = false;
            this.target_pose = new Geometry.PoseMsg();
        }

        public AuboSycServiceRequest(AuboJointsMsg current_joints, bool is_joints, double[] target_joints, bool is_pose, Geometry.PoseMsg target_pose)
        {
            this.current_joints = current_joints;
            this.is_joints = is_joints;
            this.target_joints = target_joints;
            this.is_pose = is_pose;
            this.target_pose = target_pose;
        }

        public static AuboSycServiceRequest Deserialize(MessageDeserializer deserializer) => new AuboSycServiceRequest(deserializer);

        private AuboSycServiceRequest(MessageDeserializer deserializer)
        {
            this.current_joints = AuboJointsMsg.Deserialize(deserializer);
            deserializer.Read(out this.is_joints);
            deserializer.Read(out this.target_joints, sizeof(double), 6);
            deserializer.Read(out this.is_pose);
            this.target_pose = Geometry.PoseMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.current_joints);
            serializer.Write(this.is_joints);
            serializer.Write(this.target_joints);
            serializer.Write(this.is_pose);
            serializer.Write(this.target_pose);
        }

        public override string ToString()
        {
            return "AuboSycServiceRequest: " +
            "\ncurrent_joints: " + current_joints.ToString() +
            "\nis_joints: " + is_joints.ToString() +
            "\ntarget_joints: " + System.String.Join(", ", target_joints.ToList()) +
            "\nis_pose: " + is_pose.ToString() +
            "\ntarget_pose: " + target_pose.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
