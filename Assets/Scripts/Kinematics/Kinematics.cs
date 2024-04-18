using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using UnityEngine;


public class Kinematics : MonoBehaviour
{


    // ���峣��
    private const int ARM_DOF = 6;
    const double MIN = -175;
    const double MAX = 175;
    const int MAXIMUM_ITERATOR = 10;


    private const double ZERO_THRESH = 1e-4;

    /*
    // AUBO_I5_PARAMS
    private const double a2 = 0.408;
    private const double a3 = 0.376;
    private const double d1 = 0.122;
    private const double d2 = 0.1215;
    private const double d5 = 0.1025;
    private const double d6 = 0.094;
    */

    
    // AUBO_I10_PARAMS
    private const double a2 = 0.647;
    private const double a3 = 0.6005;
    private const double d1 = 0.1632;
    private const double d2 = 0.2013;
    private const double d5 = 0.1025;
    private const double d6 = 0.094;
    

    //�ؽڽǶ�����
    //private double MaxQ = 175.0 / 180.0 * Math.PI;
    private double[,] AngleLimit = new double[6, 2]
    {
        {-175.0 / 180.0 * Math.PI, 175.0 / 180.0 * Math.PI},
        {-175.0 / 180.0 * Math.PI, 175.0 / 180.0 * Math.PI},
        {-175.0 / 180.0 * Math.PI, 175.0 / 180.0 * Math.PI},
        {-175.0 / 180.0 * Math.PI, 175.0 / 180.0 * Math.PI},
        {-175.0 / 180.0 * Math.PI, 175.0 / 180.0 * Math.PI},
        {-175.0 / 180.0 * Math.PI, 175.0 / 180.0 * Math.PI}
    };

    //��е������
    AuboControl i_aubocontrol;

    //�ֿ�������
    HapticPlugin i_hapticcontrol;
    int Button1On;

    bool TeleoperationEngaged = false;

    double LinearScale = 2;

    public bool is_linear_only = false;
    public bool is_angular_only = false;

    //�ƶ���Χ����
    const double move_linear_thresh_min = 0.001;  //�����ƶ���С��Χ 0.005
    const double move_linear_thresh_max = 0.03;  //�����ƶ����Χ 
    const double move_angular_thresh_min = 5.0 / 180 * Math.PI;  //�����ƶ���С�Ƕȷ�Χ 20
    const double move_angular_thresh_max = 80.0 / 180 * Math.PI;  //�����ƶ����Ƕȷ�Χ

    private bool is_linear_flag = false;
    private bool is_angular_flag = false;

    // slaver
    private double[] slaver_initial_joints = new double[] { 0, 0.115909, 1.829596, 0.142811, 1.621238, 0 };
    [SerializeField]
    private double[] slaver_current_joints = new double[6];
    [SerializeField]
    private double[] slaver_current_position = new double[3];
    private Matrix4x4 slaver_current_oritation = new Matrix4x4();
    private double[,] slaver_current_matrix = new double[4, 4];

    private double[] slaver_start_joints = new double[6];
    private double[] slaver_start_position = new double[3];
    private Matrix4x4 slaver_start_oritation = new Matrix4x4();





    // master

    private double[] master_current_position = new double[3];
    [SerializeField]
    private double[] master_current_rpy = new double[3];
    private Matrix4x4 master_current_oritation = new Matrix4x4();


    private double[] master_start_position = new double[3];
    //private double[] master_start_oritation = new double[3];
    private Matrix4x4 master_start_oritation = new Matrix4x4();


    private double[] master_previous_position = new double[3];
    [SerializeField]
    private double[] master_previous_rpy = new double[3];
    private Matrix4x4 master_previous_oritation = new Matrix4x4();




    // private double[,] master_current_matrix = new double[4, 4];

    // transform from touch to ros
    private Matrix4x4 Transform_Ros_Touch = new Matrix4x4();






    // Start is called before the first frame update
    void Start()
    {
        //i_hapticcontrol = GameObject.Find("HapticActor").GetComponent<HapticPlugin>();
        i_aubocontrol = GameObject.Find("aubo_i5_publish").GetComponent<AuboControl>();
        i_hapticcontrol = GetComponent<HapticPlugin>();
        Button1On = i_hapticcontrol.my_Buttons[1];
        master_current_position = i_hapticcontrol.Touch_Position;  // left x, up y, forward z
        master_current_rpy = i_hapticcontrol.Touch_Rpy;   // ypr
        master_current_oritation = i_hapticcontrol.DeviceTransformRotation;

        Array.Copy(slaver_initial_joints, slaver_current_joints, slaver_initial_joints.Length);

        aubo_forward(slaver_current_matrix, slaver_current_joints);


        slaver_current_oritation = DoubleMatrixToRotation(slaver_current_matrix);
        slaver_current_position[0] = slaver_current_matrix[0, 3];
        slaver_current_position[1] = slaver_current_matrix[1, 3];
        slaver_current_position[2] = slaver_current_matrix[2, 3];

        Transform_Ros_Touch = Matrix4x4.zero;
        /*
        Transform_Ros_Touch.m02 = 1;
        Transform_Ros_Touch.m10 = -1;
        Transform_Ros_Touch.m21 = 1;
        Transform_Ros_Touch.m33 = 1;
        */
        Transform_Ros_Touch.m02 = -1;
        Transform_Ros_Touch.m10 = 1;
        Transform_Ros_Touch.m21 = 1;
        Transform_Ros_Touch.m33 = 1;

        

    }


    // Update is called once per frame
    void Update()
    {
        Button1On = i_hapticcontrol.my_Buttons[1];
        master_current_position = i_hapticcontrol.Touch_Position;  // left x, up y, forward z
        master_current_rpy = i_hapticcontrol.Touch_Rpy;   // ypr
        master_current_oritation = i_hapticcontrol.DeviceTransformRotation;

        //��һ�ν���ң����ģʽ
        if (Button1On == 1 && !TeleoperationEngaged)
        {
            //��ȡ��ʱ��ʵ�Ļ�е�۹ؽڽǶ�
            Array.Copy(i_aubocontrol.m_VirtualJointsState, slaver_current_joints, 6);
            aubo_forward(slaver_current_matrix, slaver_current_joints);
            slaver_current_oritation = DoubleMatrixToRotation(slaver_current_matrix);


            Array.Copy(master_current_position, master_start_position, master_current_position.Length);
            //����Ӧ��ʹ�ÿ���
            //master_start_position = master_current_position;
            master_start_oritation = master_current_oritation;

            Array.Copy(master_current_position, master_previous_position, master_current_position.Length);
            Array.Copy(master_current_rpy, master_previous_rpy, master_current_rpy.Length);
            //master_previous_position = master_current_position;
            master_previous_oritation = master_current_oritation;

            Array.Copy(slaver_current_position, slaver_start_position, slaver_current_position.Length);
            //slaver_start_position = slaver_current_position;
            slaver_start_oritation = slaver_current_oritation;

            TeleoperationEngaged = true;

        }
        else if (Button1On == 0 && TeleoperationEngaged)
        {
            TeleoperationEngaged = false;  //��һ���˳�ң����ģʽ
        }

        if (TeleoperationEngaged)
        {
            //-----------------------------------position-----------------------------------------
            double[] master_move_position = new double[3];
            double[] master_delta_move = new double[3];
            double[] slaver_target_postion = new double[3];
            for (int i = 0; i < 3; i++)
            {
                master_move_position[i] = (master_current_position[i] - master_start_position[i]) * 0.001; //mm->m
                master_delta_move[i] = (master_current_position[i] - master_previous_position[i]) * 0.001;
                //master_previous_position[i] = master_current_position[i];
            }
            // �ж�λ����
            double delta_move_flag = Math.Sqrt(Math.Pow(master_delta_move[0], 2) + Math.Pow(master_delta_move[1], 2) + Math.Pow(master_delta_move[2], 2));
            if ((delta_move_flag > move_linear_thresh_max) && (!is_angular_only))
            {
                is_linear_flag = false;
                UnityEngine.Debug.Log("����仯���������¿�ʼ��");
                return;
            }
            else if ((delta_move_flag > move_linear_thresh_min) && (!is_angular_only))
            {
                slaver_target_postion[0] = slaver_start_position[0] + master_move_position[2] * LinearScale * (-1); // x z
                slaver_target_postion[1] = slaver_start_position[1] + master_move_position[0] * LinearScale; //y x
                slaver_target_postion[2] = slaver_start_position[2] + master_move_position[1] * LinearScale; // z y
                Array.Copy(master_current_position, master_previous_position, master_current_position.Length);
                is_linear_flag = true;
                //UnityEngine.Debug.Log("linear move" + delta_move_flag.ToString());
            }
            else
            {
                slaver_target_postion[0] = slaver_current_position[0];
                slaver_target_postion[1] = slaver_current_position[1];
                slaver_target_postion[2] = slaver_current_position[2];
                is_linear_flag = false;
            }


            //------------------------------------rotation---------------------------------------------------
            Matrix4x4 master_move_rotation = new Matrix4x4();
            Matrix4x4 slaver_move_rotation = new Matrix4x4();
            Matrix4x4 slaver_target_rotation = new Matrix4x4();
            double[] master_delta_rpy = new double[3];
            for (int i = 0; i < 3; i++)
            {
                master_delta_rpy[i] = Math.Abs(master_current_rpy[i] - master_previous_rpy[i]);
                //master_previous_rpy[i] = master_current_rpy[i];
            }
            //�ж���ת��  -- ��RPY�����ж�
            double delta_move_rpy = master_delta_rpy[0] + master_delta_rpy[1] + master_delta_rpy[2];
            if ((delta_move_rpy > move_angular_thresh_max) && (!is_linear_only))
            {
                is_angular_flag = false;
                UnityEngine.Debug.Log("�Ƕȱ仯���������¿�ʼ��");
                return;
            }
            else if ((delta_move_rpy > move_angular_thresh_min) && (!is_linear_only))
            {
                master_move_rotation = master_current_oritation * (master_start_oritation.transpose);
                slaver_move_rotation = Transform_Ros_Touch * master_move_rotation * (Transform_Ros_Touch.transpose);
                slaver_target_rotation = slaver_move_rotation * slaver_start_oritation;
                Array.Copy(master_current_rpy, master_previous_rpy, master_current_rpy.Length);
                is_angular_flag = true;
                //UnityEngine.Debug.Log("angular move" + delta_move_rpy.ToString());
            }
            else
            {
                slaver_target_rotation = slaver_current_oritation;
                is_angular_flag = false;
            }


            //------------------------------------target--------------------------------------------
            if (is_angular_flag || is_linear_flag)
            {
                double[,] slaver_target_pose = GetSlaverTargetPose(slaver_target_postion, slaver_target_rotation);
                double[] temp_inverse;
                bool flag = GetInverseResult(slaver_target_pose, slaver_current_joints, out temp_inverse);
                if (flag)
                {
                    Array.Copy(temp_inverse, slaver_current_joints, temp_inverse.Length);
                    aubo_forward(slaver_current_matrix, slaver_current_joints);
                    slaver_current_position[0] = slaver_current_matrix[0, 3];
                    slaver_current_position[1] = slaver_current_matrix[1, 3];
                    slaver_current_position[2] = slaver_current_matrix[2, 3];
                    slaver_current_oritation = DoubleMatrixToRotation(slaver_current_matrix);

                    //i_aubocontrol.PubJoints(slaver_current_joints);

                    //AuboJointsMsg pub_joints = new AuboJointsMsg(slaver_current_joints);
                    //i_aubocontrol.m_Ros.Publish(i_aubocontrol.m_JointPubName, pub_joints);
                    i_aubocontrol.SetJointState(slaver_current_joints);
                }
                else
                {
                    UnityEngine.Debug.Log("No Solution!");
                }


            }



        }





    }


    public Matrix4x4 MatrixDoubleTo4x4(double[,] d_matrix)
    {
        Matrix4x4 f_matrix = new Matrix4x4();

        f_matrix[0, 0] = (float)d_matrix[0, 0];
        f_matrix[0, 1] = (float)d_matrix[0, 1];
        f_matrix[0, 2] = (float)d_matrix[0, 2];
        f_matrix[0, 3] = (float)d_matrix[0, 3];

        f_matrix[1, 0] = (float)d_matrix[1, 0];
        f_matrix[1, 1] = (float)d_matrix[1, 1];
        f_matrix[1, 2] = (float)d_matrix[1, 2];
        f_matrix[1, 3] = (float)d_matrix[1, 3];

        f_matrix[2, 0] = (float)d_matrix[2, 0];
        f_matrix[2, 1] = (float)d_matrix[2, 1];
        f_matrix[2, 2] = (float)d_matrix[2, 2];
        f_matrix[2, 3] = (float)d_matrix[2, 3];

        f_matrix[3, 0] = (float)d_matrix[3, 0];
        f_matrix[3, 1] = (float)d_matrix[3, 1];
        f_matrix[3, 2] = (float)d_matrix[3, 2];
        f_matrix[3, 3] = (float)d_matrix[3, 3];

        return f_matrix;
    }

    public double[,] Matrix4x4ToDoubleArray(Matrix4x4 f_matrix)
    {
        double[,] d_matrix = new double[4, 4];

        d_matrix[0, 0] = f_matrix.m00;
        d_matrix[0, 1] = f_matrix.m01;
        d_matrix[0, 2] = f_matrix.m02;
        d_matrix[0, 3] = f_matrix.m03;

        d_matrix[1, 0] = f_matrix.m10;
        d_matrix[1, 1] = f_matrix.m11;
        d_matrix[1, 2] = f_matrix.m12;
        d_matrix[1, 3] = f_matrix.m13;

        d_matrix[2, 0] = f_matrix.m20;
        d_matrix[2, 1] = f_matrix.m21;
        d_matrix[2, 2] = f_matrix.m22;
        d_matrix[2, 3] = f_matrix.m23;

        d_matrix[3, 0] = f_matrix.m30;
        d_matrix[3, 1] = f_matrix.m31;
        d_matrix[3, 2] = f_matrix.m32;
        d_matrix[3, 3] = f_matrix.m33;

        return d_matrix;
    }

    public Matrix4x4 DoubleMatrixToRotation(double[,] d_matrix)
    {
        Matrix4x4 f_matrix = new Matrix4x4();

        f_matrix[0, 0] = (float)d_matrix[0, 0];
        f_matrix[0, 1] = (float)d_matrix[0, 1];
        f_matrix[0, 2] = (float)d_matrix[0, 2];
        f_matrix[0, 3] = 0f;

        f_matrix[1, 0] = (float)d_matrix[1, 0];
        f_matrix[1, 1] = (float)d_matrix[1, 1];
        f_matrix[1, 2] = (float)d_matrix[1, 2];
        f_matrix[1, 3] = 0f;

        f_matrix[2, 0] = (float)d_matrix[2, 0];
        f_matrix[2, 1] = (float)d_matrix[2, 1];
        f_matrix[2, 2] = (float)d_matrix[2, 2];
        f_matrix[2, 3] = 0f;

        f_matrix[3, 0] = (float)d_matrix[3, 0];
        f_matrix[3, 1] = (float)d_matrix[3, 1];
        f_matrix[3, 2] = (float)d_matrix[3, 2];
        f_matrix[3, 3] = (float)d_matrix[3, 3];

        return f_matrix;
    }

    public double[,] GetSlaverTargetPose(double[] position, Matrix4x4 oritation)
    {
        double[,] target = new double[4, 4];

        target[0, 0] = oritation.m00;
        target[0, 1] = oritation.m01;
        target[0, 2] = oritation.m02;

        target[1, 0] = oritation.m10;
        target[1, 1] = oritation.m11;
        target[1, 2] = oritation.m12;

        target[2, 0] = oritation.m20;
        target[2, 1] = oritation.m21;
        target[2, 2] = oritation.m22;

        target[0, 3] = position[0];
        target[1, 3] = position[1];
        target[2, 3] = position[2];

        target[3, 0] = 0;
        target[3, 1] = 0;
        target[3, 2] = 0;
        target[3, 3] = 1;

        return target;
    }



    static int SIGN(double x)
    {
        return (x > 0) ? 1 : (x < 0) ? -1 : 0;
    }

    static double AntiSinCos(double sA, double cA)
    {
        double eps = 1e-8;
        double angle = 0;
        if ((Math.Abs(sA) < eps) && (Math.Abs(cA) < eps))
        {
            return 0;
        }
        if (Math.Abs(cA) < eps)
            angle = Math.PI / 2.0 * SIGN(sA);
        else if (Math.Abs(sA) < eps)
        {
            if (SIGN(cA) == 1)
                angle = 0;
            else
                angle = Math.PI;
        }
        else
        {
            angle = Math.Atan2(sA, cA);
        }

        return angle;
    }


    public double[] RotMatrixtoRPY(double[,] R)
    {
        double[] rpy = new double[3];
        double qz, qy, qx;
        double a = 1e-5;

        if ((Math.Abs(R[0, 0]) < a) && (Math.Abs(R[1, 0]) < a))
        {
            //singularity
            qz = 0;     //yaw
            qy = Math.Atan2(-R[2, 0], R[0, 0]);  // pitch
            qx = Math.Atan2(-R[1, 2], R[1, 1]);  //roll
        }
        else
        {
            qz = Math.Atan2(R[1, 0], R[0, 0]);
            double sz = Math.Sin(qz);
            double cz = Math.Cos(qz);
            qy = Math.Atan2(-R[2, 0], cz * R[0, 0] + sz * R[1, 0]);
            qx = Math.Atan2(sz * R[0, 2] - cz * R[1, 2], cz * R[1, 1] - sz * R[0, 1]);
        }
        rpy[0] = qx;
        rpy[1] = qy;
        rpy[2] = qz;

        return rpy;
    }

    // ע��˳��x,y,z,w
    public double[] RotMatrixtoQuat(double[,] R)
    {
        double[] quat = new double[4];

        double temp1 = R[0, 0] + R[1, 1] + R[2, 2] + 1;
        if (Math.Abs(temp1) < 1e-6)
            temp1 = 0.0;
        double w = (1.0 / 2) * Math.Sqrt(temp1);
        double x = (R[2, 1] - R[1, 2]) / (4 * w);
        double y = (R[0, 2] - R[2, 0]) / (4 * w);
        double z = (R[1, 0] - R[0, 1]) / (4 * w);

        quat[0] = x;
        quat[1] = y;
        quat[2] = z;
        quat[3] = w;

        return quat;
    }

    // input radian
    // ���˶�ѧ
    public void aubo_forward(double[,] T, double[] q)
    {
        double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
        double C1 = Math.Cos(q1), C2 = Math.Cos(q2), C4 = Math.Cos(q4), C5 = Math.Cos(q5), C6 = Math.Cos(q6);
        double C23 = Math.Cos(q2 - q3), C234 = Math.Cos(q2 - q3 + q4), C2345 = Math.Cos(q2 - q3 + q4 - q5), C2345p = Math.Cos(q2 - q3 + q4 + q5);
        double S1 = Math.Sin(q1), S2 = Math.Sin(q2), S4 = Math.Sin(q4), S5 = Math.Sin(q5), S6 = Math.Sin(q6);
        double S23 = Math.Sin(q2 - q3), S234 = Math.Sin(q2 - q3 + q4);

        T[0, 0] = -C6 * S1 * S5 + C1 * (C234 * C5 * C6 - S234 * S6);
        T[0, 1] = S1 * S5 * S6 - C1 * (C4 * C6 * S23 + C23 * C6 * S4 + C234 * C5 * S6);
        T[0, 2] = C5 * S1 + C1 * C234 * S5;
        T[0, 3] = (d2 + C5 * d6) * S1 - C1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

        T[1, 0] = C234 * C5 * C6 * S1 + C1 * C6 * S5 - S1 * S234 * S6;
        T[1, 1] = -C6 * S1 * S234 - (C234 * C5 * S1 + C1 * S5) * S6;
        T[1, 2] = -C1 * C5 + C234 * S1 * S5;
        T[1, 3] = -C1 * (d2 + C5 * d6) - S1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

        T[2, 0] = C5 * C6 * S234 + C234 * S6;
        T[2, 1] = C234 * C6 - C5 * S234 * S6;
        T[2, 2] = S234 * S5;
        T[2, 3] = d1 + a2 * C2 + a3 * C23 + d5 * C234 + d6 * C2345 / 2 - d6 * C2345p / 2;

        T[3, 0] = 0;
        T[3, 1] = 0;
        T[3, 2] = 0;
        T[3, 3] = 1;
    }

    // ���˶�ѧ ���������Ŀ  8��⣺6��8��
    // output radian
    // ʹ��out������Ҫʵ������double [,] q_sols; // no need to initialize
    public static int aubo_inverse(out double[,] q_sols, double[,] T)
    {
        q_sols = new double[6, 8];
        bool singularity = false;

        int num_sols = 0;
        double nx = T[0, 0], ox = T[0, 1], ax = T[0, 2], px = T[0, 3];
        double ny = T[1, 0], oy = T[1, 1], ay = T[1, 2], py = T[1, 3];
        double nz = T[2, 0], oz = T[2, 1], az = T[2, 2], pz = T[2, 3];

        //////////////////////// shoulder rotate joint (q1) //////////////////////////////
        double[] q1 = new double[2];

        double A1 = d6 * ay - py;
        double B1 = d6 * ax - px;
        double R1 = A1 * A1 + B1 * B1 - d2 * d2;

        if (R1 < 0.0)
            return num_sols;
        else
        {
            double R12 = Math.Sqrt(R1);
            q1[0] = AntiSinCos(A1, B1) - AntiSinCos(d2, R12);
            q1[1] = AntiSinCos(A1, B1) - AntiSinCos(d2, -R12);
            for (int i = 0; i < 2; i++)
            {
                while (q1[i] > Math.PI)
                    q1[i] -= 2 * Math.PI;
                while (q1[i] < -Math.PI)
                    q1[i] += 2 * Math.PI;
            }
        }

        double[,] q5 = new double[2, 2];

        for (int i = 0; i < 2; i++)
        {
            double C1 = Math.Cos(q1[i]), S1 = Math.Sin(q1[i]);
            double B5 = -ay * C1 + ax * S1;
            double M5 = (-ny * C1 + nx * S1);
            double N5 = (-oy * C1 + ox * S1);

            double R5 = Math.Sqrt(M5 * M5 + N5 * N5);

            q5[i, 0] = AntiSinCos(R5, B5);
            q5[i, 1] = AntiSinCos(-R5, B5);
        }

        double q6;
        double[] q3 = new double[2], q2 = new double[2], q4 = new double[2];

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                double C1 = Math.Cos(q1[i]), S1 = Math.Sin(q1[i]);
                double S5 = Math.Sin(q5[i, j]);

                double A6 = (-oy * C1 + ox * S1);
                double B6 = (ny * C1 - nx * S1);

                if (Math.Abs(S5) < ZERO_THRESH)
                {
                    singularity = true;
                    break;
                }
                else
                    q6 = AntiSinCos(A6 * S5, B6 * S5);

                double C6 = Math.Cos(q6);
                double S6 = Math.Sin(q6);

                double pp1 = C1 * (ax * d6 - px + d5 * ox * C6 + d5 * nx * S6) + S1 * (ay * d6 - py + d5 * oy * C6 + d5 * ny * S6);
                double pp2 = -d1 - az * d6 + pz - d5 * oz * C6 - d5 * nz * S6;
                double B3 = (pp1 * pp1 + pp2 * pp2 - a2 * a2 - a3 * a3) / (2 * a2 * a3);

                if ((1 - B3 * B3) < ZERO_THRESH)
                {
                    singularity = true;
                    continue;
                }
                else
                {
                    double Sin3 = Math.Sqrt(1 - B3 * B3);
                    q3[0] = AntiSinCos(Sin3, B3);
                    q3[1] = AntiSinCos(-Sin3, B3);
                }

                for (int k = 0; k < 2; k++)
                {
                    double C3 = Math.Cos(q3[k]), S3 = Math.Sin(q3[k]);
                    double A2 = pp1 * (a2 + a3 * C3) + pp2 * (a3 * S3);
                    double B2 = pp2 * (a2 + a3 * C3) - pp1 * (a3 * S3);

                    q2[k] = AntiSinCos(A2, B2);

                    double C2 = Math.Cos(q2[k]), S2 = Math.Sin(q2[k]);

                    double A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6);
                    double B4 = oz * C6 + nz * S6;
                    double A41 = pp1 - a2 * S2;
                    double B41 = pp2 - a2 * C2;

                    q4[k] = AntiSinCos(A4, B4) - AntiSinCos(A41, B41);
                    while (q4[k] > Math.PI)
                        q4[k] -= 2 * Math.PI;
                    while (q4[k] < -Math.PI)
                        q4[k] += 2 * Math.PI;

                    q_sols[0, num_sols] = q1[i]; q_sols[1, num_sols] = q2[k];
                    q_sols[2, num_sols] = q3[k]; q_sols[3, num_sols] = q4[k];
                    q_sols[4, num_sols] = q5[i, j]; q_sols[5, num_sols] = q6;
                    num_sols++;
                }
            }
        }

        return num_sols;
    }

    //input current transformation matrix and last joint angle;
    //out matrixxd q_sols;
    //��֤����Ƿ��������˶�ѧ����������������ͬ�Ľ�
    public double[,] ProcessIK(double[,] T)
    {
        int numSols = 0;
        int realSols = 0;
        double[,] qSols = new double[6, 8];
        double[,] qTemp = new double[6, 8];
        double[,] qRealSols;


        numSols = aubo_inverse(out qSols, T);


        for (int i = 0; i < numSols; i++)
        {
            double[,] TT = new double[4, 4];
            double[] temp = new double[6];
            double err = 0;

            // ��ȡqSols�ĵ�i��
            for (int row = 0; row < 6; row++)
            {
                temp[row] = qSols[row, i];
            }


            aubo_forward(TT, temp);

            for (int j = 0; j < 4; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    err += Math.Abs(TT[j, k] - T[j, k]);
                }
            }
            if (err < 1e-5)
            {
                realSols++;
                for (int row = 0; row < 6; row++)
                {
                    qTemp[row, realSols - 1] = qSols[row, i];
                }
            }
        }

        if (realSols != numSols)
        {
            Console.WriteLine("IK ERROR");
        }

        // ��ȡqTemp����Ч����
        qRealSols = new double[6, realSols];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < realSols; j++)
            {
                qRealSols[i, j] = qTemp[i, j];
            }
        }

        // ���û����Ч�Ľ�
        if (realSols == 0)
        {
            // ������Ҫ�����޽�����
        }

        return qRealSols;
    }

    //��֤���������ʵ��е�۹ؽڽǶ��Ƿ��������Ҫ��
    public bool VerifyIK(double[,] qSols, double[] qInput)
    {
        bool flag = false;
        int numSolutions = qSols.GetLength(1); // qSols����������������

        for (int i = 0; i < numSolutions; i++)
        {
            double err = 0;
            for (int j = 0; j < qInput.Length; j++)
            {
                // ����qInput��qSols�ĵ�i��֮��Ĳ�ֵ��ŷ�Ͼ���
                err += Math.Pow(qInput[j] - qSols[j, i], 2);
            }
            err = Math.Sqrt(err); // ���ŷ�Ͼ���ļ���

            if (err < ZERO_THRESH)
            {
                flag = true;
                break; // �ҵ�����һ�������������������˳�ѭ��
            }
        }

        return flag;
    }

    //choose solution ;input all q_sols,and last q_old;
    //out put the nearest q solution;
    //�ҳ���ԭ��е�۽Ƕ�����������q_choose
    public static bool ChooseIKonRefJoint(double[,] qSols, double[] qRef, out double[] qChoose)
    {
        int numSolutions = qSols.GetLength(1); // ��ȡ�����������qSols������
        if (numSolutions == 0)
        {
            qChoose = null;
            return false;
        }

        qChoose = new double[qRef.Length]; // ��ʼ��qChoose��������qRef������ͬ
        double minSum = double.MaxValue; // ���ڴ洢��С���
        int index = 0; // �洢��С����Ӧ�Ľ������

        for (int i = 0; i < numSolutions; i++)
        {
            double err = 0;
            for (int j = 0; j < qRef.Length; j++)
            {
                // ����qRef��qSols�ĵ�i��֮��Ĳ�ֵ��ŷ�Ͼ���
                err += Math.Pow(qRef[j] - qSols[j, i], 2);
            }
            err = Math.Sqrt(err); // ���ŷ�Ͼ���ļ���

            if (err < minSum)
            {
                index = i;
                minSum = err; // ������С�����Ӧ�Ľ������
            }
        }

        // ѡȡ��qRef��ӽ��Ľ���ΪqChoose
        for (int i = 0; i < qRef.Length; i++)
        {
            qChoose[i] = qSols[i, index];
        }

        return true;
    }

    //��������ض��Ƕ�Ҫ������������ؽ�1����0�ȣ��ؽ�3����0�ȣ��ؽ�5����0�Ľ�
    public static bool ChooseIKonConfiguration(double[,] qSols, int chooseMode, out double[] qChoose)
    {
        int N = qSols.GetLength(1); // ��ȡ�����������qSols������
        qChoose = null;

        for (int i = 0; i < N; i++)
        {
            // ���ݽ���ض��ؽ�ֵ������
            int flag = (qSols[0, i] > 0 ? 1 : 0) +
                       (qSols[2, i] > 0 ? 1 << 1 : 0) +
                       (qSols[4, i] > 0 ? 1 << 2 : 0);

            if (flag == chooseMode)
            {
                // ����ҵ�ƥ������ã����������ΪqChoose
                int numRows = qSols.GetLength(0); // ��ȡ��������ÿ����ĳ���
                qChoose = new double[numRows];
                for (int row = 0; row < numRows; row++)
                {
                    qChoose[row] = qSols[row, i];
                }
                return true; // ���سɹ��ҵ���
            }
        }

        // ���û���ҵ�ƥ��Ľ⣬����false
        return false;
    }

    //ѡ������ؽڽǶ����Ƶ��������ֻ����ԭʼ��
    public static bool SelectIK(double[,] qSols, double[,] angleLimits, out double[,] qSolsSelected, int numSols)
    {
        int N = qSols.GetLength(1);
        if (N == 0)
        {
            qSolsSelected = null;
            numSols = 0;
            return false;
        }

        qSolsSelected = new double[ARM_DOF, N]; // ������ԭʼ��
        numSols = 0;

        for (int i = 0; i < N; i++)
        {
            bool valid = true;
            for (int j = 0; j < ARM_DOF; j++)
            {
                if (qSols[j, i] > angleLimits[j, 1] || qSols[j, i] < angleLimits[j, 0])
                {
                    valid = false;
                    break;
                }
            }

            if (valid)
            {
                for (int k = 0; k < ARM_DOF; k++)
                {
                    qSolsSelected[k, numSols] = qSols[k, i];
                }
                numSols++;
            }
        }

        if (numSols > 0)
        {
            // ����qSolsSelected����Ĵ�С��ƥ��ʵ��ѡ�еĽ������
            double[,] tempSols = new double[ARM_DOF, numSols];
            for (int i = 0; i < ARM_DOF; i++)
            {
                for (int j = 0; j < numSols; j++)
                {
                    tempSols[i, j] = qSolsSelected[i, j];
                }
            }
            qSolsSelected = tempSols;
            return true;
        }
        else
        {
            qSolsSelected = null;
            return false;
        }
    }

    //ѡ������ؽڽǶ����Ƶ�������������ǹؽڽǶȼӼ�2�����Ƿ��Է�������������ʵ��ʹ��ʱ�������Ƿ�������
    public bool SelectIK_Multi(double[,] qSols, double[,] angleLimits, out double[,] qSolsSelected, out byte numSols)
    {
        int N = qSols.GetLength(1);
        if (N == 0)
        {
            qSolsSelected = null;
            numSols = 0;
            return false;
        }

        qSolsSelected = new double[ARM_DOF, N * 3]; // ����������ÿ���ⶼ���ԼӼ�2PI���Է�������
        numSols = 0;
        bool valid;

        for (int i = 0; i < N; i++)
        {
            valid = true;
            for (int j = 0; j < ARM_DOF; j++)
            {
                if (qSols[j, i] > angleLimits[j, 1] || qSols[j, i] < angleLimits[j, 0])
                {
                    valid = false;
                    break;
                }
            }

            if (valid)
            {
                // �洢����������ԭʼ��
                for (int k = 0; k < ARM_DOF; k++)
                {
                    qSolsSelected[k, numSols] = qSols[k, i];
                }
                numSols++;

                double[] tmp = new double[ARM_DOF];
                for (int k = 0; k < ARM_DOF; k++)
                {
                    tmp[k] = qSols[k, i];
                }

                // ���Լ�2PI
                valid = true;
                do
                {
                    for (int j = 0; j < ARM_DOF && valid; j++)
                    {
                        tmp[j] += 2 * Math.PI;
                        if (tmp[j] > angleLimits[j, 1])
                        {
                            valid = false;
                        }
                    }

                    if (valid)
                    {
                        for (int k = 0; k < ARM_DOF; k++)
                        {
                            qSolsSelected[k, numSols] = tmp[k];
                        }
                        numSols++;
                    }

                } while (valid && numSols < N * 3);

                // ���Լ�2PI
                for (int k = 0; k < ARM_DOF; k++)
                {
                    tmp[k] = qSols[k, i];
                }
                valid = true;
                do
                {
                    for (int j = 0; j < ARM_DOF && valid; j++)
                    {
                        tmp[j] -= 2 * Math.PI;
                        if (tmp[j] < angleLimits[j, 0])
                        {
                            valid = false;
                        }
                    }

                    if (valid)
                    {
                        for (int k = 0; k < ARM_DOF; k++)
                        {
                            qSolsSelected[k, numSols] = tmp[k];
                        }
                        numSols++;
                    }

                } while (valid && numSols < N * 3);
            }
        }

        if (numSols > 0)
            return true;
        else
        {
            qSolsSelected = null;
            return false;
        }
    }


    // ʵ�ʵ��ú����������->����ؽ����ƵĽ�->��ԭ�Ƕ�����Ľ�
    public bool GetInverseResult(double[,] T_target, double[] q_ref, out double[] q_result)
    {
        q_result = null;
        int num_sols = 0;


        double[,] q_sols_all = new double[6, 8];
        //double[,] q_sols_RmZero; 
        double[,] q_sols_inlimit = new double[6, 8];
        double[,] q_sol_Valid;


        num_sols = aubo_inverse(out q_sols_all, T_target);

        //remove  zero matrixXd
        double[,] q_sols_RmZero = new double[ARM_DOF, num_sols];
        for (int i = 0; i < ARM_DOF; i++)
        {
            for (int j = 0; j < num_sols; j++)
            {
                q_sols_RmZero[i, j] = q_sols_all[i, j];
            }
        }


        if (q_sols_RmZero.GetLength(1) != 0)
        {
            // Remove not in limited data and zero matrix
            bool ret2 = SelectIK(q_sols_RmZero, AngleLimit, out q_sols_inlimit, num_sols);

            // Remove zero matrix
            // q_sol_Valid = RemoveZeroSols(q_sols_inlimit, num_sols);

            if (q_sols_inlimit.GetLength(1) != 0 && ret2)
            {
                bool ret3 = ChooseIKonRefJoint(q_sols_inlimit, q_ref, out q_result);

                if (ret3)
                {
                    Console.WriteLine("Find solution choose");
                    return true;
                }
                else
                {
                    Console.WriteLine("No solution choose");
                    return false;
                }
            }
            else
            {
                Console.WriteLine("No valid sols");
                return false;
            }
        }
        else
        {
            Console.WriteLine("Inverse result num is 0");
            return false;
        }
    }
}
