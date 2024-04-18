using System.Collections;
using System.Collections.Generic;
using System.Net;
using UnityEngine;
using UnityEngine.UI;

public class AuboVirtualController : MonoBehaviour
{

    public bool send_to_actual_Aubo = false; // 是否发送给真实机械臂

    public float joint_1_init_angle = 0;
    public bool joint_1_angle_inverse = false;
    public Slider slider_joint_1;
    public GameObject joint_1;

    public float joint_2_init_angle = 0;
    public bool joint_2_angle_inverse = false;
    public Slider slider_joint_2;
    public GameObject joint_2;

    public float joint_3_init_angle = 0;
    public bool joint_3_angle_inverse = false;
    public Slider slider_joint_3;
    public GameObject joint_3;

    public float joint_4_init_angle = 0;
    public bool joint_4_angle_inverse = false;
    public Slider slider_joint_4;
    public GameObject joint_4;

    public float joint_5_init_angle = 0;
    public bool joint_5_angle_inverse = false;
    public Slider slider_joint_5;
    public GameObject joint_5;

    public float joint_6_init_angle = 0;
    public bool joint_6_angle_inverse = false;
    public Slider slider_joint_6;
    public GameObject joint_6;

    public float stiffness = 0;
    public float damping = 0;
    public float forceLimit = 0;
    public float targetVelocity = 0;

    private float joint_1_now_angle;
    private float joint_2_now_angle;
    private float joint_3_now_angle;
    private float joint_4_now_angle;
    private float joint_5_now_angle;
    private float joint_6_now_angle;

    // Start is called before the first frame update
    void Start()
    {
        slider_joint_1.value = joint_1_init_angle;
        slider_joint_2.value = joint_2_init_angle;
        slider_joint_3.value = joint_3_init_angle;
        slider_joint_4.value = joint_4_init_angle;
        slider_joint_5.value = joint_5_init_angle;
        slider_joint_6.value = joint_6_init_angle;

        joint_1_now_angle = joint_1_init_angle;
        joint_2_now_angle = joint_2_init_angle;
        joint_3_now_angle = joint_3_init_angle;
        joint_4_now_angle = joint_4_init_angle;
        joint_5_now_angle = joint_5_init_angle;
        joint_6_now_angle = joint_6_init_angle;
    }

    // Update is called once per frame
    void Update()
    {
/*        UpdateArmRotation(joint_1, "y", slider_joint_1.value - joint_1_now_angle, joint_1_angle_inverse);
        UpdateArmRotation(joint_2, "y", slider_joint_2.value - joint_2_now_angle, joint_2_angle_inverse);
        UpdateArmRotation(joint_3, "y", slider_joint_3.value - joint_3_now_angle, joint_3_angle_inverse);
        UpdateArmRotation(joint_4, "y", slider_joint_4.value - joint_4_now_angle, joint_4_angle_inverse);
        UpdateArmRotation(joint_5, "y", slider_joint_5.value - joint_5_now_angle, joint_5_angle_inverse);
        UpdateArmRotation(joint_6, "y", slider_joint_6.value - joint_6_now_angle, joint_6_angle_inverse);


        joint_1_now_angle = slider_joint_1.value;
        joint_2_now_angle = slider_joint_2.value;
        joint_3_now_angle = slider_joint_3.value;
        joint_4_now_angle = slider_joint_4.value;
        joint_5_now_angle = slider_joint_5.value;
        joint_6_now_angle = slider_joint_6.value;*/
    }

    private void UpdateArmRotation(GameObject joint, string rotation_axis, float rotation_angle, bool isInverse)
    {
        float angle = rotation_angle;
        if (isInverse) {
            angle = -angle;
        }

        if (rotation_axis.ToLower().Equals("x"))
            joint.transform.Rotate(angle, 0, 0, Space.Self);
        if (rotation_axis.ToLower().Equals("y"))
            joint.transform.Rotate(0, angle, 0, Space.Self);
        if (rotation_axis.ToLower().Equals("z"))
            joint.transform.Rotate(0, 0, angle, Space.Self);
    }

    // 根据Articulation进行旋转
    private void UpdateArmRotationByArticulation(GameObject joint, float rotation_angle, bool isInverse) {
        ArticulationBody articulation_joint = joint.GetComponent<ArticulationBody>();
        ArticulationDrive drive = articulation_joint.xDrive;
        if (isInverse)
            drive.target = -rotation_angle;
        else
            drive.target = rotation_angle;
        drive.stiffness = stiffness;
        drive.damping = damping;
        drive.forceLimit = forceLimit;
        drive.targetVelocity = targetVelocity;
        articulation_joint.xDrive = drive;
    }

    private void FixedUpdate()
    {
        UpdateArmRotationByArticulation(joint_1, slider_joint_1.value - joint_1_now_angle, joint_1_angle_inverse);
        UpdateArmRotationByArticulation(joint_2, slider_joint_2.value - joint_2_now_angle, joint_2_angle_inverse);
        UpdateArmRotationByArticulation(joint_3, slider_joint_3.value - joint_3_now_angle, joint_3_angle_inverse);
        UpdateArmRotationByArticulation(joint_4, slider_joint_4.value - joint_4_now_angle, joint_4_angle_inverse);
        UpdateArmRotationByArticulation(joint_5, slider_joint_5.value - joint_5_now_angle, joint_5_angle_inverse);
        UpdateArmRotationByArticulation(joint_6, slider_joint_6.value - joint_6_now_angle, joint_6_angle_inverse);
    }
}
