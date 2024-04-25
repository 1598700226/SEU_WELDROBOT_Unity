using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;

public class UnityPublish_MoveCommand : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/smoother_cmd_vel";
    public float move_speed = 0.1f;
    public float turn_speed = 0.1f;
    public float max_speed = 0.3f;
    public float min_speed = 0.1f;

    public Button forwardLeftBtn;
    public Button forwardBtn;
    public Button forwardRightBtn;

    public Button LeftBtn;
    public Button StopBtn;
    public Button RightBtn;

    public Button BackLeftBtn;
    public Button BackBtn;
    public Button BackRightBtn;


    private ButtonKeepDownCheck buttonKeepDownCheck_fl;
    private ButtonKeepDownCheck buttonKeepDownCheck_f;
    private ButtonKeepDownCheck buttonKeepDownCheck_fr;

    private ButtonKeepDownCheck buttonKeepDownCheck_l;
    private ButtonKeepDownCheck buttonKeepDownCheck_stop;
    private ButtonKeepDownCheck buttonKeepDownCheck_r;

    private ButtonKeepDownCheck buttonKeepDownCheck_bl;
    private ButtonKeepDownCheck buttonKeepDownCheck_b;
    private ButtonKeepDownCheck buttonKeepDownCheck_br;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>(topicName);

        buttonKeepDownCheck_fl = forwardLeftBtn.GetComponent<ButtonKeepDownCheck>();
        buttonKeepDownCheck_f = forwardBtn.GetComponent<ButtonKeepDownCheck>();
        buttonKeepDownCheck_fr = forwardRightBtn.GetComponent<ButtonKeepDownCheck>();

        buttonKeepDownCheck_l = LeftBtn.GetComponent<ButtonKeepDownCheck>();
        buttonKeepDownCheck_stop = StopBtn.GetComponent<ButtonKeepDownCheck>();
        buttonKeepDownCheck_r = RightBtn.GetComponent<ButtonKeepDownCheck>();

        buttonKeepDownCheck_bl = BackLeftBtn.GetComponent<ButtonKeepDownCheck>();
        buttonKeepDownCheck_b = BackBtn.GetComponent<ButtonKeepDownCheck>();
        buttonKeepDownCheck_br = BackRightBtn.GetComponent<ButtonKeepDownCheck>();
    }

    // Update is called once per frame
    void Update()
    {
        listenCarControl();
    }

    public void SendMoveCommandtoTopic(double x, double y, double z, double th) {
        RosMessageTypes.Geometry.Vector3Msg lineSpeed = new RosMessageTypes.Geometry.Vector3Msg(
                x * move_speed,
                y * move_speed,
                z * move_speed
            );

        RosMessageTypes.Geometry.Vector3Msg angularSpeed = new RosMessageTypes.Geometry.Vector3Msg(
            0,
            0,
            th * turn_speed
        );

        RosMessageTypes.Geometry.TwistMsg twistMsg = new RosMessageTypes.Geometry.TwistMsg(lineSpeed, angularSpeed);
        ros.Publish(topicName, twistMsg);
    }

    void listenCarControl()
    {
        // 前进
        if (buttonKeepDownCheck_fl.isButtonPressed)
        {
            SendMoveCommandtoTopic(1, 0, 0, 1);
        }
        if (buttonKeepDownCheck_f.isButtonPressed)
        {
            SendMoveCommandtoTopic(1, 0, 0, 0);
        }
        if (buttonKeepDownCheck_fr.isButtonPressed)
        {
            SendMoveCommandtoTopic(1, 0, 0, -1);
        }
        // 后退
        if (buttonKeepDownCheck_bl.isButtonPressed)
        {
            SendMoveCommandtoTopic(-1, 0, 0, -1);
        }
        if (buttonKeepDownCheck_b.isButtonPressed)
        {
            SendMoveCommandtoTopic(-1, 0, 0, 0);
        }
        if (buttonKeepDownCheck_br.isButtonPressed)
        {
            SendMoveCommandtoTopic(-1, 0, 0, 1);
        }
        // 左转
        if (buttonKeepDownCheck_l.isButtonPressed)
        {
            SendMoveCommandtoTopic(0, 0, 0, 1);
        }
        if (buttonKeepDownCheck_stop.isButtonPressed)
        {
            SendMoveCommandtoTopic(0, 0, 0, 0);
        }
        // 右转
        if (buttonKeepDownCheck_r.isButtonPressed)
        {
            SendMoveCommandtoTopic(0, 0, 0, -1);
        }
    }

    public void SetSpeed(float speed) 
    {
        float itemSpeed = speed > max_speed ? max_speed : speed < min_speed ? min_speed : speed;
        move_speed = itemSpeed;
        turn_speed = itemSpeed;
    }
}
