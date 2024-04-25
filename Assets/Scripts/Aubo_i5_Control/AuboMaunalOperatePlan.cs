using RosMessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class AuboMaunalOperatePlan : MonoBehaviour
{
    public bool isMaunalOperateMode = false;
    public List<double[]> positon = new List<double[]>();
    public List<Quaternion<FLU>> orientation = new List<Quaternion<FLU>>();

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ClearData() 
    {
        positon.Clear();
        orientation.Clear();
    }

    public void AddEndPointPositionAndOrientation(PointMsg posiont, QuaternionMsg quaternion) 
    {
        if(!isMaunalOperateMode)
        {
            Debug.Log("【AuboMaunalOperatePlan】AddEndPointPositionAndOrientation fail，当前非手动拖拽示教模式");
            return;
        }

        double[] posi = new double[3] {
            posiont.x,
            posiont.y,
            posiont.z
            };
        Quaternion<FLU> quat = new Quaternion<FLU>(
            (float)quaternion.x, 
            (float)quaternion.y, 
            (float)quaternion.z, 
            (float)quaternion.w
            );
        positon.Add(posi);
        orientation.Add(quat);
    }

    public void GetPointPositonAndOrientation(out List<double[]> positon, out List<Quaternion<FLU>> orientation)
    {
        positon = this.positon;
        orientation = this.orientation;
    }
}
