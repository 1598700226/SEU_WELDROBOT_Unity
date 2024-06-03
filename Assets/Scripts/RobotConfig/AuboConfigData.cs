[System.Serializable]
public class AuboConfigData
{
    public double initJoint1;
    public double initJoint2;
    public double initJoint3;
    public double initJoint4;
    public double initJoint5;
    public double initJoint6;

    public AuboConfigData()
    {

    }

    public AuboConfigData(double initJoint1, double initJoint2, double initJoint3,
                          double initJoint4, double initJoint5, double initJoint6)
    {
        this.initJoint1 = initJoint1;
        this.initJoint2 = initJoint2;
        this.initJoint3 = initJoint3;
        this.initJoint4 = initJoint4;
        this.initJoint5 = initJoint5;
        this.initJoint6 = initJoint6;
    }

    public override string ToString()
    {
        return $"initJoint1 {initJoint1}; initJoint2 {initJoint2}; initJoint3 {initJoint3};" +
            $" initJoint4 {initJoint4}; initJoint5 {initJoint5}; initJoint6 {initJoint6}";
    }
}
