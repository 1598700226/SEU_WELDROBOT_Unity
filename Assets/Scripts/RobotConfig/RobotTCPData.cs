[System.Serializable]
public class RobotTCPData
{
    public float tcp_x;
    public float tcp_y;
    public float tcp_z;

    public RobotTCPData()
    { 
    
    }

    public RobotTCPData(float tcp_x, float tcp_y, float tcp_z)
    {
        this.tcp_x = tcp_x;
        this.tcp_y = tcp_y;
        this.tcp_z = tcp_z;
    }

    public override string ToString()
    {
        return $"x {tcp_x}; y {tcp_y}; z {tcp_z}";
    }
}