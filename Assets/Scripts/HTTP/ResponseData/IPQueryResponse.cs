[System.Serializable]
public class IPQueryResponse
{
    public string status;
    public string info;
    public string infocode;
    public string province;
    public string city;
    public string adcode;
    public string rectangle;

    public override string ToString()
    {
        return $"status={status}, info={info}, infocode={infocode}, province={province}, city={city}, adcode={adcode}, rectangle={rectangle}";
    }
}