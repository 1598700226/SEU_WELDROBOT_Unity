using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;

public class GaoDeApi : MonoBehaviour
{
    public RawImage rawImage;
    public TMP_Text queryText;
    [Header("��ͼ����ϵ��1-17")]
    public string ZOOM;

    public IPQueryResponse IPQueryResponse;

    private static string GAODE_KEY = "d1169efd4c61b21248c70733bc731f0b";                    // ��Կ
    private static string GAODE_MAP_HTTPS_URL = "https://restapi.amap.com/v3/staticmap";     // ��̬��ͼ��ѯURL
    private static string GAODE_IP_HTTPS_URL = "https://restapi.amap.com/v3/ip";             // IP��λ��ѯURL

    private string query_longitude = string.Empty;          // ����ip��ѯ���ľ���
    private string query_latitude = string.Empty;           // ����ip��ѯ����γ��


    private void Start()
    {
    }

    public void RefreshRequest(string ip, string longitude, string latitude) 
    {
        // �����γ��δ���������IP��ѯλ�ã�֮���ȡ��̬ͼƬ
        if (string.IsNullOrEmpty(longitude) || string.IsNullOrEmpty(latitude))
        {
            StartCoroutine(GetIPandImage(ip));
        }
        // ���ݾ�γ�ȣ�����ݾ�γ�Ȼ�ȡͼƬλ��
        else
        {
            if ((double.TryParse(longitude, out double longitude_d) && double.TryParse(latitude, out double latitude_d)))
            {
                StartCoroutine(GetStaticImageCoroutine(GaoDeApi_GetStaticMapURL(GAODE_KEY, longitude, latitude, ZOOM)));
            }
        }
    }

    IEnumerator GetIPandImage(string ip) 
    {
        yield return StartCoroutine(GetIPPositionCoroutine(GaoDeApi_GetIPQueryURL(GAODE_KEY, ip)));

        if (!query_longitude.Equals(string.Empty) && !query_latitude.Equals(string.Empty))
        {
            yield return StartCoroutine(GetStaticImageCoroutine(
                GaoDeApi_GetStaticMapURL(GAODE_KEY,
                    query_longitude,
                    query_latitude, ZOOM)
                ));
        }
    }

    // ���þ�̬ͼƬ��ѯ�ӿ�
    IEnumerator GetStaticImageCoroutine(string imageUrl)
    {
        using (UnityWebRequest uwr = UnityWebRequestTexture.GetTexture(imageUrl))
        {
            yield return uwr.SendWebRequest();

            if (uwr.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError("Failed to download image: " + uwr.error);
            }
            else
            {
                // ���Content-Type
                string contentType = uwr.GetResponseHeader("Content-Type");
                Debug.Log(contentType);
                if (contentType != null && contentType.StartsWith("image"))
                {
                    var texture = DownloadHandlerTexture.GetContent(uwr);
                    rawImage.texture = texture;
                    Debug.Log("Success to download Gaode map image");
                }
                else 
                {
                    Debug.Log("Failed to download image: Maybe param error");
                }
            }
        }
    }

    // ����ip��ѯ�ӿ�
    IEnumerator GetIPPositionCoroutine(string url)
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get(url))
        {
            // ��������
            yield return webRequest.SendWebRequest();
            query_longitude = string.Empty;
            query_latitude = string.Empty;

            if (webRequest.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError("Error: " + webRequest.error);
            }
            else
            {
                // ����JSON����
                IPQueryResponse = JsonUtility.FromJson<IPQueryResponse>(webRequest.downloadHandler.text);
                // ��ӡ���ؽ��
                Debug.Log("ipquery result: " + IPQueryResponse.ToString());
                string lable = $"״̬��{IPQueryResponse.info}, �ص㣺{IPQueryResponse.province},{IPQueryResponse.city}";
                queryText.text = lable;
                if (!string.IsNullOrEmpty(IPQueryResponse.rectangle))
                {
                    query_longitude = ((double.Parse(IPQueryResponse.rectangle.Split(";")[0].Split(",")[0]) + 
                        double.Parse(IPQueryResponse.rectangle.Split(";")[1].Split(",")[0])) / 2).ToString("F6");
                    query_latitude = ((double.Parse(IPQueryResponse.rectangle.Split(";")[0].Split(",")[1]) +
                        double.Parse(IPQueryResponse.rectangle.Split(";")[1].Split(",")[1])) / 2).ToString("F6");
                }
            }
        }
    }

    /*****************************************************�ߵµ�ͼapi********************************************************/
    // �ο�api�ĵ���https://lbs.amap.com/api/webservice/guide/api/staticmaps
    private string GaoDeApi_GetStaticMapURL(string gaode_key, string longitude, string latitude, string zoom) 
    {
        string location = longitude + "," + latitude;
        string markers = "small,0x0000FF,A:" + location;

        if (string.IsNullOrEmpty(zoom))
        {
            return GAODE_MAP_HTTPS_URL + $"?size=200*200&scale=2&key={gaode_key}&location={location}&markers={markers}";
        }
        else 
        {
            return GAODE_MAP_HTTPS_URL + $"?size=200*200&scale=2&key={gaode_key}&location={location}&markers={markers}&zoom={zoom}";
        }
    }

    private string GaoDeApi_GetIPQueryURL(string gaode_key, string ip)
    {
        if (string.IsNullOrEmpty(ip))
        {
            return GAODE_IP_HTTPS_URL + $"?key={gaode_key}";
        }
        else 
        {
            return GAODE_IP_HTTPS_URL + $"?key={gaode_key}&ip={ip}";
        }
    }
}
