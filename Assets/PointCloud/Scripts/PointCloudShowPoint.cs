using UnityEngine;
using System.Collections;
using System.IO;
using System.Threading;
using System;
using System.Collections.Generic;
using Unity.VisualScripting.Antlr3.Runtime.Tree;
using Unity.VisualScripting;
using UnityEngine.UIElements;
using System.Linq;

public class PointCloudShowPoint : MonoBehaviour {

    // File
    public string dataPath;
	private string filename;
	public Material matVertex;

	// PointCloud
	private GameObject pointCloud;

	public float scale = 1;
	public bool invertYZ = false;
	public bool realRefreshPotinCloud = false;
	public bool pointCloudShowOnce = true;
	public int pointCloudDelayShow = 5;
	public int numPoints;
	public int numPointGroups;
	public int numTriangles;

    private int limitPoints = 65000;
    private Vector3[] points;
	private Color[] colors;
	private Vector3[] triangles;

    void Start () {
		// Create Resources folder
		//createFolders ();

		// Get Filename
		filename = Path.GetFileName(dataPath);

        // 启动协程，每隔5秒执行一次ReadFile函数
        StartCoroutine(loadScene());
	}

    void Update()
    {
		// 单次更新
		if (!realRefreshPotinCloud && pointCloudShowOnce) {
            pointCloudShowOnce = false;
            StartCoroutine(loadScene());
        }

		// 持续更新
		if (realRefreshPotinCloud) {
            pointCloudShowOnce = false;
            StartCoroutine(loadScene());
        }
    }


    IEnumerator loadScene(){

		Debug.Log("更新环境 FilePath：" + filename);
		using (Mutex fileLock = new Mutex(false, "FileLockName"))
		{

			try
			{
				// 尝试获取文件锁定
				if (fileLock.WaitOne(TimeSpan.FromSeconds(1))) // 等待1秒
				{
					loadPointCloud();
				}
				else
				{
					Debug.Log("无法获取文件锁定，写入文件被取消。");
				}
			}
			finally
			{
				// 释放文件锁定
				fileLock.ReleaseMutex();
			}
		}

        yield return new WaitForSecondsRealtime(pointCloudDelayShow);
    }


    IEnumerator loadPointCloud() {
		// Check what file exists
		if (File.Exists(dataPath))
		{
            File.Copy(dataPath, dataPath + "_unity", true);
			loadOFF(dataPath + "_unity");
        }
   		else
			Debug.Log("File '" + dataPath + "' could not be found");

		yield return null;
	}


    // Start Coroutine of reading the points from the OFF file and creating the meshes
    IEnumerator loadOFF(string dPath){

        // Read file
        using (StreamReader sr = new StreamReader(dPath))
		{
			sr.ReadLine(); // OFF
			string[] buffer = sr.ReadLine().Split(); // nPoints, nFaces

			numPoints = int.Parse(buffer[0]);
            numTriangles = int.Parse(buffer[1]);

            points = new Vector3[numPoints];
			colors = new Color[numPoints];
			triangles = new Vector3[numTriangles];

			for (int i = 0; i < numPoints; i++)
			{
				buffer = sr.ReadLine().Split();
				if (!invertYZ)
					points[i] = new Vector3(float.Parse(buffer[0]) * scale, float.Parse(buffer[1]) * scale, float.Parse(buffer[2]) * scale);
				else
					points[i] = new Vector3(float.Parse(buffer[0]) * scale, float.Parse(buffer[2]) * scale, float.Parse(buffer[1]) * scale);

				if (buffer.Length >= 5)
					colors[i] = new Color(int.Parse(buffer[3]) / 255.0f, int.Parse(buffer[4]) / 255.0f, int.Parse(buffer[5]) / 255.0f);
				else
					colors[i] = Color.cyan;
			}

            yield return null;
            for (int i = 0; i < numTriangles; i++)
			{
                buffer = sr.ReadLine().Split();
				triangles[i] = new Vector3(int.Parse(buffer[1]), int.Parse(buffer[2]), int.Parse(buffer[3]));
            }
            yield return null;

            // Instantiate Point Groups
            numPointGroups = Mathf.CeilToInt(numPoints * 1.0f / limitPoints * 1.0f);
			if (pointCloud != null)
			{
				Transform parentTransform = pointCloud.transform;
				foreach (Transform childTransform in parentTransform)
				{
                    Destroy(childTransform.gameObject);
				}
			}
			else
			{
				pointCloud = new GameObject(filename);
			}

			for (int i = 0; i < numPointGroups - 1; i++)
			{
				InstantiateMesh(i, limitPoints);
                yield return null;
            }
			InstantiateMesh(numPointGroups - 1, numPoints - (numPointGroups - 1) * limitPoints);
            yield return null;
        }
	}

	
	void InstantiateMesh(int meshInd, int nPoints){
		// Create Mesh
		GameObject pointGroup = new GameObject (filename + meshInd);
		pointGroup.AddComponent<MeshFilter> ();
		pointGroup.AddComponent<MeshRenderer> ();

        pointGroup.GetComponent<Renderer>().material = matVertex;
		Mesh mesh = CreateMesh(meshInd, nPoints, limitPoints);
		pointGroup.GetComponent<MeshFilter>().mesh = mesh;
        pointGroup.transform.parent = pointCloud.transform;
	}

	Mesh CreateMesh(int id, int nPoints, int limitPoints){
		
		Mesh mesh = new Mesh ();

        Vector3[] myPoints = new Vector3[nPoints]; 
		int[] indecies = new int[nPoints];
		Color[] myColors = new Color[nPoints];
        for (int i=0;i<nPoints;++i) {
			myPoints[i] = points[id*limitPoints + i];
			indecies[i] = i;
			myColors[i] = colors[id*limitPoints + i];
		}

		mesh.vertices = myPoints;
		mesh.colors = myColors;

        mesh.SetIndices(indecies, MeshTopology.Points, 0);
		mesh.uv = new Vector2[nPoints];
		mesh.normals = new Vector3[nPoints];

		return mesh;
	}
}
