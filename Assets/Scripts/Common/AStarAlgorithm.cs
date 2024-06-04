using JetBrains.Annotations;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

public class AStarAlgorithm : MonoBehaviour
{
    private static int NODE_SIZE = 32;
    private static int OBSTACLE_GRAY = 0;

    /****************自定义优先队列*******************/
    private class PriorityQueueNode<T>
    {
        public T Data { get; private set; }
        public float Priority { get; private set; }

        public PriorityQueueNode(T data, float priority)
        {
            this.Data = data;
            this.Priority = priority;
        }
    }
    private class PriorityQueue<T>
    {
        private List<PriorityQueueNode<T>> queue = new List<PriorityQueueNode<T>>();

        public void Enqueue(T item, float priority)
        {
            queue.Add(new PriorityQueueNode<T>(item, priority));
            queue.Sort((x, y) => x.Priority.CompareTo(y.Priority));
        }

        public T Dequeue()
        {
            if (queue.Count == 0)
            {
                Debug.LogError("Attempt to dequeue from empty queue.");
                return default(T);
            }
            T item = queue[0].Data;
            queue.RemoveAt(0);
            return item;
        }

        public T Peek()
        {
            return queue.Count > 0 ? queue[0].Data : default(T);
        }

        public int Count
        {
            get { return queue.Count; }
        }

        public List<PriorityQueueNode<T>> Datas
        { 
            get 
            { 
                return queue; 
            } 
        }
    }

    private class Node
    {
        // Change this depending on what the desired size is for each element in the grid
        public Node Parent;
        public Vector2 Position;
        public Vector2 Center
        {
            get
            {
                return new Vector2(Position.x * NODE_SIZE + NODE_SIZE / 2, Position.y * NODE_SIZE + NODE_SIZE / 2);
            }
        }
        public float DistanceToTarget;
        public float Cost;
        public float Weight;
        public float F
        {
            get
            {
                if (DistanceToTarget != -1 && Cost != -1)
                    return DistanceToTarget + Cost;
                else
                    return -1;
            }
        }
        public bool Walkable;

        public Node(Vector2 pos, bool walkable, float weight = 1)
        {
            Parent = null;
            Position = pos;
            DistanceToTarget = -1;
            Cost = 1;
            Weight = weight;
            Walkable = walkable;
        }
    }

    public class Astar
    {
        List<List<Node>> Grid;
        int GridRows
        {
            get
            {
                return Grid[0].Count;
            }
        }
        int GridCols
        {
            get
            {
                return Grid.Count;
            }
        }

        public Astar(int[,] map, int imgWidth, int imgHeight, int node_size = 32, int obstacle_gray = 0)
        {
            AStarAlgorithm.NODE_SIZE = node_size;
            AStarAlgorithm.OBSTACLE_GRAY = obstacle_gray;
            Grid = GetAStarMap(map, imgWidth, imgHeight);
        }

        /// <summary>
        /// 输入图像起点和终点坐标，返回A星路径
        /// </summary>
        /// <param name="Start"></param>
        /// <param name="End"></param>
        /// <returns></returns>
        public List<Vector2> getPath(Vector2 Start, Vector2 End) {
            List<Vector2> path = new List<Vector2>();
            Stack<Node> nodes = FindPath(Start, End);
            if (nodes == null || nodes.Count == 0)
            {
                return null;
            }

            while (nodes.Count > 0)
            {
                Node node = nodes.Pop();
                Vector2 vector2 = new Vector2(node.Center.x, node.Center.y);
                path.Add(vector2);
            }
            return path;
        }

        private Stack<Node> FindPath(Vector2 Start, Vector2 End)
        {
            Node start = new Node(new Vector2((int)(Start.x / NODE_SIZE), (int)(Start.y / NODE_SIZE)), true);
            Node end = new Node(new Vector2((int)(End.x / NODE_SIZE), (int)(End.y / NODE_SIZE)), true);

            Stack<Node> Path = new Stack<Node>();

            PriorityQueue<Node> OpenList = new PriorityQueue<Node>();

            List<Node> ClosedList = new List<Node>();
            List<Node> adjacencies;
            Node current = start;

            // add start node to Open List
            OpenList.Enqueue(start, start.F);

            while (OpenList.Count != 0 && !ClosedList.Exists(x => x.Position == end.Position))
            {
                current = OpenList.Dequeue();
                ClosedList.Add(current);
                adjacencies = GetAdjacentNodes(current);

                foreach (Node n in adjacencies)
                {
                    if (!ClosedList.Contains(n) && n.Walkable)
                    {
                        bool isFound = false;
                        foreach (var oLNode in OpenList.Datas)
                        {
                            if (oLNode.Data == n)
                            {
                                isFound = true;
                            }
                        }
                        if (!isFound)
                        {
                            n.Parent = current;
                            n.DistanceToTarget = Math.Abs(n.Position.x - end.Position.x) + Math.Abs(n.Position.y - end.Position.y);
                            n.Cost = n.Weight + n.Parent.Cost;
                            OpenList.Enqueue(n, n.F);
                        }
                    }
                }
            }

            // construct path, if end was not closed return null
            if (!ClosedList.Exists(x => x.Position == end.Position))
            {
                return null;
            }

            // if all good, return path
            Node temp = ClosedList[ClosedList.IndexOf(current)];
            if (temp == null) return null;
            do
            {
                Path.Push(temp);
                temp = temp.Parent;
            } while (temp != start && temp != null);
            return Path;
        }

        private List<Node> GetAdjacentNodes(Node n)
        {
            List<Node> temp = new List<Node>();

            int row = (int)n.Position.y;
            int col = (int)n.Position.x;

            if (row + 1 < GridRows)
            {
                temp.Add(Grid[col][row + 1]);
            }
            if (row - 1 >= 0)
            {
                temp.Add(Grid[col][row - 1]);
            }
            if (col - 1 >= 0)
            {
                temp.Add(Grid[col - 1][row]);
            }
            if (col + 1 < GridCols)
            {
                temp.Add(Grid[col + 1][row]);
            }

            return temp;
        }

        private List<List<Node>> GetAStarMap(int[,] map, int imgWidth, int imgHeight) 
        {
            List<List<Node>> temp = new List<List<Node>>();
            for (int i = 0; i < imgWidth / NODE_SIZE; i++)
            {
                temp.Add(new List<Node>());
                for (int j = 0; j < imgHeight / NODE_SIZE; j++)
                {
                    bool walkable = isWalkable(map, i, j);
                    temp[i].Add(new Node(new Vector2(i, j), walkable));
                }
            }
            return temp;
        }

        private bool isWalkable(int[,] map, int i, int j)
        {
            for (int x = 0; x < NODE_SIZE; x++)
            {
                for (int y = 0; y < NODE_SIZE; y++)
                {
                    if (map[j * NODE_SIZE + y, i * NODE_SIZE + x] >= OBSTACLE_GRAY)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
}
