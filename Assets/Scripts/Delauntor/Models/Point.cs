using Algorithm.Delauntor.Interfaces;

namespace Algorithm.Delauntor.Models
{
    public struct Point : IPoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public int Index { get; set; }

        public Point(double x, double y, int index)
        {
            X = x;
            Y = y;
            Index = index;
        }
        public override string ToString() => $"{X},{Y}";
    }

}
