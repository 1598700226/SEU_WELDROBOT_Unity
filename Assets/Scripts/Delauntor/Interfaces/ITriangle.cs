using System.Collections.Generic;

namespace Algorithm.Delauntor.Interfaces
{
    public interface ITriangle
    {
        IEnumerable<IPoint> Points { get; }
        int Index { get; }
    }
}
