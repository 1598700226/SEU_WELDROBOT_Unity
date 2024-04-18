using System.Collections.Generic;

namespace Algorithm.Delauntor.Interfaces
{
    public interface IVoronoiCell
    {
        IPoint[] Points { get; }
        int Index { get; }
    }
}
