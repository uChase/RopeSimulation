using System;

public interface IRopeSolver
{
    void Step(RopeNode[] nodes, float dt, Action<RopeNode[]> computeForces);
}
