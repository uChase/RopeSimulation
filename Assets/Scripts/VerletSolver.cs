using System;
using UnityEngine;

public class VerletSolver : IRopeSolver
{
    public void Step(RopeNode[] nodes, float dt, Action<RopeNode[]> computeForces)
    {
        computeForces(nodes);

        float invDt = dt > 0f ? 1f / dt : 0f;

        for (int i = 0; i < nodes.Length; i++)
        {
            if (nodes[i].pinned)
            {
                nodes[i].previousPosition = nodes[i].position;
                nodes[i].velocity = Vector3.zero;
                continue;
            }

            Vector3 acceleration = nodes[i].GetAcceleration();
            Vector3 currentPosition = nodes[i].position;
            Vector3 newPosition = 2f * currentPosition - nodes[i].previousPosition + acceleration * (dt * dt);

            nodes[i].previousPosition = currentPosition;
            nodes[i].position = newPosition;
            nodes[i].velocity = (newPosition - currentPosition) * invDt;
        }
    }
}
