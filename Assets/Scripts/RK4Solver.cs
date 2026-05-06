using System;
using UnityEngine;

public class RK4Solver : IRopeSolver
{
    private Vector3[] x0;
    private Vector3[] v0;
    private Vector3[] k1x;
    private Vector3[] k1v;
    private Vector3[] k2x;
    private Vector3[] k2v;
    private Vector3[] k3x;
    private Vector3[] k3v;
    private Vector3[] k4x;
    private Vector3[] k4v;

    public void Step(RopeNode[] nodes, float dt, Action<RopeNode[]> computeForces)
    {
        int n = nodes.Length;

        EnsureCapacity(n);

        // Store initial positions and velocities.
        for (int i = 0; i < n; i++)
        {
            x0[i] = nodes[i].position;
            v0[i] = nodes[i].velocity;
        }

        // RK4 evaluates four derivative estimates
        // k1: derivative at the beginning of the interval
        // k2: derivative at the midpoint using k1
        // k3: derivative at the midpoint using k2
        // k4: derivative at the end of the interval using k3
        EvaluateStage(nodes, computeForces, x0, v0, x0, v0, 0f, k1x, k1v);
        EvaluateStage(nodes, computeForces, x0, v0, k1x, k1v, 0.5f * dt, k2x, k2v);
        EvaluateStage(nodes, computeForces, x0, v0, k2x, k2v, 0.5f * dt, k3x, k3v);
        EvaluateStage(nodes, computeForces, x0, v0, k3x, k3v, dt, k4x, k4v);

        float sixth = dt / 6f;

        // Combine the slopes with RK4 weights to compute the final position and velocity.
        for (int i = 0; i < n; i++)
        {
            if (nodes[i].pinned)
            {
                nodes[i].position = x0[i];
                nodes[i].previousPosition = x0[i];
                nodes[i].velocity = Vector3.zero;
                continue;
            }

            Vector3 dx = sixth * (k1x[i] + 2f * k2x[i] + 2f * k3x[i] + k4x[i]);
            Vector3 dv = sixth * (k1v[i] + 2f * k2v[i] + 2f * k3v[i] + k4v[i]);

            nodes[i].previousPosition = x0[i];
            nodes[i].position = x0[i] + dx;
            nodes[i].velocity = v0[i] + dv;
        }
    }

    private static void EvaluateStage(
        RopeNode[] nodes,
        Action<RopeNode[]> computeForces,
        Vector3[] x0,
        Vector3[] v0,
        Vector3[] kxPrev,
        Vector3[] kvPrev,
        float h,
        Vector3[] kxOut,
        Vector3[] kvOut)
    {
        int n = nodes.Length;


        //  Move the nodes to the intermediate state for this stage of RK4.
        // for k1, h = 0, so this is the initial state. 
        // for k2 and k3, h = 0.5*dt, so this is the midpoint state using the previous slope.
        // for k4, h = dt, so this is the end state using the previous slope.
        for (int i = 0; i < n; i++)
        {
            if (nodes[i].pinned)
            {
                nodes[i].position = x0[i];
                nodes[i].velocity = Vector3.zero;
                continue;
            }
            nodes[i].position = x0[i] + h * kxPrev[i];
            nodes[i].velocity = v0[i] + h * kvPrev[i];
        }

        computeForces(nodes);

        // get the derivatives at this stage: kxOut = velocity, kvOut = acceleration
        for (int i = 0; i < n; i++)
        {
            if (nodes[i].pinned)
            {
                kxOut[i] = Vector3.zero;
                kvOut[i] = Vector3.zero;
                continue;
            }
            kxOut[i] = nodes[i].velocity;
            kvOut[i] = nodes[i].GetAcceleration();
        }
    }

    private void EnsureCapacity(int n)

    {
        // If the arrays already match the rope length, reuse them.
        if (x0 != null && x0.Length == n)
            return;
        x0 = new Vector3[n];
        v0 = new Vector3[n];
        k1x = new Vector3[n];
        k1v = new Vector3[n];
        k2x = new Vector3[n];
        k2v = new Vector3[n];
        k3x = new Vector3[n];
        k3v = new Vector3[n];
        k4x = new Vector3[n];
        k4v = new Vector3[n];
    }
}
