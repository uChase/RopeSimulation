using UnityEngine;

[System.Serializable]
public struct RopeNode
{
    public Vector3 position;
    public Vector3 previousPosition;
    public Vector3 velocity;
    public Vector3 force;

    public float mass;
    public bool pinned;

    public RopeNode(Vector3 startPosition, float nodeMass = 1.0f, bool isPinned = false)
    {
        position = startPosition;
        previousPosition = startPosition;
        velocity = Vector3.zero;
        force = Vector3.zero;
        mass = nodeMass;
        pinned = isPinned;
    }

    public void ClearForces()
    {
        force = Vector3.zero;
    }

    public void ApplyForce(Vector3 newForce)
    {
        force += newForce;
    }

    public readonly Vector3 GetAcceleration()
    {
        if (mass <= 0f) return Vector3.zero;
        return force / mass;
    }
}