using System.Collections.Generic;
using UnityEngine;

public class WindBox : MonoBehaviour
{
    // Every enabled WindBox is in this list; ropes iterate it inside ComputeForces.
    public static readonly List<WindBox> Active = new();

    [Header("Volume")]
    [Tooltip("Box size in this transform's local space. The box is centered on the GameObject's position.")]
    public Vector3 size = new(2f, 2f, 3f);

    [Header("Wind")]
    [Tooltip("Force magnitude applied to each rope node inside the box (Newtons).")]
    public float strength = 20f;

    [Tooltip("Direction of the wind in this transform's local space. +Z = the GameObject's forward.")]
    public Vector3 localDirection = Vector3.forward;

    [Tooltip("0 = constant force across the whole volume, 1 = force ramps from zero at back to full strength at front.")]
    [Range(0f, 1f)] public float edgeFalloff = 0.3f;

    void OnEnable()
    {
        if (!Active.Contains(this)) Active.Add(this);
    }

    void OnDisable()
    {
        Active.Remove(this);
    }

    // Returns the wind force this box would apply at `worldPoint`, or zero if outside.
    public Vector3 GetWindForce(Vector3 worldPoint)
    {
        Vector3 local = transform.InverseTransformPoint(worldPoint);
        Vector3 half = size * 0.5f;

        if (Mathf.Abs(local.x) > half.x ||
            Mathf.Abs(local.y) > half.y ||
            Mathf.Abs(local.z) > half.z)
        {
            return Vector3.zero;
        }

        Vector3 dirLocal = localDirection.sqrMagnitude > 1e-6f
            ? localDirection.normalized
            : Vector3.forward;

        // Project the point onto the wind direction.
        // Back of the box is negative along dirLocal.
        // Front of the box is positive along dirLocal.
        float alongWind = Vector3.Dot(local, dirLocal);

        // Distance from center to edge in the wind direction.
        // This handles directions other than just local +Z.
        float halfLengthAlongWind =
            Mathf.Abs(dirLocal.x) * half.x +
            Mathf.Abs(dirLocal.y) * half.y +
            Mathf.Abs(dirLocal.z) * half.z;

        // Convert from [-halfLength, +halfLength] to [0, 1].
        // 0 = back, 1 = front.
        float t = halfLengthAlongWind > 1e-4f
            ? Mathf.InverseLerp(-halfLengthAlongWind, halfLengthAlongWind, alongWind)
            : 1f;

        float falloff = Mathf.Lerp(1f, 1f - edgeFalloff, t);

        Vector3 dirWorld = transform.TransformDirection(dirLocal);
        return dirWorld * (strength * falloff);
    }

    void OnDrawGizmos()
    {
        bool on = isActiveAndEnabled;

        Color faint = on
            ? new Color(0.25f, 0.85f, 1f, 0.18f)
            : new Color(0.5f, 0.5f, 0.5f, 0.08f);

        Color edge = on
            ? new Color(0.25f, 0.85f, 1f, 0.9f)
            : new Color(0.5f, 0.5f, 0.5f, 0.4f);

        Matrix4x4 prev = Gizmos.matrix;
        Gizmos.matrix = transform.localToWorldMatrix;

        Gizmos.color = faint;
        Gizmos.DrawCube(Vector3.zero, size);

        Gizmos.color = edge;
        Gizmos.DrawWireCube(Vector3.zero, size);

        if (on)
        {
            Vector3 d = localDirection.sqrMagnitude > 1e-6f
                ? localDirection.normalized
                : Vector3.forward;

            float reach = 0.5f * Mathf.Min(size.x, Mathf.Min(size.y, size.z));

            // Draw wind direction arrow line.
            Gizmos.DrawLine(Vector3.zero, d * reach);

            // Draw small marker at back and front.
            Vector3 half = size * 0.5f;
            float halfLengthAlongWind =
                Mathf.Abs(d.x) * half.x +
                Mathf.Abs(d.y) * half.y +
                Mathf.Abs(d.z) * half.z;

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(-d * halfLengthAlongWind, reach * 0.08f);

            Gizmos.color = Color.red;
            Gizmos.DrawSphere(d * halfLengthAlongWind, reach * 0.08f);
        }

        Gizmos.matrix = prev;
    }
}