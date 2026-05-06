using UnityEngine;
using UnityEngine.InputSystem;

public class ThirdPersonCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;
    [Tooltip("Vertical offset (above target.position) for the orbit pivot — usually around chest/head height.")]
    public float pivotHeight = 1.4f;

    [Header("Orbit")]
    public float distance = 5f;
    [Tooltip("Mouse delta multiplier. Pixels-per-frame already, so values around 0.1–0.3 feel natural.")]
    public float mouseSensitivity = 0.15f;
    public float minPitch = -25f;
    public float maxPitch = 70f;
    public bool invertY = false;

    [Header("Smoothing")]
    [Tooltip("Approximate time (s) for the camera to catch up to the target. Smaller = snappier, larger = more cinematic drag. ~0.1 feels good for 3rd-person follow.")]
    [Range(0f, 0.5f)] public float positionSmoothTime = 0.12f;

    [Header("Cursor")]
    public bool lockCursorOnStart = true;

    private float yaw;
    private float pitch;
    private Vector3 followVelocity;

    void Start()
    {
        if (lockCursorOnStart)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }

        Vector3 angles = transform.eulerAngles;
        yaw = angles.y;
        pitch = angles.x;

        if (target == null)
        {
            PlayerController p = FindFirstObjectByType<PlayerController>();
            if (p != null) target = p.transform;
        }
    }

    void Update()
    {
        Keyboard kb = Keyboard.current;
        Mouse mouse = Mouse.current;
        if (kb != null && kb.escapeKey.wasPressedThisFrame)
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
        else if (mouse != null && mouse.leftButton.wasPressedThisFrame && Cursor.lockState != CursorLockMode.Locked)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
    }

    void LateUpdate()
    {
        if (target == null) return;

        Mouse mouse = Mouse.current;
        if (mouse != null && Cursor.lockState == CursorLockMode.Locked)
        {
            Vector2 delta = mouse.delta.ReadValue();
            yaw += delta.x * mouseSensitivity;
            pitch += (invertY ? delta.y : -delta.y) * mouseSensitivity;
            pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
        }

        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
        Vector3 pivot = target.position + Vector3.up * pivotHeight;
        Vector3 desired = pivot + rotation * Vector3.back * distance;

        transform.position = positionSmoothTime > 0f
            ? Vector3.SmoothDamp(transform.position, desired, ref followVelocity, positionSmoothTime)
            : desired;
        transform.rotation = Quaternion.LookRotation(pivot - transform.position, Vector3.up);
    }
}
