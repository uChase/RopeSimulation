using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class PlayerController : MonoBehaviour
{
    [Header("References")]
    public Transform cameraTransform;

    [Header("Hop")]
    public float hopForwardSpeed = 3.5f;
    public float hopUpSpeed = 3.5f;
    public float hopCooldown = 0.08f;

    [Header("Facing")]
    public bool faceHopDirection = true;
    public Vector3 modelForward = Vector3.forward;
    public float rotationSpeed = 540f;

    [Header("Ground Check")]
    public float groundCheckOffset = 0.9f;
    public float groundCheckRadius = 0.25f;
    public LayerMask groundLayer = ~0;

    [Header("Blade")]
    [Tooltip("Transform that spins when activated.")]
    public Transform blade;

    [Tooltip("Wind volume that pushes rope nodes while the blade is spinning.")]
    public WindBox bladeWind;

    [Tooltip("Key that toggles blade spinning.")]
    public Key bladeToggleKey = Key.Space;

    [Tooltip("Blade spin speed in degrees per second.")]
    public float bladeSpinSpeed = 720f;

    [HideInInspector]
    public bool bladeSpinning = false;

    private Rigidbody rb;
    private float lastHopTime = -10f;
    private readonly Collider[] groundHits = new Collider[8];

    private Quaternion faceTarget;

    // Input cached from Update, applied in FixedUpdate
    private Vector3 desiredHopDir;
    private bool wantsHop;

    void Start()
    {
        rb = GetComponent<Rigidbody>();

        rb.constraints = RigidbodyConstraints.FreezeRotation;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        if (cameraTransform == null && Camera.main != null)
            cameraTransform = Camera.main.transform;

        faceTarget = rb.rotation;

        if (bladeWind != null) bladeWind.enabled = bladeSpinning;
    }

    void Update()
    {
        ReadInput();
        HandleBladeInput();
    }

    void FixedUpdate()
    {
        TryHop();

        if (faceHopDirection)
        {
            Quaternion nextRotation = Quaternion.RotateTowards(
                rb.rotation,
                faceTarget,
                rotationSpeed * Time.fixedDeltaTime
            );

            rb.MoveRotation(nextRotation);
        }
    }

    void ReadInput()
    {
        Keyboard kb = Keyboard.current;

        wantsHop = false;
        desiredHopDir = Vector3.zero;

        if (cameraTransform == null || kb == null)
            return;

        Vector3 forwardXZ = FlattenY(cameraTransform.forward);
        Vector3 rightXZ = FlattenY(cameraTransform.right);

        Vector3 dir = Vector3.zero;

        if (kb.wKey.isPressed) dir += forwardXZ;
        if (kb.sKey.isPressed) dir -= forwardXZ;
        if (kb.aKey.isPressed) dir -= rightXZ;
        if (kb.dKey.isPressed) dir += rightXZ;

        if (dir.sqrMagnitude < 1e-4f)
            return;

        desiredHopDir = dir.normalized;
        wantsHop = true;
    }

    void TryHop()
    {
        if (!wantsHop) return;
        if (Time.fixedTime - lastHopTime < hopCooldown) return;
        if (!IsGrounded()) return;

        Vector3 dir = desiredHopDir;

        Vector3 v = rb.linearVelocity;
        v.x = dir.x * hopForwardSpeed;
        v.y = hopUpSpeed;
        v.z = dir.z * hopForwardSpeed;
        rb.linearVelocity = v;

        if (faceHopDirection)
        {
            Vector3 mf = modelForward.sqrMagnitude > 1e-6f
                ? modelForward.normalized
                : Vector3.forward;

            faceTarget =
                Quaternion.LookRotation(dir, Vector3.up)
                * Quaternion.Inverse(Quaternion.LookRotation(mf, Vector3.up));
        }

        lastHopTime = Time.fixedTime;
    }

    bool IsGrounded()
    {
        Vector3 origin = rb.position + Vector3.down * groundCheckOffset;

        int n = Physics.OverlapSphereNonAlloc(
            origin,
            groundCheckRadius,
            groundHits,
            groundLayer,
            QueryTriggerInteraction.Ignore
        );

        for (int i = 0; i < n; i++)
        {
            Collider c = groundHits[i];

            if (c.attachedRigidbody == rb) continue;
            if (c.transform.IsChildOf(transform)) continue;

            return true;
        }

        return false;
    }

    static Vector3 FlattenY(Vector3 v)
    {
        v.y = 0f;
        return v.sqrMagnitude > 1e-6f ? v.normalized : Vector3.forward;
    }

    void HandleBladeInput()
    {
        Keyboard kb = Keyboard.current;
        if (kb == null) return;

        if (kb[bladeToggleKey].wasPressedThisFrame)
        {
            bladeSpinning = !bladeSpinning;
            if (bladeWind != null) bladeWind.enabled = bladeSpinning;
        }
    }

    void LateUpdate()
    {
        SpinBlade();
    }

    void SpinBlade()
    {
        if (!bladeSpinning) return;
        if (blade == null) return;

        blade.Rotate(Vector3.right, bladeSpinSpeed * Time.deltaTime, Space.Self);
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(
            transform.position + Vector3.down * groundCheckOffset,
            groundCheckRadius
        );
    }
}