using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class Rope : MonoBehaviour
{
    public enum SolverType { Verlet, RK4 }

    [Header("Geometry")]
    public int nodeCount = 25;
    public float segmentLength = 0.2f;
    public Vector3 endDirection = Vector3.right;

    [Header("Mesh")]
    [Min(0.001f)] public float radius = 0.05f;
    [Range(3, 32)] public int radialSegments = 10;
    public bool capEnds = true;

    [Header("Physics")]
    public Vector3 gravity = new(0f, -9.81f, 0f);
    [Min(0.0001f)] public float nodeMass = 0.1f;
    [Min(0f)] public float linearDamping = 0.05f;

    [Header("Solver")]
    public SolverType solverType = SolverType.Verlet;
    [Min(1)] public int substeps = 4;

    [Header("Constraints")]
    [Min(0f)] public float constraintStabilization = 60f;
    public bool useBendingConstraint = true;
    [Range(0f, 1f)] public float bendingStrength = 0.18f;

    [Header("Anchor")]
    public bool pinFirstNode = true;
    public Transform anchor;

    private RopeNode[] nodes;
    private IRopeSolver solver;
    private SolverType activeSolverType;

    private MeshFilter meshFilter;
    private Mesh ropeMesh;
    private Vector3[] vertices;
    private Vector3[] normals;
    private Vector2[] uvs;
    private int[] triangles;
    private Vector3[] frameTangent;
    private Vector3[] frameNormal;
    private Vector3[] frameBinormal;
    private int builtRadialSegments;
    private int builtNodeCount;
    private bool builtCaps;

    // Lagrange-multiplier tridiagonal solver buffers (one entry per constraint k = 0..n-2).
    private Vector3[] lagD;        // d_k = p_{k+1} - p_k
    private Vector3[] lagVd;       // relative velocity v_{k+1} - v_k
    private float[] lagSub;        // sub-diagonal of J M^-1 J^T
    private float[] lagDiag;       // diagonal
    private float[] lagSup;        // super-diagonal
    private float[] lagRhs;        // right-hand side
    private float[] lagPrime;      // Thomas-algorithm scratch (sup' after sweep)
    private float[] lagRhsPrime;   // Thomas-algorithm scratch (rhs' after sweep)
    private float[] lagLambda;     // solved tensions

    void Start()
    {
        // get initial state of the rope nodes, select the solver, and initialize the mesh topology and vertex buffers
        InitializeNodes();
        SelectSolver();
        InitializeMesh();
    }

    void FixedUpdate()
    {
        if (nodes == null || nodes.Length == 0) return;

        if (activeSolverType != solverType) SelectSolver();

        nodes[0].pinned = pinFirstNode;

        if (pinFirstNode)
        {
            Vector3 anchorPos = anchor != null ? anchor.position : nodes[0].position;
            nodes[0].position = anchorPos;
            nodes[0].previousPosition = anchorPos;
            nodes[0].velocity = Vector3.zero;
        }

        float subDt = Time.fixedDeltaTime / substeps;
        for (int s = 0; s < substeps; s++)
        {
            solver.Step(nodes, subDt, ComputeForces);
            if (useBendingConstraint) ApplyBendingConstraint();
        }
    }

    void LateUpdate()
    {
        UpdateMesh();
    }

    void InitializeNodes()
    {
        nodes = new RopeNode[nodeCount];
        Vector3 dir = endDirection.sqrMagnitude > 1e-6f ? endDirection.normalized : Vector3.right;
        Vector3 origin = transform.position;

        for (int i = 0; i < nodeCount; i++)
        {
            Vector3 pos = origin + dir * (i * segmentLength);
            bool pinned = pinFirstNode && i == 0;
            nodes[i] = new RopeNode(pos, nodeMass, pinned);
        }
    }

    void SelectSolver()
    {
        if (solverType == SolverType.RK4)
            solver = new RK4Solver();
        else
            solver = new VerletSolver();
        activeSolverType = solverType;
    }

    void InitializeMesh()
    {
        meshFilter = GetComponent<MeshFilter>();
        ropeMesh = new Mesh { name = "RopeMesh" };
        ropeMesh.MarkDynamic();
        meshFilter.mesh = ropeMesh;
        BuildMeshTopology();
        UpdateMesh();
    }

    void BuildMeshTopology()
    {
        // count vertices
        int rings = nodes.Length;
        int ringVerts = rings * radialSegments;
        int capVerts = capEnds ? 2 : 0;
        int totalVerts = ringVerts + capVerts;

        // count triangles and indices
        int sideTris = (rings - 1) * radialSegments * 2;
        int capTris = capEnds ? radialSegments * 2 : 0;
        int totalIndices = (sideTris + capTris) * 3;

        // Allocate arrays
        vertices = new Vector3[totalVerts];
        normals = new Vector3[totalVerts];
        uvs = new Vector2[totalVerts];
        triangles = new int[totalIndices];

        // each ring needs a coordinate frame
        // tangent points along the rope, normal and binormal define the cross section orientation
        frameTangent = new Vector3[rings];
        frameNormal = new Vector3[rings];
        frameBinormal = new Vector3[rings];

        // UVs are laid out in a cylindrical pattern along the rope, with optional caps at the ends.
        for (int i = 0; i < rings; i++)
        {
            // v goes from 0 to 1 along the length of the rope 
            float v = rings > 1 ? (float)i / (rings - 1) : 0f;
            for (int j = 0; j < radialSegments; j++)
            {
                // u goes from 0 to 1 around the circumference of the rope
                int idx = i * radialSegments + j;
                uvs[idx] = new Vector2((float)j / radialSegments, v);
            }
        }
        // simple uv layout for the caps, centered at 0.5 in u and spanning 0 to 1 in v
        if (capEnds)
        {
            uvs[ringVerts] = new Vector2(0.5f, 0f);
            uvs[ringVerts + 1] = new Vector2(0.5f, 1f);
        }

        // most importantly, define the triangle indices for the mesh topology
        int t = 0;
        for (int i = 0; i < rings - 1; i++)
        {
            for (int j = 0; j < radialSegments; j++)
            {
                int j1 = (j + 1) % radialSegments;
                int a = i * radialSegments + j;
                int b = i * radialSegments + j1;
                int c = (i + 1) * radialSegments + j;
                int d = (i + 1) * radialSegments + j1;
                triangles[t++] = a; triangles[t++] = b; triangles[t++] = c;
                triangles[t++] = b; triangles[t++] = d; triangles[t++] = c;
            }
        }

        // build the end caps as triangle fans if enabled. 
        if (capEnds)
        {
            int startCenter = ringVerts;
            int endCenter = ringVerts + 1;
            for (int j = 0; j < radialSegments; j++)
            {
                int j1 = (j + 1) % radialSegments;
                triangles[t++] = startCenter;
                triangles[t++] = j1;
                triangles[t++] = j;
            }
            int last = (rings - 1) * radialSegments;
            for (int j = 0; j < radialSegments; j++)
            {
                int j1 = (j + 1) % radialSegments;
                triangles[t++] = endCenter;
                triangles[t++] = last + j;
                triangles[t++] = last + j1;
            }
        }

        // assign the arrays to the mesh. We can update vertices and normals later without rebuilding the topology.
        ropeMesh.Clear();
        ropeMesh.vertices = vertices;
        ropeMesh.triangles = triangles;
        ropeMesh.uv = uvs;
        ropeMesh.normals = normals;

        // checks if topology needs to be rebuilt in case parameters changed in the editor
        builtRadialSegments = radialSegments;
        builtNodeCount = rings;
        builtCaps = capEnds;
    }

    // Get frame of each node
    void ComputeFrames()
    {
        int n = nodes.Length;
        // get the tangent by looking at the direction to the next node
        for (int i = 0; i < n; i++)
        {
            Vector3 t;
            if (n == 1) t = Vector3.right;
            else if (i == 0) t = nodes[1].position - nodes[0].position;
            else if (i == n - 1) t = nodes[n - 1].position - nodes[n - 2].position;
            else t = nodes[i + 1].position - nodes[i - 1].position; // use central difference for interior nodes for smoother tangents
            if (t.sqrMagnitude < 1e-12f) t = Vector3.right;
            frameTangent[i] = t.normalized;
        }

        // get the normal and binormal by constructing an arbitrary normal for the first node
        Vector3 helper = Mathf.Abs(Vector3.Dot(frameTangent[0], Vector3.up)) < 0.95f
            ? Vector3.up
            : Vector3.right;
        frameNormal[0] = Vector3.Cross(frameTangent[0], helper).normalized;
        frameBinormal[0] = Vector3.Cross(frameTangent[0], frameNormal[0]).normalized;

        // transport the normal and binormal along the rope using parallel transport to minimize twisting of the frame.
        for (int i = 1; i < n; i++)
        {
            // rotation that aligns the previous tangent to the current tangent
            Quaternion rot = Quaternion.FromToRotation(frameTangent[i - 1], frameTangent[i]);
            // transport the previous normal and binormal using this rotation
            Vector3 carriedNormal = rot * frameNormal[i - 1];
            // recompute to ensure no drift in the frame over time
            frameBinormal[i] = Vector3.Cross(frameTangent[i], carriedNormal).normalized;
            frameNormal[i] = Vector3.Cross(frameBinormal[i], frameTangent[i]).normalized;
        }
    }

    void UpdateMesh()
    {
        if (ropeMesh == null) return;

        // if we need to rebuild the topology due to parameter changes
        if (builtRadialSegments != radialSegments || builtNodeCount != nodes.Length || builtCaps != capEnds)
        {
            BuildMeshTopology();
        }

        // get the coordinate frames
        ComputeFrames();


        Transform tr = transform;
        int rings = nodes.Length;
        for (int i = 0; i < rings; i++)
        {
            Vector3 center = nodes[i].position;
            Vector3 nrm = frameNormal[i];
            Vector3 bin = frameBinormal[i];


            for (int j = 0; j < radialSegments; j++)
            {
                // loop around the ring angle
                float angle = (float)j / radialSegments * Mathf.PI * 2f;
                float cs = Mathf.Cos(angle);
                float sn = Mathf.Sin(angle);
                // get the vertex position by starting at the center and moving out along the normal and binormal according to the angle
                Vector3 outward = cs * nrm + sn * bin;
                // convert into local space of the rope object for the mesh vertices
                int idx = i * radialSegments + j;
                vertices[idx] = tr.InverseTransformPoint(center + outward * radius);
                normals[idx] = tr.InverseTransformDirection(outward);
            }
        }

        // cap vertices are just one radius along the tangent direction from the center of the end nodes
        if (capEnds)
        {
            int ringVerts = rings * radialSegments;
            vertices[ringVerts] = tr.InverseTransformPoint(nodes[0].position);
            normals[ringVerts] = tr.InverseTransformDirection(-frameTangent[0]);
            vertices[ringVerts + 1] = tr.InverseTransformPoint(nodes[rings - 1].position);
            normals[ringVerts + 1] = tr.InverseTransformDirection(frameTangent[rings - 1]);
        }


        // sends the updated vertex positions and normals to the mesh. The triangles and uvs don't change so we don't need to reassign those.
        ropeMesh.vertices = vertices;
        ropeMesh.normals = normals;

        // update bounds for culling and lighting
        ropeMesh.RecalculateBounds();
    }

    // Computes total force on each node
    void ComputeForces(RopeNode[] state)
    {
        int n = state.Length;

        // External forces: gravity + linear drag + active wind boxes
        int windBoxes = WindBox.Active.Count;
        for (int i = 0; i < n; i++)
        {
            state[i].ClearForces();
            state[i].ApplyForce(gravity * state[i].mass);
            if (linearDamping > 0f)
                state[i].ApplyForce(-linearDamping * state[i].velocity);

            for (int b = 0; b < windBoxes; b++)
                state[i].ApplyForce(WindBox.Active[b].GetWindForce(state[i].position));
        }

        // n - 1 constraints for n nodes
        int m = n - 1;
        if (m <= 0) return;

        // make sure our buffers are allocated
        EnsureLagrangeBuffers(m);

        // Get the distance and relative velocity for each constraint. We use for for all constraint equaitions
        for (int k = 0; k < m; k++)
        {
            lagD[k] = state[k + 1].position - state[k].position;
            lagVd[k] = state[k + 1].velocity - state[k].velocity;
        }

        // Prepare the baumgarte stabilization terms
        float omega = constraintStabilization; // how aggressively we correct errors
        float velGain = 2f * omega;
        float posGain = omega * omega;
        float L2 = segmentLength * segmentLength;

        // for each row build one row of A lambda = b. A is tridiagonal with (lagSub, lagDiag, lagSup), and b is lagRhs.
        for (int k = 0; k < m; k++)
        {
            // get the inverse masses, treat pinned nodes as infinite mass 
            float wk = state[k].pinned ? 0f : 1f / state[k].mass;
            float wk1 = state[k + 1].pinned ? 0f : 1f / state[k + 1].mass;

            Vector3 d = lagD[k];
            Vector3 vd = lagVd[k];
            float dd = d.sqrMagnitude;

            // Diagonal: |d_k|^2 (w_k + w_{k+1}); tiny term keeps the system non-singular
            // when both endpoints are pinned or a segment collapses to zero length.
            lagDiag[k] = (wk + wk1) * dd + 1e-8f;
            lagSub[k] = k > 0 ? -wk * Vector3.Dot(d, lagD[k - 1]) : 0f;
            lagSup[k] = k < m - 1 ? -wk1 * Vector3.Dot(d, lagD[k + 1]) : 0f;

            // constraint violation and its derivative
            float cVal = 0.5f * (dd - L2);
            float cDot = Vector3.Dot(d, vd);

            // external force contribution, how the external forces change constraint acceleration
            Vector3 fextDiff = wk1 * state[k + 1].force - wk * state[k].force;

            // calculate the right hand side with baumgarte stabilization
            lagRhs[k] = -vd.sqrMagnitude
                      - Vector3.Dot(d, fextDiff)
                      - velGain * cDot
                      - posGain * cVal;
        }

        // solve the tridiagonal system for the Lagrange multipliers
        SolveTridiagonal(m);

        // For each node i we have the force from the previous segment (if i > 0) and the next segment (if i < n - 1)
        // Recall the force is just the Lagrange multiplier times the gradient of the constraint, which is +/-d for the distance constraint
        for (int i = 0; i < n; i++)
        {
            if (i > 0) state[i].ApplyForce(lagD[i - 1] * lagLambda[i - 1]);
            if (i < m) state[i].ApplyForce(-lagD[i] * lagLambda[i]);
        }
    }

    // Thomas algorithm
    void SolveTridiagonal(int m)
    {
        // normalize the first row
        lagPrime[0] = lagSup[0] / lagDiag[0];
        lagRhsPrime[0] = lagRhs[0] / lagDiag[0];

        // forward sweep
        for (int k = 1; k < m; k++)
        {
            // eliminate the sub-diagonal entry by subtracting a scaled version of the previous row
            float denom = lagDiag[k] - lagSub[k] * lagPrime[k - 1];
            // safeguard against near-zero denominators
            if (Mathf.Abs(denom) < 1e-12f) denom = denom < 0f ? -1e-12f : 1e-12f;
            // store the modified coefficients for the current row
            lagPrime[k] = lagSup[k] / denom;
            lagRhsPrime[k] = (lagRhs[k] - lagSub[k] * lagRhsPrime[k - 1]) / denom;
        }

        // back substitution
        lagLambda[m - 1] = lagRhsPrime[m - 1];
        for (int k = m - 2; k >= 0; k--)
        {
            // solve for lambda_k using the modified coefficients and the value of lambda_{k+1} we just computed
            lagLambda[k] = lagRhsPrime[k] - lagPrime[k] * lagLambda[k + 1];
        }
    }

    // Make sure the Lagrange solver buffers are allocated and large enough for m constraints.
    void EnsureLagrangeBuffers(int m)
    {
        if (lagD != null && lagD.Length >= m) return;
        lagD = new Vector3[m];
        lagVd = new Vector3[m];
        lagSub = new float[m];
        lagDiag = new float[m];
        lagSup = new float[m];
        lagRhs = new float[m];
        lagPrime = new float[m];
        lagRhsPrime = new float[m];
        lagLambda = new float[m];
    }

    void ApplyBendingConstraint()
    {
        for (int i = 1; i < nodes.Length - 1; i++)
        {
            Vector3 prev = nodes[i - 1].position;
            Vector3 curr = nodes[i].position;
            Vector3 next = nodes[i + 1].position;

            Vector3 midpoint = 0.5f * (prev + next);
            Vector3 offset = midpoint - curr;

            bool prevPinned = nodes[i - 1].pinned;
            bool currPinned = nodes[i].pinned;
            bool nextPinned = nodes[i + 1].pinned;

            if (currPinned) continue;

            if (!prevPinned && !nextPinned)
            {
                // push current node towards the midpoint, and pull the neighbors away 
                Vector3 c = offset * (bendingStrength * 0.5f);
                nodes[i].position += offset * bendingStrength;
                nodes[i - 1].position -= c;
                nodes[i + 1].position -= c;
            }
            else
            {
                nodes[i].position += offset * bendingStrength;
            }
        }
    }

    void OnDrawGizmosSelected()
    {
        if (nodes == null) return;
        Gizmos.color = Color.yellow;
        for (int i = 0; i < nodes.Length; i++)
        {
            Gizmos.DrawSphere(nodes[i].position, radius * 0.6f);
        }
    }
}
