using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;

public class RopeBenchmark : MonoBehaviour
{
    public enum Mode { Ramp, Fixed }

    [Header("Solver")]
    public Rope.SolverType solverType = Rope.SolverType.Verlet;

    [Header("Mode")]
    public Mode mode = Mode.Ramp;

    [Tooltip("Number of ropes spawned in Fixed mode.")]
    [Min(1)] public int fixedCount = 50;

    [Tooltip("Ropes added per ramp stage.")]
    [Min(1)] public int ropesPerStage = 5;

    [Tooltip("Hard upper bound on total ropes regardless of performance.")]
    [Min(1)] public int maxRopes = 500;

    [Header("Timing")]
    [Tooltip("Seconds to wait after Play starts before spawning/measuring. Helps avoid editor/game startup spikes.")]
    [Min(0f)] public float initialWarmupSeconds = 2f;

    [Tooltip("Seconds to let a newly spawned stage settle before measuring.")]
    [Min(0f)] public float stageSettleSeconds = 1f;

    [Tooltip("Seconds of actual measurement after settle time.")]
    [Min(0.25f)] public float measureSeconds = 3f;

    [Tooltip("Ignore frames above this frame time while collecting samples. Useful for editor hiccups. Set high to disable.")]
    [Min(1f)] public float maxAcceptedFrameMs = 250f;

    [Header("Stop Conditions")]
    [Tooltip("Ramp stops when average FPS is below this for several consecutive measured stages.")]
    public float stopFpsThreshold = 30f;

    [Tooltip("How many failing measured stages in a row are required before stopping.")]
    [Min(1)] public int consecutiveFailStagesToStop = 2;

    [Tooltip("Also stop if p95 frame time exceeds this many ms for several consecutive stages. 33.3ms is about 30 FPS.")]
    public float stopP95FrameMs = 33.3f;

    [Header("Rope Settings")]
    [Min(2)] public int nodeCount = 25;
    public float segmentLength = 0.2f;
    [Min(0.0001f)] public float nodeMass = 0.1f;
    [Min(1)] public int substeps = 4;
    public float constraintStabilization = 60f;
    [Min(0f)] public float linearDamping = 0.05f;

    [Tooltip("Optional material for the rope mesh.")]
    public Material ropeMaterial;

    [Tooltip("Rope mesh radius. Lower this with low radialSegments to isolate physics cost from rendering cost.")]
    [Min(0.001f)] public float ropeRadius = 0.04f;

    [Range(3, 32)] public int ropeRadialSegments = 6;

    [Header("Layout")]
    public Vector3 layoutOrigin = new(0f, 5f, 0f);
    public Vector3 spacing = new(1.5f, 0f, 1.5f);
    [Min(1)] public int gridColumns = 10;

    [Header("Output")]
    public bool logToConsole = true;
    public string csvFileName = "rope_benchmark.csv";
    public bool drawHud = true;

    [Tooltip("Write per-frame constraint violation samples to a separate time-series CSV.")]
    public bool writeViolationTimeSeries = true;

    [Tooltip("Filename for the per-frame violation time-series CSV.")]
    public string violationCsvFileName = "rope_benchmark_violation.csv";

    private enum BenchState
    {
        InitialWarmup,
        StageSettle,
        Measuring,
        Finished
    }

    private BenchState state = BenchState.InitialWarmup;

    private readonly List<Rope> spawned = new();
    private readonly List<string> csvRows = new();
    private readonly List<float> frameMsSamples = new();
    private readonly List<string> violationCsvRows = new();
    private readonly List<float> maxViolationSamples = new();
    private readonly List<float> meanViolationSamples = new();
    private readonly List<float> rmsViolationSamples = new();

    private float stateStartTime;
    private int consecutiveFailStages;

    private string finishReason = "";

    private float lastAvgFps;
    private float lastAvgMs;
    private float lastMedianMs;
    private float lastP95Ms;
    private float lastMaxViolation;
    private float lastMeanViolation;
    private float lastRmsViolation;

    void Start()
    {
        Application.targetFrameRate = -1;
        QualitySettings.vSyncCount = 0;

        csvRows.Add(
            "solver,count,nodeCount,substeps,radialSegments,avgFps,avgMs,medianMs,p95Ms,minMs,maxMs,samples,maxAbsViolation,meanAbsViolation,rmsViolation"
        );

        if (writeViolationTimeSeries)
        {
            violationCsvRows.Add(
                "solver,count,timeInStage,frameMs,maxAbsViolation,meanAbsViolation,rmsViolation"
            );
        }

        stateStartTime = Time.unscaledTime;

        if (logToConsole)
        {
            Debug.Log(
                $"[Bench {solverType}] Starting benchmark. " +
                $"Initial warmup: {initialWarmupSeconds:F1}s"
            );
        }
    }

    void Update()
    {
        if (state == BenchState.Finished) return;

        var kb = Keyboard.current;
        if (kb != null && kb.escapeKey.wasPressedThisFrame)
        {
            Finish("aborted by user");
            return;
        }

        float elapsed = Time.unscaledTime - stateStartTime;

        switch (state)
        {
            case BenchState.InitialWarmup:
                if (elapsed >= initialWarmupSeconds)
                {
                    SpawnInitialRopes();
                    BeginStageSettle();
                }
                break;

            case BenchState.StageSettle:
                if (elapsed >= stageSettleSeconds)
                {
                    BeginMeasuring();
                }
                break;

            case BenchState.Measuring:
                CollectFrameSample();

                if (elapsed >= measureSeconds)
                {
                    EndMeasurementWindow();
                }
                break;
        }
    }

    void SpawnInitialRopes()
    {
        int initialCount = mode == Mode.Fixed ? fixedCount : ropesPerStage;
        initialCount = Mathf.Min(initialCount, maxRopes);

        for (int i = 0; i < initialCount; i++)
            SpawnOne();

        if (logToConsole)
            Debug.Log($"[Bench {solverType}] Spawned initial ropes: {spawned.Count}");
    }

    void BeginStageSettle()
    {
        state = BenchState.StageSettle;
        stateStartTime = Time.unscaledTime;

        if (logToConsole)
        {
            Debug.Log(
                $"[Bench {solverType}] Settling stage with {spawned.Count} ropes " +
                $"for {stageSettleSeconds:F1}s..."
            );
        }
    }

    void BeginMeasuring()
    {
        frameMsSamples.Clear();
        maxViolationSamples.Clear();
        meanViolationSamples.Clear();
        rmsViolationSamples.Clear();

        state = BenchState.Measuring;
        stateStartTime = Time.unscaledTime;

        if (logToConsole)
        {
            Debug.Log(
                $"[Bench {solverType}] Measuring {spawned.Count} ropes " +
                $"for {measureSeconds:F1}s..."
            );
        }
    }

    void CollectFrameSample()
    {
        float dt = Time.unscaledDeltaTime;
        if (dt <= 0f) return;

        float frameMs = dt * 1000f;
        if (frameMs > maxAcceptedFrameMs) return;

        frameMsSamples.Add(frameMs);

        float maxAbs = 0f;
        double sumAbs = 0d;
        double sumSq = 0d;
        int cnt = 0;
        for (int i = 0; i < spawned.Count; i++)
        {
            spawned[i].AccumulateConstraintViolation(ref maxAbs, ref sumAbs, ref sumSq, ref cnt);
        }

        float meanAbs = cnt > 0 ? (float)(sumAbs / cnt) : 0f;
        float rms = cnt > 0 ? Mathf.Sqrt((float)(sumSq / cnt)) : 0f;

        maxViolationSamples.Add(maxAbs);
        meanViolationSamples.Add(meanAbs);
        rmsViolationSamples.Add(rms);

        lastMaxViolation = maxAbs;
        lastMeanViolation = meanAbs;
        lastRmsViolation = rms;

        if (writeViolationTimeSeries)
        {
            float timeInStage = Time.unscaledTime - stateStartTime;
            violationCsvRows.Add(
                $"{solverType},{spawned.Count},{timeInStage:F4},{frameMs:F3},{maxAbs:G7},{meanAbs:G7},{rms:G7}"
            );
        }
    }

    void EndMeasurementWindow()
    {
        int count = spawned.Count;

        if (frameMsSamples.Count == 0)
        {
            csvRows.Add($"{solverType},{count},{nodeCount},{substeps},{ropeRadialSegments},0,0,0,0,0,0,0,0,0,0");

            if (logToConsole)
                Debug.LogWarning($"[Bench {solverType}] No valid samples collected for count={count}.");

            Finish("no valid frame samples");
            return;
        }

        frameMsSamples.Sort();

        float sum = 0f;
        float minMs = frameMsSamples[0];
        float maxMs = frameMsSamples[^1];

        for (int i = 0; i < frameMsSamples.Count; i++)
            sum += frameMsSamples[i];

        float avgMs = sum / frameMsSamples.Count;
        float avgFps = 1000f / avgMs;
        float medianMs = PercentileSorted(frameMsSamples, 0.50f);
        float p95Ms = PercentileSorted(frameMsSamples, 0.95f);

        lastAvgFps = avgFps;
        lastAvgMs = avgMs;
        lastMedianMs = medianMs;
        lastP95Ms = p95Ms;

        float peakMaxViolation = 0f;
        float meanViolation = 0f;
        float rmsViolation = 0f;
        if (maxViolationSamples.Count > 0)
        {
            double sumMean = 0d;
            double sumRms = 0d;
            for (int i = 0; i < maxViolationSamples.Count; i++)
            {
                if (maxViolationSamples[i] > peakMaxViolation) peakMaxViolation = maxViolationSamples[i];
                sumMean += meanViolationSamples[i];
                sumRms += rmsViolationSamples[i];
            }
            meanViolation = (float)(sumMean / maxViolationSamples.Count);
            rmsViolation = (float)(sumRms / maxViolationSamples.Count);
        }

        csvRows.Add(
            $"{solverType}," +
            $"{count}," +
            $"{nodeCount}," +
            $"{substeps}," +
            $"{ropeRadialSegments}," +
            $"{avgFps:F2}," +
            $"{avgMs:F3}," +
            $"{medianMs:F3}," +
            $"{p95Ms:F3}," +
            $"{minMs:F3}," +
            $"{maxMs:F3}," +
            $"{frameMsSamples.Count}," +
            $"{peakMaxViolation:G7}," +
            $"{meanViolation:G7}," +
            $"{rmsViolation:G7}"
        );

        if (logToConsole)
        {
            Debug.Log(
                $"[Bench {solverType}] ropes={count} | " +
                $"avgFPS={avgFps:F1} | " +
                $"avg={avgMs:F2}ms | " +
                $"median={medianMs:F2}ms | " +
                $"p95={p95Ms:F2}ms | " +
                $"samples={frameMsSamples.Count} | " +
                $"violation peak={peakMaxViolation:G4} mean={meanViolation:G4} rms={rmsViolation:G4}"
            );
        }

        bool failedFps = avgFps < stopFpsThreshold;
        bool failedP95 = p95Ms > stopP95FrameMs;
        bool failedStage = failedFps || failedP95;

        if (failedStage)
            consecutiveFailStages++;
        else
            consecutiveFailStages = 0;

        if (mode == Mode.Fixed)
        {
            Finish("fixed-mode measurement complete");
            return;
        }

        if (consecutiveFailStages >= consecutiveFailStagesToStop)
        {
            Finish(
                $"performance failed for {consecutiveFailStages} consecutive stages " +
                $"avgFPS={avgFps:F1}, p95={p95Ms:F1}ms"
            );
            return;
        }

        if (spawned.Count >= maxRopes)
        {
            Finish($"reached maxRopes ({maxRopes})");
            return;
        }

        for (int i = 0; i < ropesPerStage && spawned.Count < maxRopes; i++)
            SpawnOne();

        BeginStageSettle();
    }

    float PercentileSorted(List<float> sortedValues, float percentile)
    {
        if (sortedValues.Count == 0) return 0f;
        if (sortedValues.Count == 1) return sortedValues[0];

        float index = percentile * (sortedValues.Count - 1);
        int lower = Mathf.FloorToInt(index);
        int upper = Mathf.CeilToInt(index);

        if (lower == upper)
            return sortedValues[lower];

        float t = index - lower;
        return Mathf.Lerp(sortedValues[lower], sortedValues[upper], t);
    }

    void Finish(string reason)
    {
        state = BenchState.Finished;
        finishReason = reason;

        if (!string.IsNullOrEmpty(csvFileName))
        {
            string fullPath = Path.Combine(Application.persistentDataPath, csvFileName);

            var sb = new StringBuilder();
            foreach (string row in csvRows)
                sb.AppendLine(row);

            File.WriteAllText(fullPath, sb.ToString());

            Debug.Log($"Benchmark CSV → {fullPath}");
        }

        if (writeViolationTimeSeries && !string.IsNullOrEmpty(violationCsvFileName) && violationCsvRows.Count > 1)
        {
            string fullPath = Path.Combine(Application.persistentDataPath, violationCsvFileName);

            var sb = new StringBuilder();
            foreach (string row in violationCsvRows)
                sb.AppendLine(row);

            File.WriteAllText(fullPath, sb.ToString());

            Debug.Log($"Violation time-series CSV → {fullPath}");
        }

        Debug.Log($"[Bench {solverType}] DONE — {reason}; total ropes: {spawned.Count}");
    }

    void SpawnOne()
    {
        int idx = spawned.Count;

        int row = idx / gridColumns;
        int col = idx % gridColumns;

        Vector3 pos = layoutOrigin + new Vector3(
            col * spacing.x,
            row * spacing.y,
            row * spacing.z
        );

        GameObject go = new($"BenchRope_{idx}");
        go.transform.position = pos;
        go.transform.SetParent(transform);

        Rope rope = go.AddComponent<Rope>();

        rope.solverType = solverType;
        rope.nodeCount = nodeCount;
        rope.segmentLength = segmentLength;
        rope.nodeMass = nodeMass;
        rope.substeps = substeps;
        rope.constraintStabilization = constraintStabilization;
        rope.linearDamping = linearDamping;
        rope.radius = ropeRadius;
        rope.radialSegments = ropeRadialSegments;

        if (ropeMaterial != null)
        {
            MeshRenderer mr = go.GetComponent<MeshRenderer>();
            if (mr != null)
                mr.sharedMaterial = ropeMaterial;
        }

        spawned.Add(rope);
    }

    void OnGUI()
    {
        if (!drawHud) return;

        var style = new GUIStyle(GUI.skin.box)
        {
            fontSize = 16,
            alignment = TextAnchor.UpperLeft,
            padding = new RectOffset(10, 10, 8, 8),
        };

        style.normal.textColor = Color.white;

        float frameMs = Time.unscaledDeltaTime > 0f ? Time.unscaledDeltaTime * 1000f : 0f;
        float fps = frameMs > 0f ? 1000f / frameMs : 0f;
        float elapsed = Time.unscaledTime - stateStartTime;

        string text =
            $"Rope Benchmark — {solverType}\n" +
            $"Mode: {mode}     State: {state}\n" +
            $"Ropes: {spawned.Count}\n" +
            $"Current FPS: {fps,6:F1}     Current Frame: {frameMs,5:F2} ms\n";

        switch (state)
        {
            case BenchState.InitialWarmup:
                text += $"Initial warmup: {Mathf.Min(elapsed, initialWarmupSeconds):F1} / {initialWarmupSeconds:F1}s";
                break;

            case BenchState.StageSettle:
                text += $"Stage settle: {Mathf.Min(elapsed, stageSettleSeconds):F1} / {stageSettleSeconds:F1}s";
                break;

            case BenchState.Measuring:
                text +=
                    $"Measuring: {Mathf.Min(elapsed, measureSeconds):F1} / {measureSeconds:F1}s\n" +
                    $"Samples: {frameMsSamples.Count}\n" +
                    $"Violation max={lastMaxViolation:G4}  mean={lastMeanViolation:G4}  rms={lastRmsViolation:G4}";
                break;

            case BenchState.Finished:
                text +=
                    $"Finished: {finishReason}\n" +
                    $"Last avg FPS: {lastAvgFps:F1}\n" +
                    $"Last avg: {lastAvgMs:F2}ms   median: {lastMedianMs:F2}ms   p95: {lastP95Ms:F2}ms\n" +
                    $"Last violation max={lastMaxViolation:G4}  mean={lastMeanViolation:G4}  rms={lastRmsViolation:G4}";
                break;
        }

        text += "\nEsc aborts and flushes CSV.";

        GUI.Box(new Rect(10, 10, 540, 220), text, style);
    }
}