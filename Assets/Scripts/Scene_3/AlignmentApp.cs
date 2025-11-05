using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Networking;
using TMPro;

public class AlignmentApp : MonoBehaviour
{
    [SerializeField] private Button btnMethodToggle;   
    [SerializeField] private Button btnVizToggle;      
    [SerializeField] private Button btnLoadP;
    [SerializeField] private Button btnLoadQ;
    [SerializeField] private Button btnAlign;
    [SerializeField] private TMP_InputField inputPathP;
    [SerializeField] private TMP_InputField inputPathQ;
    [SerializeField] private TMP_Text statusText;

    [SerializeField] private TMP_InputField inputMaxIters;     
    [SerializeField] private TMP_InputField inputInlierThresh;  
    [SerializeField] private TMP_InputField inputRandomSeed;    
    [SerializeField] private TMP_InputField inputScaleFactor;  

    [SerializeField] private float pointSize = 0.03f;
    [SerializeField] private Material matP;
    [SerializeField] private Material matQ;
    [SerializeField] private Material matQAligned;
    [SerializeField] private Material matMotionLine;

    [SerializeField] private RegistrationMethod method = RegistrationMethod.Rigid;
    [SerializeField] private int maxIterations = 200;
    [SerializeField] private float inlierThreshold = 0.02f;
    [SerializeField] private int randomSeed = 1234;

    private Vector3[] P, Q;
    private AlignResult result;
    private bool viewMotion = false;

    private Transform rootP, rootQ, rootQAligned, rootLines;
    private float scaleFactor = 0.1f;

    private void Awake()
    {
        if (btnMethodToggle) btnMethodToggle.onClick.AddListener(ToggleMethod);
        if (btnVizToggle)    btnVizToggle.onClick.AddListener(ToggleViz);
        if (btnLoadP)        btnLoadP.onClick.AddListener(LoadP);
        if (btnLoadQ)        btnLoadQ.onClick.AddListener(LoadQ);
        if (btnAlign)        btnAlign.onClick.AddListener(DoAlign);

        if (inputMaxIters)
        {
            inputMaxIters.text = maxIterations.ToString(CultureInfo.InvariantCulture);
            inputMaxIters.onEndEdit.AddListener(_ => ApplyUserParams());
        }
        if (inputInlierThresh)
        {
            inputInlierThresh.text = inlierThreshold.ToString(CultureInfo.InvariantCulture);
            inputInlierThresh.onEndEdit.AddListener(_ => ApplyUserParams());
        }
        if (inputRandomSeed)
        {
            inputRandomSeed.text = randomSeed.ToString(CultureInfo.InvariantCulture);
            inputRandomSeed.onEndEdit.AddListener(_ => ApplyUserParams());
        }

        if (inputScaleFactor)
        {
            inputScaleFactor.text = scaleFactor.ToString(CultureInfo.InvariantCulture);
            inputScaleFactor.onEndEdit.AddListener(_ => ApplyScaleFactor());
        }

        rootP = new GameObject("P_Points").transform;
        rootQ = new GameObject("Q_Points").transform;
        rootQAligned = new GameObject("Q_Aligned").transform;
        rootLines = new GameObject("Motion_Lines").transform;

        UpdateMethodButton();
        UpdateVizButton();
        Print("Ready. Load P and Q files.");
    }

    private void ApplyUserParams()
    {
        if (inputMaxIters && !string.IsNullOrWhiteSpace(inputMaxIters.text))
        {
            if (int.TryParse(inputMaxIters.text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var iters))
                maxIterations = Mathf.Max(1, iters);
        }

        if (inputInlierThresh && !string.IsNullOrWhiteSpace(inputInlierThresh.text))
        {
            if (float.TryParse(inputInlierThresh.text, NumberStyles.Float, CultureInfo.InvariantCulture, out var thr))
                inlierThreshold = Mathf.Max(0f, thr);
        }

        if (inputRandomSeed && !string.IsNullOrWhiteSpace(inputRandomSeed.text))
        {
            if (int.TryParse(inputRandomSeed.text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var seed))
                randomSeed = seed;
        }
    }

    private void ApplyScaleFactor()
    {
        if (inputScaleFactor && !string.IsNullOrWhiteSpace(inputScaleFactor.text))
        {
            if (float.TryParse(inputScaleFactor.text, NumberStyles.Float, CultureInfo.InvariantCulture, out var factor))
                scaleFactor = Mathf.Max(0f, factor);
        }
    }

    private void ClearChildren(Transform t)
    {
        for (int i = t.childCount - 1; i >= 0; i--)
            Destroy(t.GetChild(i).gameObject);
    }

    private void LoadP()
    {
        StartCoroutine(PrepareAndLoad(inputPathP ? inputPathP.text.Trim() : "", isP: true));
    }

    private void LoadQ()
    {
        StartCoroutine(PrepareAndLoad(inputPathQ ? inputPathQ.text.Trim() : "", isP: false));
    }

    private IEnumerator PrepareAndLoad(string userInput, bool isP)
    {
        if (string.IsNullOrWhiteSpace(userInput))
        {
            Print("Path/Name is empty.");
            yield break;
        }

        Print("Preparing file: " + userInput);

        string realPath = null;

        if (userInput.StartsWith("res:", System.StringComparison.OrdinalIgnoreCase))
        {
            string resName = userInput.Substring(4).Trim();
            if (string.IsNullOrEmpty(resName))
            {
                Print("Invalid resources key.");
                yield break;
            }

            TextAsset ta = Resources.Load<TextAsset>(resName);
            if (ta == null)
            {
                Print($"Resources.Load failed: {resName}");
                yield break;
            }

            realPath = Path.Combine(Application.persistentDataPath, resName + ".txt");
            File.WriteAllText(realPath, ta.text);
        }
        else if (userInput.StartsWith("/") || userInput.StartsWith("file:", System.StringComparison.OrdinalIgnoreCase))
        {
            realPath = userInput.StartsWith("file:", System.StringComparison.OrdinalIgnoreCase)
                ? new System.Uri(userInput).LocalPath
                : userInput;

            if (!File.Exists(realPath))
            {
                Print("File not found: " + realPath);
                yield break;
            }
        }
        else
        {
            string saPath = Path.Combine(Application.streamingAssetsPath, userInput);
            string dstPath = Path.Combine(Application.persistentDataPath, userInput);

            if (!File.Exists(dstPath))
            {
#if UNITY_ANDROID
                using (var req = UnityWebRequest.Get(saPath))
                {
                    yield return req.SendWebRequest();
                    if (req.result != UnityWebRequest.Result.Success)
                    {
                        Print("StreamingAssets read failed: " + req.error);
                        yield break;
                    }
                    File.WriteAllBytes(dstPath, req.downloadHandler.data);
                }
#else
                if (saPath.StartsWith("jar:") || saPath.StartsWith("file://"))
                {
                    using (var req = UnityWebRequest.Get(saPath))
                    {
                        yield return req.SendWebRequest();
                        if (req.result != UnityWebRequest.Result.Success)
                        {
                            Print("StreamingAssets read failed: " + req.error);
                            yield break;
                        }
                        File.WriteAllBytes(dstPath, req.downloadHandler.data);
                    }
                }
                else
                {
                    if (!File.Exists(saPath))
                    {
                        Print("StreamingAssets file not found: " + saPath);
                        yield break;
                    }
                    File.WriteAllBytes(dstPath, File.ReadAllBytes(saPath));
                }
#endif
            }
            realPath = dstPath;
        }

        try
        {
            var pts = FilePointReader.LoadPointFile(realPath);
            if (isP)
            {
                P = pts;
                DrawPointCloud(P, rootP, matP);
                Print($"Loaded P: {P.Length} points");
            }
            else
            {
                Q = pts;
                DrawPointCloud(Q, rootQ, matQ);
                Print($"Loaded Q: {Q.Length} points");
            }
        }
        catch (System.SystemException e)
        {
            Print("Load error: " + e.Message);
        }
    }

    private void DoAlign()
    {
        ApplyUserParams();

        if (P == null || Q == null)
        {
            Print("Please load P and Q first.");
            return;
        }

        result = RansacAligner.AlignRansac(P, Q, method, maxIterations, inlierThreshold, randomSeed);
        if (!result.success)
        {
            Print("Alignment failed. Not enough inliers.");
            return;
        }

        Redraw();
        PrintResult();
    }

    private void Redraw()
    {
        ClearChildren(rootQAligned);
        ClearChildren(rootLines);

        if (result.success)
        {
            var Qa = new Vector3[Q.Length];
            for (int i = 0; i < Q.Length; i++)
                Qa[i] = RansacAligner.TransformPoint(result, Q[i]);

            if (!viewMotion)
            {
                DrawPointCloud(P, rootP, matP);
                DrawPointCloud(Q, rootQ, matQ);
                DrawPointCloud(Qa, rootQAligned, matQAligned);
            }
            else
            {
                ClearChildren(rootP);
                ClearChildren(rootQ);
                DrawPointCloud(Qa, rootQAligned, matQAligned);
                DrawMotionLines(Q, Qa, rootLines, matMotionLine);
            }
        }
    }

    private void PrintResult()
    {
        var Rm = result.R;
        var Rrow0 = $"{Rm.m00:+0.000;-0.000} {Rm.m01:+0.000;-0.000} {Rm.m02:+0.000;-0.000}";
        var Rrow1 = $"{Rm.m10:+0.000;-0.000} {Rm.m11:+0.000;-0.000} {Rm.m12:+0.000;-0.000}";
        var Rrow2 = $"{Rm.m20:+0.000;-0.000} {Rm.m21:+0.000;-0.000} {Rm.m22:+0.000;-0.000}";
        string scaleStr = (method == RegistrationMethod.Similarity) ? $"\ns = {result.s:0.000}" : "\ns = 1.000";
        Print($"Inliers: {result.inlierCount}\nR =\n{Rrow0}\n{Rrow1}\n{Rrow2}\nT = ({result.T.x:+0.000;-0.000}, {result.T.y:+0.000;-0.000}, {result.T.z:+0.000;-0.000}){scaleStr}");
    }

    private void DrawPointCloud(Vector3[] pts, Transform parent, Material mat)
    {
        ClearChildren(parent);
        foreach (var p in pts)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.transform.SetParent(parent, false);
            go.transform.position = p * scaleFactor;
            go.transform.localScale = Vector3.one * pointSize;
            var r = go.GetComponent<Renderer>();
            if (mat != null) r.sharedMaterial = mat;
            var col = go.GetComponent<Collider>();
            if (col) Destroy(col);
        }
    }

    private void DrawMotionLines(Vector3[] qOrig, Vector3[] qAligned, Transform parent, Material mat)
    {
        ClearChildren(parent);
        int n = Mathf.Min(qOrig.Length, qAligned.Length);
        for (int i = 0; i < n; i++)
        {
            var go = new GameObject("line_" + i);
            go.transform.SetParent(parent, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.positionCount = 2;
            lr.useWorldSpace = true;
            lr.SetPosition(0, qOrig[i] * scaleFactor);
            lr.SetPosition(1, qAligned[i] * scaleFactor);
            lr.startWidth = lr.endWidth = Mathf.Max(0.5f * pointSize, 0.005f);
            if (mat) lr.sharedMaterial = mat;
            lr.numCapVertices = 4;
            lr.numCornerVertices = 4;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
        }
    }

    private void ToggleMethod()
    {
        method = (method == RegistrationMethod.Rigid) ? RegistrationMethod.Similarity : RegistrationMethod.Rigid;
        UpdateMethodButton();
    }

    private void ToggleViz()
    {
        viewMotion = !viewMotion;
        UpdateVizButton();
        Redraw();
    }

    private void UpdateMethodButton()
    {
        if (btnMethodToggle && btnMethodToggle.GetComponentInChildren<TMP_Text>())
            btnMethodToggle.GetComponentInChildren<TMP_Text>().text = $"Method: {method}";
    }

    private void UpdateVizButton()
    {
        if (btnVizToggle && btnVizToggle.GetComponentInChildren<TMP_Text>())
            btnVizToggle.GetComponentInChildren<TMP_Text>().text = viewMotion ? "Viz: Motion" : "Viz: Points";
    }

    private void Print(string msg)
    {
        if (statusText) statusText.text = msg;
        Debug.Log(msg);
    }
}
