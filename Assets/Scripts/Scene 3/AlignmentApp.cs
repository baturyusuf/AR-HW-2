using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class AlignmentApp : MonoBehaviour
{
    [Header("UI References")]
    [SerializeField] private Button btnMethodToggle;     // Rigid <-> Similarity
    [SerializeField] private Button btnVizToggle;        // Points <-> Motion
    [SerializeField] private Button btnLoadP;
    [SerializeField] private Button btnLoadQ;
    [SerializeField] private Button btnAlign;
    [SerializeField] private TMP_InputField inputPathP;  // veya UnityEngine.UI.InputField
    [SerializeField] private TMP_InputField inputPathQ;
    [SerializeField] private TMP_Text statusText;        // çıktı yazısı

    [Header("Rendering")]
    [SerializeField] private float pointSize = 0.03f;
    [SerializeField] private Material matP;
    [SerializeField] private Material matQ;
    [SerializeField] private Material matQAligned;
    [SerializeField] private Material matMotionLine;

    [Header("RANSAC Params")]
    [SerializeField] private RegistrationMethod method = RegistrationMethod.Rigid;
    [SerializeField] private int maxIterations = 200;
    [SerializeField] private float inlierThreshold = 0.02f;
    [SerializeField] private int randomSeed = 1234;

    // Data
    private Vector3[] P, Q;
    private AlignResult result;
    private bool viewMotion = false;

    // Scene holders
    private Transform rootP, rootQ, rootQAligned, rootLines;

    private void Awake()
    {
        // UI wiring
        if (btnMethodToggle) btnMethodToggle.onClick.AddListener(ToggleMethod);
        if (btnVizToggle)    btnVizToggle.onClick.AddListener(ToggleViz);
        if (btnLoadP)        btnLoadP.onClick.AddListener(LoadP);
        if (btnLoadQ)        btnLoadQ.onClick.AddListener(LoadQ);
        if (btnAlign)        btnAlign.onClick.AddListener(DoAlign);

        // parents
        rootP = new GameObject("P_Points").transform;
        rootQ = new GameObject("Q_Points").transform;
        rootQAligned = new GameObject("Q_Aligned").transform;
        rootLines = new GameObject("Motion_Lines").transform;

        UpdateMethodButton();
        UpdateVizButton();
        Print("Ready. Load P and Q files.");
    }

    private void ClearChildren(Transform t)
    {
        for (int i = t.childCount - 1; i >= 0; i--)
            Destroy(t.GetChild(i).gameObject);
    }

    private void LoadP()
    {
        try
        {
            P = FilePointReader.LoadPointFile(inputPathP.text.Trim());
            DrawPointCloud(P, rootP, matP);
            Print($"Loaded P: {P.Length} points");
        }
        catch (System.SystemException e)
        {
            Print("Load P error: " + e.Message);
        }
    }

    private void LoadQ()
    {
        try
        {
            Q = FilePointReader.LoadPointFile(inputPathQ.text.Trim());
            DrawPointCloud(Q, rootQ, matQ);
            Print($"Loaded Q: {Q.Length} points");
        }
        catch (System.SystemException e)
        {
            Print("Load Q error: " + e.Message);
        }
    }

    private void DoAlign()
    {
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

        // Görselleştirme
        Redraw();

        // Sonuç yazdır
        var Rm = result.R;
        var Rrow0 = $"{Rm.m00:+0.000;-0.000} {Rm.m01:+0.000;-0.000} {Rm.m02:+0.000;-0.000}";
        var Rrow1 = $"{Rm.m10:+0.000;-0.000} {Rm.m11:+0.000;-0.000} {Rm.m12:+0.000;-0.000}";
        var Rrow2 = $"{Rm.m20:+0.000;-0.000} {Rm.m21:+0.000;-0.000} {Rm.m22:+0.000;-0.000}";
        string scaleStr = (method == RegistrationMethod.Similarity) ? $"\ns = {result.s:0.000}" : "\ns = 1.000";
        Print($"Inliers: {result.inlierCount}\nR =\n{Rrow0}\n{Rrow1}\n{Rrow2}\nT = ({result.T.x:+0.000;-0.000}, {result.T.y:+0.000;-0.000}, {result.T.z:+0.000;-0.000}){scaleStr}");
    }

    private void Redraw()
    {
        ClearChildren(rootQAligned);
        ClearChildren(rootLines);

        // her iki görselleştirme modu için önce hizalanmış Q noktalarını hesapla
        if (result.success)
        {
            var Qa = new Vector3[Q.Length];
            for (int i = 0; i < Q.Length; i++)
                Qa[i] = RansacAligner.TransformPoint(result, Q[i]);

            if (!viewMotion)
            {
                // Mod 1: Orijinal + hizalanmış noktalar (3 renk)
                DrawPointCloud(P, rootP, matP);         // P: color A
                DrawPointCloud(Q, rootQ, matQ);         // Q orijinal: color B
                DrawPointCloud(Qa, rootQAligned, matQAligned); // Q aligned: color C
            }
            else
            {
                // Mod 2: Sadece Q transform + hareket çizgisi
                ClearChildren(rootP);
                ClearChildren(rootQ);
                DrawPointCloud(Qa, rootQAligned, matQAligned);
                DrawMotionLines(Q, Qa, rootLines, matMotionLine);
            }
        }
    }

    private void DrawPointCloud(Vector3[] pts, Transform parent, Material mat)
    {
        ClearChildren(parent);
        foreach (var p in pts)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.transform.SetParent(parent, false);
            go.transform.position = p;
            go.transform.localScale = Vector3.one * pointSize;
            var r = go.GetComponent<Renderer>();
            if (mat != null) r.sharedMaterial = mat;
            // Gereksiz collider'ı kaldır
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
            lr.SetPosition(0, qOrig[i]);
            lr.SetPosition(1, qAligned[i]);
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
