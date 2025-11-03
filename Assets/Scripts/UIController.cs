using System.IO;
using UnityEngine;
using TMPro;

public class UIController : MonoBehaviour
{
    [Header("Views")]
    [SerializeField] private PointCloudView pView;
    [SerializeField] private PointCloudView qView;
    [SerializeField] private PointCloudView qpView;
    [SerializeField] private MovementLines lines;

    [Header("UI")]
    [SerializeField] private TextMeshProUGUI paramsText;

    [Header("Materials")]
    [SerializeField] private Material matP;
    [SerializeField] private Material matQ;
    [SerializeField] private Material matQp;

    [Header("Files (leave empty to use StreamingAssets/P.txt & Q.txt)")]
    [SerializeField] private string pathP = "";
    [SerializeField] private string pathQ = "";

    [Header("RANSAC Params")]
    [SerializeField] private bool withScaleDefault = false; // false=Rigid, true=Similarity
    [SerializeField] private int iters = 1000;
    [SerializeField] private float inlierThreshold = 0.03f;

    // State
    private Vector3[] P, Q, Qp;
    private bool withScale;
    private bool showLines = false;

    void Awake()
    {
        withScale = withScaleDefault;
        UpdateParamsTextIdle();
        ClearAllViews();
    }

    void ClearAllViews()
    {
        if (pView) pView.Clear();
        if (qView) qView.Clear();
        if (qpView) qpView.Clear();
        if (lines) lines.Clear();
    }

    string DefaultPPath() => Path.Combine(Application.streamingAssetsPath, "P.txt");
    string DefaultQPath() => Path.Combine(Application.streamingAssetsPath, "Q.txt");

    // UI Buttons
    public void ToggleMethod()
    {
        withScale = !withScale;
        UpdateParamsTextIdle();
    }

    public void ToggleView()
    {
        showLines = !showLines;
        RedrawView();
    }

    public void LoadP()
    {
        string pth = string.IsNullOrWhiteSpace(pathP) ? DefaultPPath() : pathP;
        P = PointCloudIO.LoadTxt(pth);
        RedrawView();
        UpdateParamsTextIdle();
    }

    public void LoadQ()
    {
        string pth = string.IsNullOrWhiteSpace(pathQ) ? DefaultQPath() : pathQ;
        Q = PointCloudIO.LoadTxt(pth);
        RedrawView();
        UpdateParamsTextIdle();
    }

    public void RunRegistration()
    {
        if (P == null || Q == null || P.Length < 3 || Q.Length < 3)
        {
            if (paramsText) paramsText.text = "Load P and Q (>=3 points).";
            return;
        }

        // RANSAC kayıt
        var res = Registration.RansacRegister(P, Q, withScale, iters, inlierThreshold);

        // Q' üret
        Qp = new Vector3[Q.Length];
        for (int i = 0; i < Q.Length; i++)
            Qp[i] = (Vector3)(res.R.MultiplyPoint3x4(Q[i] * res.Scale) + res.T);

        // Görsel
        RedrawView();

        // Yazı
        string mode = withScale ? "Similarity (s, R, T)" : "Rigid (R, T)";
        string Rtxt =
            $"[{res.R.m00:F3} {res.R.m01:F3} {res.R.m02:F3}]\n" +
            $"[{res.R.m10:F3} {res.R.m11:F3} {res.R.m12:F3}]\n" +
            $"[{res.R.m20:F3} {res.R.m21:F3} {res.R.m22:F3}]";
        string Ttxt = $"[{res.T.x:F3}, {res.T.y:F3}, {res.T.z:F3}]";
        string Stxt = withScale ? $"\ns = {res.Scale:F5}" : "";
        if (paramsText)
            paramsText.text = $"{mode}\nInliers: {res.Inliers}\nR = \n{Rtxt}\nT = {Ttxt}{Stxt}";
    }

    void RedrawView()
    {
        // Temizle
        if (pView) pView.Clear();
        if (qView) qView.Clear();
        if (qpView) qpView.Clear();
        if (lines) lines.Clear();

        // Noktaları çiz
        if (P != null && pView) { pView.ShowPoints(P); TintChildren(pView.transform, matP); }
        if (Q != null && qView) { qView.ShowPoints(Q); TintChildren(qView.transform, matQ); }

        // Q' varsa göster
        if (Qp != null && qpView)
        {
            if (!showLines)
            {
                qpView.ShowPoints(Qp);
                TintChildren(qpView.transform, matQp);
            }
            else
            {
                lines.Show(Q, Qp);
            }
        }
    }

    void TintChildren(Transform parent, Material mat)
    {
        if (!parent || !mat) return;
        for (int i = 0; i < parent.childCount; i++)
        {
            var rend = parent.GetChild(i).GetComponent<Renderer>();
            if (rend) rend.sharedMaterial = mat;
        }
    }

    void UpdateParamsTextIdle()
    {
        if (!paramsText) return;
        string mode = withScale ? "Method: Similarity (s,R,T)" : "Method: Rigid (R,T)";
        string pInfo = (P == null) ? "P: not loaded" : $"P: {P.Length} pts";
        string qInfo = (Q == null) ? "Q: not loaded" : $"Q: {Q.Length} pts";
        string view = showLines ? "View: Movement Lines" : "View: Points";
        paramsText.text = $"{mode}\n{pInfo} | {qInfo}\n{view}\nIters={iters}  τ={inlierThreshold}";
    }
}
