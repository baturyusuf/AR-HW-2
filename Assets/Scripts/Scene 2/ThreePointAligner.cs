using System;
using System.Collections.Generic;
using UnityEngine;

public enum ThreePointMethod { Rigid, Similarity }

public class ThreePointAligner : MonoBehaviour
{
    [Header("Input Files (choose one of these ways)")]
    public TextAsset PFile;                 // (Seçenek 1) Inspector’a .txt sürükle-bırak
    public TextAsset QFile;                 // (Seçenek 1)
    public string PPathOrStreamingName;     // (Seçenek 2) Yol: mutlak veya StreamingAssets içinde dosya adı
    public string QPathOrStreamingName;     // (Seçenek 2)

    [Header("Parsed Points (ReadOnly)")]
    public List<Vector3> P; // P kümesi
    public List<Vector3> Q; // Q kümesi

    [Header("Pick matching indices (YOU choose which points match)")]
    public int p1Index = 0, p2Index = 1, p3Index = 2;
    public int q1Index = 0, q2Index = 1, q3Index = 2;

    [Header("Method")]
    public ThreePointMethod method = ThreePointMethod.Rigid;

    [Header("Result (Read-Only)")]
    public Matrix4x4 R = Matrix4x4.identity;
    public Vector3 T = Vector3.zero;
    public float s = 1f;

    [Header("Apply result")]
    public bool applyToQTransforms = false;    // Q sahnede Transform olarak varsa uygula
    public Transform[] QTransforms;            // (opsiyonel) Q'yu temsil eden Transform’lar
                                              // yoksa sadece veriler üzerinde hesap yapar

    // =============================
    // 0) Dosyaları yükle
    // =============================
    [ContextMenu("Load Files")]
    public void LoadFiles()
    {
        try
        {
            if (PFile != null) P = PointCloudFileReader.LoadFromTextAsset(PFile);
            else if (!string.IsNullOrEmpty(PPathOrStreamingName)) P = PointCloudFileReader.LoadFromPath(PPathOrStreamingName);
            else throw new Exception("Provide PFile or PPathOrStreamingName.");

            if (QFile != null) Q = PointCloudFileReader.LoadFromTextAsset(QFile);
            else if (!string.IsNullOrEmpty(QPathOrStreamingName)) Q = PointCloudFileReader.LoadFromPath(QPathOrStreamingName);
            else throw new Exception("Provide QFile or QPathOrStreamingName.");

            Debug.Log($"Loaded P={P.Count} points, Q={Q.Count} points.");
        }
        catch (Exception ex)
        {
            Debug.LogError("LoadFiles error: " + ex.Message);
        }
    }

    // =============================
    // 1) Neden 3 nokta? — Kod tarafında: 3 eşleşik indisi alıyoruz.
    // =============================
    Vector3 p1, p2, p3, q1, q2, q3;

    // =============================
    // 2-3) P ve Q için yerel eksen takımları (ONB) kur
    // =============================
    struct ONB
    {
        public Vector3 e1, e2, e3;   // dik birim vektörler
        public Matrix4x4 B;          // sütunları e1 e2 e3 olan baz matrisi
        public bool valid;
    }

    void Start(){
        //LoadFiles();
    }

    // -----------------------------
    // 4) R = BQ * BP^T
    // 5) T = q1 - R * p1
    // 6) Uygulama fonksiyonları
    // -----------------------------

    // =============================
    // ANA AKIŞ: 3-nokta hizalama (Rigid/Similarity)
    // =============================
    [ContextMenu("Align Using 3 Points")]
    public void Align3Points()
    {
        if (P == null || Q == null || P.Count < 3 || Q.Count < 3)
        {
            Debug.LogError("Load the files first and ensure P,Q have >=3 points.");
            return;
        }
        if (!CheckIndices()) return;

        // Seçilen üçer nokta:
        p1 = P[p1Index]; p2 = P[p2Index]; p3 = P[p3Index];
        q1 = Q[q1Index]; q2 = Q[q2Index]; q3 = Q[q3Index];

        // 2) P tarafında ONB kur
        var onbP = BuildONB(p1, p2, p3);
        if (!onbP.valid)
        {
            Debug.LogError("P ONB could not be built. Points may be collinear or too close.");
            return;
        }

        // 3) Q tarafında ONB kur
        var onbQ = BuildONB(q1, q2, q3);
        if (!onbQ.valid)
        {
            Debug.LogError("Q ONB could not be built. Points may be collinear or too close.");
            return;
        }

        // 4) R = BQ * BP^T
        var BP_T = onbP.B.transpose;
        R = onbQ.B * BP_T;

        // 5) T = q1 - R * p1  (Rigid temel formül)
        T = q1 - R.MultiplyPoint3x4(p1);

        // Similarity ise ölçek s'yi de bul (6'nın “neden” kısmı similarity’de s de var)
        if (method == ThreePointMethod.Similarity)
        {
            // Üçgen kenar uzunluklarının ortalamasından tekil s:
            float lp12 = (p2 - p1).magnitude;
            float lp13 = (p3 - p1).magnitude;
            float lp23 = (p3 - p2).magnitude;
            float avgP = (lp12 + lp13 + lp23) / 3f;

            float lq12 = (q2 - q1).magnitude;
            float lq13 = (q3 - q1).magnitude;
            float lq23 = (q3 - q2).magnitude;
            float avgQ = (lq12 + lq13 + lq23) / 3f;

            if (avgP <= 1e-8f) { Debug.LogError("Degenerate P triangle."); return; }
            s = avgQ / avgP;
        }
        else
        {
            s = 1f;
        }

        Debug.Log($"Alignment done. Method={method}, s={s:F6}\nR=\n{Pretty(R)}\nT={T}");

        // 6) (opsiyonel) sahnedeki Q Transform’larına uygula
        if (applyToQTransforms && QTransforms != null && QTransforms.Length > 0)
        {
            ApplyToQTransforms();
        }
    }

    // =============================
    // 6) Uygula: q_hat = s*R*p + T
    // =============================
    public void ApplyToQTransforms()
    {
        foreach (var t in QTransforms)
        {
            if (t == null) continue;
            Vector3 p = t.position;
            Vector3 aligned = R.MultiplyPoint3x4(p * s) + T;
            t.position = aligned;
        }
        Debug.Log("Applied transform to QTransforms.");
    }

    // =============================
    // 7-8-9) Doğrulama/sağlamlık: 
    // Inspector’dan “Gizmo çizdirme” ile görsel kontrol yapabilirsin.
    // Aşağıda ONB kurulumunun doğru çalışıp çalışmadığını görmeye yarayan yardımcılar var.
    // =============================
    void OnDrawGizmosSelected()
    {
        // P ve Q seçilmişse küçük eksenleri görselleştir:
        if (P != null && Q != null && 
            p1Index >= 0 && p2Index >= 0 && p3Index >= 0 &&
            q1Index >= 0 && q2Index >= 0 && q3Index >= 0 &&
            p1Index < P.Count && p2Index < P.Count && p3Index < P.Count &&
            q1Index < Q.Count && q2Index < Q.Count && q3Index < Q.Count)
        {
            var onbP = BuildONB(P[p1Index], P[p2Index], P[p3Index]);
            var onbQ = BuildONB(Q[q1Index], Q[q2Index], Q[q3Index]);

            if (onbP.valid) DrawBasis(P[p1Index], onbP, 0.1f);
            if (onbQ.valid) DrawBasis(Q[q1Index], onbQ, 0.1f);
        }
    }

    // ----------------- Yardımcılar -----------------
    bool CheckIndices()
    {
        if (p1Index == p2Index || p1Index == p3Index || p2Index == p3Index)
        {
            Debug.LogError("P indices must be distinct.");
            return false;
        }
        if (q1Index == q2Index || q1Index == q3Index || q2Index == q3Index)
        {
            Debug.LogError("Q indices must be distinct.");
            return false;
        }
        if (p1Index < 0 || p2Index < 0 || p3Index < 0 ||
            q1Index < 0 || q2Index < 0 || q3Index < 0 ||
            p1Index >= P.Count || p2Index >= P.Count || p3Index >= P.Count ||
            q1Index >= Q.Count || q2Index >= Q.Count || q3Index >= Q.Count)
        {
            Debug.LogError("Index out of range.");
            return false;
        }
        return true;
    }

    static ONB BuildONB(Vector3 a, Vector3 b, Vector3 c)
    {
        // e1 = normalize(b-a)
        Vector3 v1 = b - a;
        float n1 = v1.magnitude;
        if (n1 < 1e-8f) return new ONB { valid = false };
        Vector3 e1 = v1 / n1;

        // e2: (c-a)'yı e1'e dikleştir, normalize et
        Vector3 v2 = c - a;
        Vector3 v2p = v2 - Vector3.Dot(v2, e1) * e1;
        float n2 = v2p.magnitude;
        if (n2 < 1e-8f) return new ONB { valid = false };
        Vector3 e2 = v2p / n2;

        // e3 = e1 x e2
        Vector3 e3 = Vector3.Cross(e1, e2);
        float n3 = e3.magnitude;
        if (n3 < 1e-8f) return new ONB { valid = false };
        e3 /= n3;

        Matrix4x4 B = Matrix4x4.identity;
        B.SetColumn(0, new Vector4(e1.x, e1.y, e1.z, 0f));
        B.SetColumn(1, new Vector4(e2.x, e2.y, e2.z, 0f));
        B.SetColumn(2, new Vector4(e3.x, e3.y, e3.z, 0f));
        B.SetColumn(3, new Vector4(0, 0, 0, 1f));

        return new ONB { e1 = e1, e2 = e2, e3 = e3, B = B, valid = true };
    }

    static void DrawBasis(Vector3 origin, ONB onb, float len)
    {
        // X'=e1 (kırmızı), Y'=e2 (yeşil), Z'=e3 (mavi)
        Gizmos.color = Color.red;
        Gizmos.DrawLine(origin, origin + onb.e1 * len);
        Gizmos.color = Color.green;
        Gizmos.DrawLine(origin, origin + onb.e2 * len);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(origin, origin + onb.e3 * len);
    }

    static string Pretty(Matrix4x4 M)
    {
        return
            $"{M[0,0]:F6}\t{M[0,1]:F6}\t{M[0,2]:F6}\t{M[0,3]:F6}\n" +
            $"{M[1,0]:F6}\t{M[1,1]:F6}\t{M[1,2]:F6}\t{M[1,3]:F6}\n" +
            $"{M[2,0]:F6}\t{M[2,1]:F6}\t{M[2,2]:F6}\t{M[2,3]:F6}\n" +
            $"{M[3,0]:F6}\t{M[3,1]:F6}\t{M[3,2]:F6}\t{M[3,3]:F6}";
    }

    [ContextMenu("List approximate matches (eps=1e-6)")]
public void ListApproxMatches()
{
    if (P == null || Q == null) { Debug.LogError("Load files first."); return; }

    const float eps = 1e-6f;
    var matches = FindApproxMatches(P, Q, eps);
    Debug.Log($"Approx matches (|P∩Q|) = {matches.Count} with eps={eps}");
    int shown = 0;
    foreach (var (ip, iq) in matches)
    {
        Debug.Log($"P[{ip}] ~ Q[{iq}]  =>  {P[ip]}");
        if (++shown >= 10) { Debug.Log("... (truncated)"); break; }
    }
}

private static List<(int ip, int iq)> FindApproxMatches(List<Vector3> P, List<Vector3> Q, float eps)
{
    var result = new List<(int ip, int iq)>();
    float eps2 = eps * eps;
    // Basit ama yeterli: her P noktasını Q ile karşılaştır
    for (int i = 0; i < P.Count; i++)
    {
        Vector3 p = P[i];
        for (int j = 0; j < Q.Count; j++)
        {
            if ((Q[j] - p).sqrMagnitude <= eps2)
            {
                result.Add((i, j));
                break; // bir P için bir Q yeter
            }
        }
    }
    return result;
}
[ContextMenu("Auto-pick 3 matching indices")]
public void AutoPick3()
{
    if (P == null || Q == null) { Debug.LogError("Load files first."); return; }

    const float eps = 1e-6f;
    if (TryAutoPick3MatchingIndices(P, Q, eps,
        out (int ip, int iq) a,
        out (int ip, int iq) b,
        out (int ip, int iq) c))
    {
        p1Index = a.ip; q1Index = a.iq;
        p2Index = b.ip; q2Index = b.iq;
        p3Index = c.ip; q3Index = c.iq;

        Debug.Log($"AutoPick success: P[{p1Index}],P[{p2Index}],P[{p3Index}]  <->  Q[{q1Index}],Q[{q2Index}],Q[{q3Index}]");
    }
    else
    {
        Debug.LogError("Could not auto-pick 3 non-collinear matching points. Check files or eps.");
    }
}

private static bool TryAutoPick3MatchingIndices(
    List<Vector3> P, List<Vector3> Q, float eps,
    out (int ip, int iq) a,
    out (int ip, int iq) b,
    out (int ip, int iq) c)
{
    a = b = c = (-1, -1);
    var matches = FindApproxMatches(P, Q, eps);
    if (matches.Count < 3) return false;

    // Üç farklı P indeksi ve üç farklı Q indeksi seç, kollineer olmasın
    for (int i = 0; i < matches.Count; i++)
    {
        for (int j = i + 1; j < matches.Count; j++)
        {
            if (matches[j].ip == matches[i].ip || matches[j].iq == matches[i].iq) continue;
            for (int k = j + 1; k < matches.Count; k++)
            {
                if (matches[k].ip == matches[i].ip || matches[k].ip == matches[j].ip) continue;
                if (matches[k].iq == matches[i].iq || matches[k].iq == matches[j].iq) continue;

                Vector3 p1 = P[matches[i].ip], p2 = P[matches[j].ip], p3 = P[matches[k].ip];
                if (!IsCollinear(p1, p2, p3))
                {
                    a = matches[i]; b = matches[j]; c = matches[k];
                    return true;
                }
            }
        }
    }
    return false;
}

private static bool IsCollinear(Vector3 a, Vector3 b, Vector3 c, float tol = 1e-8f)
{
    Vector3 v1 = b - a;
    Vector3 v2 = c - a;
    Vector3 cross = Vector3.Cross(v1, v2);
    return cross.sqrMagnitude < tol;
}

}
