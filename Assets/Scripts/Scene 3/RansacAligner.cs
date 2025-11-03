using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

/// Kayıt metodu: Rigid (s=1) veya Similarity (s serbest). Q -> P dönüşümünü çözer.
public enum RegistrationMethod { Rigid, Similarity }

public struct AlignResult
{
    public bool success;
    public Matrix4x4 R;    // salt rotasyon matrisi
    public Quaternion q;   // rotasyon
    public Vector3 T;      // translasyon
    public float s;        // ölçek (Rigid için 1)
    public int inlierCount;
    public List<(int qi, int pi)> inlierMatches; // q index -> p index
}

public static class RansacAligner
{
    /// Ana RANSAC akışı. Q kümesini P'ye hizalar (Q->P).
    public static AlignResult AlignRansac(Vector3[] P, Vector3[] Q,
                                          RegistrationMethod method,
                                          int maxIters = 200,
                                          float inlierThresh = 0.02f,
                                          int randomSeed = 1234)
    {
        if (P == null || Q == null || P.Length < 3 || Q.Length < 3)
            return new AlignResult { success = false };

        Random.InitState(randomSeed);

        AlignResult best = new AlignResult { success = false, inlierCount = -1 };

        // RANSAC: 3 nokta eşleşmesi hipotezle (Q'dan 3, P'den 3) + 3! permütasyon
        for (int it = 0; it < maxIters; it++)
        {
            if (!TryPickDistinct(Q.Length, 3, out var qIdx) ||
                !TryPickDistinct(P.Length, 3, out var pIdx))
                continue;

            var qTriplet = new Vector3[] { Q[qIdx[0]], Q[qIdx[1]], Q[qIdx[2]] };
            var pTriplet = new Vector3[] { P[pIdx[0]], P[pIdx[1]], P[pIdx[2]] };

            if (IsCollinear(qTriplet) || IsCollinear(pTriplet))
                continue;

            // 3! permütasyon dene (qTriplet sıra sabit, pTriplet'i permütasyonla eşleştir)
            int[][] perms = new int[][]
            {
                new[]{0,1,2}, new[]{0,2,1},
                new[]{1,0,2}, new[]{1,2,0},
                new[]{2,0,1}, new[]{2,1,0}
            };

            foreach (var perm in perms)
            {
                Vector3[] qS = { qTriplet[0], qTriplet[1], qTriplet[2] };
                Vector3[] pS = { pTriplet[perm[0]], pTriplet[perm[1]], pTriplet[perm[2]] };

                var hypo = SolveClosedForm(pS, qS, method); // Q->P çöz

                if (!hypo.success) continue;

                // İnlier say
                var inliers = new List<(int qi, int pi)>(Mathf.Min(P.Length, Q.Length));
                int count = CountInliers(P, Q, hypo, inlierThresh, inliers);

                if (count > best.inlierCount)
                {
                    best = hypo;
                    best.inlierCount = count;
                    best.inlierMatches = inliers;
                    best.success = true;
                }
            }
        }

        if (!best.success || best.inlierCount < 3)
            return new AlignResult { success = false };

        // Refine: inlierlarla yeniden çöz + 1 ICP benzeri iyileştirme
        var refined = RefineWithInliers(P, Q, best, method, inlierThresh);
        return refined.success ? refined : best;
    }

    // --- İnliers sayımı: Q'yu dönüştürüp P'de en yakını bul ---
    private static int CountInliers(Vector3[] P, Vector3[] Q, AlignResult tr,
                                    float thresh, List<(int qi, int pi)> outPairs)
    {
        int count = 0;
        for (int qi = 0; qi < Q.Length; qi++)
        {
            var qT = TransformPoint(tr, Q[qi]);
            int bestPi = -1;
            float bestD2 = float.MaxValue;
            for (int pi = 0; pi < P.Length; pi++)
            {
                float d2 = (P[pi] - qT).sqrMagnitude;
                if (d2 < bestD2) { bestD2 = d2; bestPi = pi; }
            }
            if (bestD2 <= thresh * thresh)
            {
                count++;
                outPairs?.Add((qi, bestPi));
            }
        }
        return count;
    }

    private static AlignResult RefineWithInliers(Vector3[] P, Vector3[] Q, AlignResult prev,
                                                 RegistrationMethod method, float thresh)
    {
        // 1) Mevcut eşleşmelerle çöz
        var pairs = prev.inlierMatches;
        if (pairs == null || pairs.Count < 3)
            return prev;

        var pList = new List<Vector3>(pairs.Count);
        var qList = new List<Vector3>(pairs.Count);
        foreach (var (qi, pi) in pairs) { qList.Add(Q[qi]); pList.Add(P[pi]); }

        var step1 = SolveClosedForm(pList.ToArray(), qList.ToArray(), method);
        if (!step1.success) return prev;

        // 2) Q'ları step1 ile dönüştür, yeniden en yakın komşu eşleştir, tekrar çöz
        var newPairs = new List<(int qi, int pi)>();
        CountInliers(P, Q, step1, thresh, newPairs);

        if (newPairs.Count < 3) return step1;

        pList.Clear(); qList.Clear();
        foreach (var (qi, pi) in newPairs) { qList.Add(Q[qi]); pList.Add(P[pi]); }
        var step2 = SolveClosedForm(pList.ToArray(), qList.ToArray(), method);
        if (!step2.success) return step1;

        step2.inlierCount = newPairs.Count;
        step2.inlierMatches = newPairs;
        step2.success = true;
        return step2;
    }

    // Q->P kapalı form çözümü (Horn yöntemi - quaternion), Similarity için ölçek eklenir
    public static AlignResult SolveClosedForm(Vector3[] P, Vector3[] Q, RegistrationMethod method)
    {
        if (P.Length != Q.Length || P.Length < 3)
            return new AlignResult { success = false };

        // 1) Merkezler
        var meanP = Mean(P);
        var meanQ = Mean(Q);
        var Pc = Center(P, meanP);
        var Qc = Center(Q, meanQ);

        // 2) Çapraz kovaryans S = sum( p' * q'^T )  (Q -> P)
        double Sxx=0, Sxy=0, Sxz=0, Syx=0, Syy=0, Syz=0, Szx=0, Szy=0, Szz=0;
        for (int i = 0; i < P.Length; i++)
        {
            var p = Pc[i]; var qv3 = Qc[i]; // <<< isim değişti
            Sxx += p.x * qv3.x; Sxy += p.x * qv3.y; Sxz += p.x * qv3.z;
            Syx += p.y * qv3.x; Syy += p.y * qv3.y; Syz += p.y * qv3.z;
            Szx += p.z * qv3.x; Szy += p.z * qv3.y; Szz += p.z * qv3.z;
        }


        // 3) Horn'un 4x4 simetrik K matrisi
        double traceS = Sxx + Syy + Szz;
        double[,] K = new double[4, 4];
        K[0,0] = traceS;
        K[0,1] = Syz - Szy;
        K[0,2] = Szx - Sxz;
        K[0,3] = Sxy - Syx;

        K[1,0] = K[0,1];
        K[1,1] = Sxx - Syy - Szz;
        K[1,2] = Sxy + Syx;
        K[1,3] = Szx + Sxz;

        K[2,0] = K[0,2];
        K[2,1] = K[1,2];
        K[2,2] = -Sxx + Syy - Szz;
        K[2,3] = Syz + Szy;

        K[3,0] = K[0,3];
        K[3,1] = K[1,3];
        K[3,2] = K[2,3];
        K[3,3] = -Sxx - Syy + Szz;

        // 4) En büyük özdeğere karşılık gelen özvektörü (q = [w,x,y,z]) güç iterasyonu ile bul
        double[] qv = PowerIterationSymmetric(K, 50);
        var q = new Quaternion((float)qv[1], (float)qv[2], (float)qv[3], (float)qv[0]);
        q.Normalize();

        // 5) Rotasyon ve ölçek
        float s = 1f;
        if (method == RegistrationMethod.Similarity)
        {
            // s = sum( q' · (R p') ) / sum( ||p'||^2 ), burada R, Q->P yönünde kullanılacak
            // Dikkat: biz Q->P çözüyoruz. Ölçek, P ~ s*R*Q + T olacak şekilde.
            double num = 0.0, den = 0.0;
            for (int i = 0; i < P.Length; i++)
            {
                Vector3 Rp = q * Qc[i];      // R * Q'
                num += Vector3.Dot(Pc[i], Rp);
                den += Vector3.Dot(Qc[i], Qc[i]);
            }
            s = (den > 1e-12) ? (float)(num / den) : 1f;
        }

        // 6) Translasyon: Pmean = s*R*Qmean + T  =>  T = Pmean - s*R*Qmean
        var RQmean = q * meanQ;
        var T = meanP - s * RQmean;

        // R matrisine ihtiyaç duyanlar için (salt rotasyon)
        var Rm = Matrix4x4.Rotate(q);

        return new AlignResult
        {
            success = true,
            R = Rm,
            q = q,
            T = T,
            s = s
        };
    }

    // Yardımcılar
    private static Vector3 Mean(Vector3[] A)
    {
        Vector3 m = Vector3.zero;
        foreach (var v in A) m += v;
        return m / A.Length;
    }

    private static Vector3[] Center(Vector3[] A, Vector3 mean)
    {
        var arr = new Vector3[A.Length];
        for (int i = 0; i < A.Length; i++) arr[i] = A[i] - mean;
        return arr;
    }

    private static bool TryPickDistinct(int n, int k, out int[] idx)
    {
        idx = new int[k];
        if (n < k) return false;
        // basit örnekleme
        var used = new HashSet<int>();
        int t = 0, guard = 0;
        while (t < k && guard < 1000)
        {
            int r = Random.Range(0, n);
            if (used.Add(r)) idx[t++] = r;
            guard++;
        }
        return t == k;
    }

    private static bool IsCollinear(Vector3[] tri)
    {
        var a = tri[1] - tri[0];
        var b = tri[2] - tri[0];
        return Vector3.Cross(a, b).magnitude < 1e-6f;
    }

    private static double[] PowerIterationSymmetric(double[,] M, int iters)
    {
        // 4x4 simetrik
        double[] v = new double[] { 1, 0, 0, 0 };
        Normalize(v);
        for (int i = 0; i < iters; i++)
        {
            double[] w = new double[4];
            for (int r = 0; r < 4; r++)
            {
                w[r] = 0;
                for (int c = 0; c < 4; c++) w[r] += M[r, c] * v[c];
            }
            v = w;
            Normalize(v);
        }
        return v;
    }

    private static void Normalize(double[] v)
    {
        double s = 0; foreach (var x in v) s += x * x;
        s = Math.Sqrt(s);
        if (s < 1e-12) return;
        for (int i = 0; i < v.Length; i++) v[i] /= s;
    }

    public static Vector3 TransformPoint(AlignResult tr, Vector3 qPoint)
    {
        return tr.s * (tr.q * qPoint) + tr.T;
    }
}
