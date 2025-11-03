using System;
using System.Collections.Generic;
using UnityEngine;

public struct RigidResult
{
    public Matrix4x4 R;
    public Vector3 T;
    public float Scale;
    public int Inliers;
}

public static class Registration
{
    // -----------------------------
    // PUBLIC API
    // -----------------------------
    public static RigidResult RansacRegister(
        Vector3[] P, Vector3[] Q,
        bool withScale,
        int iters = 1000,
        float inlierThresh = 0.03f)
    {
        System.Random rng = new System.Random(123);
        int bestInliers = -1;
        Matrix4x4 bestR = Matrix4x4.identity;
        Vector3 bestT = Vector3.zero;
        float bestS = 1f;

        if (P == null || Q == null || P.Length < 3 || Q.Length < 3)
            return new RigidResult { R = Matrix4x4.identity, T = Vector3.zero, Scale = 1f, Inliers = 0 };

        for (int it = 0; it < iters; it++)
        {
            // --- 1) Q’dan 3 indeks seç ve dejenere üçlüleri ele ---
            int a = rng.Next(Q.Length), b = rng.Next(Q.Length), c = rng.Next(Q.Length);
            if (a == b || b == c || a == c) { it--; continue; }
            var qA = Q[a]; var qB = Q[b]; var qC = Q[c];
            if (IsDegenerateTriplet(qA, qB, qC)) { it--; continue; }

            // --- 2) P’de en yakın komşular ---
            var pA = Nearest(qA, P);
            var pB = Nearest(qB, P);
            var pC = Nearest(qC, P);

            Vector3[] P3 = new[] { pA, pB, pC };
            Vector3[] Q3 = new[] { qA, qB, qC };

            // --- 3) Hipotez ---
            (Matrix4x4 R, Vector3 T, float s) = withScale
                ? Umeyama(P3, Q3)          // s,R,T
                : KabschNoScale(P3, Q3);   // R,T (s=1)

            // mantıksız ölçekleri diskalifiye et (RANSAC içinde)
            if (withScale && (s < 0.2f || s > 5f)) continue;

            // --- 4) İnlier say ---
            int inl = 0;
            for (int i = 0; i < Q.Length; i++)
            {
                Vector3 q = Q[i];
                Vector3 qh = (Vector3)(R.MultiplyPoint3x4(q * s) + T);
                Vector3 pnn = Nearest(qh, P);
                if ((qh - pnn).magnitude < inlierThresh) inl++;
            }

            if (inl > bestInliers)
            {
                bestInliers = inl; bestR = R; bestT = T; bestS = s;
            }
        }

        // --- 5) Refit: en iyi hipotezle inlier eşleşmeleri oluştur, kapalı form tekrar çöz ---
        if (bestInliers > 0)
        {
            List<Vector3> Pin = new(); List<Vector3> Qin = new();
            for (int i = 0; i < Q.Length; i++)
            {
                Vector3 qh = (Vector3)(bestR.MultiplyPoint3x4(Q[i] * bestS) + bestT);
                Vector3 pnn = Nearest(qh, P);
                if ((qh - pnn).magnitude < inlierThresh)
                {
                    // EŞLEŞME: Q[i] ↔ Pnn
                    Qin.Add(Q[i]);
                    Pin.Add(pnn);
                }
            }
            if (Pin.Count >= 3)
            {
                (Matrix4x4 Rf, Vector3 Tf, float sf) = withScale
                    ? Umeyama(Pin, Qin)
                    : KabschNoScale(Pin, Qin);

                // son bir kıskaç
                if (withScale) sf = Mathf.Clamp(sf, 0.2f, 5f);

                bestR = Rf; bestT = Tf; bestS = sf; bestInliers = Pin.Count;
            }
        }

        return new RigidResult { R = bestR, T = bestT, Scale = bestS, Inliers = bestInliers };
    }

    public static (Matrix4x4 R, Vector3 T, float s) KabschNoScale(IList<Vector3> P, IList<Vector3> Q)
    {
        // P ≈ R*Q + T (ölçeksiz)
        return Umeyama(P, Q, forceUnitScale: true);
    }

    // P ≈ s * R * Q + T  (SVD olmadan: R için polar approx, s için dot-product oranı)
    public static (Matrix4x4 R, Vector3 T, float s) Umeyama(
        IList<Vector3> P, IList<Vector3> Q,
        bool forceUnitScale = false,
        float sClampMin = 1e-3f, float sClampMax = 1e+3f)
    {
        if (P.Count != Q.Count) throw new Exception("Umeyama: P ve Q aynı sayıda nokta içermeli.");

        Centroids(P, out Vector3 cP);
        Centroids(Q, out Vector3 cQ);

        Matrix4x4 Sigma = CrossCovariance(P, Q, cP, cQ); // 3x3
        Matrix4x4 R = PolarDecomposition(Sigma);
        ZeroTranslation(ref R);

        float s = 1f;
        if (!forceUnitScale)
        {
            // s = sum_i <p_i', R q_i'> / sum_i ||q_i'||^2
            float num = 0f, den = 0f;
            for (int i = 0; i < Q.Count; i++)
            {
                Vector3 qp = P[i] - cP;
                Vector3 qq = Q[i] - cQ;
                Vector3 Rqq = (Vector3)R.MultiplyVector(qq);
                num += Vector3.Dot(qp, Rqq);
                den += qq.sqrMagnitude;
            }
            if (den > 1e-12f) s = num / den;
            s = Mathf.Clamp(s, sClampMin, sClampMax);
        }

        Vector3 T = cP - (Vector3)(R.MultiplyPoint3x4(cQ * s));
        return (R, T, s);
    }

    // -----------------------------
    // PRIVATE HELPERS
    // -----------------------------
    static bool IsDegenerateTriplet(Vector3 a, Vector3 b, Vector3 c)
    {
        // neredeyse aynı veya neredeyse koliner üçlüleri ele
        if ((a - b).sqrMagnitude < 1e-6f) return true;
        if ((a - c).sqrMagnitude < 1e-6f) return true;
        if ((b - c).sqrMagnitude < 1e-6f) return true;
        Vector3 u = (b - a).normalized;
        Vector3 v = (c - a).normalized;
        float s = Vector3.Cross(u, v).magnitude; // sin(theta)
        return s < 0.05f; // ~<3°
    }

    static Vector3 Nearest(Vector3 x, Vector3[] P)
    {
        float best = float.MaxValue; int bi = 0;
        for (int i = 0; i < P.Length; i++)
        {
            float d = (x - P[i]).sqrMagnitude;
            if (d < best) { best = d; bi = i; }
        }
        return P[bi];
    }

    static void Centroids(IList<Vector3> A, out Vector3 c)
    {
        c = Vector3.zero;
        for (int i = 0; i < A.Count; i++) c += A[i];
        c /= Mathf.Max(1, A.Count);
    }

    static Matrix4x4 CrossCovariance(IList<Vector3> P, IList<Vector3> Q, Vector3 cP, Vector3 cQ)
    {
        Matrix4x4 H = Matrix4x4.zero;
        for (int i = 0; i < P.Count; i++)
        {
            Vector3 p = P[i] - cP; Vector3 q = Q[i] - cQ;
            H.m00 += p.x * q.x; H.m01 += p.x * q.y; H.m02 += p.x * q.z;
            H.m10 += p.y * q.x; H.m11 += p.y * q.y; H.m12 += p.y * q.z;
            H.m20 += p.z * q.x; H.m21 += p.z * q.y; H.m22 += p.z * q.z;
        }
        H.m33 = 1f;
        return H;
    }

    static Matrix4x4 Transpose(Matrix4x4 M)
    {
        Matrix4x4 T = Matrix4x4.identity;
        T.m00 = M.m00; T.m01 = M.m10; T.m02 = M.m20;
        T.m10 = M.m01; T.m11 = M.m11; T.m12 = M.m21;
        T.m20 = M.m02; T.m21 = M.m12; T.m22 = M.m22;
        return T;
    }

    static void ZeroTranslation(ref Matrix4x4 M)
    {
        M.m03 = 0f; M.m13 = 0f; M.m23 = 0f;
    }

    // Newton–Schulz ≈ polar; ardından ortonormalizasyon + det>0 düzeltmesi
    static Matrix4x4 PolarDecomposition(Matrix4x4 A)
    {
        Matrix4x4 X = A;
        for (int i = 0; i < 8; i++)
        {
            Matrix4x4 Xt = Transpose(X);
            Matrix4x4 XtX = Xt * X;
            Matrix4x4 threeI = MatScale(Matrix4x4.identity, 3f);
            Matrix4x4 term = MatSub(threeI, XtX);
            X = MatScale(X * term, 0.5f);
        }
        Orthonormalize(ref X);
        ZeroTranslation(ref X);
        X.m33 = 1f;
        return X;
    }

    static Matrix4x4 MatScale(Matrix4x4 A, float s)
    {
        Matrix4x4 M = A;
        M.m00 *= s; M.m01 *= s; M.m02 *= s; M.m03 *= s;
        M.m10 *= s; M.m11 *= s; M.m12 *= s; M.m13 *= s;
        M.m20 *= s; M.m21 *= s; M.m22 *= s; M.m23 *= s;
        M.m30 *= s; M.m31 *= s; M.m32 *= s; M.m33 *= s;
        return M;
    }

    static Matrix4x4 MatSub(Matrix4x4 A, Matrix4x4 B)
    {
        Matrix4x4 M = new Matrix4x4();
        M.m00 = A.m00 - B.m00; M.m01 = A.m01 - B.m01; M.m02 = A.m02 - B.m02; M.m03 = A.m03 - B.m03;
        M.m10 = A.m10 - B.m10; M.m11 = A.m11 - B.m11; M.m12 = A.m12 - B.m12; M.m13 = A.m13 - B.m13;
        M.m20 = A.m20 - B.m20; M.m21 = A.m21 - B.m21; M.m22 = A.m22 - B.m22; M.m23 = A.m23 - B.m23;
        M.m30 = A.m30 - B.m30; M.m31 = A.m31 - B.m31; M.m32 = A.m32 - B.m32; M.m33 = A.m33 - B.m33;
        return M;
    }

    static void Orthonormalize(ref Matrix4x4 M)
    {
        Vector3 c0 = new Vector3(M.m00, M.m10, M.m20);
        Vector3 c1 = new Vector3(M.m01, M.m11, M.m21);

        c0 = c0.normalized;
        c1 = (c1 - Vector3.Dot(c1, c0) * c0).normalized;
        Vector3 c2 = Vector3.Cross(c0, c1);

        // sağ ellilik
        float det = Vector3.Dot(c0, Vector3.Cross(c1, c2));
        if (det < 0f) c2 = -c2;

        M.m00 = c0.x; M.m10 = c0.y; M.m20 = c0.z;
        M.m01 = c1.x; M.m11 = c1.y; M.m21 = c1.z;
        M.m02 = c2.x; M.m12 = c2.y; M.m22 = c2.z;
    }
}
