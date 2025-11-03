using System;
using System.IO;
using System.Linq;
using System.Globalization;
using UnityEngine;

public static class PointCloudIO
{
    // /StreamingAssets altında ise path = Path.Combine(Application.streamingAssetsPath, "P.txt")
    public static Vector3[] LoadTxt(string path)
    {
        var lines = File.ReadAllLines(path)
                        .Where(l => !string.IsNullOrWhiteSpace(l))
                        .ToArray();

        int n = int.Parse(lines[0].Trim());
        float[] xs = ParseLine(lines[1], n);
        float[] ys = ParseLine(lines[2], n);
        float[] zs = ParseLine(lines[3], n);

        var pts = new Vector3[n];
        for (int i = 0; i < n; i++) pts[i] = new Vector3(xs[i], ys[i], zs[i]);
        return pts;
    }

    static float[] ParseLine(string line, int n)
    {
        var parts = line.Split(new[]{' ','\t'}, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != n) throw new Exception("Count mismatch");
        var arr = new float[n];
        for (int i = 0; i < n; i++) arr[i] = float.Parse(parts[i], CultureInfo.InvariantCulture);
        return arr;
    }
}
