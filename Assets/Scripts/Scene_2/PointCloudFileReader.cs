using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public static class PointCloudFileReader
{
    /// <summary>
    /// Format:
    /// num_pts
    /// x1 x2 ... xn
    /// y1 y2 ... yn
    /// z1 z2 ... zn
    /// </summary>
    public static List<Vector3> LoadFromString(string content)
    {
        var ci = CultureInfo.InvariantCulture;
        string[] rawLines = content.Replace("\r", "").Split('\n');
        var lines = new List<string>();
        foreach (var ln in rawLines)
        {
            var s = ln.Trim();
            if (s.Length > 0) lines.Add(s);
        }
        if (lines.Count < 4)
            throw new Exception("File format error: need 4 lines (num_pts + 3 coordinate lines).");

        int n = int.Parse(lines[0].Split(new[]{' ','\t'}, StringSplitOptions.RemoveEmptyEntries)[0], ci);

        float[] xs = ParseFloatArray(lines[1], n, ci);
        float[] ys = ParseFloatArray(lines[2], n, ci);
        float[] zs = ParseFloatArray(lines[3], n, ci);

        var pts = new List<Vector3>(n);
        for (int i = 0; i < n; i++)
            pts.Add(new Vector3(xs[i], ys[i], zs[i]));
        return pts;
    }

    public static List<Vector3> LoadFromTextAsset(TextAsset ta)
    {
        if (ta == null) throw new Exception("TextAsset is null.");
        return LoadFromString(ta.text);
    }

    public static List<Vector3> LoadFromPath(string absoluteOrStreamingPath)
    {
        string path = absoluteOrStreamingPath;
        // Eğer sadece dosya adı verdiyse, StreamingAssets içinde ara:
        if (!File.Exists(path))
        {
            string tryStreaming = Path.Combine(Application.streamingAssetsPath, absoluteOrStreamingPath);
            if (File.Exists(tryStreaming)) path = tryStreaming;
        }
        if (!File.Exists(path)) throw new Exception($"File not found: {absoluteOrStreamingPath}");
        var content = File.ReadAllText(path);
        return LoadFromString(content);
    }

    private static float[] ParseFloatArray(string line, int expectedCount, CultureInfo ci)
    {
        var toks = line.Split(new[]{' ','\t'}, StringSplitOptions.RemoveEmptyEntries);
        if (toks.Length != expectedCount)
            throw new Exception($"Line has {toks.Length} numbers, expected {expectedCount}.");
        var arr = new float[expectedCount];
        for (int i = 0; i < expectedCount; i++)
            arr[i] = float.Parse(toks[i], ci);
        return arr;
    }
}
