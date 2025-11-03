using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public static class FilePointReader
{
    // Format:
    // num_pts
    // x1 x2 … xn
    // y1 y2 … yn
    // z1 z2 … zn
    public static Vector3[] LoadPointFile(string path)
    {
        if (!File.Exists(path))
            throw new FileNotFoundException("File not found", path);

        var lines = File.ReadAllLines(path);
        if (lines.Length < 4)
            throw new Exception("Invalid file: expected at least 4 lines");

        int n = int.Parse(lines[0].Trim());
        var xs = ParseFloatArray(lines[1], n);
        var ys = ParseFloatArray(lines[2], n);
        var zs = ParseFloatArray(lines[3], n);

        var pts = new Vector3[n];
        for (int i = 0; i < n; i++)
            pts[i] = new Vector3(xs[i], ys[i], zs[i]);
        return pts;
    }

    private static float[] ParseFloatArray(string line, int expected)
    {
        var sp = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
        if (sp.Length != expected)
            throw new Exception($"Invalid component count: expected {expected}, got {sp.Length}");
        var arr = new float[expected];
        for (int i = 0; i < expected; i++)
            arr[i] = float.Parse(sp[i], CultureInfo.InvariantCulture);
        return arr;
    }
}
