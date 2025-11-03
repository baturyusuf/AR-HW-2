using UnityEngine;

public class MovementLines : MonoBehaviour
{
    [SerializeField] Material lineMat;
    public void Show(Vector3[] fromQ, Vector3[] toQ)
    {
        Clear();
        for (int i=0;i<Mathf.Min(fromQ.Length, toQ.Length); i++)
        {
            var go = new GameObject("line_"+i);
            go.transform.SetParent(transform);
            var lr = go.AddComponent<LineRenderer>();
            lr.material = lineMat;
            lr.positionCount = 2;
            lr.widthMultiplier = 0.01f;
            lr.useWorldSpace = true;
            lr.SetPosition(0, fromQ[i]);
            lr.SetPosition(1, toQ[i]);
        }
    }
    public void Clear() { for (int i=transform.childCount-1;i>=0;i--) Destroy(transform.GetChild(i).gameObject); }
}
