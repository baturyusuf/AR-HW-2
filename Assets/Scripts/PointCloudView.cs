using UnityEngine;

public class PointCloudView : MonoBehaviour
{
    [SerializeField] GameObject pointPrefab; // küçük sphere (0.01–0.03)
    public void ShowPoints(Vector3[] pts)
    {
        Clear();
        foreach (var p in pts)
        {
            var go = Instantiate(pointPrefab, p, Quaternion.identity, transform);
        }
    }
    public void Clear()
    {
        for (int i = transform.childCount-1; i>=0; i--) Destroy(transform.GetChild(i).gameObject);
    }
}
