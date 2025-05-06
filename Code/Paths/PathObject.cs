using UnityEngine;

[CreateAssetMenu(fileName = "Path", menuName = "PathObject")]
public class PathObject : ScriptableObject
{
    public string name;
    public float pathDirectness;
    public float pathDifficulty;
    public PathNodeLogic[] nodeLogic;
    public int maxAmountInWorld;
    public float spawnProbability;
    public float pathSmoothness;
}
