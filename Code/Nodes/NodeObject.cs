using System;
using UnityEngine;
[CreateAssetMenu(fileName = "Node", menuName = "NodeObject")]
public class NodeObject : ScriptableObject
{
    public Sprite sprite;
    public float difficulty;
    public float treasure;
    [Header("Boss")]
    public float bossSpawnChance;
    public RelativeBossStatistics[] relativeBossStatistics;
}
[Serializable]
public class RelativeBossStatistics
{
    public Boss boss;
    public float baseSpawnProbability;
    public float difficultyMultiplier = 1;
}
