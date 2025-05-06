using System.Collections.Generic;
using UnityEngine;

public class Vector2Comparer : IEqualityComparer<Vector2>
{
    private readonly float _tolerance;

    public Vector2Comparer(float tolerance)
    {
        _tolerance = tolerance;
    }

    public bool Equals(Vector2 a, Vector2 b)
    {
        return Vector2.Distance(a, b) < _tolerance;
    }

    public int GetHashCode(Vector2 position)
    {
        return Mathf.RoundToInt(position.x / _tolerance).GetHashCode() ^
               Mathf.RoundToInt(position.y / _tolerance).GetHashCode();
    }
}
public static class Vector2Extensions
{
    public static bool ContainsPosition(this List<Vector2> list, Vector2 position, float tolerance = 0.1f)
    {
        foreach (Vector2 item in list)
        {
            if (Vector2.Distance(item, position) <= tolerance)
                return true;
        }
        return false;
    }
    public static bool TryFindSimilar(this List<Vector2> list, Vector2 target, float tolerance, out int index)
    {
        for (int i = 0; i < list.Count; i++)
        {
            if (Vector2.Distance(list[i], target) <= tolerance)
            {
                index = i;
                return true;
            }
        }

        index = -1;
        return false;
    }
    public static bool Contains(this List<Vector2> list, Vector2 target, Vector2Comparer comparer)
    {
        foreach (Vector2 item in list)
        {
            if (comparer.Equals(item, target))
                return true;
        }
        return false;
    }
    public static bool TryGetValueSimilar<T>(this Dictionary<Vector2, T> dict, Vector2 key, float tolerance, out T value)
    {
        foreach (var pair in dict)
        {
            if (Vector2.Distance(pair.Key, key) <= tolerance)
            {
                value = pair.Value;
                return true;
            }
        }

        value = default;
        return false;
    }
}