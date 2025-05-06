
using UnityEngine;
using System.Collections.Generic;
using System;
using UnityEngine.UI;
[Serializable]
public class Path 
{
    
    public List<Vector2> nodes = new List<Vector2>();
    public List<NodeObject> nodeObjects = new List<NodeObject>();
    public Dictionary<NodeObject, int> placedNodes = new Dictionary<NodeObject, int>();
    public PathObject pathObject;
    public Dictionary<Vector2, NodeObject> nodeObjectsAtCrossPoints = new Dictionary<Vector2, NodeObject>(new Vector2Comparer(0.1f));
    public Path(List<Vector2> nodes, PathObject pathObject)
    {
        this.nodes = nodes;
        this.pathObject = pathObject;
    }
    public void GeneratePath()
    {
        // Process each node position in the path
        for (int i = 0; i < nodes.Count; i++)
        {
            // Skip if this node already has a node type assigned (e.g., it's an intersection)
            // Add this if you want to avoid overwriting intersection nodes

            // 1. Build list of eligible node types based on what's already placed
            List<PathNodeLogic> eligibleNodes = new List<PathNodeLogic>();
            foreach (PathNodeLogic nodeLogic in pathObject.nodeLogic)
            {
                bool prerequisitesMet = true;

                // If nodesInFront is defined, check if the prerequisites are met
                if (nodeLogic.nodesInFront != null && nodeLogic.nodesInFront.Length > 0)
                {
                    prerequisitesMet = false;
                    // Check if ANY of the prerequisite nodes exist in the path so far
                    foreach (NodeObject requiredNode in nodeLogic.nodesInFront)
                    {
                        if (nodeObjects.Contains(requiredNode))
                        {
                            prerequisitesMet = true;
                            break;
                        }
                    }
                }

                if (prerequisitesMet)
                {
                    eligibleNodes.Add(nodeLogic);
                }
            }

            // If no eligible nodes, use a fallback or skip
            if (eligibleNodes.Count == 0)
            {
                Debug.LogWarning($"No eligible nodes for position {i} in path {pathObject.name}");
                continue;
            }

            // 2. Proper weighted random selection
            NodeObject chosenNodeObject = GetWeightedRandomNode(eligibleNodes);

            // 3. Apply the chosen node
            if (chosenNodeObject != null)
            {
                nodeObjects.Add(chosenNodeObject);

                // Add the node to the placed counts dictionary if it doesn't exist
                if (!placedNodes.ContainsKey(chosenNodeObject))
                {
                    placedNodes[chosenNodeObject] = 0;
                }

                // Increment the count of this type of node
                placedNodes[chosenNodeObject]++;

                // Update the sprite if we can
                if (WorldLevelGenerator.Instance.vectorToNodeMap.TryGetValue(nodes[i], out NodeData nodeData) &&
                    nodeData.nodeGameObject != null)
                {
                    WorldLevelGenerator.Instance.vectorToNodeMap[nodes[i]].nodeObject = chosenNodeObject;
                    Debug.Log("assigning " + nodes[i] + " this nodeObject: " + chosenNodeObject);
                    Image image = nodeData.nodeGameObject.GetComponent<Image>();
                    if (image != null)
                    {
                        image.sprite = chosenNodeObject.sprite;
                    }
                    else
                    {
                        Debug.LogWarning($"SpriteRenderer missing for node at position {i}");
                    }
                }
                else
                {
                    Debug.LogWarning($"NodeData or GameObject missing for node at position {i}");
                }
            }
            else
            {
                Debug.LogError($"Failed to select a node for position {i} in path {pathObject.name}");
            }
        }
    }

    // Proper weighted random selection method
    private NodeObject GetWeightedRandomNode(List<PathNodeLogic> nodeLogics)
    {
        // Calculate total weight
        float totalWeight = 0;
        foreach (PathNodeLogic nodeLogic in nodeLogics)
        {
            if (nodeLogic != null && nodeLogic.relevantNode != null)
            {
                totalWeight += nodeLogic.spawnMultiplier;
            }
        }

        // Guard against zero total weight
        if (totalWeight <= 0)
        {
            Debug.LogWarning("Total weight is zero or negative");
            return nodeLogics.Count > 0 ? nodeLogics[0].relevantNode : null;
        }

        // Generate random value between 0 and total weight
        float random = (float)(WorldLevelGenerator.Instance.prng.NextDouble() * totalWeight);

        // Find the selected node
        float currentWeight = 0;
        foreach (PathNodeLogic nodeLogic in nodeLogics)
        {
            if (nodeLogic == null || nodeLogic.relevantNode == null)
                continue;

            currentWeight += nodeLogic.spawnMultiplier;

            // Found our selection
            if (random <= currentWeight)
            {
                return nodeLogic.relevantNode;
            }
        }

        // Fallback
        return nodeLogics.Count > 0 ? nodeLogics[0].relevantNode : null;
    }
    
   
}
