using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Procedural world level generator that creates a Slay the Spire-like map layout
/// </summary>
public class WorldLevelGenerator : MonoBehaviour
{
    #region Singleton and Core Properties
    
    public static WorldLevelGenerator Instance;
    public static float tolerance = 0.1f; // Tolerance for position matching in UI space
    
    // Dictionary to quickly access node data by position
    public Dictionary<Vector2, NodeData> vectorToNodeMap = new Dictionary<Vector2, NodeData>(new Vector2Comparer(tolerance));
    
    // Random number generator
    public System.Random prng;
    
    #endregion

    #region Difficulty Settings
    
    [Header("Difficulty Settings")]
    [SerializeField] private float worldLevelBaseDifficulty = 1.0f;
    [SerializeField] private float worldLevelDifficultyIncrease = 2.5f;
    [SerializeField] private int currentWorld = 0;
    [Tooltip("Controls how winding paths are (higher = more winding)")]
    public float indirectnessImpact = 18.0f;
    
    #endregion

    #region Debug Properties
    
    [Header("Debug")]
    public List<Vector2> pathConnectionDebugList = new List<Vector2>();
    public List<PathDebug> pathDebugs = new List<PathDebug>();
    public List<Vector2> initNodesDebug = new List<Vector2>();
    [SerializeField] private bool debugPositionsVisually;
    [SerializeField] private GameObject imageDebug;
    
    #endregion

    #region Path Properties
    
    [Header("Paths")]
    [SerializeField] private bool chosenPathObjectsIsStatic;
    [SerializeField] private PathObject[] possiblePaths;
    [SerializeField] private List<PathObject> chosenPathObjects = new List<PathObject>();
    [SerializeField] private List<Path> chosenPaths = new List<Path>();
    [SerializeField] private int minAmountOfPaths = 4;
    [SerializeField] private int maxAmountOfPaths = 4;
    
    // Dictionary to track path intersections
    private Dictionary<Vector2, List<Path>> pathsConnectionPointDictionary = 
        new Dictionary<Vector2, List<Path>>(new Vector2Comparer(tolerance));
    
    public List<Path> paths = new List<Path>();
    private int levelSeedOffset;
    
    #endregion

    #region UI Properties
    
    [Header("UI")]
    public RectTransform UI_Container;
    public RectTransform UI_Screen;
    [SerializeField] private UILine line;
    [SerializeField] private float lineWidth = 5.0f;
    
    #endregion

    #region Node Settings
    
    [Header("Node Settings")]
    [SerializeField] private GameObject nodePrefab;
    [SerializeField] private float nodeMinDistance = 45.0f;
    [SerializeField] private float nodeMaxDistance = 95.0f;
    [SerializeField] private int nodesToGenerate = 40;
    
    #endregion

    #region Spawn Area
    
    [Header("Spawn Area")]
    [SerializeField] private float circleSpawnRadius = 5.0f;
    [SerializeField] private Transform spawnCenter;
    
    #endregion

    #region Start/End Settings
    
    [Header("Start/End Settings")]
    public Vector2 startPosition = new Vector2(-300, 0);
    public Vector2 endPosition = new Vector2(300, 0);
    [SerializeField] private float specialNodeScale = 2.0f;
    
    #endregion

    #region Connection Visualization
    
    [Header("Connection Visualization")]
    [SerializeField] private bool showConnections = true;
    [SerializeField] private Color connectionColor = Color.white;
    [SerializeField] private float connectionWidth = 0.1f;
    
    #endregion

    #region Private Fields
    
    // Store references to nodes
    private Vector2 startNodePos;
    private Vector2 endNodePos;
    private Vector2 centre;
    private List<Vector2> initialNodes = new List<Vector2>();
    private List<UILine> connectionLines = new List<UILine>();
    
    #endregion

    #region Initialization

    private void Awake()
    {
        // Singleton setup
        if (Instance == null) Instance = this;
        else Destroy(this);

        // Reset collections to ensure clean state
        initialNodes = new List<Vector2>();
        vectorToNodeMap = new Dictionary<Vector2, NodeData>(new Vector2Comparer(tolerance));
    }
    
    private void Start()
    {
        // Initialize random number generator
        prng = new System.Random(ProceduralMain.Instance != null ? 
            ProceduralMain.Instance.seed : (int)System.DateTime.Now.Ticks);

        // Calculate proper radius based on start and end positions
        float distance = Vector2.Distance(startPosition, endPosition);
        circleSpawnRadius = Mathf.Max(100f, distance / 2) * 2;

        // Calculate center point
        centre = (startPosition + endPosition) / 2;

        Debug.Log($"Start: {startPosition}, End: {endPosition}, Centre: {centre}, Radius: {circleSpawnRadius}");

        GenerateMap();
    }

    #endregion

    #region Public Methods

    /// <summary>
    /// Resets and regenerates the world map
    /// </summary>
    public void ResetWorld()
    {
        levelSeedOffset = 0;
        currentWorld = 0;
        
        if (UI_Container != null) 
            Destroy(UI_Container.gameObject);
            
        initialNodes = new List<Vector2>();
        vectorToNodeMap = new Dictionary<Vector2, NodeData>(new Vector2Comparer(tolerance));
        
        // Reinitialize random number generator
        prng = new System.Random(ProceduralMain.Instance != null ? 
            ProceduralMain.Instance.seed : (int)System.DateTime.Now.Ticks);

        // Recalculate radius and center
        float distance = Vector2.Distance(startPosition, endPosition);
        circleSpawnRadius = Mathf.Max(100f, distance / 2) * 2;
        centre = (startPosition + endPosition) / 2;
        
        GenerateMap();
    }

    /// <summary>
    /// Gets the current world difficulty level
    /// </summary>
    public float GetBaseWorldDifficulty()
    {
        return worldLevelBaseDifficulty + currentWorld * worldLevelDifficultyIncrease;
    }

    /// <summary>
    /// Generate a new world map
    /// </summary>
    [ContextMenu("Generate Map")]
    public void GenerateMap()
    {
        currentWorld++;
        Debug.Log("Generating map...");
        
        // Clean up existing map
        if (UI_Container != null) 
            Destroy(UI_Container.gameObject);
            
        // Create new UI container
        GameObject UI_CONTAINER_GameObject = new GameObject("UI_CONTAINER");
        UI_Container = UI_CONTAINER_GameObject.AddComponent<RectTransform>();
        UI_Container.transform.parent = UI_Screen;
        UI_Container.transform.localPosition = Vector2.zero;
        UI_Container.transform.localScale = Vector3.one;
        
        // Clear all collections
        vectorToNodeMap.Clear();
        initialNodes.Clear();
        pathConnectionDebugList.Clear();
        pathsConnectionPointDictionary.Clear();
        connectionLines.Clear();
        
        if (!chosenPathObjectsIsStatic)
            chosenPathObjects.Clear();
            
        chosenPaths.Clear();
        paths.Clear();
        
        // Clear any existing data
        ClearExistingNodesAndConnections();

        // Select paths to use
        if (!chosenPathObjectsIsStatic)
            chosenPathObjects = GeneratePathList(possiblePaths);
            
        if (chosenPathObjects.Count == 0)
        {
            Debug.LogError("Failed to generate any paths!");
            return;
        }
        
        // Create start and end nodes
        CreateStartAndEndNodes();

        // Generate intermediate nodes
        SpawnNodesWithConnections();

        // Track all nodes that are part of paths
        List<Vector2> mergedNodeList = new List<Vector2>();

        Debug.Log($"Creating {chosenPathObjects.Count} paths between {startNodePos} and {endNodePos}");

        // Generate each path
        foreach (var pathObject in chosenPathObjects)
        {
            // Find a path between start and end
            List<Vector2> pathNodes = FindPath(startNodePos, endNodePos, pathObject.pathDirectness, pathObject.pathSmoothness);

            if (pathNodes.Count < 2)
            {
                Debug.LogWarning($"Path {pathObject.name} failed to generate a valid path!");
                continue;
            }

            // Create debug object for visualization
            pathDebugs.Add(new PathDebug(pathNodes));

            // Create path object and visualize it
            Path newPath = new Path(pathNodes, pathObject);
            paths.Add(newPath);
            VisualizePath(newPath);

            // Add all nodes from this path to our merged list
            mergedNodeList.AddRange(pathNodes);

            // Add to chosen paths list
            chosenPaths.Add(newPath);

            Debug.Log($"Created path with {pathNodes.Count} nodes using {pathObject.name}");
        }

        // Clear nodes list as we've created all necessary nodes
        initialNodes.Clear();

        // Process intersections to find where paths cross
        List<Vector2> relevantList = new List<Vector2>();
        foreach (Vector2 nodePos in mergedNodeList)
        {
            if (relevantList.ContainsPosition(nodePos))
            {
                pathConnectionDebugList.Add(nodePos);
            }
            else
            {
                relevantList.Add(nodePos);
            }
        }
        
        // Sort paths by directness
        chosenPaths = SortPathListAfterDirectness(chosenPaths);
        
        // Generate the path content for each path
        foreach (Path path in chosenPaths)
        {
            path.GeneratePath();
        }

        // Generate seeds for node objects
        GenerateSeedsForNodeObjects();
        
        // Update node neighbors
        GenerateNewNeighbours();
        
        // Notify navigator that map is ready
        UIWorldLevelGeneratorNavigator.Instance.MapGenerationFinished();
        Debug.Log("Map generation complete!");
    }

    #endregion

    #region Node Generation Methods
    
    /// <summary>
    /// Creates the start and end nodes for the map
    /// </summary>
    private void CreateStartAndEndNodes()
    {
        // Create start node
        GameObject startNodeObj = Instantiate(nodePrefab, startPosition, Quaternion.identity, transform);
        startNodeObj.transform.localScale *= specialNodeScale;
        startNodeObj.name = "StartNode";
        NodeData startNode = new NodeData(startPosition);
        startNode.nodeGameObject = startNodeObj;
        startNodeObj.transform.parent = UI_Container;
        startNodeObj.transform.localPosition = startPosition;
        startNodeObj.GetComponent<Button>().onClick.AddListener(() =>
        {
            UIWorldLevelGeneratorNavigator.Instance.GetUIInput(startPosition);
        });
        
        // Create end node
        GameObject endNodeObj = Instantiate(nodePrefab, endPosition, Quaternion.identity, transform);
        endNodeObj.transform.localScale *= specialNodeScale;
        endNodeObj.name = "EndNode";
        NodeData endNode = new NodeData(endPosition);
        endNode.nodeGameObject = endNodeObj;
        endNodeObj.transform.parent = UI_Container;
        endNodeObj.transform.localPosition = endPosition;
        endNodeObj.GetComponent<Button>().onClick.AddListener(() =>
        {
            UIWorldLevelGeneratorNavigator.Instance.GetUIInput(endPosition);
        });
        
        // Store positions
        startNodePos = startPosition;
        endNodePos = endPosition;

        // Add to dictionaries and lists
        vectorToNodeMap[startPosition] = startNode;
        vectorToNodeMap[endPosition] = endNode;
        initialNodes.Add(startPosition);
        initialNodes.Add(endPosition);
    }

    /// <summary>
    /// Generate random seeds for node content
    /// </summary>
    private void GenerateSeedsForNodeObjects()
    {
        foreach (KeyValuePair<Vector2, NodeData> kvp in vectorToNodeMap)
        {
            kvp.Value.localSeed = levelSeedOffset + (ProceduralMain.Instance.seed * ProceduralMain.Instance.seedImpactOnLevelChange);
            levelSeedOffset++;
        }
    }

    /// <summary>
    /// Set up neighbor connections between nodes
    /// </summary>
    private void GenerateNewNeighbours()
    {
        // Clear existing neighbors
        foreach (KeyValuePair<Vector2, NodeData> kvp in vectorToNodeMap)
        {
            kvp.Value.neighbors.Clear();
        }
        
        // Set up new connections based on paths
        foreach (Path path in chosenPaths)
        {
            for (int i = 0; i < path.nodes.Count - 1; i++)
            {
                vectorToNodeMap[path.nodes[i]].neighbors.Add(path.nodes[i + 1]);
            }
        }
    }

    /// <summary>
    /// Generate intermediate nodes in the map
    /// </summary>
    private void SpawnNodesWithConnections()
    {
        centre = (startPosition + endPosition) / 2;
        Vector2 pathDirection = (endPosition - startPosition).normalized;
        Vector2 perpendicular = new Vector2(-pathDirection.y, pathDirection.x); // Perpendicular direction
        float pathLength = Vector2.Distance(startPosition, endPosition);

        // Calculate corridor width - narrower than a full circle
        float corridorWidth = pathLength * 0.4f; // Control corridor width

        Debug.Log($"Path length: {pathLength}, Corridor width: {corridorWidth}");

        int nodesPlaced = 0;
        int maxAttempts = nodesToGenerate * 10;
        int attemptCounter = 0;
        int failedConnections = 0;

        // Generate nodes in a corridor formation
        while (nodesPlaced < nodesToGenerate && attemptCounter < maxAttempts)
        {
            attemptCounter++;

            // Position along the path (from 0.1 to 0.9 to avoid clustering at endpoints)
            float pathProgress = 0.1f + (float)prng.NextDouble() * 0.8f;

            // Distance from the center line (narrower near endpoints, wider in middle)
            float maxOffset = corridorWidth * Mathf.Sin(pathProgress * Mathf.PI);
            float offset = ((float)prng.NextDouble() * 2 - 1) * maxOffset;

            // Calculate position in corridor
            Vector2 pathPoint = startPosition + pathDirection * (pathLength * pathProgress);
            Vector2 potentialPosition = pathPoint + perpendicular * offset;

            bool isValid = true;
            List<Vector2> potentialNeighbors = new List<Vector2>();

            // Check against existing nodes for minimum distance & find neighbors
            foreach (Vector2 existingNode in initialNodes)
            {
                float distance = Vector2.Distance(potentialPosition, existingNode);

                if (distance < nodeMinDistance)
                {
                    isValid = false;
                    break;
                }

                if (distance <= nodeMaxDistance)
                {
                    potentialNeighbors.Add(existingNode);
                }
            }

            // Force connection if needed but respect minimum distance
            if (potentialNeighbors.Count == 0 && initialNodes.Count > 2)
            {
                // Find the closest node
                Vector2 closestNode = initialNodes[0];
                float closestDist = float.MaxValue;

                foreach (Vector2 node in initialNodes)
                {
                    float dist = Vector2.Distance(potentialPosition, node);
                    if (dist < closestDist)
                    {
                        closestDist = dist;
                        closestNode = node;
                    }
                }

                // Force connection if within acceptable range
                if (closestDist < nodeMaxDistance * 1.25f) // Slightly relaxed max distance
                {
                    potentialNeighbors.Add(closestNode);
                    isValid = true;
                }
                else
                {
                    // Skip this position - too isolated
                    failedConnections++;
                    continue;
                }
            }

            // Create the node if position is valid
            if (isValid)
            {
                NodeData newNode = new NodeData(potentialPosition);
                if (!vectorToNodeMap.ContainsKey(potentialPosition))
                    vectorToNodeMap.Add(potentialPosition, newNode);
                else 
                    vectorToNodeMap[potentialPosition] = newNode;
                    
                initialNodes.Add(potentialPosition);
                initNodesDebug.Add(potentialPosition);

                // Visualize if debugging is enabled
                if (debugPositionsVisually)
                {
                    GameObject iDebug = Instantiate(imageDebug, Vector2.zero, Quaternion.identity);
                    iDebug.transform.parent = UI_Screen;
                    iDebug.transform.localPosition = potentialPosition;
                }

                // Create bidirectional connections
                foreach (Vector2 neighbor in potentialNeighbors)
                {
                    newNode.neighbors.Add(neighbor);
                    vectorToNodeMap[neighbor].neighbors.Add(newNode.position);
                }

                nodesPlaced++;
                failedConnections = 0;
            }
            else
            {
                failedConnections++;
            }

            // Adaptive placement strategy - if we're struggling to place nodes
            if (failedConnections > 20)
            {
                // Temporarily reduce minimum distance requirement
                float originalMinDistance = nodeMinDistance;
                nodeMinDistance *= 0.8f;
                Debug.LogWarning($"Temporarily reducing min distance to {nodeMinDistance} to place more nodes");

                // Reset after a few attempts
                if (failedConnections > 30)
                {
                    nodeMinDistance = originalMinDistance;
                    failedConnections = 0;
                }
            }
        }

        // Ensure connectivity of all nodes (not just start/end)
        EnsureNetworkConnectivity();

        // Log connectivity information
        int totalConnections = 0;
        foreach (Vector2 node in initialNodes)
        {
            totalConnections += vectorToNodeMap[node].neighbors.Count;
        }

        Debug.Log($"Generated {initialNodes.Count} nodes with {totalConnections} connections");
        Debug.Log($"Average connections per node: {(float)totalConnections / initialNodes.Count:F2}");
    }

    /// <summary>
    /// Ensure all nodes are connected to the network
    /// </summary>
    private void EnsureNetworkConnectivity()
    {
        // First ensure start/end are connected
        EnsureNodeConnectivity(startNodePos);
        EnsureNodeConnectivity(endNodePos);

        // Check all nodes to ensure none are isolated
        foreach (Vector2 nodePos in initialNodes)
        {
            NodeData node = vectorToNodeMap[nodePos];

            // If node has no connections, connect it to the closest node
            if (node.neighbors.Count == 0)
            {
                Vector2 closestNodePos = Vector2.zero;
                float closestDistance = float.MaxValue;

                foreach (Vector2 otherNodePos in initialNodes)
                {
                    if (Vector2.Distance(nodePos, otherNodePos) < tolerance)
                        continue; // Skip self

                    float distance = Vector2.Distance(nodePos, otherNodePos);
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        closestNodePos = otherNodePos;
                    }
                }

                // Add bidirectional connection
                if (closestDistance < float.MaxValue)
                {
                    Debug.Log($"Creating forced connection from isolated node at {nodePos} to {closestNodePos}");
                    node.neighbors.Add(closestNodePos);
                    vectorToNodeMap[closestNodePos].neighbors.Add(nodePos);
                }
            }
        }

        // Check if the network is fully connected (all nodes reachable from start)
        HashSet<Vector2> reachableNodes = new HashSet<Vector2>(new Vector2Comparer(tolerance));
        Queue<Vector2> nodesToExplore = new Queue<Vector2>();

        // Start from start node
        nodesToExplore.Enqueue(startNodePos);
        reachableNodes.Add(startNodePos);

        // Breadth-first search to find all reachable nodes
        while (nodesToExplore.Count > 0)
        {
            Vector2 current = nodesToExplore.Dequeue();

            foreach (Vector2 neighbor in vectorToNodeMap[current].neighbors)
            {
                if (!reachableNodes.Contains(neighbor))
                {
                    reachableNodes.Add(neighbor);
                    nodesToExplore.Enqueue(neighbor);
                }
            }
        }

        // If not all nodes are reachable, create bridges between disconnected components
        if (reachableNodes.Count < initialNodes.Count)
        {
            Debug.LogWarning($"Network not fully connected - only {reachableNodes.Count} of {initialNodes.Count} nodes reachable");

            // Find and connect unreachable nodes
            foreach (Vector2 nodePos in initialNodes)
            {
                if (!reachableNodes.Contains(nodePos))
                {
                    // Find closest reachable node
                    Vector2 closestReachable = Vector2.zero;
                    float closestDistance = float.MaxValue;

                    foreach (Vector2 reachableNode in reachableNodes)
                    {
                        float distance = Vector2.Distance(nodePos, reachableNode);
                        if (distance < closestDistance)
                        {
                            closestDistance = distance;
                            closestReachable = reachableNode;
                        }
                    }

                    // Create connection to make this node reachable
                    Debug.Log($"Creating bridge connection from unreachable node at {nodePos} to {closestReachable}");
                    vectorToNodeMap[nodePos].neighbors.Add(closestReachable);
                    vectorToNodeMap[closestReachable].neighbors.Add(nodePos);

                    // Now this node is reachable
                    reachableNodes.Add(nodePos);
                    nodesToExplore.Enqueue(nodePos);

                    // Continue BFS to find more nodes
                    while (nodesToExplore.Count > 0)
                    {
                        Vector2 current = nodesToExplore.Dequeue();

                        foreach (Vector2 neighbor in vectorToNodeMap[current].neighbors)
                        {
                            if (!reachableNodes.Contains(neighbor))
                            {
                                reachableNodes.Add(neighbor);
                                nodesToExplore.Enqueue(neighbor);
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Ensure start and end nodes are connected to the network
    /// </summary>
    private void EnsureStartEndConnectivity()
    {
        EnsureNodeConnectivity(startNodePos);
        EnsureNodeConnectivity(endNodePos);
    }

    /// <summary>
    /// Ensure a specific node is connected to the network
    /// </summary>
    private void EnsureNodeConnectivity(Vector2 nodePos)
    {
        NodeData node = vectorToNodeMap[nodePos];

        // If node already has connections, nothing to do
        if (node.neighbors.Count > 0)
            return;

        // Find the closest node to connect to
        Vector2 closestNodePos = Vector2.zero;
        float closestDistance = float.MaxValue;

        foreach (Vector2 otherNodePos in initialNodes)
        {
            if (Vector2.Distance(transform.InverseTransformPoint(nodePos), transform.InverseTransformPoint(otherNodePos)) < 0.001f)
                continue; // Skip self

            float distance = Vector2.Distance(transform.InverseTransformPoint(nodePos), transform.InverseTransformPoint(otherNodePos));
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestNodePos = otherNodePos;
            }
        }

        // Add bidirectional connection
        if (closestDistance < float.MaxValue)
        {
            Debug.Log($"Creating forced connection from {nodePos} to {closestNodePos}");
            node.neighbors.Add(closestNodePos);
            vectorToNodeMap[closestNodePos].neighbors.Add(nodePos);
        }
    }

    #endregion

    #region Path Generation Methods

    /// <summary>
    /// Selects a subset of path objects based on weighted probability
    /// </summary>
    private List<PathObject> GeneratePathList(PathObject[] list)
    {
        if (list == null || list.Length == 0)
        {
            Debug.LogError("No path objects provided!");
            return new List<PathObject>();
        }

        List<PathObject> allPathObjects = new List<PathObject>();
        float totalProbabilityWeight = 0;

        foreach (var path in list)
        {
            if (path == null) continue;

            for (int i = 0; i < path.maxAmountInWorld; i++)
            {
                allPathObjects.Add(path);
            }
            totalProbabilityWeight += path.spawnProbability * path.maxAmountInWorld;
        }

        if (allPathObjects.Count == 0)
        {
            Debug.LogError("No valid path objects found!");
            return new List<PathObject>();
        }

        // Determine how many paths to generate
        int maxPaths = Mathf.Clamp(maxAmountOfPaths, 2, allPathObjects.Count);
        int minPaths = Mathf.Clamp(minAmountOfPaths, 1, maxPaths - 1);
        int chosenPathAmount = prng.Next(minPaths, maxPaths + 1); // +1 because Next is exclusive on upper bound

        List<PathObject> chosenPaths = new List<PathObject>();
        int safetyCounter = 0;
        const int MAX_TRIES = 100;

        // Weighted random selection without replacement
        while (chosenPaths.Count < chosenPathAmount && safetyCounter < MAX_TRIES && allPathObjects.Count > 0)
        {
            // Generate a random value between 0 and the current total weight
            float randomValue = (float)prng.NextDouble() * totalProbabilityWeight;
            float currentSum = 0;

            // Find the item that corresponds to this random value
            for (int i = 0; i < allPathObjects.Count; i++)
            {
                currentSum += allPathObjects[i].spawnProbability;

                if (currentSum >= randomValue)
                {
                    // Add this path to our chosen list
                    PathObject selectedPath = allPathObjects[i];
                    chosenPaths.Add(selectedPath);

                    // Remove this path and update the total probability
                    totalProbabilityWeight -= selectedPath.spawnProbability;
                    allPathObjects.RemoveAt(i);

                    // Remove any duplicates of this path (same instance) to prevent multiple selections
                    for (int j = allPathObjects.Count - 1; j >= 0; j--)
                    {
                        if (object.ReferenceEquals(allPathObjects[j], selectedPath))
                        {
                            totalProbabilityWeight -= selectedPath.spawnProbability;
                            allPathObjects.RemoveAt(j);
                        }
                    }

                    break;
                }
            }

            safetyCounter++;
        }

        // If we didn't get enough paths, add some randomly
        if (chosenPaths.Count < minPaths && allPathObjects.Count > 0)
        {
            // If we didn't get enough paths, add some randomly
            int remaining = minPaths - chosenPaths.Count;
            remaining = Mathf.Min(remaining, allPathObjects.Count);

            for (int i = 0; i < remaining; i++)
            {
                int index = prng.Next(0, allPathObjects.Count);
                chosenPaths.Add(allPathObjects[index]);
                allPathObjects.RemoveAt(index);
            }
        }

        Debug.Log($"Selected {chosenPaths.Count} paths out of {list.Length} available types");
        return chosenPaths;
    }

    /// <summary>
    /// Sort paths by directness (less direct paths first)
    /// </summary>
    private List<Path> SortPathListAfterDirectness(List<Path> paths)
    { 
        bool listSorted = false;
        while(!listSorted)
        {
            bool potentialListSorted = true;
            for(int i = 0; i < paths.Count - 1; i++)
            {
                if (paths[i].pathObject.pathDirectness > paths[i+1].pathObject.pathDirectness)
                {
                    Path path = paths[i];
                    paths[i] = paths[i+1];
                    paths[i + 1] = path;
                    potentialListSorted = false;
                }
            }
            if (potentialListSorted) listSorted = true;
        }
        return paths;
    }

    /// <summary>
    /// Find a path between two nodes with configurable directness
    /// </summary>
    private List<Vector2> FindPath(Vector2 startPos, Vector2 endPos, float directness, float pathSmoothness)
    {
        Debug.Log($"Finding path from {startPos} to {endPos} with directness {directness}");

        // Create a path-specific dictionary to isolate node connections for this path
        Dictionary<Vector2, NodeData> pathSpecificNodes = new Dictionary<Vector2, NodeData>(new Vector2Comparer(tolerance));

        // Clone all nodes to create path-specific versions with isolated neighbors
        foreach (Vector2 originalNodePos in initialNodes)
        {
            // Create a deep copy of the node with the same position but empty neighbors
            NodeData originalNode = vectorToNodeMap[originalNodePos];
            NodeData clonedNode = new NodeData(originalNodePos);
            pathSpecificNodes[originalNodePos] = clonedNode;
        }

        // Copy neighbor relationships while using the cloned nodes
        foreach (Vector2 originalNodePos in initialNodes)
        {
            NodeData originalNode = vectorToNodeMap[originalNodePos];
            NodeData clonedNode = pathSpecificNodes[originalNodePos];

            // Copy all neighbor connections to the cloned node
            foreach (Vector2 neighborPos in originalNode.neighbors)
            {
                if (pathSpecificNodes.ContainsKey(neighborPos))
                {
                    clonedNode.neighbors.Add(neighborPos);
                }
            }
        }

        // Validate directness value and apply indirectnessImpact
        directness = Mathf.Clamp01(directness);

        // If positions are the same, return single node path
        if (Vector2.Distance(startPos, endPos) < tolerance)
        {
            return new List<Vector2> { startPos };
        }

        // A* algorithm data structures
        List<Vector2> openSet = new List<Vector2>();
        HashSet<Vector2> closedSet = new HashSet<Vector2>(new Vector2Comparer(tolerance));
        Dictionary<Vector2, Vector2> cameFrom = new Dictionary<Vector2, Vector2>(new Vector2Comparer(tolerance));
        Dictionary<Vector2, float> gScore = new Dictionary<Vector2, float>(new Vector2Comparer(tolerance));
        Dictionary<Vector2, float> fScore = new Dictionary<Vector2, float>(new Vector2Comparer(tolerance));

        // For non-direct paths, add variation with random weights for edges
        Dictionary<Vector2, Dictionary<Vector2, float>> edgeWeights =
            new Dictionary<Vector2, Dictionary<Vector2, float>>(new Vector2Comparer(tolerance));

        // Initialize pathfinding variables
        openSet.Add(startPos);
        foreach (Vector2 nodePos in initialNodes)
        {
            gScore[nodePos] = float.MaxValue;
            fScore[nodePos] = float.MaxValue;

            // Pre-calculate random edge weights for non-direct paths
            if (directness < 0.99f)
            {
                edgeWeights[nodePos] = new Dictionary<Vector2, float>(new Vector2Comparer(tolerance));
                NodeData node = pathSpecificNodes[nodePos];

                foreach (Vector2 neighborPos in node.neighbors)
                {
                    // Apply indirectnessImpact to create more variation
                    float effectiveDirectness = Mathf.Pow(directness, indirectnessImpact);

                    // Determine range of randomness based on indirectnessImpact
                    float minRange, maxRange;

                    if (indirectnessImpact > 5)
                    {
                        // High impact - extreme variation
                        minRange = Mathf.Lerp(0.9f, 0.01f, 1 - effectiveDirectness);
                        maxRange = Mathf.Lerp(1.1f, 50f, 1 - effectiveDirectness);
                    }
                    else if (indirectnessImpact > 2)
                    {
                        // Medium impact
                        minRange = Mathf.Lerp(0.9f, 0.05f, 1 - effectiveDirectness);
                        maxRange = Mathf.Lerp(1.1f, 20f, 1 - effectiveDirectness);
                    }
                    else
                    {
                        // Normal impact
                        minRange = Mathf.Lerp(0.9f, 0.1f, 1 - effectiveDirectness);
                        maxRange = Mathf.Lerp(1.1f, 10f, 1 - effectiveDirectness);
                    }

                    // Generate random weight
                    float weight = (float)prng.NextDouble();
                    weight = (indirectnessImpact > 1 && weight < 0.5f) ?
                             weight * weight :  // Square small values to create more extreme weights
                             weight;

                    float randomValue = Mathf.Lerp(minRange, maxRange, weight);
                    edgeWeights[nodePos][neighborPos] = randomValue;
                }
            }
        }

        gScore[startPos] = 0;

        // Initial heuristic based on directness
        float effectiveDirectnessForHeuristic = Mathf.Pow(directness, Mathf.Max(1, indirectnessImpact * 0.5f));

        if (effectiveDirectnessForHeuristic > 0.7f)
        {
            // Standard A* heuristic for direct paths
            fScore[startPos] = Vector2.Distance(startPos, endPos);
        }
        else
        {
            // For indirect paths, use reduced initial heuristic influence
            float heuristicWeight = Mathf.Lerp(0.1f, 0.7f, effectiveDirectnessForHeuristic);
            fScore[startPos] = Vector2.Distance(startPos, endPos) * heuristicWeight;
        }

        // For low directness, we'll sometimes consider non-optimal nodes to add variation
        int explorationCounter = 0;

        // A* main loop
        while (openSet.Count > 0)
        {
            Vector2 currentPos;

            // More aggressive random node selection based on indirectness
            float randomNodeThreshold = Mathf.Pow(directness, Mathf.Max(1, indirectnessImpact * 0.25f));

            if (directness < 0.8f && openSet.Count > 1 &&
                (float)prng.NextDouble() > randomNodeThreshold)
            {
                // Pick a random node instead of the optimal one
                int randomIndex = prng.Next(0, openSet.Count);
                currentPos = openSet[randomIndex];
                explorationCounter++;

                // Limit random exploration 
                if (explorationCounter > initialNodes.Count / (indirectnessImpact > 1 ? 1 : 2))
                {
                    // Reset and use best node
                    explorationCounter = 0;
                    currentPos = FindNodeWithLowestFScore(openSet, fScore);
                }
            }
            else
            {
                currentPos = FindNodeWithLowestFScore(openSet, fScore);
            }

            // Goal reached?
            if (Vector2.Distance(currentPos, endPos) < tolerance)
            {
                // Reconstruct path
                List<Vector2> path = new List<Vector2> { currentPos };
                while (cameFrom.ContainsKey(currentPos))
                {
                    currentPos = cameFrom[currentPos];
                    path.Insert(0, currentPos);
                }

                // For very low directness, introduce deliberate detours
                if (directness < 0.5f && path.Count > 3)
                {
                    // Calculate detour count based on directness and indirectnessImpact
                    int detourCount = Mathf.RoundToInt((1 - directness) * indirectnessImpact * 3);
                    Debug.Log($"Adding {detourCount} detours based on directness {directness} and impact {indirectnessImpact}");

                    for (int i = 0; i < detourCount; i++)
                    {
                        List<Vector2> detoured = IntroduceDetour(path, pathSpecificNodes, pathSmoothness);
                        if (detoured.Count > path.Count)
                        {
                            path = detoured;
                        }
                    }
                }
                
                Debug.Log($"Path found with {path.Count} nodes");
                return path;
            }

            openSet.Remove(currentPos);
            closedSet.Add(currentPos);

            // Get the node's neighbors from our path-specific dictionary
            if (!pathSpecificNodes.TryGetValue(currentPos, out NodeData currentNode))
            {
                continue;
            }

            // Check all neighbors
            foreach (Vector2 neighborPos in currentNode.neighbors)
            {
                if (closedSet.Contains(neighborPos))
                    continue;

                // Calculate distance between current and neighbor
                float edgeDistance = Vector2.Distance(currentPos, neighborPos);

                // Apply edge variation for non-direct paths
                if (directness < 0.99f && edgeWeights.ContainsKey(currentPos) &&
                    edgeWeights[currentPos].ContainsKey(neighborPos))
                {
                    edgeDistance *= edgeWeights[currentPos][neighborPos];
                }

                // Calculate tentative gScore
                float tentativeGScore = gScore[currentPos] + edgeDistance;

                // Add to open set if not already there
                if (!openSet.Contains(neighborPos))
                {
                    openSet.Add(neighborPos);
                }
                else if (tentativeGScore >= gScore[neighborPos])
                    continue;

                // This path is better than any previous one
                cameFrom[neighborPos] = currentPos;
                gScore[neighborPos] = tentativeGScore;

                // Calculate heuristic with improved impact of directness
                float heuristic = CalculateHeuristic(neighborPos, endPos, directness);
                fScore[neighborPos] = gScore[neighborPos] + heuristic;
            }
        }

        // No path found - create a direct path with some intermediate nodes
        Debug.LogWarning($"No path found from {startPos} to {endPos}, creating fallback path");
        return CreateFallbackPath(startPos, endPos);
    }

    /// <summary>
    /// Create a fallback path when A* fails to find a valid path
    /// </summary>
    private List<Vector2> CreateFallbackPath(Vector2 startPos, Vector2 endPos)
    {
        List<Vector2> fallbackPath = new List<Vector2> { startPos };

        // Get direction and total distance
        Vector2 direction = endPos - startPos;
        float totalDistance = direction.magnitude;

        // Calculate how many intermediate nodes we need based on maxDistance
        int nodesNeeded = Mathf.CeilToInt(totalDistance / nodeMaxDistance);

        // Get all nodes sorted by distance along the path
        List<KeyValuePair<Vector2, float>> potentialNodes = new List<KeyValuePair<Vector2, float>>();

        foreach (Vector2 nodePos in initialNodes)
        {
            // Skip start and end
            if (Vector2.Distance(nodePos, startPos) < tolerance ||
                Vector2.Distance(nodePos, endPos) < tolerance)
                continue;

            // Calculate projection along path direction
            Vector2 toNode = nodePos - startPos;
            float projectionAlongPath = Vector2.Dot(toNode, direction.normalized);

            // Only consider nodes that are between start and end
            if (projectionAlongPath > 0 && projectionAlongPath < totalDistance)
            {
                potentialNodes.Add(new KeyValuePair<Vector2, float>(nodePos, projectionAlongPath));
            }
        }

        // Sort by distance along path
        potentialNodes.Sort((a, b) => a.Value.CompareTo(b.Value));

        // Now select nodes that respect the max distance
        Vector2 lastAddedNode = startPos;
        foreach (var node in potentialNodes)
        {
            float distToLast = Vector2.Distance(node.Key, lastAddedNode);

            // Only add if within max distance from last node
            if (distToLast <= nodeMaxDistance)
            {
                fallbackPath.Add(node.Key);
                lastAddedNode = node.Key;
            }
        }

        // If we can connect to the end node, add it
        if (Vector2.Distance(lastAddedNode, endPos) <= nodeMaxDistance)
        {
            fallbackPath.Add(endPos);
        }
        else
        {
            // End node is too far from last node in path
            // Find the best node to add before end
            Vector2 bestNode = Vector2.zero;
            float bestDistance = float.MaxValue;

            foreach (Vector2 nodePos in initialNodes)
            {
                float distFromLast = Vector2.Distance(nodePos, lastAddedNode);
                float distToEnd = Vector2.Distance(nodePos, endPos);

                if (distFromLast <= nodeMaxDistance && distToEnd <= nodeMaxDistance &&
                    distFromLast + distToEnd < bestDistance)
                {
                    bestNode = nodePos;
                    bestDistance = distFromLast + distToEnd;
                }
            }

            // If we found a valid connector node, add it
            if (bestDistance < float.MaxValue)
            {
                fallbackPath.Add(bestNode);
            }

            // Add end node
            fallbackPath.Add(endPos);
        }

        return fallbackPath;
    }

    /// <summary>
    /// Find the node with the lowest f-score in the open set
    /// </summary>
    private Vector2 FindNodeWithLowestFScore(List<Vector2> openSet, Dictionary<Vector2, float> fScore)
    {
        Vector2 best = openSet[0];
        float lowestFScore = fScore[best];

        for (int i = 1; i < openSet.Count; i++)
        {
            float score = fScore[openSet[i]];
            if (score < lowestFScore)
            {
                best = openSet[i];
                lowestFScore = score;
            }
        }

        return best;
    }

    /// <summary>
    /// Calculate heuristic for A* pathfinding with directness factor
    /// </summary>
    private float CalculateHeuristic(Vector2 current, Vector2 end, float directness)
    {
        float directDistance = Vector2.Distance(current, end);

        // Apply indirectnessImpact to the directness value
        float effectiveDirectness = Mathf.Pow(directness, indirectnessImpact);

        if (effectiveDirectness >= 0.9f)
        {
            // Standard A* heuristic for direct paths
            return directDistance;
        }
        else if (effectiveDirectness <= 0.1f)
        {
            // For very indirect paths, use minimal heuristic guidance
            // and add substantial random variation
            float baseHeuristic = directDistance * 0.1f;
            float randomVariation = (float)(prng.NextDouble() * directDistance * 5f * indirectnessImpact);
            return baseHeuristic + randomVariation;
        }
        else
        {
            // For paths with moderate directness, scale the heuristic and randomness
            float heuristicWeight = Mathf.Lerp(0.1f, 0.9f, effectiveDirectness);
            float randomWeight = Mathf.Lerp(5f * indirectnessImpact, 0.5f, effectiveDirectness);

            return directDistance * heuristicWeight +
                (float)(prng.NextDouble() * directDistance * randomWeight);
        }
    }

    /// <summary>
    /// Introduce a detour in the path for more natural, organic path shapes
    /// </summary>
    private List<Vector2> IntroduceDetour(List<Vector2> originalPath, Dictionary<Vector2, NodeData> pathNodes, float pathSmoothness)
    {
        if (originalPath.Count < 3 || pathNodes.Count < 4)
            return originalPath;

        // We'll create a new path using only existing nodes
        List<Vector2> newPath = new List<Vector2>();
        newPath.Add(originalPath[0]); // Always keep start

        // For each segment in the original path
        for (int i = 0; i < originalPath.Count - 1; i++)
        {
            Vector2 current = originalPath[i];
            Vector2 next = originalPath[i + 1];

            // If this segment is too long, find intermediate nodes
            float segmentLength = Vector2.Distance(current, next);
            if (segmentLength > nodeMaxDistance)
            {
                // Find nodes that can create a valid sub-path
                List<Vector2> subPath = FindNodesForSegment(current, next);

                // Add all but the last node (which will be added in the next iteration)
                for (int j = 0; j < subPath.Count - 1; j++)
                {
                    newPath.Add(subPath[j]);
                }
            }
            else
            {
                // Only add the current node - next will be added in next iteration
                if (i > 0) // Avoid adding start twice
                    newPath.Add(current);
            }
        }

        // Add the end node
        newPath.Add(originalPath[originalPath.Count - 1]);

        return newPath;
    }

    /// <summary>
    /// Find existing nodes to create a path between two points
    /// </summary>
    private List<Vector2> FindNodesForSegment(Vector2 start, Vector2 end)
    {
        List<Vector2> subPath = new List<Vector2>();
        subPath.Add(start);

        // Get direction and distance
        Vector2 direction = end - start;
        float totalDistance = direction.magnitude;

        // Find all nodes that are between these points
        List<KeyValuePair<Vector2, float>> potentialNodes = new List<KeyValuePair<Vector2, float>>();

        foreach (Vector2 nodePos in initialNodes)
        {
            // Skip start and end
            if (Vector2.Distance(nodePos, start) < tolerance ||
                Vector2.Distance(nodePos, end) < tolerance)
                continue;

            // Check if this node is near the line between start and end
            Vector2 startToNode = nodePos - start;
            float projectionAlongPath = Vector2.Dot(startToNode, direction.normalized);

            // Only consider nodes between start and end
            if (projectionAlongPath > 0 && projectionAlongPath < totalDistance)
            {
                // Calculate perpendicular distance from line
                Vector2 projectedPoint = start + direction.normalized * projectionAlongPath;
                float perpDistance = Vector2.Distance(nodePos, projectedPoint);

                // Only consider nodes close to the line
                if (perpDistance < nodeMaxDistance * 0.5f)
                {
                    potentialNodes.Add(new KeyValuePair<Vector2, float>(nodePos, projectionAlongPath));
                }
            }
        }

        // Sort by distance along path
        potentialNodes.Sort((a, b) => a.Value.CompareTo(b.Value));

        // Create a path through these nodes
        Vector2 lastNode = start;
        foreach (var node in potentialNodes)
        {
            if (Vector2.Distance(node.Key, lastNode) <= nodeMaxDistance)
            {
                subPath.Add(node.Key);
                lastNode = node.Key;
            }
        }

        // Add end node if it's within range
        if (Vector2.Distance(lastNode, end) <= nodeMaxDistance)
        {
            subPath.Add(end);
        }

        return subPath;
    }

    #endregion

    #region Visualization Methods

    /// <summary>
    /// Clear existing nodes and connections
    /// </summary>
    private void ClearExistingNodesAndConnections()
    {
        initialNodes.Clear();
        vectorToNodeMap.Clear();
        pathsConnectionPointDictionary.Clear();
        paths.Clear();

        foreach (Transform child in transform)
        {
            Destroy(child.gameObject);
        }

        foreach (UILine line in connectionLines)
        {
            if (line != null) Destroy(line.gameObject);
        }
        connectionLines.Clear();
    }

    /// <summary>
    /// Create UI visualization for a path
    /// </summary>
    private void VisualizePath(Path path)
    {
        GameObject pathParentObject = new GameObject(path.pathObject.name);

        // Set up as UI element
        pathParentObject.transform.SetParent(UI_Container);
        RectTransform pathRect = pathParentObject.AddComponent<RectTransform>();
        pathRect.anchoredPosition = Vector2.zero;

        List<Vector2> pathNodePositions = path.nodes;

        // Create all nodes
        for (int i = 0; i < pathNodePositions.Count; i++)
        {
            Vector2 nodePos = pathNodePositions[i];

            // Check if we already have a NodeData for this position
            if (!vectorToNodeMap.TryGetValue(nodePos, out NodeData node))
            {
                // Create a new NodeData for this position
                node = new NodeData(nodePos);
                vectorToNodeMap[nodePos] = node;
            }

            // Create GameObject if it doesn't exist
            if (node.nodeGameObject == null)
            {
                // Instantiate as UI element
                GameObject nodeObj = Instantiate(nodePrefab, Vector3.zero, Quaternion.identity);
                nodeObj.GetComponent<Button>().onClick.AddListener(() =>
                {
                    UIWorldLevelGeneratorNavigator.Instance.GetUIInput(nodePos);
                });
                
                nodeObj.transform.parent = UI_Container;
                nodeObj.transform.localPosition = nodePos;
                
                // Set position using RectTransform
                RectTransform rect = nodeObj.GetComponent<RectTransform>();
                node.nodeGameObject = nodeObj;
            }

            // Register this point as an intersection if multiple paths share it
            if (!pathsConnectionPointDictionary.ContainsKey(nodePos))
            {
                pathsConnectionPointDictionary.Add(nodePos, new List<Path>());
            }

            // Add this path to the node's path list if not already there
            if (!pathsConnectionPointDictionary[nodePos].Contains(path))
            {
                pathsConnectionPointDictionary[nodePos].Add(path);
            }
        }

        // Create connections between nodes
        for (int i = 0; i < pathNodePositions.Count - 1; i++)
        {
            Vector2 posA = pathNodePositions[i];
            Vector2 posB = pathNodePositions[i + 1];

            // Create line renderer for connection
            UILine line = Instantiate(this.line, Vector3.zero, Quaternion.identity);
            line.CreateLine(posA, posB, lineWidth);
            line.lineImage.color = connectionColor;
            
            connectionLines.Add(line);
        }
    }

    /// <summary>
    /// Create node types at path intersections
    /// </summary>
    private void CreatePathContent()
    {
        Debug.Log($"Creating path content at {pathsConnectionPointDictionary.Count} intersection points");

        foreach (KeyValuePair<Vector2, List<Path>> connectionPoint in pathsConnectionPointDictionary)
        {
            // Skip nodes that only have one path going through them
            if (connectionPoint.Value.Count <= 1)
                continue;

            // Try to get the node data for this position
            if (vectorToNodeMap.TryGetValue(connectionPoint.Key, out NodeData nodeData) && nodeData.nodeGameObject != null)
            {
                // Get the sprite renderer to change its appearance
                SpriteRenderer image = nodeData.nodeGameObject.GetComponent<SpriteRenderer>();

                // Get a node object appropriate for this intersection
                NodeObject nodeObject = GetRelevantNodeForPathsConnection(connectionPoint.Value);
                nodeData.nodeObject = nodeObject;
                vectorToNodeMap[connectionPoint.Key].nodeObject = nodeObject;
                
                // Apply the sprite if we have a valid node object
                if (nodeObject != null && image != null)
                {
                    image.sprite = nodeObject.sprite;
                    Debug.Log($"Set node at {connectionPoint.Key} to {nodeObject.name}");
                }
                else
                {
                    Debug.LogWarning($"Failed to assign node sprite at {connectionPoint.Key}");
                }
            }
            else
            {
                Debug.LogWarning($"No node game object found at intersection position {connectionPoint.Key}");
            }
        }
    }

    /// <summary>
    /// Get a node type suitable for a path intersection
    /// </summary>
    private NodeObject GetRelevantNodeForPathsConnection(List<Path> paths)
    {
        Dictionary<NodeObject, float> nodeTypeScores = new Dictionary<NodeObject, float>();

        // For each path that intersects at this point
        foreach (Path path in paths)
        {
            // Skip invalid paths
            if (path == null || path.pathObject == null || path.pathObject.nodeLogic == null)
                continue;

            // Calculate the total weight for normalization
            float totalWeight = 0;
            foreach (PathNodeLogic logic in path.pathObject.nodeLogic)
            {
                // Skip null entries
                if (logic == null || logic.relevantNode == null)
                    continue;

                totalWeight += logic.spawnMultiplier;
            }

            // Skip paths with no valid weights
            if (totalWeight <= 0)
                continue;

            // For each possible node type in this path
            foreach (PathNodeLogic logic in path.pathObject.nodeLogic)
            {
                // Skip null entries
                if (logic == null || logic.relevantNode == null)
                    continue;

                // Initialize if not present
                if (!nodeTypeScores.ContainsKey(logic.relevantNode))
                {
                    nodeTypeScores.Add(logic.relevantNode, 0);
                }

                // Calculate normalized probability
                float normalizedWeight = logic.spawnMultiplier / totalWeight;

                // Adjust score based on how many of this type were already placed
                float placementAdjustment = 1.0f;
                if (path.placedNodes != null && path.placedNodes.ContainsKey(logic.relevantNode))
                {
                    // Reduce probability if we've already placed many of this type
                    placementAdjustment = 1.0f / (path.placedNodes[logic.relevantNode] + 1.0f);
                }

                // Add to cumulative score
                nodeTypeScores[logic.relevantNode] += normalizedWeight * placementAdjustment;
            }
        }

        // Now select a node type using weighted random selection
        float totalScore = 0;
        foreach (float score in nodeTypeScores.Values)
        {
            totalScore += score;
        }

        // If we have no valid nodes, return null
        if (totalScore <= 0 || nodeTypeScores.Count == 0)
        {
            Debug.LogWarning("No valid node types found for intersection");
            return null;
        }

        // Select randomly based on weights
        float randomValue = (float)prng.NextDouble() * totalScore;
        float runningTotal = 0;

        foreach (KeyValuePair<NodeObject, float> nodePair in nodeTypeScores)
        {
            runningTotal += nodePair.Value;
            if (randomValue <= runningTotal)
            {
                return nodePair.Key;
            }
        }

        // Fallback to highest scoring node if random selection fails
        NodeObject highestScoreNode = null;
        float highestScore = 0;

        foreach (KeyValuePair<NodeObject, float> nodePair in nodeTypeScores)
        {
            if (nodePair.Value > highestScore)
            {
                highestScore = nodePair.Value;
                highestScoreNode = nodePair.Key;
            }
        }

        return highestScoreNode;
    }

    #endregion
}

/// <summary>
/// Store node data with position, connections and UI representation
/// </summary>
[Serializable]
/// <summary>
/// Store node data with position, connections and UI representation
/// </summary>
[Serializable]
public class NodeData
{
    public int localSeed;
    public NodeObject nodeObject;
    public Vector2 position;
    public List<Vector2> neighbors = new List<Vector2>();
    public GameObject nodeGameObject;
    
    public NodeData(Vector2 position)
    {
        this.position = position;
    }
}

/// <summary>
/// Defines the logic for node placement in paths
/// </summary>
[Serializable]
public class PathNodeLogic
{
    public NodeObject relevantNode;
    public NodeObject[] nodesInFront;
    public float spawnMultiplier = 1;
}

/// <summary>
/// Debug class to visualize paths
/// </summary>
[Serializable]
public class PathDebug
{
    public List<Vector2> path = new List<Vector2>();
    
    public PathDebug(List<Vector2> path)
    {
        this.path = path;a
    }
}
