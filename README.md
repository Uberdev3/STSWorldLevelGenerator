# STSWorldLevelGenerator
 The code for my Slay the Spire like world generator.Unity World Level Generator
A procedural world map generator that creates Slay the Spire-like path-based level progression for Unity games.

Overview
This world level generator creates procedural map layouts resembling the path-based progression system found in games like Slay the Spire. The generator produces a network of interconnected nodes with multiple branching paths between a start and end point.
Key Features

Multiple procedurally generated paths with configurable directness
Node-based progression system with custom node types
UI-based rendering system for 2D game worlds
Dynamic path generation with configurable difficulty progression
Customizable node encounters through path objects

How It Works
Core Components

Node Generation - Creates a network of nodes within a defined area
Path Finding - Uses a modified A* algorithm to generate varied paths between nodes
Visualization - Renders paths and nodes using Unity UI components
Node Objects - Attaches gameplay functionality to nodes based on path types

Generation Process

Initialization

Define start and end positions
Set up generation parameters (min/max paths, node distances, etc.)


Node Network Creation

Place start and end nodes
Generate intermediate nodes in a corridor formation
Ensure proper connectivity between all nodes


Path Generation

Select path objects based on probability weights
Find paths between start and end with varying directness
Apply smoothing and detours for visual variety


Node Assignment

Identify path intersections
Assign node types based on path properties
Generate seeds for deterministic node content


Visualization

Render nodes as UI elements
Draw connections between nodes
Apply styling based on configuration



Customization Options

Directness Impact: Controls how straight or winding paths are
Path Objects: Define node types and their distribution along paths
Difficulty Progression: Scales challenge based on world level
Node Distribution: Controls node placement density and patterns

Integration
This system is designed to be part of a larger game framework and works with:

ProceduralMain component for seed management
UIWorldLevelGeneratorNavigator for player movement on the map
Path and NodeObject components for gameplay content

Usage

Attach the WorldLevelGenerator script to a GameObject in your scene
Configure the Inspector settings:

Set up UI containers and prefabs
Configure node distances and counts
Define path objects and their properties
Set start/end positions


Call GenerateMap() to create a new procedural map
Use UIWorldLevelGeneratorNavigator to handle player movement between nodes

Configuration Parameters
Basic Settings

World Level Base Difficulty: Base difficulty level
World Level Difficulty Increase: How much difficulty scales per level
Indirectness Impact: Controls path variation (higher = more winding paths)

Path Settings

Min/Max Amount Of Paths: Range for number of paths to generate
Possible Paths: Collection of path types that can be selected
Chosen Path Objects Is Static: Whether paths are regenerated each time

Node Settings

Node Min/Max Distance: Controls spacing between nodes
Nodes To Generate: Total intermediate nodes to place
Node Prefab: UI element used for nodes

UI Settings

UI_Container/Screen: References to UI elements for rendering
Line: Line prefab for connections
Line Width: Visual thickness of connections

Start/End Settings

Start/End Position: World positions for level endpoints
Special Node Scale: Size multiplier for start/end nodes

Examples
Straight Paths
LevelGenerator.indirectnessImpact = 1.0f;
worldLevelGenerator.GenerateMap();

Winding Paths
LevelGenerator.indirectnessImpact = 5.0f;
worldLevelGenerator.GenerateMap();

High Difficulty Scaling
LevelGenerator.worldLevelBaseDifficulty = 1.0f;
worldLevelGenerator.worldLevelDifficultyIncrease = 2.5f;
