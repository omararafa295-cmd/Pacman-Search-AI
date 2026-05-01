# Pacman AI - Search Algorithms 👻

## 📌 Overview
This project implements various artificial intelligence search algorithms to navigate Pacman through complex mazes efficiently. The primary goal is to find the most optimal path to collect all dots while avoiding obstacles, demonstrating practical applications of pathfinding and graph search techniques.

## 🚀 Algorithms Implemented
In this project, I developed the core logic for several AI search algorithms:
- **Depth-First Search (DFS):** Explores the deepest nodes in the search tree first.
- **Breadth-First Search (BFS):** Explores the shallowest nodes first, guaranteeing the shortest path in unweighted graphs.
- **Uniform Cost Search (UCS):** Finds the optimal path based on varying step costs (useful for environments with varying terrain).
- **A* Search:** Utilizes custom heuristics (like Manhattan distance) to find the most optimal path efficiently, combining the benefits of UCS and greedy search.

## 🛠️ Tech Stack & Concepts
- **Language:** Python 3
- **Core Concepts:** Data Structures (Stacks, Queues, Priority Queues), Graph Theory, Heuristics Design, AI Agents.

## 📂 My Contribution
My primary code and logic implementation for this project are located in:
* `search.py`: Contains the core search algorithms (DFS, BFS, UCS, A*).
* `searchAgents.py`: Contains the search-based agents and heuristics used to solve specific routing tasks.

*(Note: The underlying game engine and graphics were provided as part of the UC Berkeley CS188 Introduction to AI course structure. My work focuses entirely on the AI and algorithmic implementation).*

## 🎮 Getting Started (How to Run)
To run the game and test the different algorithms, clone the repository and use the following commands in your terminal:

**1. Run Depth-First Search (DFS):**
```bash
python pacman.py -l mediumMaze -p SearchAgent -a fn=dfs

**2. Run Breadth-First Search (BFS):**
```bash
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs

**3. Run A Search (with Manhattan Heuristic):**
```bash
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

Developed by Omar Mounir Arafa | Software Engineer & Full-Stack Developer

