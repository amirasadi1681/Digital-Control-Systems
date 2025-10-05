# Maze-Following Robot Using Search Algorithms
[![View Presentation](https://img.shields.io/badge/Prezi_Presentation-Click_to_View-blue?style=flat-square&logo=prezi)](https://prezi.com/view/wHjIw9nZQi5z6kG3UhBq/)

This project presents the design and implementation of an **autonomous maze-following robot** that combines **PID control** with **search algorithms (BFS and DFS)** for intelligent path planning and navigation.  
The system was implemented in Python and MATLAB, integrating control theory with algorithmic decision-making to achieve stable and efficient movement through a maze.

---

## 📘 Project Overview

The goal of this project is to develop a robot capable of exploring and solving a maze using both **feedback control** and **graph-based search algorithms**.  
The robot uses a **PID controller** for real-time motion correction and a **BFS/DFS-based path planner** to determine the optimal or complete traversal path.

The system was tested in a simulated environment, combining control dynamics and logical pathfinding for smooth and efficient navigation.

---

## ⚙️ Key Features

- **PID Control for Motion Stability:**  
  A discrete-time PID controller ensures smooth speed and heading control, correcting errors in real time.

- **BFS & DFS Path Planning:**  
  Implemented Breadth-First Search (BFS) and Depth-First Search (DFS) algorithms for maze exploration and optimal route generation.

- **Autonomous Navigation:**  
  The robot makes local and global navigation decisions based on environmental feedback and precomputed paths.

- **Simulation and Analysis:**  
  Full simulation and control loop validation were performed in Python and MATLAB.

---

## 🧩 System Components

- **`PID.py`:** Implements the discrete PID controller used for velocity and orientation control.  
- **`Maze_controller.py`:** Contains the main maze-solving logic using BFS and DFS algorithms.  
- **`Location.py`:** Tracks the robot’s position, heading, and updates during movement.  

---

## 📊 Simulation and Results

The robot successfully navigated multiple maze layouts using a combination of control and search strategies.  
PID control maintained stable motion, while BFS and DFS efficiently mapped and solved the maze.

Key observations:
- Smooth and stable motion under digital control.  
- Successful maze solving using BFS and DFS.  
- Robust performance under turns, dead-ends, and path changes.

### 🔹 Maze Simulation
![Maze Simulation](docs/figures/maze_sim.png)
*Simulation of the robot navigating through the maze environment using PID and BFS/DFS algorithms.*

---

## 🚀 How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/amirasadi1681/Maze-Following-Robot.git
   cd Maze-Following-Robot
