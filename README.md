# Time-Based Grid Search & Multi-Agent Path Planning  
A Technical Implementation of Space-Time A*, Safe Interval Path Planning (SIPP), and Priority-Based Multi-Agent Coordination

---

This repository provides a comprehensive and modular implementation of **time-augmented path planning algorithms** for dynamic environments and multi-robot systems. It includes **Space-Time A*** (STA*), **Safe Interval Path Planning** (SIPP), and a **priority-based multi-agent planner** built on top of a spatiotemporal reservation mechanism. The goal of this project is to offer a research-grade, easily extensible codebase for students, engineers, and researchers working on robotic motion planning, dynamic obstacle avoidance, and multi-agent coordination.

Time-based path planning extends the traditional 2D grid representation into a **3D space-time lattice**, enabling planners to guarantee both **spatial** and **temporal** safety. This is essential for environments with moving obstacles, autonomous fleets, warehouse robots, and multi-agent navigation problems.

---

## 🚀 Overview

Classical grid-based path planning considers only spatial feasibility. However, when obstacles move or multiple agents simultaneously operate in the same environment, spatial reasoning is insufficient. The algorithms in this repository explicitly incorporate time as part of the search space, enabling:

- Collision-free navigation around dynamic obstacles  
- Resolution of head-on conflicts  
- Avoidance of edge conflicts (swapping collisions)  
- Temporal coordination across multiple robots  
- Optimal or near-optimal arrival times  

Key algorithms implemented:

### ✅ **Space-Time A\***  
A direct extension of A* into the state space (x, y, t).  
Each node represents a state at a specific time step, enabling fine-grained temporal reasoning.  
Ideal for baseline implementations and for understanding the underlying mechanics of time-augmented search.

### ✅ **Safe Interval Path Planning (SIPP)**  
A significantly more efficient planner that compresses time into **intervals of guaranteed safety**, thereby reducing the number of states.  
SIPP retains optimality while drastically reducing node expansions, making it appropriate for scenarios with many dynamic obstacles.

### ✅ **Priority-Based Multi-Agent Planning**  
A scalable technique for coordinating multiple robots.  
Agents are assigned priorities, planned sequentially, and inserted into a **reservation table** that ensures no future agent can violate temporal or spatial safety.

---

## 🧩 System Architecture

The system is structured into three layers:

### **1. Environment & Dynamic Obstacle Model**
- Represents static obstacles, dynamic movement schedules, and time-dependent occupancy.
- Exposes `is_blocked(x, y, t)` for all planners.

### **2. Single-Agent Planner**
- Either STA* or SIPP
- Outputs a time-parameterized trajectory:  
  `(x0, y0, t0), (x1, y1, t1), …`

### **3. Multi-Agent Coordinator**
- Uses a global reservation table to prevent conflicts.
- Handles vertex and edge collision checking.
- Resolves multi-agent interactions using priority ordering.

---

## 📐 Technical Details

### **State Representation**
A planner state is defined as:
s = (x, y, t)


For SIPP, the state additionally contains a safe interval:



I = [t_low, t_high]


### **Transition Model**
- Actions include: move in 4 or 8 directions, or wait.
- Each transition increments time by Δt = 1 (configurable).

### **Heuristic Function**
Time-admissible heuristic:



h(s) = ManhattanDistance(s, goal) / max_speed


Ensures optimality under the discrete-time model.

### **Conflict Models**
- **Vertex Conflict:**  two agents occupy (x, y, t)
- **Edge Conflict:** agents swap cells simultaneously
- **Temporal Conflict:** arrival time intersects unsafe intervals

---

## 📊 Performance & Complexity

| Algorithm | State Space | Notes |
|----------|-------------|-------|
| Space-Time A* | O(|V| × T) | Simple but expands many redundant temporal states |
| SIPP | O(|V| × |I(c)|) | Much smaller state-space, optimal, highly scalable |
| Priority Planner | O(N × C_single) | Fast but not globally optimal |

SIPP typically achieves **5×–15× reduction** in expansions compared to STA*, especially in dense or highly dynamic environments.

---

## 📂 Repository Structure



├── sta/ # Space-Time A* implementation
├── sipp/ # Safe Interval Path Planning
├── multi_agent/ # Priority-based planner with reservation tables
├── env/ # Grid, dynamic obstacles, and time-dependent models
├── examples/ # Demo scripts and test scenarios
└── media/ # GIFs, images, visualizations


---

## 📸 Visual Demonstrations

Animations and visualizations are provided to illustrate:

- Time-expanded search layers  
- Safe interval segmentation  
- Multi-agent conflict resolution  
- Spatiotemporal reservation dynamics  

(You may include GIFs from PythonRobotics or your own simulations.)

---

## ✅ Applications

This project is ideal for:

- Autonomous ground robots  
- Multi-drone coordination  
- Warehouse robots (AGV fleets)  
- Crowd simulation  
- Dynamic obstacle avoidance research  
- Undergraduate and graduate robotics courses  

---

## 📚 References

- Phillips & Likhachev — *SIPP: Safe Interval Path Planning*
- Atsushi Sakai — *PythonRobotics: Time-Based Grid Search*  
- Silver — *Cooperative Pathfinding*  
- Erdmann & Lozano-Pérez — *On Multiple Moving Objects*

---

## 🛠 Future Work

- Integration with Conflict-Based Search (CBS)  
- Continuous-time SIPP variants  
- Learning-based dynamic obstacle prediction  
- Receding-horizon multi-agent planning  

---
