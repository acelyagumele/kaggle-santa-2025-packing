🎄 Kaggle Santa 2025: Hybrid Geometry Optimization Engine
📌 Project Overview
This repository contains a high-performance hybrid optimization engine developed for the Kaggle Santa 2025 challenge. The project addresses a complex NP-Hard 2D Bin Packing problem: placing between 1 and 200 intricate Christmas tree polygons into the smallest possible square bounding box without any overlaps.

Achievement: Ranked in the Top 14% globally among 3,395 competing teams.

🏗 System Architecture
The system utilizes a Two-Tier Architecture to bridge the gap between raw computational speed and absolute geometric validation:

C++ High-Performance Solver (The Worker): \* A custom Computational Geometry engine built from scratch (using cross-product and ray-casting) to handle million-scale collision tests per second.

Implements multiple heuristic search strategies to navigate the complex solution space.

Python Orchestration & Fail-Safe Layer (The Arbiter): \* Manages the execution pipeline and automates hyperparameter tuning via subprocess.

Utilizes STRtree (Spatial Indexing) and Shapely for a rigorous final validation, ensuring submissions are 100% free of overlap errors.

🔧 Engineering Challenges & Solutions

1. Overcoming Floating-Point Precision Loss
   A critical challenge was the precision loss between C++ (double) and Kaggle’s evaluation servers, where mikroscopic gaps in memory could be interpreted as overlaps during file I/O.

Solution: Implemented a Local Kaggle Evaluator Simulator in Python. Outputs are parsed as arbitrary-precision Decimal objects (25-digit precision) and scaled by 1e18 before validation.

C++ Side: Armed the collision detection engine with a strict 1e-11 safety tolerance to ensure results are "stricter" than the evaluation metric.

2. Escaping Local Minima
   Standard Simulated Annealing often stalled in suboptimal configurations. Multiple heuristics were developed to overcome this:

Earthquake Heuristic: Temporarily increasing thermodynamic temperature to "shake" the system and explore new formations.

Dynamic Gravity: Phased logic that pushes polygons towards the South-West corner to compact them, followed by a center-gravity phase for final refinement.

📂 Project Structure
C++ Optimization Modules (src/cpp/)
optimizer_exploration.cpp: Aggressive Simulated Annealing core designed to break local minima using high-temperature "shake" phases.

optimizer_lns.cpp: Advanced Large Neighborhood Search module that identifies "culprit" polygons causing score bottlenecks and applies localized Ruin-and-Recreate heuristics.

optimizer_finetune.cpp: A hybrid core combining Simulated Annealing with Differential Evolution (Mini-DE) for population-based local search.

optimizer_microsteps.cpp: Precision-focused solver using ultra-small step sizes (0.0001) and center-gravity for final millimeter-level compaction.

Python Layer (src/python/)
orchestrator.py: The main manager that accelerates geometry templates with Numba (JIT) and performs fail-safe validation using STRtree Spatial Indexing.
