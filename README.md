# 🎄 Kaggle Santa 2025: Christmas Tree Packing Challenge

[![Kaggle Rank](https://img.shields.io/badge/Kaggle-Top%2014%25-gold?style=for-the-badge)](https://www.kaggle.com/competitions/santa-2025)
[![Tech Stack](https://img.shields.io/badge/Tech-C%2B%2B%20%7C%20Python%20%7C%20Shapely-blue?style=for-the-badge)](https://github.com/yourusername/kaggle-santa-2025-packing)

<div>
  <a href="https://youtube.com/shorts/sIQvgCirOdE?feature=share" target="_blank">
    <img src="https://img.shields.io/badge/YouTube-Watch%20Full%20Solution-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="Watch on YouTube"/>
  </a>
</div>

---

## 📌 Project Overview

This repository contains a high-performance hybrid optimization engine developed for the **Kaggle Santa 2025** challenge. The project addresses a complex **NP-Hard 2D Bin Packing** problem: placing between 1 and 200 intricate Christmas tree polygons into the smallest possible square bounding box without any overlaps.

> **Achievement:** Ranked in the **Top 14%** globally among 3,395 competing teams.

---

## 🏗 System Architecture

The system utilizes a **Two-Tier Architecture** to bridge the gap between raw computational speed and absolute geometric validation:

### ⚙️ C++ High-Performance Solver (The Worker)

- **Custom Engine:** A computational geometry engine built from scratch (using cross-product and ray-casting) to handle million-scale collision tests per second.
- **Heuristic Search:** Implements multiple strategies to navigate the complex solution space, including Simulated Annealing and Differential Evolution.

### 🐍 Python Orchestration & Fail-Safe (The Arbiter)

- **Execution Pipeline:** Manages the pipeline and automates hyperparameter tuning via `subprocess`.
- **Validation:** Utilizes **STRtree (Spatial Indexing)** and `Shapely` for a rigorous final validation, ensuring submissions are 100% free of overlap errors.

---

## 🔧 Engineering Challenges & Solutions

### 1. Overcoming Floating-Point Precision Loss

A critical challenge was the precision loss between C++ (`double`) and Kaggle’s evaluation servers, where microscopic gaps in memory could be interpreted as overlaps during file I/O.

- **Solution:** Implemented a **Local Kaggle Evaluator Simulator** in Python.
- **Precision:** Outputs are parsed as arbitrary-precision **Decimal** objects (25-digit precision) and scaled by $10^{18}$ before validation.
- **Safety:** The C++ engine uses a strict $1e^{-11}$ safety tolerance to ensure results are "stricter" than the evaluation metric.

### 2. Escaping Local Minima

- **Earthquake Heuristic:** Aggressive "shake" phases using high-temperature Simulated Annealing to break local minima.
- **Dynamic Gravity:** A phased strategy that pushes polygons towards a corner to compact them, followed by a center-gravity phase.
- **LNS Strategy:** Identifying bottleneck "culprit" polygons and applying localized **Ruin-and-Recreate** tactics.

---

## 📂 Project Structure

| File                        | Description                                                        |
| :-------------------------- | :----------------------------------------------------------------- |
| `optimizer_exploration.cpp` | Aggressive SA core for breaking local minima.                      |
| `optimizer_lns.cpp`         | **Large Neighborhood Search** for bottleneck-focused optimization. |
| `optimizer_finetune.cpp`    | Hybrid core with **Differential Evolution (Mini-DE)**.             |
| `optimizer_microsteps.cpp`  | Precision-focused solver using ultra-small step sizes.             |
| `orchestrator.py`           | Python manager with **Numba (JIT)** and **STRtree** validation.    |

---

## 🚀 Getting Started

To run the optimization engine, ensure you have the required dependencies and follow these steps:

### 1. Prerequisites

- **Python 3.8+**: The orchestrator requires `pandas`, `numpy`, and `shapely` libraries.

- **C++ Compiler**: A modern compiler (GCC/G++ recommended) is needed to build the solver cores.
- **Initial Data**: A valid `submission.csv` file must be present in the root directory, as the orchestrator reads existing configurations to refine them.

### 2. Compilation

The Python orchestrator communicates with the C++ core via `subprocess`. You must compile the desired C++ module into an executable named `mysolver` (the default name expected by the script):

```bash
# Example: Compiling the exploration engine
g++ -O3 src/cpp/optimizer_exploration.cpp -o mysolver
```

### 3. Execution

The orchestrator will randomly select problem groups (1-200 trees), execute the C++ solver with varying seeds, and validate the results using STRtree Spatial Indexing.

```bash
# Start the hybrid optimization loop
python src/python/orchestrator.py
```

Note: The script will automatically update submission.csv whenever a better non-overlapping score is found during the validation phase.

### Testing the Code

The `submission.csv` included in this repository represents our final optimized state from the competition. Because the packing is already highly optimized, running the orchestrator directly might not yield new improvements.

If you would like to test the overlap detection and the optimization pipeline:

1. Open `submission.csv` and intentionally modify a coordinate (e.g., change an `x` or `y` value slightly) to force an overlap.
2. Run `python src/python/orchestrator.py`.
3. The system will detect the invalid state and use the C++ solvers to resolve the collision and re-pack the group.
