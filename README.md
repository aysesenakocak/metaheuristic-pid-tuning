# metaheuristic-pid-tuning
Tuning PID and FOPID controllers for dynamic systems in MATLAB/Simulink using metaheuristic optimization algorithms (PSO, etc.).

# Control-Optimization-Lab (MATLAB & Simulink)

A modular benchmark framework for automated controller tuning across dynamical systems using metaheuristic and evolutionary optimization algorithms.

---

## 📌 Overview

Designing control parameters manually via trial-and-error or heuristic methods (e.g., classical Ziegler-Nichols) often yields suboptimal responses, especially in the presence of actuator saturations and nonlinearities. 

This repository serves as an extensible testbed to:
1. Couple **MATLAB-based heuristic optimizers** with high-fidelity **Simulink models**.
2. Benchmark controllers ranging from classical **PID** to fractional-order **FOPID** and modern robust schemes.
3. Evaluate closed-loop transient and steady-state responses against integral performance indices (ITAE, IAE, ISE) and multi-objective penalty constraints.

---

## 🗂️ Repository Structure

Each experimental setup is self-contained within its own directory containing its Simulink model (`.slx`), objective definition (`cost.m`), driver script, and dedicated technical notes:

```text
├── 01_DC_Motor_Speed_PID_PSO/       # Completed: Armature-controlled DC motor with PSO
├── 02_DC_Motor_Speed_FOPID_PSO/     # Planned: Fractional-order (5-parameter) expansion
├── 03_Inverted_Pendulum_LQR_GA/     # Planned: Nonlinear balance system
├── common/                          # Shared performance indices and utilities
├── .gitignore
└── README.md
