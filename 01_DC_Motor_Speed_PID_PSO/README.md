# 01: DC Motor Velocity Control via PSO-Tuned PID

This benchmark demonstrates the automated tuning of a classical 3-parameter PID controller for an armature-controlled DC motor subjected to actuator saturation ($\pm 24\text{ V}$) using Particle Swarm Optimization (PSO) in MATLAB and Simulink.

---

## 📌 System Description & Reference Model

The plant model and physical parameters are adopted from the widely recognized benchmark by the **University of Michigan Control Tutorials for MATLAB & Simulink (CTMS)**:
- **Reference:** [CTMS - DC Motor Speed: Simulink Controller Design](https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SimulinkControl)

The electromechanical dynamics are governed by the following differential equations:

$$V(t) = R \cdot i(t) + L \frac{di(t)}{dt} + K_e \cdot \omega(t)$$

$$J \frac{d\omega(t)}{dt} + B \cdot \omega(t) = K_t \cdot i(t) - T_L(t)$$

### Physical Model Parameters (CTMS Benchmark):
| Parameter | Symbol | Nominal Value | Unit |
| :--- | :---: | :---: | :---: |
| Moment of inertia of the rotor | $J$ | $0.01$ | $\text{kg}\cdot\text{m}^2$ |
| Motor viscous friction constant | $b$ | $0.1$ | $\text{N}\cdot\text{m}\cdot\text{s}$ |
| Electromotive force constant | $K_e$ | $0.01$ | $\text{V} / (\text{rad/s})$ |
| Motor torque constant | $K_t$ | $0.01$ | $\text{N}\cdot\text{m}/\text{A}$ |
| Electric resistance | $R$ | $1.0$ | $\Omega$ |
| Electric inductance | $L$ | $0.5$ | $\text{H}$ |

### Transfer Function:
From the differential equations above, the open-loop transfer function from input voltage $V(s)$ to angular speed $\omega(s)$ is:

$$P(s) = \frac{\omega(s)}{V(s)} = \frac{K}{(J s + b)(L s + R) + K^2} \quad \text{where } K = K_t = K_e$$

Substituting the numerical parameters:

$$P(s) = \frac{0.01}{0.005 s^2 + 0.06 s + 0.1001}$$

- **Actuator Constraint:** The armature input voltage $V(t)$ is constrained by a saturation block: $[-24\text{ V}, +24\text{ V}]$.
- **Control Objective:** Track a unit step angular speed setpoint ($r = 1.0\text{ rad/s}$) under closed-loop feedback.

---

## 🎯 Cost Function Formulation

To achieve rapid rise time while strictly eliminating overshoot and late-stage oscillatory behavior, the objective function combines the **Integral of Time-weighted Absolute Error (ITAE)** with a heavy overshoot penalty:

$$J(K_p, K_i, K_d) = \int_{0}^{T} t \cdot \vert{}e(t)\vert{} \, dt + 50 \cdot M_p$$

### Key Mechanics:
1. **Time Weighting ($t \cdot \vert{}e(t)\vert{}$):** Initial transient errors at small $t$ are tolerated; however, steady-state errors and long-lasting ringing are amplified linearly over time.
2. **Overshoot Penalty ($50 \cdot M_p$):** 
   $$M_p = \max\Big(0, \, \max(\omega(t)) - 1\Big)$$
   A scalar weighting multiplier of $50$ heavily penalizes overshoot, forcing the swarm away from excessively aggressive, underdamped regions.

---

## 🔍 Optimization Settings (PSO)

The tuning routine utilizes MATLAB's `particleswarm` configured as follows:

| Hyperparameter | Value | Description |
| :--- | :--- | :--- |
| **Search Space Dimension** | 3 | $[K_p, K_i, K_d]$ |
| **Swarm Size** | 15 | Number of candidate particles evaluated per iteration |
| **Max Iterations** | 20 | Upper boundary for optimization loop termination |
| **Lower Bounds (`lb`)** | `[0, 0, 0]` | Positivity constraint |
| **Upper Bounds (`ub`)** | `[500, 100, 10]` | Stability and derivative noise ceiling |

---

## 📈 Convergence Log & Results

```text
                                 Best            Mean     Stall
Iteration     f-count            f(x)            f(x)    Iterations
    0              15           1.003           5.713        0
    1              30          0.2236           8.525        0
    2              45          0.1247           6.579        0
    3              60         0.06201          0.9886        0
...
   11             180         0.02962           8.625        0
   20             315         0.02962          0.2496        9
Optimization ended: number of iterations exceeded OPTIONS.MaxIterations.
