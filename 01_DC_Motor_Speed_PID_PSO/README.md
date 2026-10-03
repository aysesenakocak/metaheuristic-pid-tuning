# 01: DC Motor Velocity Control via PSO-Tuned PID

This benchmark demonstrates the automated tuning of a parallel-form PID controller for an armature-controlled DC motor subjected to actuator saturation ($\pm 24\text{ V}$) using Particle Swarm Optimization (PSO) in MATLAB and Simulink.

## 📌 System Description & Reference Model

The dynamic model and physical parameters are adopted from the standard benchmark by the **University of Michigan Control Tutorials for MATLAB & Simulink (CTMS)**:

* **Reference:** [CTMS - DC Motor Speed: Simulink Controller Design](https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SimulinkControl)

The electro-mechanical coupling is described by Kirchhoff's voltage law for the armature circuit and Newton's second law for the mechanical rotational assembly:

$$
V(t) = R \cdot i(t) + L \frac{di(t)}{dt} + K_e \cdot \omega(t)
$$

$$
J \frac{d\omega(t)}{dt} + b \cdot \omega(t) = K_t \cdot i(t) - T_L(t)
$$

### Physical Model Parameters (CTMS Benchmark)

| Parameter | Symbol | Nominal Value | Unit | 
| ----- | ----- | ----- | ----- | 
| Moment of inertia of the rotor | $J$ | $0.01$ | $\text{kg}\cdot\text{m}^2$ | 
| Motor viscous friction constant | $b$ | $0.1$ | $\text{N}\cdot\text{m}\cdot\text{s}$ | 
| Electromotive force constant | $K_e$ | $0.01$ | $\text{V} / (\text{rad/s})$ | 
| Motor torque constant | $K_t$ | $0.01$ | $\text{N}\cdot\text{m}/\text{A}$ | 
| Electric armature resistance | $R$ | $1.0$ | $\Omega$ | 
| Electric armature inductance | $L$ | $0.5$ | $\text{H}$ | 

### Transfer Function Derivation

Assuming zero load disturbance ($T_L(s) = 0$) and motor torque-back-EMF symmetry ($K = K_t = K_e$), the open-loop transfer function mapping input armature voltage $V(s)$ to angular rotor speed $\omega(s)$ is:

$$
P(s) = \frac{\omega(s)}{V(s)} = \frac{K}{(J s + b)(L s + R) + K^2}
$$

Substituting the numerical parameters gives:

$$
P(s) = \frac{0.01}{0.005 s^2 + 0.06 s + 0.1001}
$$

## 🧩 Simulink Model Architecture

The closed-loop simulation model (`dc_motor_model_cost.slx`) implements continuous feedback tracking with physical actuator limitations and real-time state logging:

```
[ Step (r = 1.0) ] ---> (+) ---> [ PID Controller ] ---> [ Saturation (±24V) ] ---> [ DC Motor Plant ] ---> y(t) [Speed]
                         ^ (-)                                                                     |
                         |-------------------------------------------------------------------------|

```

 `docs/simulink_model.png` 

### Constituent Blocks & Functional Mechanics:

1. **Reference Input (`Step`):**

   * Supplies a unit step setpoint ($r(t) = 1.0\text{ rad/s}$) at $t = 0\text{ s}$ to evaluate step tracking, rise time, and settling characteristics.

2. **Error Summing Junction (`Sum`):**

   * Computes the instantaneous velocity tracking error:
     

     $$
     e(t) = r(t) - y(t)
     $$

   * Routes $e(t)$ to the PID controller and streams it to the MATLAB workspace for integral cost evaluation.

3. **Controller (`PID Controller`):**

   * Implements the ideal parallel PID control law:
     

     $$
     u(t) = K_p \, e(t) + K_i \int_{0}^{t} e(\tau)\,d\tau + K_d \, \frac{de(t)}{dt}
     $$

   * The gains ($K_p, K_i, K_d$) are dynamically updated by the PSO algorithm via `assignin` into the MATLAB base workspace.

4. **Actuator Constraint (`Saturation`):**

   * Restricts the commanded voltage to the physical power supply limits:
     

     $$
     V_{armature} \in [-24\text{ V}, \, +24\text{ V}]
     $$

   * **Engineering Relevance:** In unconstrained linear models, unguided optimizers can choose unrealistically high gains that demand hundreds of volts. The saturation block guarantees that solutions are physically realizable on realistic H-bridge hardware and penalizes windup-prone behaviors.

5. **DC Motor Plant (`Continuous Transfer Function` or `State-Space Subsystem`):**

   * Simulates the combined electrical and mechanical dynamics of the plant according to parameters $J, b, K, R, L$.

6. **Signal Loggers (`To Workspace`):**

   * Configured in `Timeseries` format to capture:

     * `e`: Error signal over simulation duration $T = 3\text{ s}$.

     * `y`: Rotor velocity output trajectory.

## 🎯 Cost Function Formulation

To balance rapid response time against excessive mechanical stress, the objective function combines the **Integral of Time-weighted Absolute Error (ITAE)** with a hard penalization for overshoot:

$$
J(K_p, K_i, K_d) = \int_{0}^{T} t \cdot \vert{}e(t)\vert{} \, dt + 50 \cdot M_p
$$

### Loss Component Breakdown:

* **ITAE Component (**$\int_{0}^{T} t \cdot \vert{}e(t)\vert{}\,dt$**):**

  * Early transient deviations (at small $t$) are permitted without heavy penalty, allowing the motor to accelerate.

  * Persistent tracking offsets or late-stage ringing are amplified linearly by $t$, driving steady-state error strictly to zero.

* **Overshoot Penalty (**$50 \cdot M_p$**):**
  

  $$
  M_p = \max\Big(0, \, \max(y(t)) - 1.0\Big)
  $$

  * Without an explicit penalty, optimization algorithms often minimize ITAE by driving gains aggressively high, producing large overshoots.

  * A scaling factor of $50$ heavily penalizes any trajectory exceeding the $1.0\text{ rad/s}$ setpoint, directing the swarm toward critically damped or non-overshooting profiles.

## 🔍 Particle Swarm Optimization (PSO) Setup

The search space is explored using MATLAB's Global Optimization Toolbox routine `particleswarm`:

| Hyperparameter | Value | Description | 
| ----- | ----- | ----- | 
| **Search Space Dimension** | 3 | $[K_p, K_i, K_d]$ | 
| **Swarm Size** | 15 | Number of candidate parameter vectors evaluated per iteration | 
| **Max Iterations** | 20 | Generation budget ceiling | 
| **Lower Bounds (`lb`)** | `[0, 0, 0]` | Positivity requirement for controller stability | 
| **Upper Bounds (`ub`)** | `[500, 300, 10]` | Practical hardware bounds preventing derivative noise amplification | 

## 📈 Optimization Results & Convergence

### Convergence History:

```
                                 Best            Mean     Stall
Iteration     f-count            f(x)            f(x)    Iterations
    0              15           1.003           5.713        0
    1              30          0.2236           8.525        0
    2              45          0.1247           6.579        0
    3              60         0.06201          0.9886        0
    4              75         0.06201          0.8641        1
    5              90         0.06201          0.9265        2
    6             105         0.05619           8.219        0
    7             120         0.05619           5.465        1
    8             135         0.05252           2.184        0
    9             150         0.04142           5.883        0
   10             165         0.04142           1.575        1
   11             180         0.02962           8.625        0
   12             195         0.02962           2.897        1
   ...
   20             315         0.02962          0.2496        9
Optimization ended: number of iterations exceeded OPTIONS.MaxIterations.

```

### Optimal Controller Parameters:

* **Proportional Gain (**$K_p$**):** `203.7924`

* **Integral Gain (**$K_i$**):** `46.6451`

* **Derivative Gain (**$K_d$**):** `9.9838`

* **Minimum Cost (**$J_{min}$**):** `0.0296`

### Performance Observations:

* **Zero Overshoot (**$M_p = 0\%$**):** Suppressed by the overshoot penalty term ($50 \cdot M_p$).

* **Eliminated Steady-State Error:** Convergence of $K_i \approx 46.65$ eliminates DC offset.

* **Rapid Settling:** Reaches setpoint in under 0.5 seconds without exceeding saturation limits.

## 📊 Velocity Response Plot

 `docs/response_curve.png` 

## 📂 File Manifest

* `run_pso_pid.m`: Executable driver script setting plant parameters, launching `particleswarm`, and rendering the final step response.

* `pid_cost.m`: Objective function injecting candidate gains into the base workspace, executing `sim`, and computing the scalar loss $J$.

* `dc_motor_model_cost.slx`: Closed-loop Simulink plant and controller model with voltage saturation and signal export blocks.

* `README.md`: Technical documentation for this benchmark.

## 🚀 How to Run

1. Open MATLAB and navigate to this directory:

   ```
   cd('01_DC_Motor_Speed_PID_PSO');
   
   ```

2. Confirm that `dc_motor_model_cost.slx` and `pid_cost.m` are in the current working directory.

3. Execute the driver script:

   ```
   run_pso_pid
   
   ```

4. Inspect the printed parameters and the resulting closed-loop step response figure.

## 📚 References

* Messner, W., Tilbury, D., et al. *"Control Tutorials for MATLAB & Simulink (CTMS) - DC Motor Speed: Simulink Controller Design"*, University of Michigan & Carnegie Mellon University.

  Available at: <https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SimulinkControl>
