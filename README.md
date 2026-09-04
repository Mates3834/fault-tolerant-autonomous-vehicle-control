# Observer-Based Fault Detection and Fault-Tolerant Control for Autonomous Vehicles

A research-oriented simulation framework for studying **fault detection, state estimation, and fault-tolerant control in autonomous vehicle systems**.

The project integrates:

- Linear dynamic system modelling
- LQR feedback control
- Luenberger state observation
- Residual-based fault detection
- Actuator effectiveness-loss modelling
- Sensor bias modelling
- Fault-triggered controller reconfiguration
- Closed-loop performance evaluation

The primary objective is to investigate how model-based estimation and residual monitoring can be integrated with feedback control to detect abnormal system behaviour and support fault accommodation.

---

# 1. Motivation

Autonomous systems depend on sensors, actuators, estimators, and controllers operating together in a closed-loop architecture.

A conventional controller can provide satisfactory performance under nominal conditions:

```text
Reference
    ↓
Controller
    ↓
Actuator
    ↓
Vehicle
    ↓
Sensors
    ↓
Feedback
```

However, actuator degradation or sensor faults can alter the behaviour of the closed-loop system.

A fault-tolerant architecture extends the conventional control loop with:

```text
State Estimation
      +
Residual Generation
      +
Fault Detection
      +
Controller Reconfiguration
```

The resulting architecture becomes:

```text
Reference
    ↓
Controller
    ↓
Actuator
    ↓
Vehicle
    ↓
Sensor Measurements
    ↓
State Observer
    ↓
Residual Generator
    ↓
Fault Detector
    ↓
Controller Reconfiguration
```

---

# 2. Project Objectives

The framework is designed to study:

1. Nominal closed-loop control
2. State estimation using an observer
3. Residual generation
4. Detection of abnormal system behaviour
5. Actuator effectiveness loss
6. Sensor bias faults
7. Fault-triggered controller accommodation
8. Baseline versus fault-tolerant control performance

The implementation is intentionally generic and is not associated with a specific aircraft, marine vehicle, or automotive platform.

---

# 3. System Architecture

The implemented architecture can be represented as:

```text
                    Reference
                        ↓
                 ┌─────────────┐
                 │     LQR     │
                 │ Controller  │
                 └──────┬──────┘
                        ↓
                 Control Command
                        ↓
                 ┌─────────────┐
                 │  Actuator   │
                 │ Fault Model │
                 └──────┬──────┘
                        ↓
                 ┌─────────────┐
                 │   Vehicle   │
                 │    Model    │
                 └──────┬──────┘
                        ↓
                   True State
                        ↓
                 ┌─────────────┐
                 │   Sensor    │
                 │ Fault Model │
                 └──────┬──────┘
                        ↓
                  Measurement
                        ↓
                 ┌─────────────┐
                 │ Luenberger  │
                 │  Observer   │
                 └──────┬──────┘
                        ↓
              Estimated State
                        +
                    Residual
                        ↓
                 Fault Detector
                        ↓
             Reconfiguration Logic
```

---

# 4. Dynamic System Model

The current project uses a generic two-state linear system representing lateral/yaw-like autonomous-vehicle dynamics.

The continuous-time model is:

```text
x_dot = A x + B u
```

with output:

```text
y = C x
```

where:

```text
x = system state vector
u = control input
y = measured output
```

The current state dimension is:

```text
x ∈ R²
```

The model is intentionally generic rather than representing a particular physical vehicle.

---

# 5. Discrete-Time Model

The continuous system is discretized for numerical simulation.

The resulting model is:

```text
x(k+1) =
A_d x(k)
+
B_d u(k)
```

and:

```text
y(k) =
C x(k)
```

The discrete model is used by both the controller and state observer.

---

# 6. LQR Controller

Nominal control is implemented using a Linear Quadratic Regulator.

The controller minimizes the quadratic objective:

```text
J =
Σ [
x(k)^T Q x(k)
+
u(k)^T R u(k)
]
```

where:

```text
Q = state penalty matrix
R = control penalty matrix
```

The control law is:

```text
u =
-K(x - x_ref)
```

where:

```text
K = LQR feedback gain
x_ref = reference state
```

---

# 7. Control Constraints

The generated control command is bounded:

```text
u_min ≤ u ≤ u_max
```

In the generic implementation:

```text
-1 ≤ u ≤ 1
```

This prevents the simulation from assuming unlimited control authority.

---

# 8. State Estimation

A Luenberger observer is used to estimate the internal system state.

The observer has the form:

```text
x_hat(k+1) =
A_d x_hat(k)
+
B_d u(k)
+
L[
y(k) - C x_hat(k)
]
```

where:

```text
x_hat = estimated state
L = observer gain
```

The difference:

```text
r(k) =
y(k)
-
C x_hat(k)
```

forms the observer innovation or residual.

---

# 9. Observer Pole Placement

The observer gain is calculated through pole placement.

The observer error dynamics are governed by:

```text
e(k+1) =
(A_d - LC)e(k)
```

where:

```text
e =
x - x_hat
```

Observer poles are selected to provide stable estimation-error convergence.

---

# 10. Residual Generation

Fault detection is based on the observer residual.

The residual vector is:

```text
r(k) =
y(k)
-
C x_hat(k)
```

A scalar residual score is obtained using:

```text
J_r(k) =
||r(k)||
```

Under nominal conditions, the residual is expected to remain relatively small apart from measurement noise and modelling error.

A fault may cause the residual magnitude to increase.

---

# 11. Threshold-Based Fault Detection

The current detector compares the residual score against a predefined threshold:

```text
J_r > J_threshold
```

A single threshold crossing is not immediately interpreted as a persistent fault.

Instead, persistence logic is used.

Conceptually:

```text
Residual
    ↓
Threshold Test
    ↓
Persistence Counter
    ↓
Fault Flag
```

This reduces sensitivity to isolated measurement-noise peaks.

---

# 12. Actuator Fault Model

The first fault scenario represents actuator effectiveness loss.

The effective control input becomes:

```text
u_effective =
η u_command
```

where:

```text
0 < η ≤ 1
```

Under nominal operation:

```text
η = 1
```

During actuator degradation:

```text
η < 1
```

The current demonstration introduces a predefined actuator effectiveness reduction after a specified fault-onset time.

---

# 13. Sensor Fault Model

The second fault type is a sensor bias.

The measured state becomes:

```text
y_fault =
y
+
b
+
v
```

where:

```text
b = sensor bias
v = measurement noise
```

The bias is introduced after a predefined time in the simulation.

This allows the observer residual to respond to abnormal sensor behaviour.

---

# 14. Measurement Noise

Synthetic Gaussian measurement noise is added to the sensor output.

The measurement model therefore becomes:

```text
y_m =
Cx
+
b_fault
+
v
```

This ensures that the fault detector operates in a noisy rather than perfectly deterministic environment.

---

# 15. Fault Detection Logic

The implemented fault-detection sequence is:

```text
Sensor Measurement
        ↓
Observer Prediction
        ↓
Innovation / Residual
        ↓
Residual Norm
        ↓
Threshold Comparison
        ↓
Persistence Check
        ↓
Fault Detected
```

The current implementation detects abnormal residual behaviour but does not provide a complete multi-fault diagnostic classifier.

---

# 16. Fault-Tolerant Controller

A simple fault-tolerant extension of the nominal LQR controller is included.

When actuator effectiveness is assumed to decrease, the control command can be compensated according to:

```text
u_FTC =
u_LQR / η_hat
```

where:

```text
η_hat
```

represents the assumed or estimated actuator effectiveness.

The compensated command remains subject to actuator saturation.

---

# 17. Important Implementation Detail

In the current demonstration, the actuator-loss magnitude used for controller accommodation is supplied by the synthetic scenario after fault detection.

Therefore, the current framework demonstrates:

```text
Fault Detection
      ↓
Known Synthetic Fault Magnitude
      ↓
Controller Reconfiguration
```

It does **not** yet implement online actuator-effectiveness estimation.

A future version could estimate:

```text
η_hat
```

directly from measurements and residual information.

---

# 18. Nominal Control Architecture

The baseline architecture is:

```text
Reference
    ↓
LQR
    ↓
Faulty Actuator
    ↓
Vehicle
    ↓
Sensors
    ↓
Observer
    ↓
Feedback
```

No explicit fault accommodation is performed.

---

# 19. Fault-Tolerant Architecture

The fault-tolerant architecture becomes:

```text
Reference
    ↓
LQR
    ↓
Fault Compensation
    ↓
Actuator
    ↓
Vehicle
    ↓
Sensors
    ↓
Observer
    ↓
Residual
    ↓
Fault Detection
    ↓
Reconfiguration
```

This provides a compact framework for studying active fault-tolerant control concepts.

---

# 20. Simulation Scenario

The demonstration includes:

```text
Initial Nominal Operation
        ↓
Reference Command
        ↓
Actuator Effectiveness Loss
        ↓
Residual Monitoring
        ↓
Fault Detection
        ↓
Controller Reconfiguration
        ↓
Sensor Bias Introduction
        ↓
Continued Closed-Loop Operation
```

This allows multiple abnormal conditions to be examined during the same simulation.

---

# 21. Performance Metric

The primary tracking metric is root-mean-square error:

```text
RMSE =
sqrt(
mean(
(x_1 - x_ref,1)²
)
)
```

Additional quantities available from the simulation include:

```text
Observer Residual
Control Input
Fault Detection Flag
State Estimate
True State
Reference State
```

---

# 22. Current Sanity Check

A short software sanity check of the current generic configuration produced approximately:

| Controller | Tracking RMSE |
|---|---:|
| Nominal LQR under faults | 0.246 |
| Current FTC implementation | 0.246 |

This result is intentionally reported without claiming an FTC performance improvement.

The current configuration demonstrates the software architecture and reconfiguration mechanism, but further controller/fault-estimation tuning is required before making comparative performance claims.

---

# 23. Why This Result Matters

Fault-tolerant control should not be evaluated only by the presence of reconfiguration logic.

A meaningful evaluation should demonstrate measurable differences in quantities such as:

```text
Tracking Error
Recovery Time
Peak Error
Control Effort
Residual Magnitude
Detection Delay
```

The current framework therefore provides a foundation for further fault-accommodation studies rather than claiming that the present controller is already optimal.

---

# 24. Repository Structure

```text
fault_tolerant_autonomous_vehicle_control/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── docs/
│   └── scope.md
│
├── src/
│   ├── __init__.py
│   ├── model.py
│   ├── controllers.py
│   ├── observer.py
│   ├── detector.py
│   ├── faults.py
│   └── simulation.py
│
├── examples/
│   └── run_comparison.py
│
└── results/
    └── sanity.txt
```

---

# 25. Module Description

| Module | Purpose |
|---|---|
| `model.py` | Generic linear vehicle dynamics |
| `controllers.py` | LQR and fault-tolerant LQR |
| `observer.py` | Luenberger state observer |
| `detector.py` | Residual-based fault detection |
| `faults.py` | Actuator and sensor fault models |
| `simulation.py` | Integrated closed-loop simulation |
| `run_comparison.py` | Baseline vs FTC comparison |

---

# 26. Installation

Clone the repository:

```bash
git clone <repository-url>
cd fault-tolerant-autonomous-vehicle-control
```

Install dependencies:

```bash
pip install -r requirements.txt
```

Dependencies:

```text
NumPy
SciPy
Matplotlib
```

---

# 27. Running the Simulation

Run:

```bash
python examples/run_comparison.py
```

The script executes:

```text
Nominal LQR under faults
          vs
Fault-Tolerant LQR
```

and generates plots for:

- Reference tracking
- Baseline response
- Fault-tolerant response
- Observer residual

---

# 28. Recommended Future Results

After improving the fault-accommodation mechanism, useful figures include:

```text
results/
├── nominal_vs_faulty.png
├── baseline_vs_ftc.png
├── observer_estimation.png
├── residual_history.png
├── fault_detection.png
├── control_input.png
└── recovery_comparison.png
```

A useful final comparison table would be:

| Method | RMSE | Peak Error | Detection Delay | Recovery Time | Control Effort |
|---|---:|---:|---:|---:|---:|
| Nominal LQR | measured | measured | — | measured | measured |
| LQR + FTC | measured | measured | measured | measured | measured |

Only measured simulation results should be reported.

---

# 29. Technologies

- Python
- NumPy
- SciPy
- Matplotlib
- Linear Systems
- LQR
- Luenberger Observer
- State Estimation
- Fault Detection
- Residual Analysis
- Fault-Tolerant Control
- Numerical Simulation

---

# 30. Research Areas

The project is related to:

- Autonomous Systems
- Fault Detection and Isolation
- Fault-Tolerant Control
- Guidance and Control
- State Estimation
- Observer Design
- Resilient Autonomous Systems
- Vehicle Control
- Robotics

---

# 31. Current Scope

Implemented:

```text
Generic Linear Vehicle Model
LQR
Luenberger Observer
Residual Generation
Threshold-Based Detection
Persistence Logic
Actuator Effectiveness Loss
Sensor Bias
Measurement Noise
Controller Reconfiguration
Baseline / FTC Comparison
```

---

# 32. Current Limitations

The current implementation does not include:

- Online fault-magnitude estimation
- Formal fault isolation
- Multiple-model estimation
- Unknown-input observers
- Adaptive observers
- Kalman-filter-based FDI
- MPC-based fault accommodation
- Nonlinear vehicle dynamics
- Multiple simultaneous actuator faults
- Actuator stuck faults
- Sensor dropout
- Hardware-in-the-loop testing
- Real vehicle experiments

The current system should therefore be interpreted as a **compact observer-based FDI and fault-tolerant control research framework**.

---

# 33. Future Extensions

## Online Fault Estimation

A major extension would be:

```text
Residual
    ↓
Fault Detection
    ↓
Fault Parameter Estimation
    ↓
η_hat
    ↓
Adaptive Controller Reconfiguration
```

This would remove the need to supply the actuator-loss magnitude from the simulation scenario.

## Fault Isolation

The detector could be extended from:

```text
Fault / No Fault
```

to:

```text
Actuator Fault
Sensor Fault
Model Disturbance
Normal Operation
```

using structured residuals or multiple observers.

## Kalman-Based FDI

The Luenberger observer could be compared with:

```text
Kalman Filter
Extended Kalman Filter
Unknown Input Observer
Multiple Model Estimator
```

## MPC-Based Fault Accommodation

A constrained controller could explicitly account for reduced actuator authority:

```text
Fault Estimate
      ↓
Updated Actuator Constraint
      ↓
MPC Optimization
      ↓
Feasible Control
```

## Nonlinear Vehicle Model

The current two-state system could be replaced with a higher-fidelity:

```text
UAV
USV
Ground Vehicle
```

dynamic model.

---

# 34. Public Implementation Notice

This repository contains a **generic and sanitized research implementation**.

The public model intentionally excludes:

- Real vehicle operational parameters
- Platform-specific actuator models
- Safety-critical control logic
- Proprietary fault thresholds
- Restricted system configurations
- Real operational datasets

All parameters, faults, thresholds, and scenarios are synthetic examples.

---

# 35. Status

**Research-oriented simulation framework / active development**

The current project demonstrates:

```text
Dynamic Model
      ↓
LQR Control
      ↓
State Observer
      ↓
Residual Generation
      ↓
Fault Detection
      ↓
Controller Reconfiguration
      ↓
Performance Evaluation
```

The primary focus is on **model-based fault detection, resilient feedback control, state estimation, and fault-accommodation architectures for autonomous systems**.

---

# Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- Guidance, Navigation and Control
- Fault-Tolerant Control
- State Estimation
- Model Predictive Control
- Marine Robotics
- UAV Autonomy
- Reinforcement Learning
- Multi-Agent Systems
