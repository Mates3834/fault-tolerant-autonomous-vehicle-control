# Observer-Based Fault Detection and Fault-Tolerant Control

Generic research simulation for studying actuator/sensor faults, residual-based
fault detection, state estimation, and controller reconfiguration in an
autonomous vehicle control loop.

Implemented:
- linear lateral/yaw-like plant
- LQR nominal controller
- Luenberger observer
- residual generation
- threshold-based fault detection
- actuator effectiveness-loss scenario
- sensor-bias scenario
- simple controller reconfiguration
- baseline vs fault-tolerant comparison
- RMSE and residual metrics

Run:
```bash
pip install -r requirements.txt
python examples/run_comparison.py
```
