# ICO-PID Adaptive Control for Robotic Arm Platform

This repository presents a machine learning-based approach for the real-time tuning of multiple PID motor controllers on a self-balancing robotic platform. The adaptive tuning method leverages **Input Correlation (ICO) Learning**, a biologically inspired algorithm, to estimate PID gains in nonlinear and dynamic environments.

## 📚 Project Overview

Traditional PID tuning methods often rely on detailed system models or manual calibration, which can be limiting for nonlinear systems or dynamic environments. This project proposes a scalable, model-free alternative using ICO Learning for continuous adaptation of PID parameters in both simulation and real-world applications.

### Key Features:
- Adaptive PID tuning using ICO Learning.
- Filtering and Poisson-based smoothing for stability and noise robustness.
- Ramp disturbance simulation support.
- Cascaded double-PID control for a 2-DOF robotic arm.
- MATLAB simulation environment and real-time Arduino deployment.
- Python tools for parameter and performance visualization.

---

## 📁 Repository Structure

Root/
│
├── Arduino files/ # Embedded implementation (Arduino)
│ └── ICO v3/
│ ├── Inner_motor/
│ ├── Outer_motor/
│ └── OneMaster_ICO_encode/
│
├── ICO_cpp/ # C++ implementation and core logic
│
├── plots/ # Output visualizations from simulation test plots
│
├── .vscode/ # Dev environment settings
│
├── Filterring_v1_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 2 without disturbance
├── Filterring_v2_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 1 without disturbance
├── Filterring_v3_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 2 with ramp disturbance
├── Filterring_v4_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 1 with ramp disturbance
│
├── pid_values.csv # Logged PID values from Matlab
├── ICO_values.csv # Logged ICO weights from Matlab
├── pos_U_values.csv # System output logs (position, control effort) from Matlab
├── PID_ICO_param_plot.ipynb # Jupyter notebook for analysis and plotting
├── robot_arm_simulation.gif # Simulation visualization output from Matlab


---

## 🚀 How to Use

### 1. MATLAB Simulations
Each version of the filtering simulation corresponds to a specific configuration:

| File                           | Configuration                                 |
|--------------------------------|-----------------------------------------------|
| `Filtering_v1_*.m`             | Option 2 - Standard adaptive control          |
| `Filtering_v2_*.m`             | Option 1 - Filtered adaptive control          |
| `Filtering_v3_*.m`             | Option 2 + Ramp disturbance                   |
| `Filtering_v4_*.m`             | Option 1 + Ramp disturbance                   |

Open `.slx` files in MATLAB Simulink to simulate behavior under various conditions.

### 2. Real-Time Implementation
Navigate to `Arduino files/ICO v3/` for the source code deployed on a robotic arm setup using Arduino. It supports a dual-motor cascade PID controller configuration.

- `Inner_motor/`: Controls joint-level dynamics.
- `Outer_motor/`: Balances the top segment.
- `OneMaster_ICO_encode/`: Implements ICO learning and handles motor communication.

### 3. Python Visualization
Use `PID_ICO_param_plot.ipynb` to analyze learning curves, PID parameters, and weight evolution from the `.csv` logs.

---

## 🧠 Methodology Summary

ICO Learning is used to correlate a delayed reflex signal (e.g., error) with a predictive signal (e.g., IMU position or velocity). By adapting the control gains based on this correlation, the algorithm provides:

- Low-latency responsiveness to disturbances.
- Minimal prior modeling.
- Real-time gain convergence and tuning.

Refer to the [Final_Report.pdf](link-to-final-report-if-uploaded) for theoretical formulations and stability proofs.

---

## 📊 Outputs & Logging

CSV logs include:
- `pid_values.csv`: Logged PID gain values over time.
- `ICO_values.csv`: Tracked ICO learning weights.
- `pos_U_values.csv`: Joint position and control effort data.

Use this data for post-analysis in Python or MATLAB.

---

## ✍️ Citation

If you use this work in your own research or implementation, please cite:

> Adrian Anthony, *Automatic Tuning of Multiple PID Motor Controllers for a Self-Balancing Platform via Machine Learning*, Aarhus University, 2025.

---

## 🔧 Acknowledgments

Supervised by Assoc. Prof. Danish Shaikh  
Department of Electrical and Computer Engineering  
Aarhus University, Denmark

