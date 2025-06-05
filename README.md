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

## 📌 Project Scope

- **Objective**: To develop and validate an intelligent adaptive control framework capable of online tuning of PID parameters using biologically inspired ICO learning.
- **Application**: Dual-motor cascade control of a robotic arm with IMU and encoder feedback.
- **Approach**: Integration of Hebbian-based ICO learning with traditional PID control for enhanced robustness and autonomy.

---

## 📁 Repository Structure

Root/<br>
│<br>
├── Arduino files/ # Embedded implementation (Arduino)<br>
│ └── ICO v3/<br>
│ ├── Inner_motor/<br>
│ ├── Outer_motor/<br>
│ └── OneMaster_ICO_encode/<br>
│<br>
├── ICO_cpp/ # C++ implementation and core logic<br>
│<br>
├── plots/ # Output visualizations from simulation test plots<br>
│<br>
├── .vscode/ # Dev environment settings<br>
│<br>
├── Filterring_v1_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 2 without disturbance<br>
├── Filterring_v2_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 1 without disturbance<br>
├── Filterring_v3_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 2 with ramp disturbance<br>
├── Filterring_v4_Multi_PID_ICO_control_simulateRobotArm_FK_angle.m # Simulink: Option 1 with ramp disturbance<br>
│<br>
├── pid_values.csv # Logged PID values from Matlab<br>
├── ICO_values.csv # Logged ICO weights from Matlab<br>
├── pos_U_values.csv # System output logs (position, control effort) from Matlab<br>
├── PID_ICO_param_plot.ipynb # Jupyter notebook for analysis and plotting<br>
├── robot_arm_simulation.gif # Simulation visualization output from Matlab<br>


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

Open `.m` files in MATLAB to simulate behavior under various conditions.

### 2. Real-Time Implementation
Navigate to `Arduino files/ICO v3/` for the source code deployed on a robotic arm setup using Arduino. It supports a dual-motor cascade PID controller configuration.

- `Inner_motor/`: Controls joint-level dynamics.
- `Outer_motor/`: Balances the top segment.
- `OneMaster_ICO_encode/`: Implements ICO learning and handles motor communication.

#NOTE: Still under development
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

## 🧪 Real-Time Implementation (Arduino)

Under `Arduino files/ICO v3/`, the following modules are available:

- `Inner_motor/`: Handles inner loop PID control.
- `Outer_motor/`: Manages outer loop PID control.
- `OneMaster_ICO_encode/`: Implements ICO learning and manages motor communication.

This directory contains the firmware for deploying the adaptive controller on an embedded system with the specifications used for the robot arm in the project.

---

## 📊 Outputs & Logging

CSV logs include:
- `pid_values.csv`: Logged PID gain values over time.
- `ICO_values.csv`: Tracked ICO learning weights.
- `pos_U_values.csv`: Joint position and control effort data.

Use this data for post-analysis in Python or MATLAB.

---

### Jupyter Notebook:
Use `PID_ICO_param_plot.ipynb` to visualize:

- Gain evolution
- System response under various disturbances
- ICO weight convergence

---

## ✍️ Citation

If you use this work in your own research or implementation, please cite:

> Adrian Anthony, *Automatic Tuning of Multiple PID Motor Controllers for a Self-Balancing Platform via Machine Learning*, Aarhus University, 2025.

---

## 🔧 Acknowledgments

Supervised by Assoc. Prof. Danish Shaikh  
Department of Electrical and Computer Engineering  
Aarhus University, Denmark

