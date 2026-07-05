# Finite vs. Infinite Horizon Linear Quadratic Regulator (LQR)

## Overview

This project presents a comparative implementation of finite- and infinite-horizon Linear Quadratic Regulators (LQR) for continuous-time linear systems. The controllers are designed by solving the Differential Riccati Equation (DRE) and the Algebraic Riccati Equation (ARE), respectively.

The implementation investigates the effect of the optimization horizon on controller performance by comparing state evolution, control effort, costate trajectories, and closed-loop stability.

---

## Objectives

- Design finite-horizon optimal state-feedback controllers.
- Design infinite-horizon optimal state-feedback controllers.
- Solve the Differential Riccati Equation (DRE).
- Solve the Algebraic Riccati Equation (ARE).
- Verify the costate dynamics analytically.
- Compare controller performance using numerical simulations.

---

## Mathematical Formulation

The project considers a continuous-time Linear Time-Invariant (LTI) system of the form

**State Equation**

ẋ(t) = Ax(t) + Bu(t)

where:

- **x(t)** is the system state vector.
- **u(t)** is the control input.
- **A** is the system matrix.
- **B** is the input matrix.

The objective is to determine an optimal state-feedback control law that minimizes a quadratic performance index consisting of state deviation and control effort.

Two formulations are implemented:

- **Finite Horizon LQR** – obtained by solving the Differential Riccati Equation (DRE) backward in time with a specified terminal cost.
- **Infinite Horizon LQR** – obtained by solving the Algebraic Riccati Equation (ARE) to compute the steady-state optimal feedback gain.

The resulting controllers are compared based on state trajectories, control inputs, costate dynamics, and overall closed-loop performance.
## Features

- Finite Horizon LQR
- Infinite Horizon LQR
- Differential Riccati Equation Solver
- Algebraic Riccati Equation Solver
- Forward Euler Numerical Integration
- Analytical Costate Verification
- Comparative State and Control Analysis

---

## Simulation Outputs

The implementation generates comparative plots for

- State trajectories
- Control input
- Costate trajectories
- Costate verification error

Additional performance plots will be added in future updates.

---

## Software Requirements

- MATLAB
- Control System Toolbox

---

## Folder Structure

```text
01_LQR_Finite_vs_Infinite_Horizon/
│
├── LQR_Finite_vs_Infinite_Horizon.m
├── images/
└── README.md
```

---

## Key Learning Outcomes

- Finite vs. Infinite Horizon Optimal Control
- Differential and Algebraic Riccati Equations
- Optimal State Feedback Design
- Hamiltonian-Based Optimal Control
- Costate Dynamics
- Numerical Simulation of Continuous-Time Systems

---

## Future Improvements

- Automatic figure export
- Control-energy comparison
- Riccati matrix evolution
- State norm comparison
- Optimal cost analysis
- Closed-loop eigenvalue visualization
- Modular MATLAB implementation