# Model Predictive Control - Position Control

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=simorxb/position_control_mpc)

## Summary
This project demonstrates the application of Model Predictive Control (MPC) to a classical position control problem of a damped mass system. The control algorithm is implemented and simulated using MATLAB/Simulink.

## Project Overview
Model Predictive Control (MPC) is an advanced control strategy that solves an optimization problem at each control step, predicting the system behavior over a finite horizon. This project focuses on the control of a mass-spring-damper-like system, where a mass is subject to damping and external force, with the objective of accurately controlling its position.

The system's dynamics are modeled in MATLAB, and a linear MPC controller is designed using state-space representation. The implementation highlights key aspects such as tuning, constraints, and simulation outcomes, providing insights into the effectiveness of MPC for linear position control tasks.

## System Model
The physical system considered is a mass $m = 10 ~ kg$, sliding on a surface with damping coefficient $k = 0.5 ~ N \cdot s/m$, controlled by an external force $F$. The dynamic equation governing the system is:

$$
m \cdot \ddot{z} = F - k \cdot \dot{z}
$$

### State-Space Representation
To design an MPC controller, the system is represented in state-space form:

$\dot{x} = A \mathbf{x} + B u$

$y = C \mathbf{x} + D u$

Where:
- $\mathbf{x} = [\dot{z}, z]^T$
- $u = F$

Matrices:
- $A = \left[ -k/m ~~ 0; ~~ 1 ~~ 0 \right]$
- $B = \left[ 1/m ; ~~ 0 \right]$
- $C = \left[ 0 ~~ 1 \right]$
- $D = 0$

## MPC Tuning & Implementation
The linear MPC controller was configured in MATLAB with the following specifications:

### Tuning Parameters
- **Scaling Factors**:
  - Output Variable: 2
  - Manipulated Variable: 10
- **Weights**:
  - Output Variable: 1
  - Manipulated Variable: 0
  - Manipulated Variable Rate of Change: 0.1
- **Constraints**:
  - Output Variable: $[-2, 2] ~ m$
  - Manipulated Variable: $[-10, 10] ~ N$
- **MPC Parameters**:
  - Sample Time: $0.1 ~ s$
  - Prediction Horizon: 10
  - Control Horizon: 2

### MATLAB Implementation
- The linear MPC object was created using MATLAB’s `mpc` command, utilizing the defined state-space model.
- Simulation was performed using the `sim` function over a time span of 5 seconds.

## Simulation Results
The MPC algorithm achieved smooth position control with minimal overshoot. Since the same model was used for both the controller design and validation, the system performance was nearly ideal. Future improvements could include robustness testing and disturbance rejection analysis.

## Explore Different Constraints

The previous simulation was performed with the manipulated variable ($F$) limited between -10 N and 10 N.

Now we want to explore more combinations of constraints to see how the MPC algorithm handles constraints and how they affect the response.

We will choose:

- [-5 N, 5 N]
- [-10 N, 10 N]
- [-20 N, 20 N]

### Simulation Result - Different Constraints

The simulation is performed directly in MATLAB using the command `sim`, for 6 seconds, on the nominal system, with the 3 different combinations of constraints listed above.

As expected, as the control input range is reduced, the control performance is degraded. Still, the MPC always finds the optimal solution by "knowing" the limits and the system's response in advance.

## Effect of Different Constraints on the Controlled Variable

For this analysis, we want to test the effect of constraints on the controlled variable (the position).

To make the effect more visible, the MPC controller has been modified to be more aggressive (weight on the manipulated variable rate of change = 0) and cause more overshoot.

The overshoot with no constraints is up to 1.1 m. We want to test the impact of:

- Position constraint: [-1.15, 1.15] m
- Position constraint: [-1.05, 1.05] m
- Position constraint: [-1.02, 1.02] m

### Simulation Result

The simulation is performed directly in MATLAB using the command `sim`, for 6 seconds, on the nominal system, with the 3 different combinations of constraints listed above.

We can observe how the MPC algorithm changes its control approach to prevent the controlled variable (the position) from going above the constraint.

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
