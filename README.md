# Underactuated-Robotics
Nonlinear cart–ball dynamics with cubic trajectory optimization, LQR tracking, ballistic landing prediction, animated visualization, and ROA computation over ball initial conditions.

# Cart–Ball Catching System  
MATLAB Implementation with Trajectory Planning, Animation, and ROA Analysis

This repository contains a complete MATLAB implementation of a **cart–ball catching system**, where a cart attempts to intercept a free-falling ball using:

- Ballistic landing prediction  
- Cubic trajectory generation  
- LQR tracking control  
- Closed-loop nonlinear simulation  
- Animated visualization  
- Region of Attraction (ROA) analysis

The project demonstrates trajectory tracking, optimal control, and simulation-based analysis.

---

## Features

### **1. Ball Landing Prediction**
The file `predict_landing.m` analytically computes:
- The time at which the ball reaches the cart height  
- The horizontal landing location  
- Whether a valid ballistic solution exists  

This is used to determine the reference target for the cart.

### **2. Trajectory Planning + LQR Control**
The main simulation:
- Builds a cubic polynomial trajectory for the cart  
- Designs an LQR regulator on the linearized cart dynamics  
- Adds a feedforward term to follow desired acceleration  
- Simulates the full nonlinear cart–ball system  

### **3. Closed-Loop Nonlinear Simulation**
The system state is:  
[x_c, ẋ_c, x_b, y_b, ẋ_b, ẏ_b]  
The cart dynamics also include friction.

### **4. Catch Detection**
A catch is considered successful if:  
y_b(t_hit) = y_catch  
|x_c(t_hit) - x_b(t_hit)| < ε  
where
ε = (cart_width / 2) - ball_radius  
ensuring the ball fits within the cart at impact.

### **5. Animation**
`run_traj_LQR.m` produces a smooth 2D animation of:
- Cart motion
- Ball trajectory
- Predicted landing point

### **6. ROA (Region of Attraction) Analysis**
`compute_ROA.m` generates a blue/orange map showing which initial ball positions lead to successful catches.

The ROA is computed over:
x_b(0) ∈ [−2.5, 2.5]
y_b(0) ∈ [0, 5]
resolution = 0.1 m
Blue = catch successful, Orange = catch failed.

---

## File Structure

.  
├── compute_ROA.m  
├── predict_landing.m  
├── README.md  
├── run_LQR.m  
└── run_traj_LQR.m  

---

## Main Scripts

### **run_traj_LQR.m**
Runs the complete simulation:
- Predict landing  
- Generate trajectory  
- Apply LQR  
- Simulate dynamics  
- Detect catch  
- Plot results  
- Animate the system  

### **compute_ROA.m**
Computes a 2D ROA map in \((x_{b0}, y_{b0})\) space by:
- Predicting landing for each initial ball position  
- Generating a trajectory for the cart  
- Running closed-loop simulation  
- Checking catch feasibility  

### **run_LQR.m**
Runs a simpler controller where the cart:
- Uses LQR to move directly toward the predicted landing position  
- Does **not** use a time-varying trajectory (no cubic planner)  
- Simulates the closed-loop dynamics and evaluates whether the catch is successful  

This script is useful to compare basic LQR regulation against the full trajectory-planning + LQR approach.

### **predict_landing.m** 
Solves the ballistic equation to determine:
- Landing time  
- Landing horizontal position  
- Validity of the solution  

---

## Tunable Parameters (run_traj_LQR.m || run_LQR.m || compute_ROA.m)

The simulation exposes several physical and control parameters that can be modified directly.

### Physical Parameters
params.mc = 1.0;    % cart mass [kg]  
params.b  = 0.1;    % viscous friction [N·s/m]  
params.g  = 9.81;   % gravity [m/s²]

### Ball Initial Conditions
xb0   = 0.5;        % initial horizontal ball position [m]  
yb0   = 1.0;        % initial ball height [m]  
vxb0  = 0.5;        % initial horizontal ball velocity [m/s]  
vyb0  = -0.2;       % initial vertical ball velocity [m/s]

### Cart Initial Conditions
xc0    = 0.0;       % initial cart position [m]  
xcdot0 = 0.0;       % initial cart velocity [m/s]  
y_catch = 0.0;      % cart/track height [m]  

### Actuator Limits
params.u_max = 15;   % maximum actuator force [N]  
params.u_min = -15;  % minimum actuator force [N]

### Cart and Ball Geometry
cart_w = 0.3;    % cart width [m]  
cart_h = 0.1;    % cart height [m]  
ball_r = 0.05;   % ball radius [m]

## LQR Tuning Parameters

The LQR regulator can be tuned through the weighting matrices:

Q = diag([50, 5]);   % cost on cart position error and velocity  
R = 10;              % cost on control 

Increasing Q results in more aggressive tracking, while increasing R produces smoother but slower control action.

## Remark on compute_ROA.m

In the ROA script (`compute_ROA.m`), the initial ball position **is not set manually as a single parameter**.  
Instead, the script evaluates the catch feasibility over an entire **grid** of initial positions.

You must specify:  
- The **range** of initial horizontal positions:  
  e.g., `xb0_vals = -2.5 : 0.1 : 2.5`
- The **range** of initial vertical positions:  
  e.g., `yb0_vals = 0.0 : 0.1 : 5.0`
- The **resolution** (grid spacing):  
  e.g., `0.05 m`

The script then simulates **every combination** of \((x_{b0}, y_{b0})\) in the grid and marks each point as a successful catch (blue) or a failure (orange).  
Thus, unlike `run_traj_LQR.m`, the ball initial position is **not a tunable parameter**, but part of the ROA search space.

---

## Requirements

- MATLAB R2021a or newer  
- MATLAB Control System Toolbox (required for the LQR controller) 

---

## Notes

- Helper functions such as `ref_traj` and `closed_loop_dynamics` are embedded inside the scripts.
- Only `predict_landing.m` is shared across the codebase.
- ROA computation is independent of animation for speed and efficiency.

---

## Possible Improvements

- 3D extension  
- Disturbance / wind modeling  
- ROA computation using parallel computing  

---

## Authors
Maher Abou Dargham  
Maria Ghorayeb
