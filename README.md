# 🚁 Quadcopter Controller Comparison via MATLAB/Simulink

> A simulation-based comparative study of three control strategies applied to a 6-DOF quadcopter model: **PID with Disturbance Observer**, **LQR with Luenberger State Observer**, and **Nonlinear Backstepping Controller**.

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a%2Fb-orange?logo=mathworks)
![Simulink](https://img.shields.io/badge/Simulink-R2023a%2Fb-blue?logo=mathworks)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Simulation%20Complete-brightgreen)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Quadcopter Dynamic Model](#quadcopter-dynamic-model)
- [Controllers](#controllers)
  - [1. PID with Disturbance Observer](#1-pid-with-disturbance-observer)
  - [2. LQR with Luenberger State Observer](#2-lqr-with-luenberger-state-observer)
  - [3. Nonlinear Backstepping Controller](#3-nonlinear-backstepping-controller)
- [Repository Structure](#repository-structure)
- [Simulation Parameters](#simulation-parameters)
- [Results](#results)
- [Requirements](#requirements)
- [How to Run](#how-to-run)
- [References](#references)
- [Author](#author)

---

## Overview

This project implements and compares three different control architectures on a nonlinear 6-DOF quadcopter model built in MATLAB/Simulink. The quadcopter has **4 control inputs** (rotor speeds ω₁, ω₂, ω₃, ω₄) and **12 state outputs** (P, Q, R, Φ, θ, Ψ, U, V, W, X, Y, Z).

The system is inherently underactuated and nonlinear, making it an excellent benchmark for comparing classical, optimal, and nonlinear control approaches.

| Controller | Type | Key Feature |
|---|---|---|
| PID + DOB | Classical | Disturbance rejection via observer |
| LQR + Luenberger | Optimal Linear | State estimation + cost minimization |
| Backstepping | Nonlinear | Lyapunov-based recursive design |

---

## Quadcopter Dynamic Model

### Reference Frames

The dynamics are described using two frames:
- **Inertial frame {O}**: Earth-fixed, gravity pointing in -z direction
- **Body frame {B}**: Fixed to the quadcopter centroid

The absolute position is expressed as **ε = [X Y Z]ᵀ**, attitude angles as **η = [Φ θ Ψ]ᵀ** (roll, pitch, yaw), linear velocities as **V_B = [Vx Vy Vz]ᵀ**, and angular velocities as **v = [P Q R]ᵀ**.

The rotation matrix between frames is:

$$R_1 = \begin{bmatrix} C_\psi C_\theta & C_\psi S_\theta S_\phi - S_\psi C_\phi & C_\psi S_\theta C_\phi + S_\psi S_\phi \\ S_\psi C_\theta & S_\psi S_\theta S_\phi + C_\psi C_\phi & S_\psi S_\theta C_\phi - C_\psi S_\phi \\ -S_\theta & C_\theta S_\phi & C_\theta C_\phi \end{bmatrix}$$

where $C_\theta = \cos(\theta)$ and $S_\theta = \sin(\theta)$.

### Thrust and Torque Model

Each rotor produces thrust proportional to the square of its angular speed:

$$T_i = K\omega_i^2$$

The net thrust in the body frame Z direction:

$$T = K\sum\omega_i^2$$

The roll, pitch, and yaw moments are:

$$M_\phi = L(T_4 - T_2), \quad M_\theta = L(T_3 - T_1), \quad M_\psi = B(-\omega_1^2 + \omega_2^2 - \omega_3^2 + \omega_4^2)$$

The mapping from thrust/torques to individual rotor speeds is:

$$\begin{pmatrix} T \\ m_\phi \\ m_\theta \\ m_\psi \end{pmatrix} = \begin{pmatrix} K & K & K & K \\ 0 & KL & 0 & -KL \\ -KL & 0 & KL & 0 \\ -B & B & -B & B \end{pmatrix} \begin{pmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{pmatrix}$$

### Translational Dynamics

In the inertial frame, neglecting drag:

$$\dot{U} = \left(\sin\Phi\sin\Psi + \cos\Phi\sin\theta\cos\Psi\right)\frac{T}{m} - \frac{A_x}{m}U$$

$$\dot{V} = \left(-\sin\Phi\cos\Psi + \cos\Phi\sin\theta\sin\Psi\right)\frac{T}{m} - \frac{A_y}{m}V$$

$$\dot{W} = -g + \left(\cos\Phi\cos\theta\right)\frac{T}{m} - \frac{A_z}{m}W$$

### Rotational Dynamics

The angular accelerations in the body frame (Newton-Euler formulation):

$$\dot{P} = \left(\frac{I_{xx} - I_{yy}}{I_{zz}}\right)QR - \frac{I_R}{I_{xx}}Q\Omega + \frac{M_\phi}{I_{xx}} - \frac{A_r}{I_{xx}}P$$

$$\dot{Q} = \left(\frac{I_{zz} - I_{xx}}{I_{yy}}\right)PR - \frac{I_R}{I_{yy}}P\Omega + \frac{M_\theta}{I_{yy}} - \frac{A_r}{I_{yy}}Q$$

$$\dot{R} = \left(\frac{I_{xx} - I_{yy}}{I_{zz}}\right)PQ + \frac{M_\psi}{I_{zz}} - \frac{A_r}{I_{zz}}R$$

The transformation from body angular velocities to Euler angle rates:

$$\begin{bmatrix} \dot{\Phi} \\ \dot{\theta} \\ \dot{\Psi} \end{bmatrix} = \begin{bmatrix} 1 & \sin\Phi\tan\theta & \cos\Phi\tan\theta \\ 0 & \cos\Phi & -\sin\Phi \\ 0 & \frac{\sin\Phi}{\cos\theta} & \frac{\cos\Phi}{\cos\theta} \end{bmatrix} \begin{bmatrix} P \\ Q \\ R \end{bmatrix}$$

The Simulink plant model accepts **{ω₁, ω₂, ω₃, ω₄}** as inputs and outputs the full 12-state vector **{P, Q, R, Φ, θ, Ψ, U, V, W, X, Y, Z}**, implemented via a Level-2 S-Function block.

---

## Controllers

### 1. PID with Disturbance Observer

#### Control Law

A standard PID controller is applied independently to each attitude axis and altitude. For the roll axis:

$$m_\phi = I_{xx}\left(K_{\phi,D}\dot{e}_\phi(t) + K_{\phi,P}e_\phi(t) + K_{\phi,I}\int e_\phi(t)\,dt\right)$$

where the tracking error is $e_\phi(t) = \phi_d(t) - \phi(t)$.

For altitude control:

$$T = mC_\phi C_\theta\left[g + K_{z,D}\dot{e}_z(t) + K_{z,P}e_z(t) + K_{z,I}\int e_z(t)\,dt\right]$$

#### Disturbance Observer (DOB)

To improve robustness, a disturbance observer is augmented on the roll axis. The roll dynamics under disturbance $d$ are:

$$\dot{P} = \frac{1}{I_{xx}}\left[M_\phi - (I_{yy} - I_{zz})QR - K_2 \cdot P \cdot |P|\right] + d$$

The observer is constructed as:

$$\begin{cases} d_{ob} = z_{in} + k_{ob} \cdot P \\ \dot{z}_{in} = -k_{ob} \cdot d_{ob} - k_{ob} \cdot \frac{1}{I_{xx}}\left[M_\phi - (I_{yy} - I_{zz})QR - K_2 \cdot P \cdot |P|\right] \end{cases}$$

The observer error dynamics:

$$\dot{e}^* = -k_{ob} \cdot e^* + \dot{d}$$

where $e^* = d - d_{ob}$. If $\dot{d}$ is bounded and $k_{ob} > 0$, the estimation error $e^* \to 0$ as $\dot{d} \to 0$.

The observed disturbance is fed back to correct the control law:

$$\tau_{\phi,new} = \tau_{\phi,original} + d_{ob} \cdot \frac{I_{xx}}{2\sqrt{2} \cdot T_{max} \cdot L}$$

The gain selection for PID was performed via **root locus design** (see `scripts/pid/root_locus_ddsgn.m`).

---

### 2. LQR with Luenberger State Observer

#### Linearization

The nonlinear plant is linearized about the hover equilibrium point using the Jacobian approach:

$$A_r = \frac{\partial}{\partial x}A(0), \quad B_r = B(0)$$

The linearized system (8-state attitude subsystem):

$$\dot{X}_1 = A_r X_1 + B_r U, \quad Y_1 = C X_1$$

where the state matrices $A_r$ and $B_r$ are derived from the full nonlinear model at hover.

#### LQR Optimal Gain

The LQR minimizes the quadratic cost function:

$$J = \int_{t_0}^{\infty} \left\{ U(t)^T \cdot R \cdot U(t) + [X(t) - X_d(t)]^T \cdot Q \cdot [X(t) - X_d(t)] \right\} dt$$

where $Q$ is the state weighting matrix and $R$ is the control effort weighting matrix.

The optimal control law is a static linear feedback:

$$U = -K \cdot [X(t) - X_d(t)]$$

The gain matrix $K$ is computed by solving the Algebraic Riccati Equation (ARE):

$$K = \text{LQR}(A_r, B_r, Q, R)$$

#### Luenberger State Observer

Since not all states are directly measurable, a Luenberger observer reconstructs the full state vector:

$$\dot{\hat{X}} = A\hat{X} + BU + L(Y - C\hat{X})$$

The observer gain matrix $L$ is designed to place observer poles significantly faster than the closed-loop poles, ensuring rapid state estimation convergence. Gain design is performed in `scripts/lqr/luenbergergain.m`.

---

### 3. Nonlinear Backstepping Controller

#### State-Space Form

The quadcopter dynamics are written in cascade state-space form. For the roll subsystem:

$$\begin{cases} x_1' = x_2 \\ x_2' = a_1 x_4 x_6 - a_2 x_4 \Omega + b_1 U_2 \end{cases}$$

where the model parameters are:

$$a_1 = \frac{I_{yy} - I_{zz}}{I_{xx}}, \quad a_2 = \frac{J_r}{I_{xx}}, \quad b_1 = \frac{L}{I_{xx}}$$

#### Lyapunov-Based Control Design

**Step 1:** Define the roll tracking error $\varepsilon_1 = x_1^d - x_1$. Choose Lyapunov function $V_1 = \frac{1}{2}\varepsilon_1^2$. To make $\dot{V}_1 < 0$, set:

$$x_2^d = \dot{x}_1^d + K_1\varepsilon_1$$

**Step 2:** Define $\varepsilon_2 = x_2^d - x_2$. Augment $V_2 = V_1 + \frac{1}{2}\varepsilon_2^2$. The resulting control law that ensures $\dot{V}_2 \leq 0$:

$$U_2 = \frac{1}{b_1}\left[\varepsilon_1 - K_1 x_2 - a_1 x_4 x_6 + a_2 x_4 \Omega + K_2\varepsilon_2\right]$$

The same two-step recursive procedure is applied to **pitch (θ)**, **yaw (Ψ)**, **altitude (z)**, and **position (x, y)**, yielding 6 independent control laws. The complete set of control laws for all axes:

| Axis | Control Law |
|---|---|
| Roll (U₂) | $U_2 = \frac{1}{b_1}[\varepsilon_1 - K_1 x_2 - a_1 x_4 x_6 + a_2 x_4\Omega + K_2\varepsilon_2]$ |
| Pitch (U₃) | $U_3 = \frac{1}{b_2}[\varepsilon_3 - a_3 x_2 x_6 - a_4\Omega x_2 + K_4\varepsilon_4 - K_3 x_4]$ |
| Yaw (U₄) | $U_4 = \frac{1}{b_3}[\varepsilon_5 - a_5 x_2 x_4 - K_6\varepsilon_6 - K_5 x_6]$ |
| Altitude (U₁) | $U_1 = \frac{m}{\cos(x_1)\cos(x_3)}[-\varepsilon_7 + g - K_8\varepsilon_8 - K_7 x_8]$ |
| Position Y | $U_y = \frac{m}{U_1}[-\varepsilon_9 - K_{10}\varepsilon_{10} - K_9 x_{10}]$ |
| Position X | $U_x = \frac{m}{U_1}[-\varepsilon_{11} - K_{12}\varepsilon_{12} - K_{11}x_{12}]$ |

#### Gain Design (Critical Damping Method)

Controller gains are computed using the **PID-Backstepping similarity** framework. For a second-order backstepping system, the characteristic equation is:

$$s^2 + K_D s + K_P = (s + c_1)(s + c_2)$$

For **critical damping** (no oscillation, fastest settling), we set $c_1 = c_2 = \sqrt{K_P}$, which gives:

$$K_1 = K_2 = \sqrt{P_{user}}, \quad K_D = 2\sqrt{K_P}$$

This satisfies the stability constraint $K_D \geq 2\sqrt{K_P}$ from Lyapunov analysis.

| Axis | $P_{user}$ | $K_i$ (= $\sqrt{P}$) |
|---|---|---|
| Roll (Φ) | 6.0 | 2.449 |
| Pitch (θ) | 5.0 | 2.236 |
| Yaw (Ψ) | 0.3 | 0.548 |
| Altitude (Z) | 0.3437 | 0.586 |
| Position (X/Y) | 0.7 | 0.837 |

Gain computation is automated in `scripts/backstepping/parametreler.m`.

---

## Repository Structure

```
quadcopter-controller-comparison/
│
├── models/
│   ├── quadcopter.slx              ← PID + DOB Simulink model
│   ├── lqr_quad.slx                ← LQR + Luenberger Observer model
│   └── backstepping_last_good.slx  ← Backstepping controller model
│
├── scripts/
│   ├── pid/
│   │   └── root_locus_ddsgn.m      ← PID gain design via root locus
│   ├── lqr/
│   │   ├── lqrdesign.m             ← LQR Q/R matrix design & K computation
│   │   └── luenbergergain.m        ← Luenberger observer gain design
│   └── backstepping/
│       └── parametreler.m          ← Critical damping gain calculator
│
├── results/
│   ├── figures/                    ← Simulation output plots (.fig, .png)
│   └── data/                       ← Workspace data (.mat)
│
├── docs/                           ← Reports and documentation
├── .gitignore
└── README.md
```

---

## Simulation Parameters

Physical parameters used across all three simulations (based on a real quadcopter platform):

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Mass | m | 0.468 | kg |
| Arm length | L | 0.225 | m |
| Inertia (X-axis) | Ixx | 4.856 × 10⁻³ | kg·m² |
| Inertia (Y-axis) | Iyy | 4.856 × 10⁻³ | kg·m² |
| Inertia (Z-axis) | Izz | 8.801 × 10⁻³ | kg·m² |
| Rotor inertia | Jr | 3.357 × 10⁻⁵ | kg·m² |
| Gravity | g | 9.81 | m/s² |

---

## Results

> 📸 Simulation result figures and GIFs will be added here after export from Simulink.

Planned result visualizations:
- Step response comparison (roll, pitch, yaw) for all 3 controllers
- Altitude tracking performance
- 3D trajectory tracking (circular path)
- Disturbance rejection comparison (PID+DOB vs others)
- Quadcopter 3D motion animation (GIF)

---

## Requirements

- **MATLAB R2023a or R2023b**
- **Simulink**
- **Control System Toolbox** (for `lqr()`, `place()` functions)

---

## How to Run

**1. Clone the repository:**
```bash
git clone https://github.com/msaitkaradeniz/quadcopter-controller-comparison.git
cd quadcopter-controller-comparison
```

**2. Open MATLAB and set path:**
```matlab
addpath(genpath('.'))
```

**3. Run the gain design scripts first:**
```matlab
% For PID
run('scripts/pid/root_locus_ddsgn.m')

% For LQR
run('scripts/lqr/lqrdesign.m')
run('scripts/lqr/luenbergergain.m')

% For Backstepping
run('scripts/backstepping/parametreler.m')
```

**4. Open and run the Simulink models:**
```matlab
open('models/quadcopter.slx')              % PID + DOB
open('models/lqr_quad.slx')               % LQR + Observer
open('models/backstepping_last_good.slx') % Backstepping
```

---

## References

[1] Saibi, A., Boushaki, R., & Belaidi, H. (2022). *Backstepping Control of Drone*. Engineering Proceedings, 14(4). https://doi.org/10.3390/engproc2022014004

[2] Kourani, A., & Daher, N. (2021). *Leveraging PID Gain Selection Towards Adaptive Backstepping Control for a Class of Second-Order Systems*. arXiv:2107.10697.

[3] Shahid, F., Kadri, B., Jumani, N.A., & Pirwani, Z. (2016). *Dynamical Modeling and Control of Quadrotor*. Transactions on Machine Design, 4, 50–63.

---

## Author

**msaitkaradeniz**
- GitHub: [@msaitkaradeniz](https://github.com/msaitkaradeniz)

---

> ⚠️ **Note:** Result figures and GIF animations will be populated after completing the simulation export workflow. See `results/` folder.
