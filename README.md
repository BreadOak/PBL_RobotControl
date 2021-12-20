# 2021 PBL Robot Control

## 0. Purpose of Development

[Outline]

![image](https://user-images.githubusercontent.com/75024315/146764699-65282edc-9c5c-496c-8166-ad33a24b3e72.png)

  1. MATLAB-based PMSM Driver Design for Neuromeka-CORE simulation.
  2. MATLAB to ROS2.
  3. GUI production using ROS2.

## 1. MATLAB-based PMSM Driver Design for Neuromeka-CORE simulation.
### 1.1 Development Environment
  - Window11 wls2.
  - Ubuntu 18.04, ROS2 dashing.
  - MATLAB R2021b.
  
### 1.2 Prerequisite
  - install add-on (ROS Toolbox v1.2).
  - install add-on (Robust Control Toolbox).
  - install add-on (Control System Toolbox).
  - install add-on (Signal Processing Toolbox).
  - install Multiparametric toolbox (MPT), using (install_mpt3.m) file.

### 1.3 MATLAB-based PMSM Driver Version list
  - Version_0 : Current & Velocity Control.
  - Version_1 : Select [Current / Velosity / Position] Control Mode. (Same control frequency)
  - Version_2 : Select [Current / Velosity / Position] Control Mode. (Different control frequency)
  - Version_3 : Model Predictive Control(MPC).
  - Version_4 : Multi-Parametric Toolbox(MPT) based MPC.
  - Version_5 : Disturbance Observer(DOB).
  - Version_6 : H-infinity Control. 
  - Version_alpha : Integrated Controller.
  - Version_beta : The latest version.

### 1.4 Overall Structure

[Block Diagram]
![image](https://user-images.githubusercontent.com/75024315/146320252-668cc404-c65a-4717-9e4b-b3a94f3cee56.png)

- You can use a Combination of Control Modes.
  - 0: Not use      1: Current(PI)   2: Current(MPC)      
  - 0: Not use      1: Velocity(PI)  2: Velocity(MPC) 3: Velocity(H-inf)    
  - 0: Not use      1: Position(PI)      
  - 0: Not use      1: Current(PI+DOB)

### 1.5 Future Work
  - Controller Maintenance.
  - Add More Control Modes.
  - ...
