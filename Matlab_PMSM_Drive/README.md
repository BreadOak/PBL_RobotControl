# 0.Purpose of Development
- MATLAB-based PMSM Driver Design for Neuromeka-CORE simulation.
- Connect Matlab to ROS2.
- GUI production using ROS2.

# 1.Development Environment
- Window11 wls2.
- Ubuntu 18.04, ROS2 dashing.
- MATLAB R2021b.
![image](https://user-images.githubusercontent.com/75024315/146763805-3f61129e-a4ed-4c28-b61e-69155064fbb0.png)

# 2.Prerequisite
- install add-on (ROS Toolbox v1.2).
- install add-on (Robust Control Toolbox).
- install add-on (Control System Toolbox).
- install add-on (Signal Processing Toolbox).
- install Multiparametric toolbox (MPT), using (install_mpt3.m) file.

# 3.MATLAB-based PMSM Driver Version list
- Version_0 : Current & Velocity Control.
- Version_1 : Select [Current / Velosity / Position] Control Mode. (Same control frequency)
- Version_2 : Select [Current / Velosity / Position] Control Mode. (Different control frequency)
- Version_3 : Model Predictive Control(MPC).
- Version_4 : Multi-Parametric Toolbox(MPT) based MPC.
- Version_5 : Disturbance Observer(DOB).
- Version_6 : H-infinity Control. 
- Version_alpha : Integrated controller.
- Version_beta : Revision.

# 4.Overall Structure
[block diagram]
![image](https://user-images.githubusercontent.com/75024315/146320252-668cc404-c65a-4717-9e4b-b3a94f3cee56.png)
- You can use a Combination of Control Modes.
  0: Not use      1: Current(PI)   2: Current(MPC)      
  0: Not use      1: Velocity(PI)  2: Velocity(MPC) 3: Velocity(H-inf)    %
  0: Not use      1: Position(PI)      
  0: Not use      1: Current(PI+DOB)

# 5.Future Work
- 
