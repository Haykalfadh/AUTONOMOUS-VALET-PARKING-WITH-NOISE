# Autonomous Valet Parking with DRL under LiDAR Noise Conditions

This repository contains the MATLAB/Simulink implementation of our research on **Autonomous Valet Parking (AVP)** using three state-of-the-art Deep Reinforcement Learning (DRL) algorithms: **Soft Actor-Critic (SAC)**, **Proximal Policy Optimization (PPO)**, and **Twin Delayed Deep Deterministic Policy Gradient (TD3)**.

The study evaluates each algorithm's performance under **four LiDAR noise scenarios**:
1. **Without Noise** (Baseline)
2. **Internal Noise** (Gaussian noise from internal LiDAR disturbances)
3. **External Noise** (Gaussian noise from external light interference)
4. **Combined Noise** (Internal + External)

A total of **12 experimental scenarios** were conducted (3 algorithms × 4 noise conditions).  

---

## 1. Research Overview

- **Objective:**  
  Compare SAC, PPO, and TD3 algorithms for AVP in noisy perception conditions and evaluate robustness, efficiency, and generalization.
  
- **Key Contributions:**
  1. Implementation of three DRL algorithms in MATLAB–Simulink.
  2. Four distinct LiDAR noise injection schemes for realistic perception challenges.
  3. Comprehensive evaluation using average reward, success rate, collision rate, and generalization.

- **Reference Paper:**  
  *Autonomous Valet Parking Based on Soft Actor-Critic Approach* (2025)  
  (https://drive.google.com/file/d/1mVdTLA1qDqTndRphbDPnqWv1KBYgANBk/view?usp=sharing)

- **Documentation:**
  
  ![TrainingProcess](https://github.com/user-attachments/assets/c3e76aaf-b349-4406-b573-849bc6fbe08c)
  ![TrainingResult](https://github.com/user-attachments/assets/17fbd930-93de-4464-bdaf-94c86f055455)


---

## 2. Repository Structure
├── Combined-Noise/ # Simulink environment for combined noise scenario

├── External-Noise/ # Simulink environment for external noise scenario

├── Internal-Noise/ # Simulink environment for internal noise scenario

├── Without-Noise/ # Simulink environment for baseline (no noise)

│

├── Camera.m # Camera sensor model

├── LIDARSensor.m # LiDAR sensor model

├── MainProgram-PPO.mlx # PPO main training script

├── MainProgram-SAC.mlx # SAC main training script

├── MainProgram-TD3.mlx # TD3 main training script

├── ParkingLot.m # Parking lot model

├── ParkingLotSimulator.m # Simulator for parking lot environment

├── autoParkingValetParams.m # AVP parameters

├── autoParkingValetResetFcn.m # Reset function for RL environment

├── createMPCForParking.m # MPC creation for search mode

├── getCarSegmentLengths.m # Utility function for geometry

├── getRefTraj.m # Reference trajectory generator

├── lidarSegmentIntersections.m # LiDAR ray intersection calculations

├── parkingVehicleStateFcnRRT.m # Vehicle state function for RRT

├── parkingVehicleStateJacobianFcnRRT.m # State Jacobian for RRT

├── rect2segs.m # Rectangle to segments utility

├── rlAutoParkingValetAgent.mat # Pretrained RL agent (optional)

├── vehicleStateFcn.m # Vehicle kinematic model

├── vehicleStateJacobianFcnDT.m # State Jacobian function

---

## 3. Prerequisites

- **MATLAB 2023a** or later  
- Required MATLAB Toolboxes:
  - Reinforcement Learning Toolbox
  - Model Predictive Control Toolbox
  - Deep Learning Toolbox
  - Simulink
    
---

## 4. Reproducing the Experiments
Step 1 – Select Noise Scenario

- Noise scenario environments are stored in:

  - Without-Noise/ → Baseline, no noise injected
  
  - Internal-Noise/ → Zero-mean Gaussian internal noise
  
  - External-Noise/ → Zero-mean Gaussian external noise
  
  - Combined-Noise/ → Both internal and external noise
  
- Open the .slx Simulink file inside the chosen scenario folder.

Step 2 – Select Algorithm

- Choose one of the three main training scripts:

  - MainProgram-SAC.mlx → Soft Actor-Critic
  
  - MainProgram-PPO.mlx → Proximal Policy Optimization
  
  - MainProgram-TD3.mlx → Twin Delayed DDPG

Step 3 – Select Mode

- In each main program (MainProgram-SAC.mlx, MainProgram-PPO.mlx, MainProgram-TD3.mlx), you will find the following code snippet:
```matlab
doTraining = true;

if doTraining
trainingResult = train(agent, env, trainOpts);
save("TrainingResult.mat", "agent");
else
load("rlAutoParkingValetAgent.mat", "agent");
end
```

- Set doTraining = true → The script will start training the agent from scratch using the selected algorithm and environment.

- Set doTraining = false → The script will skip training and instead load a pre-trained agent from the file rlAutoParkingValetAgent.mat or your previous Training Result for testing and evaluation.

Step 4 – Monitor Training

- Episode reward, Average reward, Episode Q0 curves will appear during training.

- Models and logs are stored in MATLAB workspace or saved manually.
