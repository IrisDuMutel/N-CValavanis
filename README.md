### Project for the course: Navigation and Control of Unmanned Aerial Vehicles 2022

This project is structured as:
- Graphs
- Models: simulink models:
    - model_PID.slx: quadcopter model integrated with PID control (inner and outer)
    - model_SMC.slx: quadcopter model integrated with PID control (x,y) and SMC control (attitude and altitude)
- Scripts: MATLAB scripts and functions:
    - init.m: initialization of all variables of the quadcopter model
    - linear_traj.m: creation of linear trajectory and references reeded for simulink
    - helix_traj.m:  creation of helix trajectory and references reeded for simulink
    - plotter.m: creates plots out of simulation and reference variables
    - trajplann3.m: function that takes coordinates and timestaps and generates a trajectory
- report.pdf: finall report of the project summarizing the main results

A quadcopter model is simulated in MATLAB&Simulink using two different control approaches: PID and SMC. Two trajectories are proposed: a simple linear trajectory and a helix trajectory.

To simulate the quadcopter, start by running init.m

To simulate the linear trajectory:

```init.m --> linear_traj.m --> model_PID.slx --> plotter.m```

To simulate the helix trajectory:

```init.m --> helix_traj.m --> model_SMC.slx --> plotter.m```