# RTD (Reachability-based Trajectory Design)
This repository presents the tools required to implement RTD, which is a method of trajectory planning for autonomous mobile robots that can guarantee safety (in static environments) and not-at-fault behavior (in dynamic environments).

## Requirements
To run the files in this repository, you will need the following:
- MATLAB R2018a or newer (versions as old as R2017a will work for most functionality).
- spotless: https://github.com/spot-toolbox/spotless
- MOSEK: https://www.mosek.com/

## Usage
All examples currently are for the FRS computation. We'll be adding examples of tracking error and online planning soon.

To run the examples, make sure that this repository is on your MATLAB path, and that spotless and MOSEK are installed correctly and on your MATLAB path. Then, in the MATLAB command window, run
> FRS_computation_example_1()

You should see MOSEK get called in the command window, and then a plot of an FRS in the state and parameter space will pop up. If this works, then you can also try running examples 2 -- 4 (see RTD/examples/offline_FRS_computation/).

## RTD Overview
RTD is a way of controlling a robot, described by a "high-fidelity" dynamic model, by generating "desired trajectories" with a lower-dimensional "trajectory-producing" model. The robot uses a tracking controller (e.g., PID or MPC) to track the desired trajectories. We estimate a worst-case bound on the tracking error dynamics, then use it with the trajectory-producing model to compute a Forward Reachable Set (FRS) offline. The FRS then contains all points in the robot's state space that are reachable while tracking the desired trajectories.

We use the FRS for online planning by first intersecting it with obstacles that we want the robot to avoid. Since the FRS contains all possible trajectories of the robot (when tracking the parameterized desired trajectories), this lets us identify all of the trajectories that could cause a crash. Then, we optimize over the remaining safe trajectories.

The online planning algorithm runs in a "receding horizon" fashion, where a desired trajectory is executed while a new one is computed. This is because robots have limited sensor information at any time, so they have to move through the world to gain more information (about the things they aren't supposed to hit). In RTD, we enforce a planning time constraint, so each planning iteration only has some limited amount of time to try and find a new trajectory. If a new trajectory can't be found, then the robot has to execute a "fail-safe" maneuver; in our prior work (see the references below), we've used coming to a stop as a fail safe since it is widely applicable. Notice that this means _every_ desired trajectory must include a fail-safe maneuver! We ensure this is possible by making the duration of the trajectories large enough for the robot to stop.

This repo demonstrates simple examples of RTD to illustrate how it works both offline and online. We'll be updating it with more examples as we make 'em!

## References
This repository implements the methods presented in the following papers:

[1] Shreyas Kousik, Sean Vaskov, Matthew Johnson-Roberson, Ramanarayan Vasudevan, "Safe Trajectory Synthesis for Autonomous Driving in Unforeseen Environments," available at https://arxiv.org/abs/1705.00091

[2] Shreyas Kousik, Sean Vaskov, Fan Bu, Matthew Johnson-Roberson, Ram Vasudevan, "Bridging the Gap Between Safety and Real-Time Performance in Receding-Horizon Trajectory Design for Mobile Robots," available at https://arxiv.org/abs/1809.06746

[3] Sean Vaskov, Utkarsh Sharma, Shreyas Kousik, Matthew Johnson-Roberson, Ramanarayan Vasudevan, "Guaranteed Safe Reachability-based Trajectory Design for a High-Fidelity Model of an Autonomous Passenger Vehicle," available at https://arxiv.org/abs/1902.01786

## Authors:
Shreyas Kousik
Sean Vaskov
Ram Vasudevan
http://roahmlab.com
