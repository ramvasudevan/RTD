# RTD
Reachability-based Trajectory Design (RTD)

Note: this repository is under construction!
This repository presents the tools required to implement RTD, which is a method of trajectory planning for autonomous mobile robots thaat can guarantee safety (in static environments) and not-at-fault behavior (in dynamic environments).

## Requirements
To run the files in this repository, you will need the following:
- MATLAB R2018a or newer (versions as old as R2017a will work for most functionality).
- Spotless: https://github.com/spot-toolbox/spotless
- MOSEK: https://www.mosek.com/
- simulator: https://github.com/skousik/simulator (note, this repo is still under construction)

## Usage
Examples are coming soon! For now, check out src/offline_FRS_computation/compute_FRS.m.

## References
This repository implements the methods presented in the following papers:

[1] Shreyas Kousik, Sean Vaskov, Matthew Johnson-Roberson, Ramanarayan Vasudevan, "Safe Trajectory Synthesis for Autonomous Driving in Unforeseen Environments," available at https://arxiv.org/abs/1705.00091

[2] Shreyas Kousik, Sean Vaskov, Fan Bu, Matthew Johnson-Roberson, Ram Vasudevan, "Bridging the Gap Between Safety and Real-Time Performance in Receding-Horizon Trajectory Design for Mobile Robots," available at https://arxiv.org/abs/1809.06746

[3] Sean Vaskov, Utkarsh Sharma, Shreyas Kousik, Matthew Johnson-Roberson, Ramanarayan Vasudevan, "Guaranteed Safe Reachability-based Trajectory Design for a High-Fidelity Model of an Autonomous Passenger Vehicle," available at https://arxiv.org/abs/1902.01786
