# SNCS22_RoutingArchitecture
This repository contains the algorithm and data files presented in the SNCS paper "MPC-based Routing and Tracking Architecture for Safe Autonomous Driving in Urban Traffic", doi:[10.1007/s42979-024-02732-5](https://doi.org/10.1007/s42979-024-02732-5).
The sequence of operations is:

1. __Path Plannig__ _(offline)_: Generate the reference travel path with maximum speed information using the GraphHopper-based routing algorithm.
2. __Velocity Plannig__ _(offline)_: Calculate the optimal velocity trajectory using the nonlinear MPC for the desired vehicle model (car/truck).
3. __Trajectory Tracking__ _(online)_: Perform real-time path and velocity tracking using the FSM approach and the linearized MPC for the desired vehicle model (car/truck).


# GraphHopper User Interface

- To use the API [Routing-API](https://docs.graphhopper.com/#tag/Routing-API) and other advanced options, you need to sign-up and create a personal API key. An API key is required to operate both the GUI and the developed routing algorithm in this repository.
- The developed files are available under [./GraphHopperUI](./GraphHopperUI) and consist of:
    - _GH_GraphHopperGUI.mlapp_: GUI to facilitate calling the GraphHopper API. The functionalities of the GUI are saved in external _*.m_ files for better code management.
    - _ComputeReferencePath.m_: Implementation of the routing algorithm described in the paper.
- For trying the different capabilities of GraphHopper, the web interface [https://graphhopper.com/maps](https://graphhopper.com/maps) can be used free of charge.


# FORTRAN Code

- To use the developed FORTRAN code, you must download OCPID-DAE1 and sqpfiltertoolbox from [https://www.unibw.de/ingmathe/research/software](https://www.unibw.de/ingmathe/research/software). Note that these toolboxes are developed and mainainted by [Univ.-Prof. Dr. rer. nat. Matthias Gerdts](mailto:matthias.gerdts@unibw.de) and have their own individual licenses.
- The code files for the described test scenario are present in the folder [./FORTRAN](./FORTRAN) and consist of:
    - _optVelCar/Trk.f90_: Solves the optimal velocity planning problem for the car/truck model.
    - _pathTrackCar/Trk.f90_: Solves the real-time path and velocity tracking problem for the car/truck model (includes the FSM approach).


# Data files
All generated data for the test scenario is saved as csv in _*.dat_ files under [./data](./data). The files are categorized as follows:

- __Path Planning__: 
    - _roadMat.dat_: Contains the reference path way-points and the lane width.
    - _velCnstr.dat_: Contains the arclength of the reference path and the corresponding maximum speed.
- __Velocity Planning__:
    - _roadDataCar/Trk.dat_: The travelled path of the solved velocity planning problem for the car/truck model.
    - _velDataCar/Trk.dat_: The optimal velocity trajectory of the solved velocity planning problem for the car/truck model.
- __Trajectory Tracking__:
    - _pathTrackCar/Trk.dat_: The resulting path and controls of the real-time tracking problem for the car/truck model.

Additionally, some python files are present to facilitate extracting the relevant data from the _*.dat_ files and generate appropriate figures.
