# novel-DMP-constraints : Experiments

The experiments were carried out on a desktop PC with an Intel® Core™ i7-9700 processor.
The code was written in c++ and organized in ros packages.
Since there are a lot 3rd party software package dependencies, and inter-dependencies between our own custom packages, we provide only the source code for the experiments and the DMP library.

Our DMP library implementation, along with the proposed off-line and on-line optimization is organized as a ros package in `ros_packages/gmp_lib` and depends on the (customized) OSQP library `ros_packages/osqp_lib`.
The `gmp_lib` is documented to some extent and further documentation will be added in the near future.

---
## Handover
The function that executes the handover is `void HandoverExperiment::execute()` in the file `handover/handover_experiment.cpp`. 
The  parameters for the proposed on-line optimization are defined in `handover/handover_params.yaml`
We use the pretrained DMP model `handover/handover_gmp_model.bin`


---

## Placing of cube inside bin
The function that placing is `void ViapointsExperiment::execute()` in the file `place_cube_in_bin/viapoints_experiment.cpp`. 
The  parameters for the proposed on-line optimization are defined in `place_cube_in_bin/viapoints_params.yaml`
We use the pretrained DMP model `place_cube_in_bin/viapoints_gmp_model.bin`