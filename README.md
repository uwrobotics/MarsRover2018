# MarsRover2018
[![Build Status](https://travis-ci.org/uwrobotics/MarsRover2018.svg?branch=master)](https://travis-ci.org/uwrobotics/MarsRover2018)

All software for the 2018 UWRT Mars Rover

# Simulation
Ensure that all submodules are loaded, and that the husky submodule is on the kinetic-devel branch. 
Comment out the spawn_husky node in "husky/husky_gazebo/launch/spawn_husky.launch" and the GPS and IMU sensors in
"husky/husky_description/urdf/husky.urdf.xacro".

Rename all mesh paths in "simulation/launch/simulation_worlds/simple_world.world" to point to the appropriate directory
in your workspace.