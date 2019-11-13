# Packages

The Catkin workspace consists of two packages. The simulation environment 'Faltland' and the package for the course 'racing'.

## Flatland 

Flatland is a performance centric 2D robot simulator.
It is intended for use as a light weight alternative to Gazebo Simulator for ground robots on a flat surface.
Flatland uses Box2D for physics simulation and it is built to integrate directly with ROS.
Flatland loads its simulation environment from YAML files and provide a plugin system for extending its functionalities.

## Racing

This package includes all contents of the course. In addition to a vehicle model based on the single-track drive.
Here you can also find a vehicle controller and the simulation configuration. 