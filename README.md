ardrone\_thinc
=============

Installation
------------


Notes
-----

* This requires https://github.com/AutonomyLab/ardrone_autonomy
* For simulation, https://github.com/thinclab/thinc_simulator
* If you want to write a program to control drones over UDP sockets, visit 
  https://github.com/thinclab/gatacdronecontrol.

Usage
-----

This package is designed to provide an endpoint for the Georgia Testbed for
Autonomous Control (GaTAC), and allows grid based motion of ARDrone Parrot
quadrotors over a UDP socket (or ROS service).  A simulator is available at
http://lhotse.cs.uga.edu/gitlab/multi-robotics/thinc_simulator.

The main ROS node is called `thinc_smart`.  SMART stands for Simulator for the
Motion of Autonomous Robotic Teams.

The package is designed to be specifically multi-agent.

For each drone that you wish to control, run each of these commands in a
separate `sh` instance (for example, run each in a separate terminal emulator)

1. Pick an id (hereafter \<id\>) for the drone (with start position
\<drone-col\>, \<drone-row\>)
2. `$ roscore` (if it is not already running)
3. `$ ROS_NAMESPACE=drone<id> rosrun ardrone_thinc thinc_smart <cols> <rows> <drone-col> <drone-row> <size of col in meters> <size of row in meters> <elevation in meters> <if flying simulated drones, last argument is 's', else last argument is 'r'>`
4. `$ rosrun ardrone_thinc thinc\sock <GaTAC-ip> <GaTAC-port> <id>`

`thinc_smart` doesn't care about whether it's running in the real world or in
a simulator.  However, for real world ARParrot Drones, each drone requires a
separate `ardrone_autonomy ardrone_driver` instance and must bind to a
seperate wireless interface.  This is most easily accomplished by using one
computer per drone, and having GaTAC connect to each computer individually
(which is no different than the simulator, except the ip addresses GaTAC needs are now no longer for a single computer).

Alternatively, it is possible to control multiple drones from a single computer by connecting them all to a single wireless access point. This requires some configuration of both your drones, and the wireless access point itself.  Follow the steps on https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones to help you get started.
