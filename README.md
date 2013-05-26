ardrone\_thinc
=============

Installation
------------

ardrone\_thinc:

```bash
$ cd ~/ros/stacks   # or equivalent
$ git clone https://github.com/dmillard/ardrone_thinc.git
$ rospack profile && rosstack profile
$ roscd ardrone_thinc
$ make
```

Notes
-----

* This requires https://github.com/AutonomyLab/ardrone_autonomy
* For simulation, use https://github.com/dmillard/thinc_simulator

Usage
-----

This package is designed to provide an endpoint for the Georgia Testbed for
Autonomous Control (GaTAC), and allows grid based motion of ARDrone Parrot
quadrotors over a UDP socket (or ROS service).  A simulator is available at
https://github.com/dmillard/thinc_simulator.

The main ROS node is called `thinc_smart`.  SMART stands for Simulator for the
Motion of Autonomous Robotic Teams.

The package is designed to be specifically multi-agent.

For each drone that you wish to control, run each of these commands in a
separate `sh` instance (for example, run each in a separate terminal emulator)

1. Pick an id (hereafter \<id\>) for the drone (with start position
\<drone-col\>, \<drone-row\>)
2. `$ roscore` (if it is not already running)
3. `$ rosrun ardrone_thinc thinc_smart <num-cols> <num-rows> <id> <drone-col>
<drone-row>`
4. `$ rosrun ardrone_thinc thinc\sock <GaTAC-ip> <GaTAC-port> <id>`

`thinc_smart` doesn't care about whether it's running in the real world or in
a simulator.  However, for real world ARParrot Drones, each drone requires a
separate `ardrone_autonomy ardrone_driver` instance and must bind to a
seperate wireless interface.  This is most easily accomplished by using one
computer per drone, and having GaTAC connect to each computer individually
(which is no different than the simulator, except the ip addresses GaTAC needs
are now no longer for a single computer).
