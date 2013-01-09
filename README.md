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

tum\_simulator (for simulation):

```bash
$ cd ~/ros/stacks   # or equivalent
$ git clone https://github.com/tum-vision/tum_simulator.git
$ rospack profile && rosstack profile
$ rosmake cvg_sim_gazebo_plugins
$ rosmake message_to_tf
```

Notes
-----

* Thresholded images are published to `thinc/thresh`.
* To run simulator, `roslaunch cvg_sim_test 3boxes_room.launch`, replacing
  3boxes\_room.launch with launchfile of your choice.

TODO
----

1. Test drone outside
2. Test THINC programs in simulator
3. Learn gazebo

