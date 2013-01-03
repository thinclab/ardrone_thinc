ardrone\_thinc
=============

Installation
------------

```bash
$ cd ~/ros/stacks   # or equivalent
$ git clone https://github.com/dmillard/ardrone\_thinc.git
$ rospack profile && rosstack profile
$ roscd ardrone_thinc
$ make
```

Notes
-----

* Thresholded images are published to `thinc/thresh`.

TODO
----

1. Stop killer attack behaviour when trying to stabilize hover
