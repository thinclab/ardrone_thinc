#!/usr/bin/env python
import roslib; roslib.load_manifest('ardrone_thinc')
import rospy, os

urdf = file('urdf/grid.urdf.xacro', 'w')

urdf.write('<?xml version="1.0"?>\n')
urdf.write('<robot name="grid">\n')
urdf.write('  <link name="base_link">\n')
for i in range(5):
    for j in range(5):
        urdf.write('    <visual>\n')
        urdf.write('      <geometry>\n')
        urdf.write('        <box size="1 1 0.01">\n')
        urdf.write('      </geometry>\n')
        urdf.write('      <origin rpy="0 0 0" xyz="%d %d 0">\n' % (i, j))
        urdf.write('    </visual>\n')
urdf.write('  </link>\n')
urdf.write('</robot>\n')
