ode_tasks
==================
Robotic tasks simulated with Open Dynamics Engine (ODE, OpenDE) over ROS architecture.

This package is provided for lectures.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Requirements
==================
- ROS core system, rospy, roscpp, std_msgs, std_srvs, geometry_msgs, tf
- ODE: we assume that the ODE is built in $HOME/prg/libode/ode-latest/ from source.
-- Use "./configure --enable-double-precision --disable-asserts" to setup.
-- Build drawstuff as well.
- Python: core, numpy


Build
==================
The repository directory should be in ROS workspace (e.g. ~/ros/).

  $ cd ode1/
  $ rosmake


Launch
==================
The main server:

  $ roslaunch ode1 arm7_door_push.launch

Press space to stop/resume the simulation.
You can test the robot with pressing: z, x, a, s, [, ], etc.  Read the source code for the detail.

Control from Python scripts:

  $ cd ode1/scripts/

Basic demo:

  $ arm7dp_core.py

Forward and inverse kinematics sample:

  $ arm7dp_kin.py

Control end effector with keys using IK:

  $ arm7dp_key.py


Troubles
==================
Send e-mails to the author.  I accept only from my students.

