Robot Kinematic simulation
===

This package implements a simple robot manipulator simulation.
Only kinematics (joint velocities + positions) are simulated over time, i.e., no
dynamics.
This makes for a less resource intensive simulation for users that only care about
robot kinematics.

The simulation accepts a `sensor_msgs/JointState` message as a command.
Joint velocities from the command message are taken as input to the simulation.

The simulated robot will use the kinematics set in the ROS parameter server
`\robot_description` parameter.

Simulation parameters
---

* ``sim_rate``: The update rate of the simulated robot. The higher this value is
set, the more fine-grained the simulation will be.
* ``realtime_rate``: The update rate of the simulation. If set to the same value as
``sim_rate``, the simulation will run at real-time. You can set this value to be higher
than ``sim_rate`` to get a faster simulation. If your computer struggles to run the simulation, try lowering this value.
* ``init``
  * ``\joint_names``: an array of joint names to be initialized with the simulation. These should match existing joint names in the robot URDF description.
  * ``\joint_positions``: the initial joint positions of the simulated robot.

  An example configuration file is provided in the ``config`` directory, which is
  loaded in the ``sim.launch`` launch file. These make use of the Rething Robotics' 
  Baxter description to run, which you can get from [here](https://github.com/RethinkRobotics/baxter_common).
