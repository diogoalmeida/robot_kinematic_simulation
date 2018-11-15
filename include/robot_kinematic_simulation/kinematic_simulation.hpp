#ifndef __KINEMATIC_SIMULATION__
#define __KINEMATIC_SIMULATION__

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

/**
  Implement a robot kinematic simulation
**/
class KinematicSimulation
{
public:
  KinematicSimulation();

private:
  ros::NodeHandle nh_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;

  /**
    Loads the URDF model and sets up the list of valid joint names. Loads pre-defined joint values.
  **/
  bool init();
};
#endif
