#ifndef __KINEMATIC_SIMULATION__
#define __KINEMATIC_SIMULATION__

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

ptrdiff_t findInVector(const std::vector<std::string> &v, const std::string &x)
{
  if (std::find(v.begin(), v.end(), x) != v.end())
  {
    return distance(v.begin(), find(v.begin(), v.end(), x));
  }

  return -1;
}

/**
  Implement a robot kinematic simulation
**/
class KinematicSimulation
{
public:
  KinematicSimulation();

  /**
    Run the simulation at the pre-defined fixed rate.
  **/
  void run();

private:
  ros::NodeHandle nh_;
  ros::Subscriber command_sub_;
  ros::Publisher state_pub_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  double rate_;

  /**
    Loads the URDF model and sets up the list of valid joint names. Loads pre-defined joint values.
  **/
  bool init();

  /**
    Resets simulation to a pre-defined state.
  **/
  bool reset();

  /**
    Acquire most recent joint velocity commands.
  **/
  void jointCommandCb(const sensor_msgs::JointStateConstPtr &msg);
};
#endif
