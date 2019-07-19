#ifndef __KINEMATIC_SIMULATION__
#define __KINEMATIC_SIMULATION__

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <urdf/model.h>
#include <mutex>
#include <thread>

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

  /**
  Resets simulation to a pre-defined state.
  **/
  bool reset();

  void update(const std::vector<double> &velocities);

 private:
  ros::NodeHandle nh_;
  ros::Publisher state_pub_, clock_pub_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  double sim_rate_, curr_time_;

  /**
    Loads the URDF model and sets up the list of valid joint names. Loads
  pre-defined joint values.
  **/
  bool init();
};
#endif
