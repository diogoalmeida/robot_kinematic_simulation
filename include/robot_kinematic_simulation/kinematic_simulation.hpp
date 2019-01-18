#ifndef __KINEMATIC_SIMULATION__
#define __KINEMATIC_SIMULATION__

#include <ros/ros.h>
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

 private:
  ros::NodeHandle nh_;
  ros::Subscriber command_sub_;
  ros::Publisher state_pub_;
  ros::ServiceServer reset_server_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::unique_ptr<std::thread> run_thread_;
  std::mutex mtx_;
  double rate_;

  /**
    Loads the URDF model and sets up the list of valid joint names. Loads
  pre-defined joint values.
  **/
  bool init();

  /**
    Resets simulation to a pre-defined state.
  **/
  bool reset();

  /**
    Simulation execution.
  **/
  void exec();

  /**
    Implements a service to reset the simulation status.
  **/
  bool resetSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
    Acquire most recent joint velocity commands.
  **/
  void jointCommandCb(const sensor_msgs::JointStateConstPtr &msg);
};
#endif
