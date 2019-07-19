#ifndef __NODE_SIMULATION__
#define __NODE_SIMULATION__

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <robot_kinematic_simulation/kinematic_simulation.hpp>

/**
  Implements the required methods to run a kinematic simulation node.

  Commands are sent via sensor_msgs/JointState through a rostopic.
**/
class NodeSimulation
{
 public:
  NodeSimulation();

  /**
  Simulation execution.
  **/
  void exec();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber command_sub_;
  ros::ServiceServer reset_server_;
  std::vector<double> joint_velocities_;
  std::vector<std::string> joint_names_;
  double compute_rate_;
  KinematicSimulation sim_;

  /**
      Initialize the simulation variables.
  **/
  bool init();

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
