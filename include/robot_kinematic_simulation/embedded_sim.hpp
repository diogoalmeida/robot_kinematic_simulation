#ifndef __EMBEDDED_SIMULATION__
#define __EMBEDDED_SIMULATION__

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <chrono>
#include <generic_control_toolbox/controller_template.hpp>
#include <robot_kinematic_simulation/kinematic_simulation.hpp>
#include <thread>

/**
  Embedds a generic controller in a kinematic simulation.
**/
class EmbeddedSimulator
{
 public:
  EmbeddedSimulator(
      std::shared_ptr<generic_control_toolbox::ControllerBase> &controller);

  void run();

 private:
  std::shared_ptr<generic_control_toolbox::ControllerBase> controller_;
  ros::ServiceServer reset_server_;
  ros::NodeHandle nh_;
  double compute_rate_;
  std::vector<std::string> joint_names_;
  KinematicSimulation sim_;

  bool init();

  /**
    Implements a service to reset the simulation status.
  **/
  bool resetSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};
#endif
