#include <robot_kinematic_simulation/embedded_sim.hpp>

EmbeddedSimulator::EmbeddedSimulator(
    std::shared_ptr<generic_control_toolbox::ControllerBase> &controller)
    : nh_("~"), controller_(controller)
{
  if (!init())
  {
    throw std::logic_error(
        "Failed to initialize the kinematic simulation node");
  }
}

bool EmbeddedSimulator::init()
{
  if (!sim_.reset())
  {
    return false;
  }

  if (!nh_.getParam("compute_rate", compute_rate_))
  {
    ROS_ERROR("Missing compute_rate");
    return false;
  }

  if (!nh_.getParam("sim_rate", sim_rate_))
  {
    ROS_ERROR("Missing sim_rate");
    return false;
  }

  reset_server_ =
      nh_.advertiseService("/state_reset", &EmbeddedSimulator::resetSrv, this);
  return true;
}

bool EmbeddedSimulator::resetSrv(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
{
  bool ret = sim_.reset();
  sim_.update(std::vector<double>(sim_.getJointNames().size(), 0.0));
  ros::spinOnce();
  ROS_WARN("Resetting kinematic simulation");
  return ret;
}

void EmbeddedSimulator::run()
{
  ros::Time prev_time;
  sensor_msgs::JointState command;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  bool ran = false;

  while (ros::ok())
  {
    if (controller_->isActive())
    {
      ran = true;
      command = controller_->updateControl(sim_.getState(),
                                           ros::Duration(1 / sim_rate_));
      std::vector<double> joint_velocities(command.name.size(), 0.0);
      for (unsigned int i = 0; i < command.name.size(); i++)
      {
        joint_velocities[i] = command.velocity[i];
      }

      sim_.update(joint_velocities);
    }
    else if (ran)  // update time
    {
      for (unsigned int i = 0; i < 100; i++)
      {
        sim_.update(std::vector<double>(sim_.getJointNames().size(), 0.0));
      }
    }

    ros::WallDuration(1 / compute_rate_).sleep();
  }
}
