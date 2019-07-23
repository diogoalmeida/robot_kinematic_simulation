#include <robot_kinematic_simulation/node_sim.hpp>

NodeSimulation::NodeSimulation() : nh_("~")
{
  if (!init())
  {
    throw std::logic_error(
        "Failed to initialize the kinematic simulation node");
  }
}

bool NodeSimulation::init()
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

  joint_velocities_.resize(sim_.getJointNames().size());

  reset_server_ =
      nh_.advertiseService("/state_reset", &NodeSimulation::resetSrv, this);
  command_sub_ =
      nh_.subscribe("/joint_command", 1, &NodeSimulation::jointCommandCb, this);
  return true;
}

bool NodeSimulation::resetSrv(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res)
{
  bool ret = sim_.reset();
  ROS_WARN("Resetting kinematic simulation");
  ros::Duration(0.2).sleep();  // wait for TF to catch up to simulation...
  return ret;
}

void NodeSimulation::jointCommandCb(const sensor_msgs::JointStateConstPtr &msg)
{
  for (unsigned int i = 0; i < msg->name.size(); i++)
  {
    ptrdiff_t idx = findInVector(sim_.getJointNames(), msg->name[i]);
    if (idx != -1)
    {
      joint_velocities_[idx] = msg->velocity[i];
    }
  }
}

void NodeSimulation::exec()
{
  ros::WallRate r(compute_rate_);
  while (ros::ok())
  {
    sim_.update(joint_velocities_);
    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematic_simulation_node");
  NodeSimulation sim;
  sim.exec();
  return 0;
}
