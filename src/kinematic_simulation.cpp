#include <robot_kinematic_simulation/kinematic_simulation.hpp>

KinematicSimulation::KinematicSimulation() : nh_("~"), curr_time_(0.0)
{
  if (!init())
  {
    throw std::logic_error("Failed to initialize the kinematic simulation");
  }
}

bool KinematicSimulation::init()
{
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to get robot description");
    return false;
  }

  // initialize joints
  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);

  for (unsigned int i = 0; i < links.size(); i++)
  {
    ROS_DEBUG_STREAM("Parsing link " << links[i]->name);
    if (links[i]->parent_joint &&
        links[i]->parent_joint->type != urdf::Joint::FIXED)
    {
      ROS_INFO_STREAM("Adding joint: " << links[i]->parent_joint->name);
      joint_names_.push_back(links[i]->parent_joint->name);
      joint_positions_.push_back(0.0);
    }
  }

  state_pub_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_states", 1);
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

  curr_time_ = 0.0;
  rosgraph_msgs::Clock clock_msg;
  clock_msg.clock = ros::Time(curr_time_);
  clock_pub_.publish(clock_msg);

  if (!reset())
  {
    return false;
  }

  ROS_INFO("Kinematic simulation initialized successfully");
  return true;
}

void KinematicSimulation::update(const std::vector<double> &velocities)
{
  if (velocities.size() != joint_names_.size())
  {
    ROS_ERROR(
        "Kinematic simulation got velocity command of incorrect size (%d vs "
        "%d)",
        velocities.size(), joint_names_.size());
  }
  rosgraph_msgs::Clock clock_msg;
  state_ = sensor_msgs::JointState();

  for (unsigned int i = 0; i < joint_names_.size(); i++)
  {
    joint_positions_[i] += velocities[i] / sim_rate_;
    state_.name.push_back(joint_names_[i]);
    state_.position.push_back(joint_positions_[i]);
    state_.velocity.push_back(velocities[i]);
    state_.effort.push_back(0.0);
  }

  curr_time_ += 1.0 / sim_rate_;
  clock_msg.clock = ros::Time(curr_time_);
  state_.header.stamp = ros::Time(curr_time_);
  state_pub_.publish(state_);
  clock_pub_.publish(clock_msg);
}

bool KinematicSimulation::reset()
{
  std::vector<std::string> names;
  std::vector<double> positions;

  if (!nh_.getParam("init/joint_names", names))
  {
    ROS_ERROR("Missing init/joint_names parameter");
    return false;
  }

  if (!nh_.getParam("init/joint_positions", positions))
  {
    ROS_ERROR("Missing init/joint_positions");
    return false;
  }

  if (names.size() != positions.size())
  {
    ROS_ERROR("Incoherent joint names and joint positions sizes");
    return false;
  }

  if (!nh_.getParam("sim_rate", sim_rate_))
  {
    ROS_ERROR("Missing sim_rate parameter");
    return false;
  }

  for (unsigned int i = 0; i < names.size(); i++)
  {
    ptrdiff_t idx = findInVector(joint_names_, names[i]);
    if (idx != -1)
    {
      joint_positions_[idx] = positions[i];
    }
  }

  state_ = sensor_msgs::JointState();
  for (unsigned int i = 0; i < joint_names_.size(); i++)
  {
    state_.name.push_back(joint_names_[i]);
    state_.position.push_back(joint_positions_[i]);
    state_.velocity.push_back(0.0);
    state_.effort.push_back(0.0);
  }

  return true;
}
