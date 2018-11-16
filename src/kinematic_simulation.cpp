#include <robot_kinematic_simulation/kinematic_simulation.hpp>

KinematicSimulation::KinematicSimulation() : nh_("~")
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
    if (links[i]->parent_joint && links[i]->parent_joint->type != urdf::Joint::FIXED)
    {
      ROS_INFO_STREAM("Adding joint: " << links[i]->parent_joint->name);
      joint_names_.push_back(links[i]->parent_joint->name);
      joint_positions_.push_back(0.0);
      joint_velocities_.push_back(0.0);
    }
  }

  if (!nh_.getParam("sim_rate", rate_))
  {
    ROS_ERROR("Missing sim_rate parameter");
    return false;
  }

  if (!reset())
  {
    return false;
  }

  command_sub_ = nh_.subscribe("/joint_command", 1, &KinematicSimulation::jointCommandCb, this);
  state_pub_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_states", 1);

  return true;
}

void KinematicSimulation::jointCommandCb(const sensor_msgs::JointStateConstPtr &msg)
{
  for (unsigned int i = 0; i < msg->name.size(); i++)
  {
    ptrdiff_t idx = findInVector(joint_names_, msg->name[i]);
    if (idx != -1)
    {
      joint_velocities_[idx] = msg->velocity[i];
    }
  }
}

void KinematicSimulation::run()
{
  ros::Rate r(rate_);
  ros::Time prev_time = ros::Time::now();
  ros::Duration dt;

  while (ros::ok())
  {
    sensor_msgs::JointState cmd;
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
      dt = (ros::Time::now() - prev_time);
      joint_positions_[i] += joint_velocities_[i]*dt.toSec();
      cmd.name.push_back(joint_names_[i]);
      cmd.position.push_back(joint_positions_[i]);
      cmd.velocity.push_back(joint_velocities_[i]);
      cmd.effort.push_back(0.0);
    }

    cmd.header.stamp = ros::Time::now();
    state_pub_.publish(cmd);
    ros::spinOnce();
    prev_time = ros::Time::now();
    r.sleep();
  }
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

  for (unsigned int i = 0; i < names.size(); i++)
  {
    ptrdiff_t idx = findInVector(joint_names_, names[i]);
    if (idx != -1)
    {
      joint_positions_[idx] = positions[i];
      joint_velocities_[idx] = 0.0;
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematic_simulation");
  KinematicSimulation sim;
  sim.run();
  return 0;
}
