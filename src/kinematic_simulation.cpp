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
      joint_velocities_.push_back(0.0);
    }
  }

  if (!reset())
  {
    return false;
  }

  reset_server_ = nh_.advertiseService("/state_reset",
                                       &KinematicSimulation::resetSrv, this);
  command_sub_ = nh_.subscribe("/joint_command", 1,
                               &KinematicSimulation::jointCommandCb, this);
  state_pub_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_states", 1);
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ROS_INFO("Kinematic simulation initialized successfully");
  return true;
}

bool KinematicSimulation::resetSrv(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
  bool ret = reset();
  ROS_WARN("Resetting kinematic simulation");
  ros::Duration(0.2).sleep();  // wait for TF to catch up to simulation...
  return ret;
}

void KinematicSimulation::jointCommandCb(
    const sensor_msgs::JointStateConstPtr &msg)
{
  std::lock_guard<std::mutex> guard(mtx_);
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
  run_thread_ = std::unique_ptr<std::thread>(
      new std::thread(&KinematicSimulation::exec, this));
  ros::spin();
}

void KinematicSimulation::exec()
{
  ros::WallRate r(sim_scale_ * rate_);
  rosgraph_msgs::Clock clock_msg;

  while (ros::ok())
  {
    {
      std::lock_guard<std::mutex> guard(mtx_);
      sensor_msgs::JointState cmd;
      for (unsigned int i = 0; i < joint_names_.size(); i++)
      {
        joint_positions_[i] += joint_velocities_[i] * 1.0 / rate_;
        cmd.name.push_back(joint_names_[i]);
        cmd.position.push_back(joint_positions_[i]);
        cmd.velocity.push_back(joint_velocities_[i]);
        cmd.effort.push_back(0.0);
      }

      curr_time_ += 1.0 / rate_;
      clock_msg.clock = ros::Time(curr_time_);
      cmd.header.stamp = ros::Time(curr_time_);
      state_pub_.publish(cmd);
      clock_pub_.publish(clock_msg);
    }

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

  if (!nh_.getParam("sim_rate", rate_))
  {
    ROS_ERROR("Missing sim_rate parameter");
    return false;
  }

  double rt_rate;
  if (!nh_.getParam("realtime_rate", rt_rate))
  {
    ROS_ERROR("Missing sim_rate parameter");
    return false;
  }

  sim_scale_ = rt_rate / rate_;

  mtx_.lock();
  for (unsigned int i = 0; i < names.size(); i++)
  {
    ptrdiff_t idx = findInVector(joint_names_, names[i]);
    if (idx != -1)
    {
      joint_positions_[idx] = positions[i];
      joint_velocities_[idx] = 0.0;
    }
  }
  mtx_.unlock();

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematic_simulation");
  KinematicSimulation sim;
  sim.run();
  return 0;
}
