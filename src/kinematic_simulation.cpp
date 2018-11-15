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
    if (links[i]->parent_joint->type == urdf::Joint::REVOLUTE)
    {
      ROS_INFO_STREAM("Adding joint: " << links[i]->parent_joint->name);
      joint_names_.push_back(links[i]->parent_joint->name);
      joint_positions_.push_back(0.0);
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematic_simulation");
  KinematicSimulation sim;
  return 0;
}
