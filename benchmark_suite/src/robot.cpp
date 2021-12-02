#include <moveit_benchmark_suite/robot.h>

using namespace moveit_benchmark_suite;

Robot::Robot(const std::string& name, const std::string& robot_description)
  : name_(name), robot_description_(robot_description)
{
}

bool Robot::initialize()
{
  loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  model_ = loader_->getModel();

  if (!model_)
  {
    ROS_WARN_STREAM("No robot model for robot description: `" + robot_description_ + "`");
    return false;
  }
  return true;
}

const std::string& Robot::getModelName() const
{
  return model_->getName();
}

const std::string& Robot::getName() const
{
  return name_;
}

const robot_model::RobotModelPtr& Robot::getModelConst() const
{
  return model_;
}

robot_model::RobotModelPtr& Robot::getModel()
{
  return model_;
}
