#include <moveit/benchmark_suite/robot.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace moveit::benchmark_suite;

Robot::Robot(const std::string& name, const std::string& robot_description)
  : name_(name), robot_description_(robot_description)
{
}

bool Robot::initialize()
{
  robot_model_loader::RobotModelLoader loader(robot_description_);
  model_ = loader.getModel();

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
