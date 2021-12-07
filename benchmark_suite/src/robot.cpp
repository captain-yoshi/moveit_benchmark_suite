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

  state_ = std::make_shared<robot_state::RobotState>(model_);
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

std::vector<std::string> Robot::getJointNames() const
{
  return state_->getVariableNames();
}

bool Robot::hasJoint(const std::string& joint) const
{
  const auto& joint_names = getJointNames();
  return (std::find(joint_names.begin(), joint_names.end(), joint) != joint_names.end());
}

const robot_model::RobotStatePtr& Robot::getStateConst() const
{
  return state_;
}

robot_model::RobotStatePtr& Robot::getState()
{
  return state_;
}

robot_model::RobotStatePtr Robot::cloneState() const
{
  auto state = allocState();
  *state = *state_;

  return state;
}

robot_model::RobotStatePtr Robot::allocState() const
{
  robot_state::RobotStatePtr state;
  state = std::make_shared<robot_state::RobotState>(getModelConst());
  state->setToDefaultValues();

  return state;
}
const Eigen::Isometry3d& Robot::getLinkTF(const std::string& name) const
{
  return state_->getGlobalLinkTransform(name);
}
