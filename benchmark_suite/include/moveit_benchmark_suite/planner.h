#pragma once

#include <moveit_benchmark_suite/robot.h>

namespace moveit_benchmark_suite
{
MOVEIT_CLASS_FORWARD(Planner);

/** Abstract interface to a planning algorithm. */
class Planner
{
public:
  Planner(const RobotPtr& robot, const std::string& name = "") : robot_(robot), name_(name){};

  // non-copyable
  Planner(Planner const&) = delete;
  void operator=(Planner const&) = delete;

  /** \brief Get the name of the planner.
   *  \return The planner's name.
   */
  const std::string& getName() const
  {
    return name_;
  };
  const RobotPtr getRobot() const
  {
    return robot_;
  };

protected:
  const std::string name_;               ///< Namespace for the planner.
  const std::string robot_description_;  ///< Namespace for the planner.
  RobotPtr robot_;
};

}  // namespace moveit_benchmark_suite
