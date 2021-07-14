
#pragma once

#include <moveit/robot_model/robot_model.h>

namespace moveit
{
namespace benchmark_suite
{
MOVEIT_CLASS_FORWARD(Robot);

class Robot
{
public:
  Robot(const std::string& name, const std::string& robot_description);

  // non-copyable
  Robot(Robot const&) = delete;
  void operator=(Robot const&) = delete;

  /** \name Initialization and Loading
      \{ */

  /** \brief Initializes a robot from a kinematic description.
   *  A default semantic description is used.
   *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
   *  \return True on success, false on failure.
   */
  bool initialize();
  const std::string& getModelName() const;

  /** \brief Get the robot's name.
   *  \return The robot's name.
   */
  const std::string& getName() const;

  /** \brief Get a const reference to the loaded robot model.
   *  \return The robot model.
   */
  const robot_model::RobotModelPtr& getModelConst() const;

  /** \brief Get a reference to the loaded robot model.
   *  \return The robot model.
   */
  robot_model::RobotModelPtr& getModel();

private:
  const std::string name_;
  const std::string robot_description_;
  robot_model::RobotModelPtr model_;
};
}  // namespace benchmark_suite
}  // namespace moveit
