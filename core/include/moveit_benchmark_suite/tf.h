/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Zachary Kingston, Constantinos Chamzas
   Desc:
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit_msgs/BoundingVolume.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <moveit/macros/class_forward.h>

#include <moveit_benchmark_suite/constants.h>
#include <moveit_benchmark_suite/geometry.h>

namespace moveit_benchmark_suite {
/** \brief Collection of methods relating to transforms and transform math.
 */
namespace TF {
/** \brief Creates the Identity pose.
 *  \return A new identity robot pose.
 */
Eigen::Isometry3d identity();

/** \brief Creates a robot pose from a linear component and zero orientation.
 *  \param[in] x X-axis translation.
 *  \param[in] y Y-ayis translation.
 *  \param[in] z Z-azis translation.
 *  \return A new robot pose from components.
 */
Eigen::Isometry3d createPoseXYZ(double x, double y, double z);

/** \brief Creates a robot pose from a linear component and zero orientation.
 *  \param[in] translation Translation component.
 *  \return A new robot pose from components.
 */
Eigen::Isometry3d createPoseXYZ(const Eigen::Ref<const Eigen::Vector3d>& translation);

/** \brief Creates a robot pose from a linear component and XYZ convention Euler angles
 *  \param[in] x X-axis translation.
 *  \param[in] y Y-ayis translation.
 *  \param[in] z Z-azis translation.
 *  \param[in] X Rotation about X.
 *  \param[in] Y Rotation about Y.
 *  \param[in] Z Rotation about Z.
 *  \return A new robot pose from components.
 */
Eigen::Isometry3d createPoseXYZ(double x, double y, double z, double X, double Y, double Z);

/** \brief Creates a robot pose from a linear component and XYZ convention Euler angles
 *  \param[in] translation Translation component.
 *  \param[in] rotation Rotational component (X, Y, Z angles).
 *  \return A new robot pose from components.
 */
Eigen::Isometry3d createPoseXYZ(const Eigen::Ref<const Eigen::Vector3d>& translation,
                                const Eigen::Ref<const Eigen::Vector3d>& rotation);

/** \brief Creates a robot pose from a linear component and a Quaternion
 *  \param[in] x X-axis translation.
 *  \param[in] y Y-axis translation.
 *  \param[in] z Z-axis translation.
 *  \param[in] W Real quaternion component.
 *  \param[in] X i quaternion component.
 *  \param[in] Y j quaternion component.
 *  \param[in] Z k quaternion component.
 *  \return A new robot pose from components.
 */
Eigen::Isometry3d createPoseQ(double x, double y, double z, double W, double X, double Y, double Z);

/** \brief Creates a robot pose from a linear component and a quaternion.
 *  \param[in] translation translation component.
 *  \param[in] rotation rotational component (W, X, Y, Z quaternion values).
 *  \return A new robot pose from components.
 */
Eigen::Isometry3d createPoseQ(const Eigen::Ref<const Eigen::Vector3d>& translation,
                              const Eigen::Ref<const Eigen::Vector4d>& rotation);

/** \brief Creates a robot pose from a linear component and a quaternion.
 *  \param[in] translation translation component.
 *  \param[in] rotation rotational component.
 *  \return A new robot pose from components.
 */
Eigen::Isometry3d createPoseQ(const Eigen::Ref<const Eigen::Vector3d>& translation, const Eigen::Quaterniond& rotation);

/** \brief Get the rotational component of a robot pose.
 *  \param[in] pose The pose to get the rotation from.
 *  \return The rotational component of the pose.
 */
Eigen::Quaterniond getPoseRotation(const Eigen::Isometry3d& pose);

/** \brief Converts a point message to an Eigen::Vector3d.
 *  \param[in] msg Message to convert.
 *  \return \a msg as an Eigen::Vector3d.
 */
Eigen::Vector3d pointMsgToEigen(const geometry_msgs::Point& msg);

/** \brief Converts an Eigen::Vector3d to a point message.
 *  \param[in] vector Vector to convert.
 *  \return \a vector as a point message.
 */
geometry_msgs::Point pointEigenToMsg(const Eigen::Vector3d& vector);

/** \brief Converts a vector message to an Eigen::Vector3d.
 *  \param[in] msg Message to convert.
 *  \return \a msg as an Eigen::Vector3d.
 */
Eigen::Vector3d vectorMsgToEigen(const geometry_msgs::Vector3& msg);

/** \brief Converts an Eigen::Vector3d to a vector message.
 *  \param[in] vector Vector to convert.
 *  \return \a vector as a vector message.
 */
geometry_msgs::Vector3 vectorEigenToMsg(const Eigen::Vector3d& vector);

/** \brief Converts a pose message to Eigen::Isometry3d.
 *  \param[in] msg Message to convert.
 *  \return \a msg an Eigen::Isometry3d.
 */
Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::Pose& msg);

/** \brief Converts an Eigen::Isometry3d to a pose message.
 *  \param[in] pose Pose to convert.
 *  \return \a pose as a pose message.
 */
geometry_msgs::Pose poseEigenToMsg(const Eigen::Isometry3d& pose);

/** \brief Converts a quaternion message to Eigen::Quaterniond.
 *  \param[in] msg Message to convert.
 *  \return \a msg an Eigen::Quaterniond.
 */
Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion& msg);

/** \brief Converts an Eigen::Quaterniond to a quaternion message.
 *  \param[in] quaternion Quaternion to convert.
 *  \return \a quaternion as a quaternion message.
 */
geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond& quaternion);

/** \brief Get a bounding volume message for given \a geometry at a \a pose.
 *  \param[in] pose Pose to place geometry at.
 *  \param[in] geometry Geometry to get bounding volume for.
 *  \return Bounding volume message for \a geometry at \a pose.
 */
moveit_msgs::BoundingVolume getBoundingVolume(const Eigen::Isometry3d& pose, const GeometryConstPtr& geometry);

/** \brief Get a position constraint message.
 *  \param[in] ee_name The name of the end-effector link.
 *  \param[in] base_name The frame of pose and orientation.
 *  \param[in] pose The pose of \a geometry in \a base_frame.
 *  \param[in] geometry The geometry describing the position constraint.
 *  \return The position constraint as a MoveIt messsage.
 */
moveit_msgs::PositionConstraint getPositionConstraint(const std::string& ee_name, const std::string& base_name,
                                                      const Eigen::Isometry3d& pose, const GeometryConstPtr& geometry);

Eigen::Vector3d samplePositionConstraint(const moveit_msgs::PositionConstraint& pc);

/** \brief Get an orientation constraint message.
 *  \param[in] ee_name The name of the end-effector link.
 *  \param[in] base_name The frame of pose and orientation.
 *  \param[in] orientation The desired orientation.
 *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
 *  \return The orientation constraint as a MoveIt message.
 */
moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string& ee_name, const std::string& base_name,
                                                            const Eigen::Quaterniond& orientation,
                                                            const Eigen::Vector3d& tolerances);

/** \brief Sample an orientation from a given \a orientation with XYZ Euler angle \a tolerances.
 *  \param[in] orientation The desired mean orientation.
 *  \param[in] tolerances XYZ Euler angle tolerances about orientation.
 *  \return The sampled orientation.
 */
Eigen::Quaterniond sampleOrientation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& tolerances);

/** \brief Sample an orientation within the XYZ Euler angle \a bounds.
 *  \param[in] bounds XYZ Euler angle bounds about orientation.
 *  \return The sampled orientation.
 */
Eigen::Quaterniond sampleOrientationUniform(const Eigen::Vector3d& bounds);

/** \brief Offset an orientation by a rotation about an axis.
 *  \param[in] orientation Orientation to offset.
 *  \param[in] axis Axis to offset orientation about.
 *  \param[in] value Value by which to offset.
 *  \return The new orientation.
 */
Eigen::Quaterniond offsetOrientation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& axis, double value);

/** \brief Sample a position within the given \a bounds using a uniform distribution.
 *  \param[in] bounds The desired mean orientation.
 *  \return The sampled position.
 */
Eigen::Vector3d samplePositionUniform(const Eigen::Vector3d& bounds);

/** \brief Sample a position from a gaussian distribution with mean zero and given standard deviation
 *  \param[in] stddev The desired standard deviation for the position.
 *  \return The sampled position.
 */
Eigen::Vector3d samplePositionGaussian(const Eigen::Vector3d& stddev);

/** \brief Sample a pose within the given position, orientation bounds.
 *  \param[in] pos_bounds The desired position bounds.
 *  \param[in] orn_bounds The desired orientation bounds.
 *  \return The sampled pose.
 */
Eigen::Isometry3d samplePoseUniform(const Eigen::Vector3d& pos_bounds, const Eigen::Vector3d& orn_bounds);

/** \brief Sample a pose with gaussian sampling for the position with given variances and
 *  uniform sampling for the orientation within the given bounds.
 *  \param[in] pos_variances The desired position variances.
 *  \param[in] orn_bounds The desired orientation bounds.
 *  \return The sampled pose.
 */
Eigen::Isometry3d samplePoseGaussian(const Eigen::Vector3d& pos_variances, const Eigen::Vector3d& orn_bounds);

/** \brief Decode a message as a transform.
 *  \param[in] tf Transform message.
 *  \return The transform.
 */
Eigen::Isometry3d transformMsgToEigen(const geometry_msgs::TransformStamped& tf);

/** \brief Encode a transform as a message.
 *  \param[in] source Source frame.
 *  \param[in] target Target frame.
 *  \param[in] tf Transform between frames.
 *  \return Transform message.
 */
geometry_msgs::TransformStamped transformEigenToMsg(const std::string& source, const std::string& target,
                                                    const Eigen::Isometry3d& tf);

/** \brief Normalize an angle between -pi to pi.
 *  \param[in] v The angle.
 *  \return The normalized angle.
 */
double angleNormalize(double v);

/** \brief Convert an angle to degrees.
 *  \param[in] v The angle in radians.
 *  \return The angle in degrees.
 */
double toDegrees(double v);

/** \brief Convert an angle to radians.
 *  \param[in] v The angle in degrees.
 *  \return The angle in radians.
 */
double toRadians(double v);

/** \brief Checks if a vector is close to zero.
 *  \param[in] v Vector to check.
 *  \param[in] tolerance Tolerance of what is considered zero.
 *  \return Whether the vector's norm is below the tolerance threshold.
 */
bool isVecZero(const Eigen::Ref<const Eigen::VectorXd>& v, double tolerance = constants::eps);

}  // namespace TF
}  // namespace moveit_benchmark_suite
