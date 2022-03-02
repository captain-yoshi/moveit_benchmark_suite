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

/* Author: Constantinos Chamzas
   Desc:
*/

#pragma once

#include <random>
#include <moveit_benchmark_suite/constants.h>

#include <Eigen/Core>

namespace moveit_benchmark_suite {
/** \brief Collection of methods relating to random sampling
 */
namespace RNG {
/** \brief Generate a random real in  [0,1).
 *  \return Sampled number.
 */
double uniform01();

/** \brief Generate a random real within given bounds: [\a lower_bound, \a upper_bound)
 *  \param[in] lower_bound Lower bound of uniform distribution.
 *  \param[in] upper_bound Upper bound of uniform distribution.
 *  \return Sampled number.
 */
double uniformReal(double lower_bound, double upper_bound);

/** \brief Generate a random integer within given bounds: [\a lower_bound, \a upper_bound)
 *  \param[in] lower_bound Lower bound of uniform distribution.
 *  \param[in] upper_bound Upper bound of uniform distribution.
 *  \return Sampled number.
 */
int uniformInt(int lower_bound, int upper_bound);

/** \brief Generate a random boolean.
 *  \return Sampled number.
 */
bool uniformBool();

/** Generate a random real using a normal distribution with mean 0 and variance 1
 *  \return Sampled number.
 */
double gaussian01();

/** \brief Generate a random real using a normal distribution with given \a mean and \e standard
 * deviation.
 *  \param[in] mean Mean of the normal distribution.
 *  \param[in] stddev Standard deviation of the normal distribution.
 *  \return Sampled number.
 */
double gaussian(double mean, double stddev);

/** \brief Generate a random real using a normal distribution with zero mean and given \e standard
 * deviation.
 *  \param[in] stddev Standard deviation of the normal distribution.
 *  \return Sampled number.
 */
double gaussian(double stddev);

/** \brief Uniform random sampling of Euler roll-pitch-yaw angles within lower bound \a lbound and
 * upper bound \a ubound computed value has the order (roll, pitch, yaw).
 *  \param[in] lbound Lower bound for roll pitch yaw.
 *  \param[in] ubound Upper bound for roll pitch yaw.
 *  \return roll-pitch-yaw vector.
 */
Eigen::Vector3d uniformRPY(const Eigen::Vector3d& lbound, const Eigen::Vector3d& ubound);

/** \brief Uniform random sampling of Euler roll-pitch-yaw angles within lower bound \a lbound and
 * upper bound \a ubound computed value has the order (roll, pitch, yaw).
 *  \param[in] bounds [-bounds, bounds] is lower and upper bound is  respectively.
 *  \return roll-pitch-yaw vector.
 */
Eigen::Vector3d uniformRPY(const Eigen::Vector3d& bounds);

/** \brief Uniform random sampling of Euler roll-pitch-yaw angles, roll, yaw in range [-pi, pi) and
 * pitch in range[-pi/2, pi/2) computed value has the order (roll, pitch, yaw).
 *  \return roll-pitch-yaw vector.
 */
Eigen::Vector3d unifromRPY();

/** \brief Generate a uniform real vector within given bounds: [\a lower_bound, \a upper_bound)
 *  \param[in] lbound Lower bound vector of uniform distribution.
 *  \param[in] ubound Upper bound vector of uniform distribution.
 *  \return Sampled vector.
 */
Eigen::Vector3d uniformVec(const Eigen::Vector3d& lbound, const Eigen::Vector3d& ubound);

/** \brief Generate a uniform real vector within given bounds: [\a -bounds, \a bounds)
 *  \param[in] bounds Upper and (negative) lower bound vector of uniform distribution.
 *  \return Sampled vector.
 */
Eigen::Vector3d uniformVec(const Eigen::Vector3d& bounds);

/** \brief Generate a random real vector using a normal distribution with given \a mean and \e
 * standard deviation
 *  \param[in] mean Mean vector of the normal distribution.
 *  \param[in] stddev Standard deviation vector (diagonal covariance) of the normal distribution.
 *  \return Sampled vector.
 */
Eigen::Vector3d gaussianVec(const Eigen::Vector3d& mean, const Eigen::Vector3d& stddev);

/** \brief Generate a random real vector using a normal distribution with \e mean zero and \e
 * standard deviation.
 *  \param[in] stddev Standard deviation vector (diagonal covariance) of the normal distribution.
 *  \return Sampled vector.
 */
Eigen::Vector3d gaussianVec(const Eigen::Vector3d& stddev);

/** \brief Choose a random element between \a start and \a end.
 *  \param[in] start Start iterator.
 *  \param[in] end End iterator.
 *  \return Chosen element.
 */
template <typename Iter>
Iter uniformSample(Iter start, Iter end)
{
  std::advance(start, uniformInt(0, std::distance(start, end) - 1));
  return start;
};

/** \brief Choose a random element from a vector.
 *  \param[in] vector Vector to sample from.
 *  \return Chosen element.
 */
template <typename Type>
Type& uniformSample(std::vector<Type> vector)
{
  return *uniformSample(vector.begin(), vector.end());
}

}  // namespace RNG
}  // namespace moveit_benchmark_suite
