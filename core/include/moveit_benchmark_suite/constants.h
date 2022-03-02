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

/* Author: Zachary Kingston
   Desc:
*/

#pragma once

#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>

namespace moveit_benchmark_suite {
namespace constants {
// common
static const double half = boost::math::constants::half<double>();
static const double third = boost::math::constants::third<double>();
static const double eps = std::numeric_limits<double>::epsilon();
static const double inf = std::numeric_limits<double>::infinity();
static const double nan = std::numeric_limits<double>::quiet_NaN();

// pi
static const double pi = boost::math::constants::pi<double>();
static const double half_pi = boost::math::constants::half_pi<double>();
static const double quarter_pi = half_pi * half;
static const double two_pi = boost::math::constants::two_pi<double>();

// tolerances
static const double ik_tolerance = 0.001;
static const unsigned int ik_attempts = 50;
static const Eigen::Vector3d ik_vec_tolerance = { ik_tolerance, ik_tolerance, ik_tolerance };
static const double cart_rot_step_size = 0.01;
static const double cart_pos_step_size = 0.01;
static const double cart_rot_jump_tol = 0.25;
static const double cart_pos_jump_tol = 0.25;

// planning
static const double default_workspace_bound = 1.0;
static const double default_allowed_planning_time = 5.0;

}  // namespace constants
}  // namespace moveit_benchmark_suite
