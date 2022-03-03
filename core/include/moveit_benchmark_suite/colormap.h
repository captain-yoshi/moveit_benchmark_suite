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

#include <Eigen/Geometry>

namespace moveit_benchmark_suite {
namespace color {
/** \brief Maps a scalar s in [0, 1] to the Viridis colormap.
 *  \param[in] s Scalar to map.
 *  \param[out] color Output color.
 */
void viridis(double s, Eigen::Ref<Eigen::Vector4d> color);

/** \brief Maps a scalar s in [0, 1] to the Cool-Warm colormap.
 *  \param[in] s Scalar to map.
 *  \param[out] color Output color.
 */
void coolwarm(double s, Eigen::Ref<Eigen::Vector4d> color);

/** \brief Maps a scalar s in [0, 1] to the Extended Kindlmann colormap.
 *  \param[in] s Scalar to map.
 *  \param[out] color Output color.
 */
void extKindlmann(double s, Eigen::Ref<Eigen::Vector4d> color);

/** \brief Maps a scalar s in [0, 1] to the Plasma colormap.
 *  \param[in] s Scalar to map.
 *  \param[out] color Output color.
 */
void plasma(double s, Eigen::Ref<Eigen::Vector4d> color);

/** \brief Maps a scalar s in [0, 1] to the Turbo colormap.
 *  \param[in] s Scalar to map.
 *  \param[out] color Output color.
 */
void turbo(double s, Eigen::Ref<Eigen::Vector4d> color);

/** \brief Maps a scalar s in [0, 1] to greyscale.
 *  \param[in] s Scalar to map.
 *  \param[out] color Output color.
 */
void grayscale(double s, Eigen::Ref<Eigen::Vector4d> color);

/** \brief Maps an RGB color to a greyscale color based on luminosity.
 *  \param[in,out] color Color to convert.
 */
void toGrayscale(Eigen::Ref<Eigen::Vector4d> color);

// Commonly used named colors.
const static Eigen::Vector4d BLACK{ 0., 0, 0, 1 };
const static Eigen::Vector4d WHITE{ 1, 1, 1, 1 };
const static Eigen::Vector4d GRAY{ 0.5, 0.5, 0.5, 1 };
const static Eigen::Vector4d RED{ 1, 0, 0, 1 };
const static Eigen::Vector4d PINK{ 1, 0.37, 0.81, 1 };
const static Eigen::Vector4d PURPLE{ 0.62, 0.32, 1, 1 };
const static Eigen::Vector4d GREEN{ 0, 1, 0, 1 };
const static Eigen::Vector4d BLUE{ 0, 0, 1, 1 };
const static Eigen::Vector4d YELLOW{ 1, 0.88, 0.12, 1 };
const static Eigen::Vector4d ORANGE{ 1, 0.6, 0.06, 1 };
const static Eigen::Vector4d BROWN{ 0.6, 0.5, 0.38, 1 };

}  // namespace color
}  // namespace moveit_benchmark_suite
