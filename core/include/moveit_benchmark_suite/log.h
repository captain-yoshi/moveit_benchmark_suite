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

#include <boost/format.hpp>
#include <ros/console.h>

/** \file Logging
 *  Functions for logging throughout Robowflex. Uses the boost::format format string syntax.
 *  See https://www.boost.org/doc/libs/1_66_0/libs/format/doc/format.html for details.
 *  All functions follow a 'printf' style, where there is a format string followed by arguments.
 *  e.g., RBX_ERROR("%s failed! Try %+08d", "test", 10)
 *        RBX_ERROR("%1%: %2%", "argument 1", "argument 2")
 *
 *  There are the following debugging levels available, in descending order of priority:
 *  - FATAL, which also causes the program to terminate.
 *  - ERROR
 *  - WARN
 *  - INFO
 *  - DEBUG, which are not visible unless enabled (e.g., with showUpToDebug())
 *
 *  Currently, all logging is done through rosconsole. It is good practice to use the defined
 *  macros here for abstraction purpose.
 */

namespace moveit_benchmark_suite {
/** \brief Logging functions.
 */
namespace log {
/** \brief Recursion base case, return string form of formatted arguments.
 *  \param[in] f Formatter.
 *  \return String of formatted arguments.
 */
std::string formatRecurse(boost::format& f);

/** \brief Recursion base case, return string form of formatted arguments.
 *  \tparam[in] T Type of first argument.
 *  \tparam[in] Args format argument types.
 *  \param[in] f Formatter.
 *  \param[in] t First argument.
 *  \param[in] args Remaining format arguments.
 *  \return String of formatted arguments.
 */
template <class T, class... Args>
std::string formatRecurse(boost::format& f, T&& t, Args&&... args)
{
  return formatRecurse(f % std::forward<T>(t), std::forward<Args>(args)...);
}

/** \brief Recursion base case, return string form of formatted arguments.
 *  \tparam[in] Args format argument types.
 *  \param[in] fmt Format string.
 *  \param[in] args Format arguments.
 *  \return String of formatted arguments.
 */
template <typename... Args>
std::string format(const std::string& fmt, Args&&... args)
{
  boost::format f(fmt);
  return formatRecurse(f, std::forward<Args>(args)...);
}

/** \brief Show all logging messages fatal and above.
 */
void showUpToFatal();

/** \brief Show all logging messages error and above.
 */
void showUpToError();

/** \brief Show all logging messages warning and above.
 */
void showUpToWarning();

/** \brief Show all logging messages info and above.
 */
void showUpToInfo();

/** \brief Show all logging messages debug and above.
 */
void showUpToDebug();
}  // namespace log
}  // namespace moveit_benchmark_suite
