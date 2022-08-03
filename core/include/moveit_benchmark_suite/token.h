/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
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

/* Author: Captain Yoshi
   Desc: Token for handling namespaces
*/

#pragma once

#include <string>
#include <vector>
#include <map>

#include <moveit_benchmark_suite/serialization/ryml.h>

namespace moveit_benchmark_suite {
std::vector<std::string> splitStr(std::string s, std::string delimiter);

std::string replaceStr(std::string subject, const std::string& search, const std::string& replace);

///
class Token
{
public:
  Token() = default;

  Token(const std::string& ns, const std::string& value = "", const std::string& del = "/");

  void reset();

  const std::string& getValue() const;
  const std::string& getDelimiter() const;
  const std::string& getNamespace() const;

  const ryml::NodeRef getNode() const;

  bool hasValue() const;
  bool isRelative() const;
  bool isAbsolute() const;

private:
  void createNode(std::size_t ctr, const std::vector<std::string>& keys, ryml::NodeRef& n);

  std::string ns_;
  std::string value_;
  std::string del_;

  bool ns_rel_ = true;  // namespace is relative or absolute
  ryml::Tree tree_;
};

}  // namespace moveit_benchmark_suite
