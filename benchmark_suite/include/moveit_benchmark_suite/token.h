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
 *   * Neither the name of Bielefeld University nor the names of its
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

/* Author: Modified version of Zachary Kingston robowflex
   Desc:
*/

#pragma once

#include <string>
#include <vector>
#include <map>

#include <yaml-cpp/yaml.h>

namespace moveit_benchmark_suite
{
std::vector<std::string> splitStr(std::string s, std::string delimiter);

std::string replaceStr(std::string subject, const std::string& search, const std::string& replace);

// Token with empty keys will revert to an empty Token
struct Token
{
  Token() = default;

  Token(const std::string& token);

  Token(const std::string& token, const std::string& value, const std::string& del = "/");

  void createTokenNode(int i, YAML::Node& n);

  void reset();

  std::string token;
  std::string value;
  std::string del;

  std::vector<std::string> keys;  // Splited token
  int n_key;
  std::string key_root;
  YAML::Node node;
};  // namespace moveit_benchmark_suite

using TokenSet = std::set<Token>;

bool operator>(const Token& t1, const Token& t2);
bool operator>=(const Token& t1, const Token& t2);
bool operator<(const Token& t1, const Token& t2);
bool operator<=(const Token& t1, const Token& t2);

namespace token
{
bool hasValue(const Token& t1);

// Check tokens overlap ex. overlaps -> "test/alpha" "test/alpha"
bool overlap(const Token& t1, const Token& t2);
bool compareToNode(const Token& t, const YAML::Node& node, YAML::Node& res);
bool compareToNode(const Token& t, const YAML::Node& node);

std::set<std::string> getChildNodeKeys(const YAML::Node& node);
std::set<std::string> getChildNodeValues(const YAML::Node& node);
std::map<std::string, std::string> getChildNodeKeyValues(const YAML::Node& node);
std::string getNodeValue(const YAML::Node& node);

}  // namespace token
}  // namespace moveit_benchmark_suite

namespace std
{
template <>
struct less<moveit_benchmark_suite::Token>

{
  bool operator()(const moveit_benchmark_suite::Token lhs, const moveit_benchmark_suite::Token rhs)

  {
    return lhs.token + lhs.value < rhs.token + rhs.value;
  }
};

}  // namespace std
