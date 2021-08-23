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

#include <moveit_benchmark_suite/token.h>

namespace moveit_benchmark_suite
{
std::vector<std::string> splitStr(std::string s, std::string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  if (delimiter.empty())
  {
    res.push_back(s);
    return res;
  }

  while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

std::string replaceStr(std::string subject, const std::string& search, const std::string& replace)
{
  size_t pos = 0;
  while ((pos = subject.find(search, pos)) != std::string::npos)
  {
    subject.replace(pos, search.length(), replace);
    pos += replace.length();
  }
  return subject;
}

Token::Token(const std::string& token, const std::string& del) : token(token), del(del)
{
  keys = splitStr(token, del);

  // Extract value
  if (!keys.empty())
  {
    if (!keys[keys.size() - 1].empty())
      value = keys[keys.size() - 1];
    keys.pop_back();
  }

  if (keys.empty() || keys[0].empty())
    reset();
  else if (keys.size() == 1 && keys[0].empty())
    reset();
  else
  {
    n_key = keys.size();
    key_root = keys[0];
    createTokenNode(0, node);
  }
}

void Token::createTokenNode(int i, YAML::Node& n)
{
  YAML::Node temp;

  if (i == n_key - 1)
    n[keys[i]] = value;
  else
  {
    createTokenNode(i + 1, temp);
    n[keys[i]] = temp;
  }
};

void Token::reset()
{
  token = "";
  value = "";

  keys = std::vector<std::string>();
  n_key = 0;
  node = YAML::Node();
};

bool operator>(const Token& t1, const Token& t2)
{
  return t1.n_key > t2.n_key;
}

bool operator>=(const Token& t1, const Token& t2)
{
  return t1.n_key >= t2.n_key;
}

bool operator<(const Token& t1, const Token& t2)
{
  return t1.n_key < t2.n_key;
}

bool operator<=(const Token& t1, const Token& t2)
{
  return t1.n_key <= t2.n_key;
}
namespace token
{
bool hasValue(const Token& t1)
{
  if (t1.value.empty())
    return false;
  return true;
}

// Check tokens overlap ex. overlaps -> "test/alpha" "test/alpha"
bool overlap(const Token& t1, const Token& t2)
{
  // Case one of the token has no key so don't overlap
  if (!t1.n_key || !t2.n_key)
    return false;

  // Case tokens first keys are different
  if (t1.n_key && t2.n_key && t1.keys[0].compare(t2.keys[0]) != 0)
    return false;

  // Case tokens are identity (same keys)
  if (t1.node.is(t2.node))
  {
    if (t1.value.empty() || t2.value.empty())  // If empty value they overlap
      return true;
    else if (t1.value.compare(t2.value) == 0)  // If same value they overlap
      return true;
    return false;
  }

  // Case tokens are not identity and have values
  if (!t1.value.empty() && !t2.value.empty() && t1.value.compare(t2.value) != 0)  // has different values don't overlap
    return false;

  // Case tokens are not identity and one has an empty value
  int n;
  if (t1 >= t2)
    n = t2.n_key;
  else
    n = t1.n_key;

  for (int i = 0; i < n; ++i)
  {
    if (t1.keys[i].compare(t2.keys[i]) != 0)
      return false;
  }

  return true;
}

bool compareToNode(const Token& t, const YAML::Node& node, YAML::Node& res)
{
  if (t.keys.empty())
    return false;

  res = YAML::Clone(node);
  for (const auto& key : t.keys)
  {
    try
    {
      if (res[key])
        res = res[key];
      else
        return false;
    }
    catch (YAML::BadSubscript& e)
    {
      return false;
    }
  }

  // compare value
  if (!t.value.empty())
  {
    std::string node_value;
    try
    {
      node_value = res.as<std::string>();
    }
    catch (YAML::BadConversion& e)
    {
      return false;
    }

    if (t.value.compare(node_value) != 0)
      return false;
  }
  return true;
}

bool compareToNode(const TokenSet& tokens, const YAML::Node& node)
{
  for (const auto& token : tokens)
  {
    if (!compareToNode(token, node))
      return false;
  }
  return true;
}

bool compareToNode(const Token& t, const YAML::Node& node)
{
  YAML::Node dummy;
  return compareToNode(t, node, dummy);
}

std::set<std::string> getChildNodeKeys(const YAML::Node& node)
{
  std::set<std::string> keys;
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    keys.insert(it->first.as<std::string>());
  }
  return keys;
}

std::set<std::string> getChildNodeValues(const YAML::Node& node)
{
  std::set<std::string> keys;
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    keys.insert(it->second.as<std::string>());
  }
  return keys;
}

std::map<std::string, std::string> getChildNodeKeyValues(const YAML::Node& node)
{
  std::map<std::string, std::string> map;
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    map.insert({ it->first.as<std::string>(), it->second.as<std::string>() });
  }
  return map;
}

std::string getNodeValue(const YAML::Node& node)
{
  std::string res;
  try
  {
    res = node.as<std::string>();
  }
  catch (YAML::BadConversion& e)
  {
    return res;
  }
  return res;
}

}  // namespace token
}  // namespace moveit_benchmark_suite
