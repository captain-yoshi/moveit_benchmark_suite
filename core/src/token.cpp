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

/* Author: Captain Yoshi
   Desc:
*/

#include <moveit_benchmark_suite/token.h>

namespace moveit_benchmark_suite {
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
  if (search.empty())
    return subject;

  size_t pos = 0;
  while ((pos = subject.find(search, pos)) != std::string::npos)
  {
    subject.replace(pos, search.length(), replace);
    pos += replace.length();
  }
  return subject;
}

Token::Token(const std::string& ns, const std::string& value, const std::string& del)
  : ns_(ns), value_(value), del_(del)
{
  // create Node from Token
  auto node = tree_.rootref();
  node |= ryml::MAP;

  auto keys = splitStr(ns, del);

  // remove first and last key if empty
  if (!keys.empty())
    if (keys[0].empty())
      keys.erase(keys.begin());
  if (keys.size() >= 2)
    if (keys[keys.size() - 1].empty())
      keys.erase(keys.end() - 1);

  createNode(0, keys, node);

  // Check if namespace is absolute or relative
  if (!del.empty() && ns.size() >= del.size() && ns.compare(0, del.size(), del) == 0)
    ns_rel_ = false;
  else
    ns_rel_ = true;
}

void Token::createNode(std::size_t ctr, const std::vector<std::string>& keys, ryml::NodeRef& n)
{
  if (keys.empty())
    return;

  else if (keys.size() == 1)
  {
    n.append_child() << ryml::key(keys[0]) |= ryml::KEYMAP;
  }
  else
  {
    auto c = n.append_child();

    for (std::size_t i = 0; i < keys.size(); ++i)
    {
      if (i == keys.size() - 1)
        break;
      if (i == keys.size() - 2)
      {
        c |= ryml::KEYVAL;
        c << ryml::key(keys[i]);
      }
      else
      {
        c |= ryml::KEYMAP;
        c << ryml::key(keys[i]);
        c = c.append_child();
      }
    }

    c << keys.back();
  }
};

void Token::reset()
{
  ns_ = "";
  value_ = "";
  del_ = "/";

  ns_rel_ = true;
  tree_.clear();
};

const std::string& Token::getValue() const
{
  return value_;
}
const std::string& Token::getDelimiter() const
{
  return del_;
}
const std::string& Token::getNamespace() const
{
  return ns_;
}
const ryml::NodeRef Token::getNode() const
{
  return tree_.rootref();
}

bool Token::hasValue() const
{
  if (value_.empty())
    return false;
  return true;
}
bool Token::isRelative() const
{
  if (ns_rel_)
    return true;
  return false;
}
bool Token::isAbsolute() const
{
  return !isRelative();
}

std::ostream & operator << (std::ostream &out, const Token &t)
{
    return out << t.getNamespace() << " (value '" << t.getValue() << "')";
}
}  // namespace moveit_benchmark_suite
