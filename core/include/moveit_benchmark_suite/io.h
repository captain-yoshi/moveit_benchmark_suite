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

#include <string>   // for std::string
#include <utility>  // for std::pair
#include <fstream>  // for std::ofstream
#include <chrono>

#include <boost/date_time.hpp>  // for date operations

#include <moveit_benchmark_suite/dataset.h>
#include <moveit_benchmark_suite/metadata.h>
#include <moveit_benchmark_suite/serialization/ryml.h>

namespace moveit_benchmark_suite {
/** \brief File and ROS Input / Output operations.
 */
namespace IO {
/** \brief Generates a UUID.
 *  \return String of UUID.
 */
std::string generateUUID();

bool isExtension(const std::string& path_string, const std::string& extension);

/** \brief Resolves `package://` URLs to their canonical form.
 *  The path does not need to exist, but the package does. Can be used to write new files in packages.
 *  \param[in] path Path to resolve.
 *  \return The canonical path, or "" on failure.
 */
const std::string resolvePackage(const std::string& path);

/** \brief Finds all package URIs within a string.
 *  \param[in] string String to search.
 *  \return List of package URIs.
 */
std::set<std::string> findPackageURIs(const std::string& string);

/** \brief Resolves `package://` URLs and relative file paths to their canonical form.
 *  \param[in] path Path to resolve.
 *  \return The canonical path, or "" on failure.
 */
const std::string resolvePath(const std::string& path, bool verbose = true);

/** \brief Resolves `package://` URLs to get the directory this path is in.
 *  \param[in] path Path to get the parent of.
 *  \return The directory that this path is contained in, or "" on failure.
 */
const std::string resolveParent(const std::string& path);

/** \brief Loads an XML or .xacro file to a string.
 *  \param[in] path File to load.
 *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
 */
const std::string loadXMLToString(const std::string& path);

/** \brief Loads a .xacro file to a string.
 *  \param[in] path File to load.
 *  \return The loaded file, or "" on failure (file does not exist or .xacro is malformed).
 */
const std::string loadXacroToString(const std::string& path);

/** \brief Loads a file to a string.
 *  \param[in] path File to load.
 *  \return The loaded file, or "" on failure (file does not exist).
 */
const std::string loadFileToString(const std::string& path);

/** \brief Runs a command \a cmd and returns stdout as a string.
 *  \param[in] cmd Command to run.
 *  \return Contents of stdout from \a cmd, or "" on failure.
 */
const std::string runCommand(const std::string& cmd);

/** \brief Loads a file to a YAML node.
 *  \param[in] path File to load.
 *  \return A pair, where the first is true on success false on failure, and second is the YAML node.
 */
ryml::substr loadFileToYAML(const std::string& path, ryml::NodeRef& node);

std::string getEnvironmentPath(const std::string& env);

/** \brief Creates a file and opens an output stream. Creates directories if they do not exist.
 *  \param[out] out Output stream to initialize.
 *  \param[in] file File to create and open.
 *  \return Absolute path after expansion.
 */
std::string createFile(std::ofstream& out, const std::string& file);

/** \brief Creates a temporary file and opens an output stream.
 *  \param[out] out Output stream to initialize.
 *  \return Filename of temporary file.
 */
std::string createTempFile(std::ofstream& out);

/** \brief Deletes a file.
 *  \param[in] file File to delete.
 */
void deleteFile(const std::string& file);

/** \brief Lists of the contents of a directory.
 *  \param[in] directory Directory to list.
 *  \return A pair of a bool and a vector of strings of filenames of the directories contents. The
 * first element will be true on success, false on failure. These filenames are absolute paths.
 */
const std::pair<bool, std::vector<std::string>> listDirectory(const std::string& directory);

/** \brief Get the hostname of the system.
 *  \return String of the hostname.
 */
const std::string getHostname();

/** \brief Get the process ID of this process.
 *  \return The process ID.
 */
std::size_t getProcessID();

/** \brief Get the thread ID of the current thread.
 *  \return The thread ID.
 */
std::size_t getThreadID();

/** \brief Get the CPU specification.
 *  \return The CPU spec.
 */
metadata::CPU getCPUMetadata();

/** \brief Get the GPU specification.
 *  \return The list of GPU spec.
 */
std::vector<metadata::GPU> getGPUMetadata();

/** \brief Get the operating system specification.
 *  \return The OS spec.
 */
metadata::OS getOSMetadata();

/** \brief Get ROS package specification.
 *  \param[in] name Name of the package.
 *  \return The Software spec.
 */
metadata::SW getROSPkgMetadata(const std::string& name);

/** \brief Get ROS package specification from pluginlib.
 *  \param[in] plugin_names Names of the plugin class from the xml plugin file
 *             e.g. <class name="ur_kinematics/UR5KinematicsPlugin" ... >
 *  \param[in] package_name Names of the ROS package
 *  \param[in] filter_prefix Package name prefix to filter out
 *  \return The software specifications.
 */
std::vector<metadata::SW> getROSPkgMetadataFromPlugins(const std::set<std::string>& plugin_names,
                                                       const std::string& package_name = "moveit_core",
                                                       const std::string& filter_prefix = "moveit");

/** \brief Get Debian package specification.
 *  \param[in] name Name of the package.
 *  \return The Software spec.
 */
metadata::SW getDebianPkgMetadata(const std::string& name);

/** \brief Get the current time (up to milliseconds)
 *  \return The time.
 */
boost::posix_time::ptime getDate(boost::posix_time::microsec_clock& clock);
boost::posix_time::ptime getDateUTC(boost::posix_time::microsec_clock& clock);
std::string getDateStr();

/** \brief Get a duration in seconds from two times.
 *  \param[in] start The start time.
 *  \param[in] finish The finish time.
 *  \return The time in seconds.
 */
double getSeconds(std::chrono::high_resolution_clock::time_point start,
                  std::chrono::high_resolution_clock::time_point end);

/** \brief Put the current thread to sleep for a desired amount of seconds.
 *  \param[in] seconds Seconds to sleep for.
 */
void threadSleep(double seconds);

/** \brief Separates a \a string into casted tokens, based upon \a separators.
 *  \tparam The type of element to cast strings into.
 *  \param[in] string String to tokenize.
 *  \param[in] separators Separators to split string on.
 *  \return The tokenized string.
 */
template <typename T>
std::vector<T> tokenize(const std::string& string, const std::string& separators = " ");

/** \brief Compare valid keys against node keys
 *  \param[in] node Node to comapre.
 *  \param[in] keys List of keys used to compare.
 *  \return True if keys matches node keys, false otherwise.
 */
bool validateNodeKeys(const ryml::NodeRef& node, const std::vector<std::string>& keys);

/** \brief Write the contents of a YAML node out to a potentially new file.
 *  \param[in] node Node to write.
 *  \param[in] file Filename to open.
 *  \return True on success, false otherwise.
 */
bool YAMLToFile(const ryml::NodeRef& node, const std::string& file);

/** \brief Dump a message (or YAML convertable object) to a file.
 *  \param[in] msg Message to dump.
 *  \param[in] file File to dump message to.
 *  \tparam T Type of the message.
 */
template <typename T>
bool messageToYAMLFile(T& msg, const std::string& file)
{
  ryml::Tree t;
  auto node = t.rootref();
  node << msg;

  return YAMLToFile(node, file);
}

/** \brief Load a message (or YAML convertable object) from a file.
 *  \param[out] msg Message to load into.
 *  \param[in] file File to load message from.
 *  \tparam T Type of the message.
 */
template <typename T>
bool YAMLFileToMessage(T& msg, const std::string& file)
{
  ryml::Tree tree;
  ryml::NodeRef node = tree.rootref();
  auto substr = IO::loadFileToYAML(file, node);
  if (substr.not_empty())
    node >> msg;

  return substr.not_empty();
}

}  // namespace IO
}  // namespace moveit_benchmark_suite
