/* Author: Zachary Kingston */

#pragma once

#include <string>   // for std::string
#include <utility>  // for std::pair
#include <fstream>  // for std::ofstream
#include <chrono>

#include <boost/date_time.hpp>  // for date operations

#include <yaml-cpp/yaml.h>  // for YAML parsing

namespace moveit_benchmark_suite
{
struct CPUInfo
{
  std::string model;
  std::string model_name;
  std::string family;
  std::string vendor_id;
  std::string architecture;
  std::string sockets;
  std::string core_per_socket;
  std::string thread_per_core;
};

struct GPUInfo
{
  std::vector<std::string> model_names;
};

struct OSInfo
{
  std::string kernel_name;
  std::string kernel_release;
  std::string distribution;
  std::string version;
};

struct RosPkgInfo
{
  std::string version;
  std::string git_branch;
  std::string git_commit;
};

/** \brief File and ROS Input / Output operations.
 */
namespace IO
{
/** \brief Generates a UUID.
 *  \return String of UUID.
 */
std::string generateUUID();

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
const std::string resolvePath(const std::string& path);

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
const std::pair<bool, YAML::Node> loadFileToYAML(const std::string& path);

std::string getFilePath(const std::string& file);
std::string getFileName(const std::string& file);
std::string getAbsFile(const std::string& file);
std::string getEnvironmentPath(const std::string& env);

/** \brief Creates a file and opens an output stream. Creates directories if they do not exist.
 *  \param[out] out Output stream to initialize.
 *  \param[in] file File to create and open.
 */
bool createFile(std::ofstream& out, const std::string& file);

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

const CPUInfo getHardwareCPU();

const GPUInfo getHardwareGPU();

const OSInfo getOSInfo();

const RosPkgInfo getMoveitInfo();
const RosPkgInfo getMoveitBenchmarkSuiteInfo();

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

/** \brief Write the contents of a YAML node out to a potentially new file.
 *  \param[in] node Node to write.
 *  \param[in] file Filename to open.
 *  \return True on success, false otherwise.
 */
bool YAMLToFile(const YAML::Node& node, const std::string& file);

/** \brief Dump a message (or YAML convertable object) to a file.
 *  \param[in] msg Message to dump.
 *  \param[in] file File to dump message to.
 *  \tparam T Type of the message.
 */
template <typename T>
bool messageToYAMLFile(T& msg, const std::string& file)
{
  YAML::Node yaml;
  yaml = msg;

  return YAMLToFile(yaml, file);
}

/** \brief Load a message (or YAML convertable object) from a file.
 *  \param[out] msg Message to load into.
 *  \param[in] file File to load message from.
 *  \tparam T Type of the message.
 */
template <typename T>
bool YAMLFileToMessage(T& msg, const std::string& file)
{
  const auto& result = IO::loadFileToYAML(file);
  if (result.first)
    msg = result.second.as<T>();

  return result.first;
}
}  // namespace IO
}  // namespace moveit_benchmark_suite
