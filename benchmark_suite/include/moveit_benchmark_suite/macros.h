/* Author: Zachary Kingston */

#pragma once
#include <boost/version.hpp>  // for boost version macros

#include <moveit/version.h>  // for MoveIt version macros

#include <ros/common.h>  // for ROS version macros

/** \def Sets flow style for YAML nodes. */
#define ROBOWFLEX_YAML_FLOW(n) n.SetStyle(YAML::EmitterStyle::Flow);

/** \def Tests if boost is at least version 1.64 */
#define IS_BOOST_164 (BOOST_VERSION >= 106400u)
