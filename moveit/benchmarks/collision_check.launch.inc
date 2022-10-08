<?xml version="1.0"?>
<launch>
  <arg name="config_file" doc="collision checking benchmark configuration file" />

  <arg name="name" default="" doc="name of the generated dataset"/>
  <arg name="output_file" default="" doc="yaml file to store benchmark data"/>

  <arg name="visualize" default="false" doc="turns on and off visualization with rviz (bool type)"/>
  <include file="$(find moveit_benchmark_suite)/launch/moveit_rviz.launch" if="$(eval bool(visualize))"/>

  <arg name="debug" default="false" doc="run benchmark in gdb"/>
  <arg unless="$(arg debug)" name="launch_prefix" value=""/>
  <arg     if="$(arg debug)" name="launch_prefix" value="gdb --args"/>

  <!-- Launch benchmark node -->
  <node name="collision_check" pkg="moveit_benchmark_suite" type="collision_check" respawn="false" clear_params="true" output="screen" launch-prefix="$(arg launch_prefix)" required="true">
    <rosparam command="load" file="$(arg config_file)"/>

    <!-- Override config file if not empty -->
    <param name="name" value="$(arg name)"/>
    <param name="config_file" value="$(arg config_file)"/>
    <param name="output_file" value="$(arg output_file)"/>
    <param name="visualize" value="$(eval bool(visualize))"/>
  </node>
</launch>
