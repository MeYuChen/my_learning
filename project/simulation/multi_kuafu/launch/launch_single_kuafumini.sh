#!/bin/bash
{
    gnome-terminal --tab "ros_bridge" -- bash -c "roslaunch udi_carla udi_carla_single_kuafumini_bridge.launch ;exec bash"
}&
sleep 2s
{
    gnome-terminal --tab "necessary_data" -- bash -c "roslaunch udi_carla udi_carla_necessary_simulation_nodes.launch;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "perception_process" -- bash -c "roslaunch udi_carla udi_carla_percep_preprocess.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "perception_tracking" -- bash -c "roslaunch udi_carla udi_carla_percep_tracking.launch ;exec bash"
}&sleep 1s
{
    gnome-terminal --tab "global_planner" -- bash -c "roslaunch udi_carla udi_carla_global_planner.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "behavior_planner" -- bash -c "roslaunch udi_carla udi_carla_behavior_planner.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "local_planner" -- bash -c "roslaunch udi_carla udi_carla_local_planner.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "control_moudle" -- bash -c "roslaunch udi_carla udi_carla_udrive_control.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "test_rviz" -- bash -c "roslaunch udi_carla udi_carla_rviz_visualization.launch;exec bash"
}
echo "launch finished!"

