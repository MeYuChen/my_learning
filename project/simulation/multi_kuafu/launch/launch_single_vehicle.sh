#!/bin/bash
{
    gnome-terminal --tab "ros_bridge" -- bash -c "roslaunch udi_carla udi_carla_single_kuafumini_bridge.launch ;exec bash"
}&
sleep 2s
{
    gnome-terminal --tab "local_planner" -- bash -c "roslaunch udi_carla udi_carla_local_planner.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "global_control_etc" -- bash -c "roslaunch udi_carla udi_carla_global_control_etc.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "perception_process" -- bash -c "roslaunch udi_carla udi_carla_percep_preprocess.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "perception_tracking" -- bash -c "roslaunch udi_carla udi_carla_percep_tracking.launch ;exec bash"
}&
sleep 1s
{
    gnome-terminal --tab "test_rviz" -- bash -c "roslaunch udi_carla udi_carla_test_rviz.launch;exec bash"
}
echo "launch finished!"

