#!/usr/bin/env python
# -*- coding: utf-8 -*-
from curses import noecho
from distutils.command.build_scripts import first_line_re
import glob
from logging import exception
import os
from pickle import TRUE
import sys
from time import sleep
from warnings import catch_warnings
from geometry_msgs.msg import Twist
import rospy
from udi_carla.msg import Object
from udi_carla.msg import Objects
import math
# To import a basic agent
from agents.navigation.basic_agent import BasicAgent

# To import a behavior agent
from agents.navigation.behavior_agent import BehaviorAgent
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
try:
    client = carla.Client('localhost',2000)
    # client.load_world('test1')
    world = client.get_world()


    # traffic_manager = client.get_trafficmanager()
    # traffic_manager.set_synchronous_mode(True)

    # spectator = world.get_spectator()

    spawn_points = world.get_map().get_spawn_points()  
    vehicles = world.get_blueprint_library().filter('*vehicle*')
    blueprint = vehicles[0]
    # max_vehicles = min([max,len(spawn_points)])
    # agent = BasicAgent(vehicles[0])
    vehicles = []
    destory_list = []
    vehicle = world.try_spawn_actor(blueprint,spawn_points[0])
    agent = BasicAgent(vehicle)
    if vehicle is not None:
        vehicles.append(vehicle)
        destory_list.append(vehicle)

    # for vehicle in vehicles:
    #     vehicle.set_autopilot(True) # 车辆通过该方法注册到交通管理器
   
    while True :
        destination = random.choice(spawn_points).location
        agent.set_destination(destination)
        agent.set_target_speed(20)
        control = agent.run_step()
        vehicle.apply_control(control)
        world.wait_for_tick()

except KeyboardInterrupt:
    client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
    del client
    del world
    print("all clean up")
