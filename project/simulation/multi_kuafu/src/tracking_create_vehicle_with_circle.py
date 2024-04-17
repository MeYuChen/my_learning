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
    # vehicles = world.get_blueprint_library().filter('*vehicle*')
    # vehicles = world.get_blueprint_library().filter('*vehicle*')
    vehicle_spawn_points =[]
    vehicle_spawn_points.append( carla.Transform (carla.Location (-9,-19,1),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (6.95,3.23,5),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (15.03,25.05,5),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (8.73,-13.67,5),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (-27.71,52.15,5),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (9,-19,1),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (12,6,5),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (20,-5,5),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (15,-13.67,5),carla.Rotation (0,0,0)))
    vehicle_spawn_points.append( carla.Transform (carla.Location (-27.-30,52.15,5),carla.Rotation (0,0,0)))
    vehicles =[]
    vehicles.append( world.get_blueprint_library().find("vehicle.diamondback.century")) #自行车
    vehicles.append( world.get_blueprint_library().find("vehicle.carlamotors.firetruck"))#消防车
    vehicles.append( world.get_blueprint_library().find("vehicle.kawasaki.ninja") )#摩托车 "
    vehicles.append( world.get_blueprint_library().find("vehicle.audi.tt"))#乘用车 
    vehicles.append( world.get_blueprint_library().find("vehicle.diamondback.century")) #自行车
    # vehicles.append( world.get_blueprint_library().find("vehicle.carlamotors.firetruck"))#消防车
    vehicles.append( world.get_blueprint_library().find("vehicle.kawasaki.ninja") )#摩托车 "
    vehicles.append( world.get_blueprint_library().find("vehicle.audi.tt"))#乘用车 
   
    # max_vehicles = min([max,len(spawn_points)])
    # agent = BasicAgent(vehicles[0])
    vehicles_s = []
    destory_list = []
    for n,blueprint in  enumerate( vehicles):
        vehicle = world.try_spawn_actor(blueprint,vehicle_spawn_points[n])
        if vehicle is not None:
            vehicles_s.append(vehicle)
            destory_list.append(vehicle)

    # for vehicle in vehicles:
    #     vehicle.set_autopilot(True) # 车辆通过该方法注册到交通管理器
    control =carla.VehicleControl()
    control.throttle = 0.49
    control.steer = 0.4
    control.brake = 0 
    control_1 =carla.VehicleControl()
    control_1.throttle = 0.3
    control_1.steer = 0.4
    control_1.brake = 0

    while True :
        for i,vehicle in enumerate( vehicles_s):
            if i>3:
                vehicle.apply_control(control)
            else:
                vehicle.apply_control(control_1)
        world.wait_for_tick()

except KeyboardInterrupt:
    client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
    del client
    del world
    print("all clean up")
