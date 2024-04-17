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
    spawn_points = world.get_map().get_spawn_points()  
    

  
    spawn_point = carla.Transform (carla.Location (-26.74,-50.40,0.5),carla.Rotation (0,0,0))
    bike = world.get_blueprint_library().find("vehicle.diamondback.century") #自行车
    vehicles_firetrunk = world.get_blueprint_library().find("vehicle.carlamotors.firetruck")#消防车
    vehicle_motorcycle = world.get_blueprint_library().find("vehicle.kawasaki.ninja") #摩托车 "
    vehicle_car = world.get_blueprint_library().find("vehicle.audi.tt")#乘用车 
    pedestrian = world.get_blueprint_library().find("walker.pedestrian.0012")#乘用车 
    blueprint = []
    blueprint.append(bike)
    blueprint.append(vehicles_firetrunk)
    blueprint.append(vehicle_motorcycle)
    blueprint.append(vehicle_car)
    # blueprint.append(pedestrian)
    destory_list = []
    vehicle = world.try_spawn_actor(blueprint[0],spawn_point)
    if vehicle is not None:
        destory_list.append(vehicle)
    control =carla.VehicleControl()
    control.throttle = 1
    control.steer = 0.3
    control.brake = 0

    location=carla.Location (-26.74,-50.40,0.5)
    delta_length = 100000
    cnt=1
    while True :
       
        if control.throttle ==0 :
            client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
            vehicle = world.try_spawn_actor(blueprint[cnt],spawn_point)
            if vehicle is not None:
                destory_list.append(vehicle)
            cnt+=1
            control.throttle = 1
            control.brake=0
        elif control.throttle == 1 and delta_length <0.1 :
            control.throttle = 0
            control.brake=1
        vehicle.apply_control(control)
        curr_localtion = vehicle.get_location()
        delta_length = math.hypot(location.x-curr_localtion.x,location.y-curr_localtion.y)
        world.wait_for_tick()

except KeyboardInterrupt:
    client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
    del client
    del world
    print("all clean up")
