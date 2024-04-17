#!/usr/bin/env python
# -*- coding: utf-8 -*-
from curses import noecho
from distutils.command.build_scripts import first_line_re
import glob
import re
from logging import exception
import os
from pickle import TRUE
import sys
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
vehicle_target_speed_km_h = 40 #km/h
pedestrin_speed_m_s = 3    # m/s
try:
    client = carla.Client('localhost',2000)
    # client.load_world('test1')
    world = client.get_world()


    # traffic_manager = client.get_trafficmanager()
    # traffic_manager.set_synchronous_mode(True)

    # spectator = world.get_spectator()

    spawn_points = world.get_map().get_spawn_points()
    spawn_point_1 = carla.Transform (carla.Location (-26.74,-50.40,0.5),carla.Rotation (0,0,0))  
    spawn_point_2 = carla.Transform (carla.Location (-26.74,-50.40,0.5),carla.Rotation (0,0,0)) 
    destination = carla.Transform (carla.Location (16.17,45.17,0.5),carla.Rotation (0,0,0))
    vehicle_models = ["vehicle.audi.a2","vehicle.audi.tt","vehicle.bmw.grandtourer","vehicle.micro.microlion","vehicle.carlamotors.carlacola",
            "vehicle.chevrolet.impala","vehicle.citroen.c3","vehicle.dodge.charger_police","vehicle.mercedes.coupe","vehicle.mini.cooper_s",
            "vehicle.nissan.micra","vehicle.nissan.patrol","vehicle.nissan.patrol_2021","vehicle.seat.leon","vehicle.toyota.prius",
            "vehicle.harley-davidson.low_rider","vehicle.yamaha.yzf","vehicle.kawasaki.ninja","vehicle.gazelle.omafiets","vehicle.audi.etron",
            "vehicle.tesla.cybertruck","vehicle.tesla.model3","vehicle.volkswagen.t2","vehicle.lincoln.mkz_2017","vehicle.ford.mustang",
            "vehicle.mercedes.coupe_2020","vehicle.lincoln.mkz_2020","vehicle.dodge.charger_2020","vehicle.dodge.charger_police_2020",
            "vehicle.ford.ambulance","vehicle.carlamotors.firetruck","vehicle.vespa.zx125","vehicle.volkswagen.t2_2021","vehicle.nissan.patrol_2021",
            "vehicle.mercedes.sprinter","vehicle.kuafu.kuafu","vehicle.kuafumini.kuafumini","vehicle.diamondback.century","vehicle.jeep.wrangler_rubicon",]
        
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
    vehicle = world.try_spawn_actor(blueprint[0],spawn_point_1)
    if vehicle is not None:
        destory_list.append(vehicle)
    print(vehicle)
    player_control = carla.WalkerControl()
    vehicle_control =carla.VehicleControl()
    vehicle_control.throttle = 1
    vehicle_control.steer = 0.5
    vehicle_control.brake = 0

    localtion = vehicle.get_location()
    vehicle_cnt = 1
    curr_location= vehicle.get_location()
    delta_legth = math.hypot(localtion.x-curr_location.x,localtion.y-curr_location.y)
    while True:
        if vehicle_control.throttle ==1 and delta_legth<0.1:
            vehicle_control.steer = 0
            client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
            vehicle = world.try_spawn_actor(blueprint[vehicle_cnt],spawn_point_1)
            vehicle_cnt+=1
            curr_location= vehicle.get_location()
            delta_legth = math.hypot(localtion.x-curr_location.x,localtion.y-curr_location.y)
            if vehicle_control.steer ==0 and delta_legth<0.1:
                vehicle_control.steer = 0.3
            else:
                vehicle.apply_control(vehicle_control)
                if delta_legth <0.1:
        world.wait_for_tick()







except KeyboardInterrupt:
    client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
    del client
    del world
    print("all clean up")
