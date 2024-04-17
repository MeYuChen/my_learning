#!/usr/bin/env python
# -*- coding: utf-8 -*-
from curses import noecho
from distutils.command.build_scripts import first_line_re
import glob
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
try:
    client = carla.Client('localhost',2000)
    # client.load_world('test1')
    world = client.get_world()


    # traffic_manager = client.get_trafficmanager()
    # traffic_manager.set_synchronous_mode(True)

    # spectator = world.get_spectator()

    spawn_points = world.get_map().get_spawn_points()
    spawn_point_1 = carla.Transform (carla.Location (-40.74,-50.40,0.5),carla.Rotation (0,0,0))  
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
        
    
    pedestrian = world.get_blueprint_library().find("walker.pedestrian.0012")#people
    pedes = world.try_spawn_actor(pedestrian,spawn_points[0])
    blueprint = []
    destory_list = []
    destory_list.append(pedes)
    vehicle_cnt = 0
    player_control = carla.WalkerControl()
    player_control.speed = 3.0
    heading =math.atan(45.17/16.17)/3.1415926*180
    heading =0
    print(heading)
    rotation = carla.Rotation(0,heading,0)
    player_control.direction= rotation.get_forward_vector()
    localtion = carla.Location (-40.74,-50.40,0.5)
    while True :
        curr_localtion = pedes.get_location()
        delta_legth = math.hypot(curr_localtion.x-localtion.x,curr_localtion.y-localtion.y)
        if delta_legth >1.0:
            localtion = curr_localtion
            heading+=10
            rotation = carla.Rotation(0,heading,0)
            player_control.direction= rotation.get_forward_vector()
        pedes.apply_control(player_control)
        world.wait_for_tick()

except KeyboardInterrupt:
    client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
    del client
    del world
    print("all clean up")
