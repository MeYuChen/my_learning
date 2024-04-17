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
import logging
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
vehicle_target_speed_km_h = 10.8 #km/h
pedestrin_speed_m_s = 2    # m/s
try:
    client = carla.Client('localhost',2000)
    # client.load_world('test1')
    world = client.get_world()
    vehicles_list = []
    walkers_list = []
    all_id = []
    client.set_timeout(10.0)
    synchronous_master = False
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    SetVehicleLightState = carla.command.SetVehicleLightState
    FutureActor = carla.command.FutureActor


    # traffic_manager = client.get_trafficmanager()
    # traffic_manager.set_synchronous_mode(True)

    # spectator = world.get_spectator()

    # spawn_points = world.get_map().get_spawn_points()
    # spawn_point_1 = carla.Transform (carla.Location (-26.74,-50.40,0.5),carla.Rotation (0,0,0))  
    # spawn_point_2 = carla.Transform (carla.Location (-27.74,-50.40,0.5),carla.Rotation (0,0,0)) 
    # destination = carla.Transform (carla.Location (16.17,45.17,0.5),carla.Rotation (0,0,0))
    # vehicle_models = ["vehicle.audi.a2","vehicle.audi.tt","vehicle.bmw.grandtourer","vehicle.micro.microlion","vehicle.carlamotors.carlacola",
    #         "vehicle.chevrolet.impala","vehicle.citroen.c3","vehicle.dodge.charger_police","vehicle.mercedes.coupe","vehicle.mini.cooper_s",
    #         "vehicle.nissan.micra","vehicle.nissan.patrol","vehicle.nissan.patrol_2021","vehicle.seat.leon","vehicle.toyota.prius",
    #         "vehicle.harley-davidson.low_rider","vehicle.yamaha.yzf","vehicle.kawasaki.ninja","vehicle.gazelle.omafiets","vehicle.audi.etron",
    #         "vehicle.tesla.cybertruck","vehicle.tesla.model3","vehicle.volkswagen.t2","vehicle.lincoln.mkz_2017","vehicle.ford.mustang",
    #         "vehicle.mercedes.coupe_2020","vehicle.lincoln.mkz_2020","vehicle.dodge.charger_2020","vehicle.dodge.charger_police_2020",
    #         "vehicle.ford.ambulance","vehicle.carlamotors.firetruck","vehicle.vespa.zx125","vehicle.volkswagen.t2_2021","vehicle.nissan.patrol_2021",
    #         "vehicle.mercedes.sprinter","vehicle.kuafu.kuafu","vehicle.kuafumini.kuafumini","vehicle.diamondback.century","vehicle.jeep.wrangler_rubicon",]
        
    bike = world.get_blueprint_library().find("vehicle.diamondback.century") #自行车
    vehicles_firetrunk = world.get_blueprint_library().find("vehicle.carlamotors.firetruck")#消防车
    vehicle_motorcycle = world.get_blueprint_library().find("vehicle.kawasaki.ninja") #摩托车 "
    vehicle_car = world.get_blueprint_library().find("vehicle.audi.tt")#乘用车 
    blueprintsWalkers = world.get_blueprint_library().filter("walker*")#乘用车 
    blueprints = []
    blueprints.append(bike)
    blueprints.append(vehicles_firetrunk)
    blueprints.append(vehicle_motorcycle)
    blueprints.append(vehicle_car)
    # # blueprint.append(pedestrian)
    # vehicle_spawn_points =[]
    # vehicle_spawn_points.append( carla.Transform (carla.Location (-9,-19,1),carla.Rotation (0,0,0)))
    # vehicle_spawn_points.append( carla.Transform (carla.Location (6.95,3.23,5),carla.Rotation (0,0,0)))
    # vehicle_spawn_points.append( carla.Transform (carla.Location (15.03,25.05,5),carla.Rotation (0,0,0)))
    # vehicle_spawn_points.append( carla.Transform (carla.Location (8.73,-13.67,5),carla.Rotation (0,0,0)))
    # vehicle_spawn_points.append( carla.Transform (carla.Location (-27.71,52.15,5),carla.Rotation (0,0,0)))
    x = -20
    y1 = 0
    y2 = 0
    spawn_points = []
    locations = []
    while x<3:
        if x != 0:
            y1 = math.sqrt(40/(x*x))
        y2 = -y1
        x+=1
        print(x,y1)
        print(x,y2)
        spawn_point = carla.Transform (carla.Location(x,y1,1),carla.Rotation (0,0,0))
        spawn_points.append(spawn_point) 
        spawn_point = carla.Transform (carla.Location(x,y2,1),carla.Rotation (0,0,0))
        spawn_points.append(spawn_point)
    destory_list = []

    # for n,blueprint in enumerate(blueprints):
    #     if blueprint.has_attribute('color'):
    #         color = random.choice(blueprint.get_attribute('color').recommended_values)
    #         blueprint.set_attribute('color', color)
    #     if blueprint.has_attribute('driver_id'):
    #         driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
    #         blueprint.set_attribute('driver_id', driver_id)
    #     blueprint.set_attribute('role_name', 'autopilot')
    #     temp = world.try_spawn_actor(blueprint,vehicle_spawn_points[n])
    #     if temp is not None:
    #         destory_list.append(temp)
    #         temp.set_autopilot()

        # prepare the light state of the cars to spawn

        # spawn the cars and set their autopilot and light state all together
        
# -------------
    # Spawn Walkers
    # -------------
    # some settings
    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
    
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, False)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    world.wait_for_tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
    while True:
        world.wait_for_tick()

except KeyboardInterrupt:
    client.apply_batch([carla.command.DestroyActor(x) for x in all_actors])
    del client
    del world
    print("all clean up")
