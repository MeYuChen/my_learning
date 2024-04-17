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
    # traffic_manager.set_synchronous_mode(False)

    # spectator = world.get_spectator()

    spawn_points = world.get_map().get_spawn_points()
    spawn_point_1 = carla.Transform (carla.Location (5.5,0,10),carla.Rotation (0,-120,0))
    spawn_point_2 = carla.Transform (carla.Location (27.5,40,10),carla.Rotation (0,-120,0))
    spawn_point_3 = carla.Transform (carla.Location (49.5,80,10),carla.Rotation (0,-120,0))
    spawn_point_4 = carla.Transform (carla.Location (71.5,120,2),carla.Rotation (0,-120,0))
    spawn_point_5 = carla.Transform (carla.Location (30.5,160,2),carla.Rotation (0,-120,0))

    spawn_point_1 = carla.Transform (carla.Location (-34.8,-72.1,5),carla.Rotation (0,-118,0))
    spawn_point_2 = carla.Transform (carla.Location (2.11,-5.52,5),carla.Rotation (0,-118,0))
    spawn_point_3 = carla.Transform (carla.Location (56.09,91.99,5),carla.Rotation (0,-118,0))
    spawn_point_4 = carla.Transform (carla.Location (-32.62,49.6,10),carla.Rotation (0,-118,0))
    spawn_point_5 = carla.Transform (carla.Location (-116.78,38.21,5),carla.Rotation (0,139.65,0))
    spawn_point_6 = carla.Transform (carla.Location (23.9,33.8,10),carla.Rotation (0,-120,0))
    # spawn_point_7 = carla.Transform (carla.Location (55.61,91.12,10),carla.Rotation (0,-110,0))
    spawn_point_8 = carla.Transform (carla.Location (77.93,170,10),carla.Rotation (0,-74,0))
    spawn_point_9 = carla.Transform (carla.Location (-90.21,119.91,2),carla.Rotation (0,20,0))
    spawn_point_10 = carla.Transform (carla.Location (-77.52,-2.16,2),carla.Rotation (0,110,0))
    models = ['dodge','audi','model3','mini','mustag','lincoln','prius','nissan','crown','impala']    
    blueprints = []
    for vehicle in world.get_blueprint_library().filter('*vehicle*'):
        if any(model in vehicle.id for model in models):
            blueprints.append(vehicle)
            print(vehicle.id)
    max_vehicles = 10
    # max_vehicles = min([max,len(spawn_points)])
    vehicles = []
    destory_list = []
    #silence_car1 = world.spawn_actor(random.choice(blueprints),spawn_point_1)
    #silence_car2 = world.spawn_actor(random.choice(blueprints),spawn_point_2)
    #silence_car3 = world.spawn_actor(random.choice(blueprints),spawn_point_3)
    #silence_car4 = world.spawn_actor(random.choice(blueprints),spawn_point_4)
    #silence_car5 = world.spawn_actor(random.choice(blueprints),spawn_point_5)
    #silence_car6 = world.spawn_actor(random.choice(blueprints),spawn_point_6)
    # silence_car7 = world.spawn_actor(random.choice(blueprints),spawn_point_7)
    #silence_car8 = world.spawn_actor(random.choice(blueprints),spawn_point_8)
    #silence_car9 = world.spawn_actor(random.choice(blueprints),spawn_point_9)
    #silence_car10 = world.spawn_actor(random.choice(blueprints),spawn_point_10)
    #destory_list.append(silence_car1)
    #destory_list.append(silence_car2)
    #destory_list.append(silence_car3)
    #destory_list.append(silence_car4)
    #destory_list.append(silence_car5)
    #destory_list.append(silence_car6)
    # destory_list.append(silence_car7)
    #destory_list.append(silence_car8)
    #destory_list.append(silence_car9)
    #destory_list.append(silence_car10)
    # enumerate(random.sample(spawn_points,max_vehicles)):
    for i,spawn_point in enumerate(spawn_points): 
        temp = world.try_spawn_actor(random.choice(blueprints),spawn_point)
        print(spawn_point)
        if temp is not None:
            vehicles.append(temp)
            destory_list.append(temp)

    for vehicle in vehicles:
        vehicle.set_autopilot(True) # 车辆通过该方法注册到交通管理器

    rospy.init_node("Udi_create_dynamic_objects_for_tracking")
    objects_pub_ = rospy.Publisher("/udi_simulation/object_vel",Objects,queue_size=10)
   
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() :
        snapshot = world.wait_for_tick()
        actors =  world.get_actors()
        objectss = Objects()
    #    for actor in actors:
    #     curractor= snapshot.find(actor.id)
        # obj = Object()
        # obj.object_id = actor.id
        # obj.object_name=actor.type_id
        # obj.x_val= curractor.get_velocity().x
        # obj.y_val = curractor.get_velocity().y
        # obj.z_val = curractor.get_velocity().z
    #     objectss.objects.append(obj)
        # print(obj)
        obj = Object()  
        currac =  snapshot.find(actors[0].id)  
        obj.object_id = actors[0] .id
        obj.object_name=actors[0] .type_id
        obj.x_val=      currac.get_velocity().x
        obj.y_val =     currac.get_velocity().y
        obj.z_val =     currac.get_velocity().z
        objectss.objects.append(obj)
        objects_pub_.publish(objectss)
        rate.sleep()

except KeyboardInterrupt:
    client.apply_batch([carla.command.DestroyActor(x) for x in destory_list])
    del client
    del world
    print("all clean up")
