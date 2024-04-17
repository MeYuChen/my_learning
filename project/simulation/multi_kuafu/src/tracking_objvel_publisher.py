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
import rospy
import re
from udi_carla.msg import Object
from udi_carla.msg import Objects
import math
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
    world = client.get_world()

    rospy.init_node("Udi_create_dynamic_objects_for_tracking")
    objects_pub_ = rospy.Publisher("/udi_simulation/object_vel",Objects,queue_size=10)
    curr_kuafu_0_location= None
   
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() :
        snapshot = world.wait_for_tick()
        actors =  world.get_actors()
        objectss = Objects()

        
        for actor in actors:
            if re.match("vehicle.kuafumini",actor.type_id):
                curr_kuafu_0_location= actor.get_location()
            obj = Object()  
            if re.match("walker",actor.type_id) or re.match("vehicle",actor.type_id) and not re.match("vehicle.kuafumini",actor.type_id):
                currac =  snapshot.find(actor.id) 

                obj.object_id = actor.id
                obj.object_name=actor .type_id
                obj.x =actor.get_location().x
                obj.y =-actor.get_location().y
                obj.z =actor.get_location().z
                obj.vel_x= currac.get_velocity().x
                obj.vel_y = -currac.get_velocity().y
                obj.vel_z = currac.get_velocity().z
                val =math.sqrt(obj.vel_x*obj.vel_x+obj.vel_y*obj.vel_y+obj.vel_z*obj.vel_z)
                obj.velocity = val
                objectss.objects.append(obj)
        print(objectss)
        objects_pub_.publish(objectss)
        rate.sleep()

except KeyboardInterrupt:
    del client
    del world
