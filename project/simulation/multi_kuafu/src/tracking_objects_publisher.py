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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from udi_msgs.msg import LocalizationEstimate
import math
import tf_conversions 
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
top_lidar_x = 1.4
top_lidar_y = 0
top_lidar_z = 1.52
def callback(msg):
    global ego_pose 
    global ego_orientation 
    global yaw
    ego_pose = msg.pose.position
    ego_orientation = msg.pose.orientation
    (r,p,yaw) = tf_conversions.transformations.euler_from_quaternion([ego_orientation.qx,ego_orientation.qy,ego_orientation.qz,ego_orientation.qw])
    
def ObjecetsVisualization(objects,objects_pub):
    markerarray =MarkerArray()
    objects = objects.objects
    for i, obj in enumerate(objects):
        arrow = Marker()
        arrow .header.frame_id = "kuafu_0/lidar_0"
        arrow.header.stamp = rospy.Time.now()
        arrow.id = i
        arrow.lifetime = rospy.Duration()
        arrow.action = Marker.ADD
        arrow.type = Marker.ARROW
        point_1  = Point()
        point_2 = Point()
        
        point_1.x = obj.x
        point_1.y = obj.y
        point_1.z = obj.z
        if (obj.velocity > 0.01):
            point_2.x = obj.x + obj.vel_x / obj.velocity
            point_2.y = obj.y + obj.vel_y / obj.velocity
            point_2.z = obj.z + obj.vel_z / obj.velocity
        else:
            point_2.x = obj.x
            point_2.y = obj.y
            point_2.z = obj.z
        arrow.points.append(point_1)
        arrow.points.append(point_2)
        arrow.scale.x = 0.3
        arrow.scale.y = 0.3
        arrow.color.r = 0
        arrow.color.g = 0
        arrow.color.b = 1
        arrow.color.a = 1  #透明度 alpha
        text =Marker()
        text .header.frame_id = "kuafu_0/lidar_0"
        text.header.stamp = rospy.Time.now()
        text.id = i + len(objects)
        text.lifetime = rospy.Duration(0.2)
        text.action = Marker.ADD
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.x = obj.x 
        text.pose.position.y = obj.y + 2
        text.pose.position.z = obj.z
        text.pose.orientation.w = 1.0
        text.scale.x = 3
        text.scale.y = 3
        text.scale.z = 1
        text.color.r = 0
        text.color.g = 0
        text.color.b = 1
        text.color.a = 1  #透明度 alpha
        text.text = str(round(obj.velocity, 2))

        box =Marker()
        box .header.frame_id = "kuafu_0/lidar_0"
        box.header.stamp = rospy.Time.now()
        box.id = i + len(objects)*2
        box.lifetime = rospy.Duration()
        box.action = Marker.ADD
        box.type = Marker.LINE_STRIP
        box.points.append(obj.polygon_point[2])  
        box.points.append(obj.polygon_point[3])  
        box.points.append(obj.polygon_point[0]) 
        box.points.append(obj.polygon_point[1]) 
        box.points.append(obj.polygon_point[2]) 
        box.scale.x = 0.1
        box.scale.y = 0.1
        box.scale.z = 0.1
        box.color.r = 1
        box.color.g = 0
        box.color.b = 0
        box.color.a = 1  #透明度 alpha
        # text.text = str((round(obj.x,2),round(obj.y,2),round(obj.z,2)))
        markerarray.markers.append(arrow)
        markerarray.markers.append(text)
        markerarray.markers.append(box)
    objects_pub.publish(markerarray)

def main():
    try:
        client = carla.Client('localhost',2000)
        world = client.get_world()

        rospy.init_node("Udi_create_dynamic_objects_for_tracking")
        objects_pub_ = rospy.Publisher("/udi_simulation/objects_real_vel",MarkerArray,queue_size=10)
                
        rate = rospy.Rate(20)
        ego_transform  = carla.Transform(carla.Location(0,0,0),carla.Rotation(0,0,0))
        while not rospy.is_shutdown() :
            snapshot = world.wait_for_tick()
            actors =  world.get_actors()
            for actor in actors:
                if (re.match("walker",actor.type_id) or re.match("vehicle",actor.type_id)):
                    size = actor.bounding_box.extent
                    print(" box size",size.x,size.y,size.z,actor.type_id)
            
            objectss = Objects()
            for actor in actors:
                obj = Object() 
                if re.match("vehicle.kuafumini",actor.type_id):
                    ego_transform = actor.get_transform()
                if (re.match("walker",actor.type_id) or re.match("vehicle",actor.type_id)) and not re.match("vehicle.kuafumini",actor.type_id):
                    
                    
                    
                    location = actor.bounding_box.location
                    print("location :",location.x,location.y,)
                    # print((location.x,location.y,location.z),(ego_transform.location.x,ego_transform.location.y,ego_transform.location.z))
                    bounding_box = actor.bounding_box.extent
                    # base_type= attributes.base_type()
                    # print(actor.type_id,bounding_box.x*2,bounding_box.y*2,bounding_box.z*2)
                    print("--------------0-----------------")
                    if re.match("vehicle",actor.type_id):
                        number_of_wheels=actor.attributes['number_of_wheels']
                        if float(number_of_wheels) == 2 and float(bounding_box.y) <0.01:
                            bounding_box.y = 0.435
                            print("modified width to:",bounding_box.y,actor.type_id)
                    currac =  snapshot.find(actor.id) 
                    obj.length = bounding_box.x
                    obj.width = bounding_box.y
                    obj.length = bounding_box.z
                    obj.object_id = actor.id
                    obj.object_name=actor .type_id
                    delta_x = actor.get_location().x - ego_transform.location.x 
                    delta_y = (ego_transform.location.y-actor.get_location().y) 
                    curr_yaw =-ego_transform.rotation.yaw/180 *math.pi
                    x_ = delta_x*math.cos(curr_yaw) + delta_y*math.sin(curr_yaw) 
                    y_ = (delta_y*math.cos(curr_yaw) - delta_x*math.sin(curr_yaw))
                    obj2vehicle_x =x_-top_lidar_x
                    obj2vehicle_y =y_ -top_lidar_y
                    obj2vehicle_z =actor.get_location().z  -ego_transform.location.z -top_lidar_z
                    obj.x = obj2vehicle_x
                    obj.y = obj2vehicle_y
                    obj.z = obj2vehicle_z
                    
                    vel_x = currac.get_velocity().x
                    vel_y = -currac.get_velocity().y
                    vel_z = currac.get_velocity().z
                    objvel2vehicle_x = vel_x*math.cos(curr_yaw) + vel_y*math.sin(curr_yaw) 
                    objvel2vehicle_y = (vel_y*math.cos(curr_yaw) - vel_x*math.sin(curr_yaw))
                    obj.vel_x =objvel2vehicle_x
                    obj.vel_y = objvel2vehicle_y
                    obj.vel_z = vel_z
                    val =math.sqrt(vel_x*vel_x+vel_y*vel_y+vel_z*vel_z)
                    obj.velocity = val

                    l = bounding_box.x
                    w = bounding_box.y
                    yaw  = -(actor.get_transform().rotation.yaw/180*math.pi+curr_yaw)
                    
                    
                    
                    print("location :",location.x,l,location.y,w)
                    dx1 = math.cos(yaw) * l
                    dy1 = math.sin(yaw) * l
                    dx2 = math.sin(yaw) * w
                    dy2 = -math.cos(yaw) * w
                    corners = []
                    delta_x = obj2vehicle_x
                    delta_y = obj2vehicle_y
                    corners.append(delta_x + dx1 + dx2 )
                    corners.append(delta_y + dy1 + dy2)
                    corners.append(delta_x + dx1 - dx2)
                    corners.append(delta_y + dy1 - dy2)
                    corners.append(delta_x - dx1 - dx2)
                    corners.append(delta_y - dy1 - dy2)
                    corners.append(delta_x - dx1 + dx2)
                    corners.append(delta_y - dy1 + dy2)
                    pose = Point()
                    pose.x = delta_x + dx1 + dx2 
                    pose.y = delta_y + dy1 + dy2
                    pose.z = 0
                    obj.polygon_point.append(pose)

                    pose = Point()
                    pose.x = delta_x + dx1 - dx2 
                    pose.y = delta_y + dy1 - dy2
                    pose.z = 0
                    obj.polygon_point.append(pose)

                    pose = Point()
                    pose.x = delta_x - dx1 - dx2 
                    pose.y = delta_y - dy1 - dy2
                    pose.z = 0
                    obj.polygon_point.append(pose)

                    pose = Point()
                    pose.x = delta_x - dx1 + dx2 
                    pose.y = delta_y - dy1 + dy2
                    pose.z = 0
                    obj.polygon_point.append(pose)

                    print("box size: ",bounding_box.x,bounding_box.y,bounding_box.z,actor.type_id)


                    # obj.polygon_point
                    objectss.objects.append(obj)
            print(objectss)
            ObjecetsVisualization(objectss,objects_pub_)
            rate.sleep()

    except KeyboardInterrupt:
        del client
        del world


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')