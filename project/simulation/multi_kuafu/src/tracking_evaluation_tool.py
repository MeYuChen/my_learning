#!/usr/bin/env python
# -*- coding: utf-8 -*-
import glob
import os
from pickle import TRUE
import sys
from time import sleep
import rospy
import re
from udi_carla.msg import simObject
from udi_carla.msg import simObjects
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from udi_msgs.msg import PredictionObstacles
import math
import csv
import shapely
import numpy as np
from shapely.geometry import Polygon, mapping 
from datetime import datetime
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import matplotlib.pyplot as plt
import carla
from shapely.geometry import Polygon, MultiPoint, mapping 

boxs=[]
class obstacles:
    def __init__(self):
        rospy.Subscriber("prediction_obstacles",PredictionObstacles,self.callback,queue_size=1)
    def callback(self,Obstacles):
       global obss 
       obss = Obstacles.prediction_obstacle

max_error_between_sim_and_tracker =0.5#m
iou_threshold = 0.20
front_valid_size = 5#m
back_valid_size = 5#m
valid_distacnce = 31 #m
err_data = []
object_number = 0
walker_number = 0
bike_number = 0
vehicle_number = 0
curr_world = None
#获取仿真的真实际数据
# 行人，车 的位置，速度，box，速度朝向，actor id


# IOU计算
def bbox_iou_eval(box1, box2):
    box1 = np.array(box1).reshape(4, 2)
    poly1 = Polygon(box1).convex_hull #POLYGON ((0 0, 0 2, 2 2, 2 0, 0 0))
    # print(type(mapping(poly1)['coordinates'])) # (((0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)),)
    poly_arr = np.array(poly1)

    box2 = np.array(box2).reshape(4, 2)
    poly2 = Polygon(box2).convex_hull

    if not poly1.intersects(poly2):  # 如果两四边形不相交
        print("two box are not intersected!!")
        iou = 0
    else:
        try:
            inter_area = poly1.intersection(poly2).area  # 相交面积
            iou = float(inter_area) / (poly1.area + poly2.area - inter_area)
        except shapely.geos.TopologicalError:
            print('shapely.geos.TopologicalError occured, iou set to 0')
            iou = 0
    return iou

def plot_polygon(trackerbox,sim_box):
    a = []
    b = []
    c = []
    d = []
    for i, x in enumerate(trackerbox):
        if i%2==0:
            a.append(x)
        else:
            b.append(x)
    for i, x in enumerate(sim_box):
        if i%2==0:
            c.append(x)
        else:
            d.append(x)
    print("trackerbox",trackerbox)
    print("sim_box",sim_box)
    plt.fill(a,b, 'r')
    plt.fill(c,d, 'b')

    # plt.fill(c, 'g')
    # for index, item in enumerate(zip(a, b), 1):
    #     plt.text(item[0], item[1], index)

    # plt.subplot(222)
    # plt.plot(c, d, 'o')
    # plt.fill(c, d, alpha=0.5)
    # for index, item in enumerate(zip(c, d), 1):
    #     plt.text(item[0], item[1], index)

    # plt.subplot(223)
    # plt.fill(e, f)

    # plt.subplot(224)
    # plt.fill("time", "signal",
    #         data={"time": [2, 4, 4], "signal": [3, 4, 3]})

    plt.show()
    

def calculate_error(obss,simobjects,ego_transform):
    column = ['   ','   ','sim_type_id','sim_id','tracker_id','sim_position','tracker_position','position_error','sim_obj_static','tracker_obj_static','is_same_status','sim_velocity','tracker_velocity','velocity_error','sim_heading','tracker_heading','heading_error','sim_velocity_vector','tracker_velocity_vector','velocity_heading_error','sim_box_width','tracker_box_width','box_width_error','sim_box_length','tracker_box_length','box_length_error','sim_box_height','tracker_box_height','box_height_error','Miss_rate']
    temp_err = []
    for i in range(30):
        temp_err.append([])
    # 添加第一列
    for a in range(len(temp_err)):
        if a<len(column):
            temp_err[a].append(column[a])
    # print(temp_err)
    #添加数据
    matched_id_list = []
    matched = False
    miss_cnt = 0
    temp_box = []
    tmep_matched_number = 0
    for tracker_obj in obss:
        iou_list = [0,0,[],[],0,0,0,0]
        sim_box = []
        tracker_box = []
        for cnt, sim_obj in enumerate(simobjects.simObjects):
            # 找出交并比最大的obj
            sim_box = [sim_obj.polygon_point[0].x,sim_obj.polygon_point[0].y,sim_obj.polygon_point[1].x,sim_obj.polygon_point[1].y,sim_obj.polygon_point[2].x,sim_obj.polygon_point[2].y,
            sim_obj.polygon_point[3].x,sim_obj.polygon_point[3].y]
            position = tracker_obj.perception_object.position
            length =  tracker_obj.perception_object.length
            width  = tracker_obj.perception_object.width
            theta = tracker_obj.perception_object.theta
            tracker_box = calculate_bounding_box(width,length,theta,position)
            
            temp_iou = bbox_iou_eval(sim_box,tracker_box)
            # if temp_iou ==0:
            #     print("width: ", width," length: ", length," theta: ", theta, " pose: ",position)
            #     plot_polygon(tracker_box,sim_box)
                # print("width: ", width," length: ", length," theta: ", theta, " pose: ",position)
            
            if temp_iou>iou_list[0]:
                iou_list[0] = temp_iou
                iou_list[1] = cnt
                iou_list[2] = tracker_box
                iou_list[3] = sim_box
                iou_list[4] = sim_obj.object_id
                iou_list[5] = datetime.now()
                iou_list[6] = sim_obj.width
                iou_list[7] = sim_obj.length
        # plot_polygon(iou_list[2],iou_list[3])
        #求obj是否在车前方
        ego_location = ego_transform.location
        sim_obj = simobjects.simObjects[iou_list[1]]
        sim_location = sim_obj.world_position

        # dist_of_ego_to_obj = math.hypot(abs(ego_location.x -sim_location.x),abs(ego_location.y +sim_location.y))
        delta_x = sim_location.x - ego_transform.location.x 
        delta_y = (ego_transform.location.y+sim_location.y) 
        dist_of_ego_to_obj = math.hypot(delta_x,delta_y)
        curr_yaw = -ego_transform.rotation.yaw/180 *math.pi
        x_ = delta_x*math.cos(curr_yaw) + delta_y*math.sin(curr_yaw)     
        if x_>=0:
            stable_range = front_valid_size
            print("object in front of ego  ------ ")
        else:
            stable_range = back_valid_size
            print("object in back of ego-------")
        print("stable_range",stable_range,dist_of_ego_to_obj)
        print(" final IOU:%s "%round(iou_list[0],4))
        if iou_list[0]>iou_threshold and (sim_obj.object_id not in matched_id_list) and (dist_of_ego_to_obj <stable_range) :            
            if sim_obj.is_walker:
                global walker_number
                walker_number += 1
            if sim_obj.is_bike :
                global bike_number
                bike_number += 1
            if sim_obj.is_vehicle:
                global vehicle_number
                vehicle_number += 1

            tmep_matched_number +=1
            temp_box.append(iou_list)
            matched_id_list.append(sim_obj.object_id)
            errordata =ErrorAnalysis(sim_obj,tracker_obj.perception_object)
            errordata.calculate_position_error()               
            errordata.calculate_velocity_error()
            errordata.calculate_velocity_heading_error()
            errordata.compare_status_of_two_obj()
            errordata.calculate_box_width_errror()
            errordata.calculate_box_length_error()
            errordata.calculate_heading_error()
            # errordata.calculate_box_error()
            
            i=2
            temp_err[i].append(errordata.sim_type_id)
            i+=1
            temp_err[i].append(errordata.sim_id)
            i+=1
            temp_err[i].append(errordata.tracker_obj_id)
            i+=1
            x = str(errordata.sim_position.x)
            y = str(errordata.sim_position.y)
            z = str(errordata.sim_position.z)
            temp_err[i].append("("+x+"_"+y+"_"+z+")")
            i+=1
            x = str(errordata.tracker_position.x)
            y = str(errordata.tracker_position.y)
            z = str(errordata.tracker_position.z)
            temp_err[i].append("("+x+"_"+y+"_"+z+")")
            i+=1
            temp_err[i].append(errordata.get_error_of_position())
            i+=1
            temp_err[i].append(errordata.is_simobj_static)
            i+=1
            temp_err[i].append(errordata.is_trackerobj_static)
            i+=1
            temp_err[i].append(errordata.is_static_matched())
            i+=1
            temp_err[i].append(errordata.sim_velocity)
            i+=1
            temp_err[i].append(errordata.tracker_velocity)
            i+=1
            temp_err[i].append(errordata.get_error_of_velocity())
            i+=1
            temp_err[i].append(errordata.sim_heading)
            i+=1
            temp_err[i].append(errordata.tracker_heading)
            i+=1
            temp_err[i].append(errordata.get_error_of_heading())
            i+=1
            x = str(errordata.sim_velocity_vector.x)
            y = str(errordata.sim_velocity_vector.y)
            z = str(errordata.sim_velocity_vector.z)
            temp_err[i].append("("+x+"_"+y+"_"+z+")")
            i+=1
            x = str(errordata.tracker_velocity_vector.x)
            y = str(errordata.tracker_velocity_vector.y)
            z = str(errordata.tracker_velocity_vector.z)
            temp_err[i].append("("+x+"_"+y+"_"+z+")")
            i+=1
            temp_err[i].append(errordata.get_error_of_velocity_heading())
            i+=1
            temp_err[i].append(errordata.sim_width)
            i+=1
            temp_err[i].append(errordata.tracker_width)
            i+=1
            temp_err[i].append(errordata.get_box_width_error())
            i+=1
            temp_err[i].append(errordata.sim_length)
            i+=1
            temp_err[i].append(errordata.tracker_length)
            i+=1
            temp_err[i].append(errordata.get_box_length_error())
            i+=1
            temp_err[i].append(errordata.sim_height)
            i+=1
            temp_err[i].append(errordata.tracker_height)
            i+=1
            temp_err[i].append(errordata.get_box_height_error())
            # print(temp_err)
            print(errordata.sim_type_id," be matched ,match IOU:%s "%round(iou_list[0],4))
        elif (dist_of_ego_to_obj <stable_range):
            #一个都没匹配上
            miss_cnt+= 1
    
    boxs.append(temp_box)
    missrate = miss_cnt / len(obss)
    print("oooo ",miss_cnt,len(obss))
    temp_err[len(column)-1].append(missrate)
    print(temp_err)
    print("current frame match finished!!!")
    global object_number 
    object_number += tmep_matched_number
    global err_data 
    err_data.append(temp_err)
    return err_data  

def export_data2csv(data_buffer,world):
    temp_err = []
    column = ['   ','','status_matching_success_rate','average_velocity_error','average_heading_error_5','average_heading_error_10','average_velocity_heading_error','average_box_width_error','average_box_length_error','average_miss_rate']
    for i in range(len(column)):
        temp_err.append([])
    # 添加第一列
    for a in range(len(column)):
            temp_err[a].append(column[a])
#      获得有车辆类型id
    header = []
    for buffer in data_buffer:
        for bf in buffer[3]:
            if bf not in header:
                header.append(bf)
    print("header-----------",header)
    temp_err[1] = header

    temp_variance = []    
    column_variance = ['   ','','status_matching_success_rate_variance',
    'velocity_variance','heading_error_5_variance','heading_error_10_variance',
    'velocity_heading_error_variance','box_width_error_variance','box_length_error_variance',
    'missrate_variance']
    for i in range(len(column_variance)):
        temp_variance.append([])
    # 添加第一列
    for a in range(len(column_variance)):
            temp_variance[a].append(column_variance[a])
#      获得有车辆类型id
    temp_variance[1] = header

    temp_acceptance_rate = []
    column_acceptance_rate = ['   ',
    '',
    'walker_status_matching_success_acceptance_rate',
    'walker_position_acceptance_rate','walker_velocity_acceptance_rate',
    'walker_heading_acceptance_rate_5','walker_heading_acceptance_rate_10',
    'walker_velocity_heading_acceptance_rate','walker_box_width_acceptance_rate',
    'walker_box_length_acceptance_rate','','bike_status_matching_success_acceptance_rate',
    'bike_position_acceptance_rate','bike_velocity_acceptance_rate','bike_heading_acceptance_rate_5',
    'bike_heading_acceptance_rate_10','bike_velocity_heading_acceptance_rate','bike_box_width_acceptance_rate',
    'bike_box_length_acceptance_rate','','vehicle_status_matching_success_acceptance_rate',
    'vehicle_position_acceptance_rate','vehicle_velocity_acceptance_rate','vehicle_heading_acceptance_rate_5',
    'vehicle_heading_acceptance_rate_10','vehicle_velocity_heading_acceptance_rate','vehicle_box_width_acceptance_rate',
    'vehicle_box_length_acceptance_rate']
    for i in range(len(column_acceptance_rate)):
        temp_acceptance_rate.append([])
    # 添加第一列
    for a in range(len(column_acceptance_rate)):
            temp_acceptance_rate[a].append(column_acceptance_rate[a])
#      获得有车辆类型id
    head = ["frame_cnt "]
    for i,data in enumerate(data_buffer):
        head.append(i+1)
    temp_acceptance_rate[1] = head

       
    column_total_error =  ['','',
    'average_status_matching_err',
    'average_position_error',
    'average_vel_err',
    'average_heading_err_5',
    'average_heading_err_10',
    'average_vel_heading_err',
    'average_box_width_err',
    'average_box_length_err',
    '',
    'average_walker_status_matching_err',
    'average_walker_position_error',
    'average_walker_vel_err',
    'average_walker_heading_err_5',
    'average_walker_heading_err_10',
    'average_walker_vel_heading_err',
    'average_walker_box_width_err',
    'average_walker_box_length_err',
    '',
    'average_bike_status_matching_err',
    'average_bike_position_error',
    'average_bike_vel_err',
    'average_bike_heading_err_5',
    'average_bike_heading_err_10',
    'average_bike_vel_heading_err',
    'average_bike_box_width_err',
    'average_bike_box_length_err',
    '',
    'average_vehicle_status_matching_err',
    'average_vehicle_position_error',
    'average_vehicle_vel_err',
    'average_vehicle_heading_err_5',
    'average_vehicle_heading_err_10',
    'average_vehicle_vel_heading_err',
    'average_vehicle_box_width_err',
    'average_vehicle_box_length_err',
    '','',
    'object_number',
    'walker_number',
    'bike_number',
    'vehicle_number',
    '','',
    'sum_status_matching_err',
    'sum_position_error',
    'sum_vel_error',
    'sum_heading_error_5',
    'sum_heading_error_10',
    'sum_vel_heading_error',
    'sum_box_width_error',
    'sum_box_length_error',
    '',
    'sum_walker_status_matching_err',
    'sum_walker_position_error',
    'sum_walker_vel_error',
    'sum_walker_heading_error_5',
    'sum_walker_heading_error_10',
    'sum_walker_vel_heading_error',
    'sum_walker_box_width_error',
    'sum_walker_box_length_error',
    '',
    'sum_bike_status_matching_err',
    'sum_bike_position_error',
    'sum_bike_vel_error',
    'sum_bike_heading_error_5',
    'sum_bike_heading_error_10',
    'sum_bike_vel_heading_error',
    'sum_bike_box_width_error',
    'sum_bike_box_length_error',
    '',
    'sum_vehicle_status_matching_err',
    'sum_vehicle_position_error',
    'sum_vehicle_vel_error',
    'sum_vehicle_heading_error_5',
    'sum_vehicle_heading_error_10',
    'sum_vehicl_vel_heading_error',
    'sum_vehicle_box_width_error',
    'sum_vehicle_box_length_error',]
    temp_total_error = [] 
    for i in range(len(column_total_error)):
        temp_total_error.append([])
    # 添加第一列
    for a in range(len(column_total_error)):
            temp_total_error[a].append(column_total_error[a])
#      获得有车辆类型id
    temp_total_error[1] = ['error_type','total_avrage_error']


    #构造每帧id的字典
    sim_id=header[1:]
    print("sim_id: ",sim_id)
    dict_range = max(sim_id)
    dict_list=[[[]for i in range(10)] for i in range(dict_range+1)]
    #获得 id的在每帧数据中的索引
    for buffer in data_buffer:
        type_id = buffer[2]
        matching_rate = buffer[10]
        position_err = buffer[7]
        vel_err = buffer[13]
        heading_err_5 = buffer[16]
        heading_err_10 = buffer[16]
        vel_heading_err =buffer[19]
        box_width_error =buffer[22]
        box_length_err = buffer[25]
        miss_rate = buffer[29]
        for i in sim_id:
            if i in buffer[3]:
                err_index = buffer[3].index(i)
                dict_list[i][0].append(matching_rate[err_index])
                dict_list[i][1].append(vel_err[err_index])
                dict_list[i][2].append(heading_err_5[err_index])
                dict_list[i][3].append(heading_err_10[err_index])
                dict_list[i][4].append(vel_heading_err[err_index])
                dict_list[i][5].append(box_width_error[err_index])
                dict_list[i][6].append(box_length_err[err_index])
                dict_list[i][7].append(miss_rate[1])
                dict_list[i][8].append(position_err[err_index])
                dict_list[i][9].append(type_id[err_index])
        # print(dict_list)

    #针对索引计算各种误差
    # 1 构造误差列表 
    aver_err_dict = [[[]for i in range(7)]for i in range(dict_range+1)]


    average_status_matching_err = 0
    average_position_error = 0
    average_vel_err = 0
    average_heading_err_5 = 0
    average_heading_err_10 = 0
    average_vel_heading_err = 0
    average_box_width_err = 0
    average_box_length_err = 0

    average_walker_status_matching_err = 0
    average_walker_position_error = 0
    average_walker_vel_err = 0
    average_walker_heading_err_5 = 0
    average_walker_heading_err_10 = 0
    average_walker_vel_heading_err = 0
    average_walker_box_width_err = 0
    average_walker_box_length_err = 0

    average_bike_status_matching_err = 0
    average_bike_position_error = 0
    average_bike_vel_err = 0
    average_bike_heading_err_5 = 0
    average_bike_heading_err_10 = 0
    average_bike_vel_heading_err = 0
    average_bike_box_width_err = 0
    average_bike_box_length_err = 0
     
    average_vehicle_status_matching_err = 0
    average_vehicle_position_error = 0
    average_vehicle_vel_err = 0
    average_vehicle_heading_err_5 = 0
    average_vehicle_heading_err_10 = 0
    average_vehicle_vel_heading_err = 0
    average_vehicle_box_width_err = 0
    average_vehicle_box_length_err = 0

    sum_status_matching_err = 0
    sum_position_error = 0
    sum_vel_error = 0
    sum_heading_error_5 = 0
    sum_heading_error_10 = 0
    sum_vel_heading_err = 0
    sum_box_width_err = 0
    sum_box_length_err = 0
     
    sum_walker_status_matching_err = 0
    sum_walker_position_error = 0
    sum_walker_vel_error = 0
    sum_walker_heading_error_5 = 0
    sum_walker_heading_error_10 = 0
    sum_walker_vel_heading_err = 0
    sum_walker_box_width_err = 0
    sum_walker_box_length_err = 0
    
    sum_bike_status_matching_err = 0
    sum_bike_position_error = 0
    sum_bike_vel_error = 0
    sum_bike_heading_error_5 = 0
    sum_bike_heading_error_10 = 0
    sum_bike_vel_heading_err = 0
    sum_bike_box_width_err = 0
    sum_bike_box_length_err = 0
    
    sum_vehicle_status_matching_err = 0
    sum_vehicle_position_error = 0
    sum_vehicle_vel_error = 0
    sum_vehicle_heading_error_5 = 0
    sum_vehicle_heading_error_10 = 0
    sum_vehicle_vel_heading_err = 0
    sum_vehicle_box_width_err = 0
    sum_vehicle_box_length_err = 0

    actors =  world.get_actors()

    for index,i in enumerate(sim_id):
        print("-------------------------id:", i)
        success_cnt = 0
        print("is same status ", dict_list[i][0])
        for rate in dict_list[i][0]:
            if rate:
                success_cnt+=1

        print("match rate: ",success_cnt, len(dict_list[i][0]),success_cnt/len(dict_list[i][0]))
        aver_err_dict[i][0] = round(success_cnt/len(dict_list[i][0]),4) #匹配成功率       
        average_status_matching_err += round(success_cnt/object_number,4)
        sum_status_matching_err+=success_cnt


        vel_error = dict_list[i][1]
        aver_vel_err = 0
        for vel_err in vel_error:
            aver_vel_err += vel_err/len(vel_error)   #速度误差
            average_vel_err += vel_err/object_number
            sum_vel_error += vel_err
        aver_err_dict[i][1] = round(aver_vel_err,4)

        heading_err_5 = dict_list[i][2]
        aver_heading_err_5 = 0
        for val in heading_err_5:
            aver_heading_err_5 += val/len(heading_err_5)
            average_heading_err_5 += val/object_number
            sum_heading_error_5 += val
            
        aver_err_dict[i][2] =round( aver_heading_err_5,4)

        heading_err_10 = dict_list[i][3]
        aver_heading_err_10 = 0
        for val in heading_err_10:
            aver_heading_err_10 += val/len(heading_err_10)
            average_heading_err_10 += val/object_number
            sum_heading_error_10 += val
        aver_err_dict[i][3] =round( aver_heading_err_10,4)


        speed_heading_err = dict_list[i][4]
        speed_heading_val = 0
        for val in speed_heading_err:
            speed_heading_val+=val/len(speed_heading_err)
            average_vel_heading_err += val/object_number
            sum_vel_heading_err += val 
        aver_err_dict[i][4]=round(speed_heading_val,4)

        box_width_err = dict_list[i][5]
        width_err = 0
        for val in box_width_err:
            width_err+=val/len(box_width_err)
            average_box_width_err += val / object_number
            sum_box_width_err += val
        aver_err_dict[i][5] = round(width_err,4)

        box_len_err =dict_list[i][6]
        len_err = 0
        for val in box_len_err:
            len_err+=val/len(box_len_err)
            average_box_length_err += val / object_number
            sum_box_length_err += val 
        aver_err_dict[i][6] = round(len_err,4)
        missrate = dict_list[i][6]

        for position_error in dict_list[i][8]:
            print(position_err)
            average_position_error += position_error/object_number
            sum_position_error += position_error

        temp_err[2].append(aver_err_dict[i][0])
        temp_err[3].append(aver_err_dict[i][1])
        temp_err[4].append(aver_err_dict[i][2])
        temp_err[5].append(aver_err_dict[i][3])
        temp_err[6].append(aver_err_dict[i][4])
        temp_err[7].append(aver_err_dict[i][5])
        temp_err[8].append(aver_err_dict[i][6])

        #分别求各类偏差，平均偏差，总偏差

        type_ids = dict_list[i][9]
        for index, rate in enumerate( dict_list[i][0]):
            type_id = type_ids[index]
            if re.match("walker",type_id):
                if rate:
                    sum_walker_status_matching_err += 1
            else:
                is_bike = False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike:
                    sum_bike_status_matching_err += 1
                else:
                    sum_vehicle_status_matching_err += 1

        vel_error = dict_list[i][1]
        for index, vel_err in enumerate( vel_error):
            if re.match("walker",type_id):
                if rate:
                    sum_walker_vel_error += vel_err
            else:
                is_bike =False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike:
                    sum_bike_vel_error += vel_err
                else:
                    sum_vehicle_vel_error += vel_err

        heading_err_5 = dict_list[i][2]
        for index, val in enumerate( heading_err_5):
            type_id = type_ids[index]
            if re.match("walker",type_id):
                sum_walker_heading_error_5 += val
            else:
                is_bike = False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike:
                    sum_bike_heading_error_5 += val
                else:
                    sum_vehicle_heading_error_5 += val

            
        heading_err_10 = dict_list[i][3]
        for index,val in enumerate( heading_err_10):
            type_id = type_ids[index]
            if re.match("walker",type_id):
                sum_walker_heading_error_10 += val
            else:
                is_bike = False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike:
                    sum_bike_heading_error_10 += val
                else:
                    sum_vehicle_heading_error_10 += val

        speed_heading_err = dict_list[i][4]
        for index, val in enumerate(speed_heading_err):
            type_id = type_ids[index]
            if re.match("walker",type_id):
                sum_walker_vel_heading_err += val
            else:
                is_bike = False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike:
                    sum_bike_vel_heading_err += val
                else:
                    sum_vehicle_vel_heading_err += val

        box_width_err = dict_list[i][5]
        for index, val in enumerate( box_width_err):
            type_id = type_ids[index]
            if re.match("walker",type_id):
                sum_walker_box_width_err += val
            else:
                is_bike = False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike :
                    sum_bike_box_width_err += val 
                else:
                    sum_vehicle_box_width_err += val

        box_len_err =dict_list[i][6]
        for index, val in enumerate(box_len_err):
            type_id = type_ids[index]
            if re.match("walker",type_id):
                sum_walker_box_length_err += val 
            else:
                is_bike = False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike :
                    sum_bike_box_length_err += val 
                else:
                    sum_vehicle_box_length_err += val
                
        for index, position_error in enumerate( dict_list[i][8]):
            type_id = type_ids[index]
            if re.match("walker",type_id):
                sum_walker_position_error += position_error
            else:
                is_bike = False
                for temp_actor in actors:
                    if re.match(type_id,temp_actor.type_id):
                        is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                        print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                        break
                if is_bike:
                    sum_bike_position_error += position_error
                else:
                    sum_vehicle_position_error += position_error
    if walker_number :
        average_walker_status_matching_err = sum_walker_status_matching_err / walker_number
        average_walker_position_error = sum_walker_position_error / walker_number
        average_walker_vel_err = sum_walker_vel_error / walker_number
        average_walker_heading_err_5 = sum_walker_heading_error_5 / walker_number
        average_walker_heading_err_10 = sum_walker_heading_error_10 / walker_number
        average_walker_vel_heading_err = sum_walker_vel_heading_err / walker_number
        average_walker_box_width_err = sum_walker_box_width_err / walker_number
        average_walker_box_length_err = sum_walker_box_length_err / walker_number
    else :
        average_walker_status_matching_err = 0
        average_walker_position_error = 0
        average_walker_vel_err = 0
        average_walker_heading_err_5 = 0
        average_walker_heading_err_10 = 0
        average_walker_vel_heading_err = 0
        average_walker_box_width_err = 0
        average_walker_box_length_err = 0
        
    if bike_number:
        average_bike_status_matching_err = sum_bike_status_matching_err / bike_number
        average_bike_position_error = sum_bike_position_error / bike_number
        average_bike_vel_err = sum_bike_vel_error / bike_number
        average_bike_heading_err_5 = sum_bike_heading_error_5 / bike_number
        average_bike_heading_err_10 = sum_bike_heading_error_10 / bike_number
        average_bike_vel_heading_err = sum_bike_vel_heading_err / bike_number
        average_bike_box_width_err = sum_bike_box_width_err / bike_number
        average_bike_box_length_err = sum_bike_box_length_err / bike_number
    else :
        average_bike_status_matching_err = 0
        average_bike_position_error = 0
        average_bike_vel_err = 0
        average_bike_heading_err_5 = 0
        average_bike_heading_err_10 = 0
        average_bike_vel_heading_err = 0
        average_bike_box_width_err = 0
        average_bike_box_length_err = 0

    if vehicle_number :
        average_vehicle_status_matching_err = sum_vehicle_status_matching_err / vehicle_number
        average_vehicle_position_error = sum_vehicle_position_error / vehicle_number
        average_vehicle_vel_err = sum_vehicle_vel_error / vehicle_number
        average_vehicle_heading_err_5 = sum_vehicle_heading_error_5 / vehicle_number
        average_vehicle_heading_err_10 = sum_vehicle_heading_error_10 / vehicle_number
        average_vehicle_vel_heading_err = sum_vehicle_vel_heading_err / vehicle_number
        average_vehicle_box_width_err = sum_vehicle_box_width_err / vehicle_number
        average_vehicle_box_length_err = sum_vehicle_box_length_err / vehicle_number
    else:
        average_vehicle_status_matching_err = 0
        average_vehicle_position_error = 0
        average_vehicle_vel_err = 0
        average_vehicle_heading_err_5 = 0
        average_vehicle_heading_err_10 = 0
        average_vehicle_vel_heading_err = 0
        average_vehicle_box_width_err = 0
        average_vehicle_box_length_err = 0

    average_miss_rate = 0
    for val in missrate:
        average_miss_rate+=val/len(missrate)
        print("missrate:",val)
    temp_err[9].append(average_miss_rate)
    
    """
    求n帧的平均误差
    """
    i = 2
    temp_total_error[i].append(average_status_matching_err)
    i += 1
    temp_total_error[i].append(average_position_error)
    i += 1
    temp_total_error[i].append(average_vel_err)
    i += 1
    temp_total_error[i].append(average_heading_err_5)
    i += 1
    temp_total_error[i].append(average_heading_err_10)
    i += 1
    temp_total_error[i].append(average_vel_heading_err)
    i += 1
    temp_total_error[i].append(average_box_width_err)
    i += 1
    temp_total_error[i].append(average_box_length_err)
    i += 2

    temp_total_error[i].append(average_walker_status_matching_err)
    i += 1
    temp_total_error[i].append(average_walker_position_error)
    i += 1
    temp_total_error[i].append(average_walker_vel_err)
    i += 1
    temp_total_error[i].append(average_walker_heading_err_5)
    i += 1
    temp_total_error[i].append(average_walker_heading_err_10)
    i += 1
    temp_total_error[i].append(average_walker_vel_heading_err)
    i += 1
    temp_total_error[i].append(average_walker_box_width_err)
    i += 1
    temp_total_error[i].append(average_walker_box_length_err)
    i += 2

    temp_total_error[i].append(average_bike_status_matching_err)
    i += 1
    temp_total_error[i].append(average_bike_position_error)
    i += 1
    temp_total_error[i].append(average_bike_vel_err)
    i += 1
    temp_total_error[i].append(average_bike_heading_err_5)
    i += 1
    temp_total_error[i].append(average_bike_heading_err_10)
    i += 1
    temp_total_error[i].append(average_bike_vel_heading_err)
    i += 1
    temp_total_error[i].append(average_bike_box_width_err)
    i += 1
    temp_total_error[i].append(average_bike_box_length_err)
    i += 2
    temp_total_error[i].append(average_vehicle_status_matching_err)
    i += 1
    temp_total_error[i].append(average_vehicle_position_error)
    i += 1
    temp_total_error[i].append(average_vehicle_vel_err)
    i += 1
    temp_total_error[i].append(average_vehicle_heading_err_5)
    i += 1
    temp_total_error[i].append(average_vehicle_heading_err_10)
    i += 1
    temp_total_error[i].append(average_vehicle_vel_heading_err)
    i += 1
    temp_total_error[i].append(average_vehicle_box_width_err)
    i += 1
    temp_total_error[i].append(average_vehicle_box_length_err)
    i += 3
    """
    object number
    """
    temp_total_error[i].append(object_number)
    i += 1
    temp_total_error[i].append(walker_number)
    i += 1
    temp_total_error[i].append(bike_number)
    i += 1
    temp_total_error[i].append(vehicle_number)
    i += 3
    
    """
    sum error
    """
    temp_total_error[i].append(average_status_matching_err)
    i += 1
    temp_total_error[i].append(sum_position_error)
    i += 1
    temp_total_error[i].append(sum_vel_error)
    i += 1
    temp_total_error[i].append(sum_heading_error_5)
    i += 1
    temp_total_error[i].append(sum_heading_error_10)
    i += 1
    temp_total_error[i].append(sum_vel_heading_err)
    i += 1
    temp_total_error[i].append(sum_box_width_err)
    i += 1
    temp_total_error[i].append(sum_box_length_err)
    i += 2

    temp_total_error[i].append(sum_walker_status_matching_err)
    i += 1
    temp_total_error[i].append(sum_walker_position_error)
    i += 1
    temp_total_error[i].append(sum_walker_vel_error)
    i += 1
    temp_total_error[i].append(sum_walker_heading_error_5)
    i += 1
    temp_total_error[i].append(sum_walker_heading_error_10)
    i += 1
    temp_total_error[i].append(sum_walker_vel_heading_err)
    i += 1
    temp_total_error[i].append(sum_walker_box_width_err)
    i += 1
    temp_total_error[i].append(sum_walker_box_length_err)
    i += 2

    temp_total_error[i].append(sum_bike_status_matching_err)
    i += 1
    temp_total_error[i].append(sum_bike_position_error)
    i += 1
    temp_total_error[i].append(sum_bike_vel_error)
    i += 1
    temp_total_error[i].append(sum_bike_heading_error_5)
    i += 1
    temp_total_error[i].append(sum_bike_heading_error_10)
    i += 1
    temp_total_error[i].append(sum_bike_vel_heading_err)
    i += 1
    temp_total_error[i].append(sum_bike_box_width_err)
    i += 1
    temp_total_error[i].append(sum_bike_box_length_err)
    i += 2
    temp_total_error[i].append(sum_vehicle_status_matching_err)
    i += 1
    temp_total_error[i].append(sum_vehicle_position_error)
    i += 1
    temp_total_error[i].append(sum_vehicle_vel_error)
    i += 1
    temp_total_error[i].append(sum_vehicle_heading_error_5)
    i += 1
    temp_total_error[i].append(sum_vehicle_heading_error_10)
    i += 1
    temp_total_error[i].append(sum_vehicle_vel_heading_err)
    i += 1
    temp_total_error[i].append(sum_vehicle_box_width_err)
    i += 1
    temp_total_error[i].append(sum_vehicle_box_length_err)



    """
    求方差
    """
    for i in sim_id:
        for rate in dict_list[i][0]:
            if rate:
                success_cnt+=1
        temp_variance[2].append(round(success_cnt/len(dict_list[i][0]),4)) #这里用平均值代替方差
        #速度平方差
        variance = CalculateVariance(aver_err_dict[i][1],dict_list[i][1])
        temp_variance[3].append(variance)
        variance = CalculateVariance(aver_err_dict[i][2],dict_list[i][2])
        temp_variance[4].append(variance)
        variance = CalculateVariance(aver_err_dict[i][3],dict_list[i][3])
        temp_variance[5].append(variance)
        variance = CalculateVariance(aver_err_dict[i][4],dict_list[i][4])
        temp_variance[6].append(variance)
        variance = CalculateVariance(aver_err_dict[i][5],dict_list[i][5])
        temp_variance[7].append(variance)
        variance = CalculateVariance(aver_err_dict[i][5],dict_list[i][5])
        temp_variance[8].append(variance)
    # calculate missrate:
    missrate_variance = CalculateVariance(average_miss_rate,missrate)
    temp_variance[9].append(missrate_variance)
    """
    求合格率
    """
    for cnt, dataframe in enumerate( data_buffer):
        data_acceptance_rate = CalculateAcceptanceRate( dataframe,world,cnt)
        data_acceptance_rate.ExtractDataAndCalculate()

        i=2
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_status_matching_success_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_position_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_velocity_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_heading_acceptance_rate_5())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_heading_acceptance_rate_10())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_velocity_heading_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_box_width_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_walker_box_length_acceptance_rate())

        i+=2
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_status_matching_success_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_position_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_velocity_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_heading_acceptance_rate_5())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_heading_acceptance_rate_10())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_velocity_heading_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_box_width_acceptance_rate())
        if data_acceptance_rate.get_bike_box_width_acceptance_rate()>0.2:
            box = boxs[cnt]

        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_bike_box_length_acceptance_rate())

        i+=2
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_status_matching_success_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_position_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_velocity_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_heading_acceptance_rate_5())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_heading_acceptance_rate_10())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_velocity_heading_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_box_width_acceptance_rate())
        i+=1
        temp_acceptance_rate[i].append(data_acceptance_rate.get_vehicle_box_length_acceptance_rate())

        
    data_buffer.append(temp_err)

    data_buffer.append(temp_variance)

    data_buffer.append(temp_acceptance_rate)

    data_buffer.append(temp_total_error)

    # 添加各类数据方差

    with open('score.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        for buffer in data_buffer:
            writer.writerows(buffer)
    return True

class CalculateAcceptanceRate:
    def __init__(self,data_frame,world,frame_cnt) :
        self.walker_status_matching_success_acceptance_rate = 0
        self.walker_position_acctptance_rate = 0
        self.walker_velocity_acceptance_rate = 0
        self.walker_heading_acceptance_rate_5 = 0
        self.walker_heading_acceptance_rate_10 = 0
        self.walker_velocity_heading_acceptance_rate = 0
        self.walker_box_width_acceptance_rate = 0
        self.walker_box_length_acceptance_rate = 0
        self.bike_status_matching_success_acceptance_rate = 0
        self.bike_position_acctptance_rate = 0
        self.bike_velocity_acceptance_rate = 0
        self.bike_heading_acceptance_rate_5 = 0
        self.bike_heading_acceptance_rate_10 = 0
        self.bike_velocity_heading_acceptance_rate = 0
        self.bike_box_width_acceptance_rate = 0
        self.bike_box_length_acceptance_rate   = 0
        self.vehicle_status_matching_success_acceptance_rate = 0
        self.vehicle_position_acctptance_rate = 0
        self.vehicle_velocity_acceptance_rate = 0
        self.vehicle_heading_acceptance_rate_5 = 0
        self.vehicle_heading_acceptance_rate_10 = 0
        self.vehicle_velocity_heading_acceptance_rate = 0
        self.vehicle_box_width_acceptance_rate = 0
        self.vehicle_box_length_acceptance_rate = 0
        self.actors = world.get_actors()
        self.data_frame = data_frame
        self.frae_cnt = frame_cnt
    def ExtractDataAndCalculate(self):
        
        walker_postion_error_list = []
        walker_status_matching_list =[]
        walker_velocity_error_list =[]
        walker_velocity_heading_error_list = []
        walker_heading_error_5_list = []
        walker_heading_error_10_list = []
        walker_box_width_error_list = []
        walker_box_length_error_list = []


        bike_postion_error_list = []
        bike_status_matching_list =[]
        bike_velocity_error_list =[]
        bike_velocity_heading_error_list = []
        bike_heading_error_5_list = []
        bike_heading_error_10_list = []
        bike_box_width_error_list = []
        bike_box_length_error_list = []


        vehicle_postion_error_list = []
        vehicle_status_matching_list =[]
        vehicle_velocity_error_list =[]
        vehicle_velocity_heading_error_list = []
        vehicle_heading_error_5_list = []
        vehicle_heading_error_10_list = []
        vehicle_box_width_error_list = []
        vehicle_box_length_error_list = []
        #获得actor名字
        actor_types = self.data_frame[2]
        print("actor_types",actor_types)
        for i, actor in enumerate(actor_types):
            if i>0:
                if re.match("walker",actor):
                    walker_postion_error_list.append(self.data_frame[7][i])
                    walker_status_matching_list.append(self.data_frame[10][i])
                    walker_velocity_error_list.append(self.data_frame[13][i])
                    walker_heading_error_5_list.append(self.data_frame[16][i])
                    walker_heading_error_10_list.append(self.data_frame[16][i])      
                    walker_velocity_heading_error_list.append(self.data_frame[19][i])
                    walker_box_width_error_list.append(self.data_frame[22][i])
                    walker_box_length_error_list.append(self.data_frame[25][i])
                    
                else :
                    is_bike =False
                    for temp_actor in self.actors:
                        if re.match(actor,temp_actor.type_id):
                            is_bike = int((temp_actor.attributes['number_of_wheels']))==2
                            print("is bike: ",is_bike,(temp_actor.attributes['number_of_wheels']))
                            break
                    if is_bike:
                        bike_postion_error_list.append(self.data_frame[7][i])
                        bike_status_matching_list.append(self.data_frame[10][i])
                        bike_velocity_error_list.append(self.data_frame[13][i])
                        bike_heading_error_5_list.append(self.data_frame[16][i])
                        bike_heading_error_10_list.append(self.data_frame[16][i])      
                        bike_velocity_heading_error_list.append(self.data_frame[19][i])
                        bike_box_width_error_list.append(self.data_frame[22][i])
                        bike_box_length_error_list.append(self.data_frame[25][i])
                    else:
                        vehicle_postion_error_list.append(self.data_frame[7][i])
                        vehicle_status_matching_list.append(self.data_frame[10][i])
                        vehicle_velocity_error_list.append(self.data_frame[13][i])
                        vehicle_heading_error_5_list.append(self.data_frame[16][i])
                        vehicle_heading_error_10_list.append(self.data_frame[16][i])      
                        vehicle_velocity_heading_error_list.append(self.data_frame[19][i])
                        vehicle_box_width_error_list.append(self.data_frame[22][i])
                        vehicle_box_length_error_list.append(self.data_frame[25][i])

        self.calculate_walker_status_matching_success_acceptance_rate(walker_status_matching_list)
        self.calculate_walker_position_acceptance_rate(walker_postion_error_list)
        self.calculate_walker_velocity_acceptance_rate(walker_velocity_error_list)
        self.calculate_walker_heading_acceptance_rate_5(walker_heading_error_5_list)
        self.calculate_walker_heading_acceptance_rate_10(walker_heading_error_10_list)
        self.calculate_walker_velocity_heading_acceptance_rate(walker_velocity_heading_error_list)
        self.calculate_walker_box_width_acceptance_rate(walker_box_width_error_list)
        self.calculate_walker_box_length_acceptance_rate(walker_box_length_error_list)

        self.calculate_bike_status_matching_success_acceptance_rate(bike_status_matching_list)
        self.calculate_bike_position_acceptance_rate(bike_postion_error_list)
        self.calculate_bike_velocity_acceptance_rate(bike_velocity_error_list)
        self.calculate_bike_heading_acceptance_rate_5(bike_heading_error_5_list)
        self.calculate_bike_heading_acceptance_rate_10(bike_heading_error_10_list)
        self.calculate_bike_velocity_heading_acceptance_rate(bike_velocity_heading_error_list)
        self.calculate_bike_box_width_acceptance_rate(bike_box_width_error_list)
        self.calculate_bike_box_length_acceptance_rate(bike_box_length_error_list)

        self.calculate_vehicle_status_matching_success_acceptance_rate(vehicle_status_matching_list)
        self.calculate_vehicle_position_acceptance_rate(vehicle_postion_error_list)
        self.calculate_vehicle_velocity_acceptance_rate(vehicle_velocity_error_list)
        self.calculate_vehicle_heading_acceptance_rate_5(vehicle_heading_error_5_list)
        self.calculate_vehicle_heading_acceptance_rate_10(vehicle_heading_error_10_list)
        self.calculate_vehicle_velocity_heading_acceptance_rate(vehicle_velocity_heading_error_list)
        self.calculate_vehicle_box_width_acceptance_rate(vehicle_box_width_error_list)
        self.calculate_vehicle_box_length_acceptance_rate(vehicle_box_length_error_list)


    """
    walker
    """
    #输出置信度合格率
    def calculate_walker_status_matching_success_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.walker_status_matching_success_acceptance_rate = -1
        else:
            acceptance_cnt =0
            for data in data_list:
                if data :
                    acceptance_cnt+=1
            self.walker_status_matching_success_acceptance_rate = acceptance_cnt / len(data_list)
    #计算几何中心<0.5的合格率
    def calculate_walker_position_acceptance_rate(self,data_list):
        if len(data_list)==0:
            self.walker_position_acctptance_rate = -1
        else:
            acceptance_cnt =0
            for data in data_list:
                if data<=0.5:
                    acceptance_cnt+=1
            self.walker_position_acctptance_rate = acceptance_cnt / len(data_list)
    #计算速度大小合格率
    def calculate_walker_velocity_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.walker_velocity_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.15:
                    acceptance_cnt+=1
            self.walker_velocity_acceptance_rate = acceptance_cnt / len(data_list)
    #计算障碍物方向偏差合格率5度
    def calculate_walker_heading_acceptance_rate_5 (self,data_list):
        if len(data_list) ==0:
            self.walker_heading_acceptance_rate_5 = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=5:
                    acceptance_cnt+=1
            self.walker_heading_acceptance_rate_5 = acceptance_cnt / len(data_list)
    #计算障碍物方向偏差合格率10度
    def calculate_walker_heading_acceptance_rate_10 (self,data_list):
        if len(data_list)==0:
            self.walker_heading_acceptance_rate_10 = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=10:
                    acceptance_cnt+=1
            self.walker_heading_acceptance_rate_10 = acceptance_cnt / len(data_list)
    #计算速度朝向合格率
    def calculate_walker_velocity_heading_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.walker_velocity_heading_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=10:
                    acceptance_cnt+=1
            self.walker_velocity_heading_acceptance_rate = acceptance_cnt / len(data_list)
    #计算box宽度合格率
    def calculate_walker_box_width_acceptance_rate (self,data_list): 
        if len(data_list)==0:
            self.walker_box_width_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.15:
                    acceptance_cnt+=1
            self.walker_box_width_acceptance_rate = acceptance_cnt / len(data_list)
    #计算box长度合格率
    def calculate_walker_box_length_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.walker_box_length_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.15:
                    acceptance_cnt+=1
            self.walker_box_length_acceptance_rate = acceptance_cnt / len(data_list)
    """
    bike
    """

    #输出置信度合格率
    def calculate_bike_status_matching_success_acceptance_rate (self,data_list):
        if len(data_list) == 0:
            self.bike_status_matching_success_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data :
                    acceptance_cnt+=1
            self.bike_status_matching_success_acceptance_rate = acceptance_cnt / len(data_list)
    #计算几何中心<0.5的合格率
    def calculate_bike_position_acceptance_rate(self,data_list):
        if len(data_list) == 0:
            self.bike_position_acctptance_rate = -1
        else:
            acceptance_cnt =0
            for data in data_list:
                if data <= 0.5:
                    acceptance_cnt+= 1
            self.bike_position_acctptance_rate = acceptance_cnt / len(data_list)
    #计算速度大小合格率
    def calculate_bike_velocity_acceptance_rate (self,data_list):
        if len(data_list) == 0:
            self.bike_velocity_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.15:
                    acceptance_cnt+=1
            self.bike_velocity_acceptance_rate = acceptance_cnt / len(data_list)
    #计算障碍物方向偏差合格率5度
    def calculate_bike_heading_acceptance_rate_5 (self,data_list):
        if len(data_list) == 0:
            self.bike_heading_acceptance_rate_5 = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=5:
                    acceptance_cnt+=1
            self.bike_heading_acceptance_rate_5 = acceptance_cnt / len(data_list)
    #计算障碍物方向偏差合格率10度
    def calculate_bike_heading_acceptance_rate_10 (self,data_list):
        if len(data_list) == 0:
            self.bike_heading_acceptance_rate_10 = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=10:
                    acceptance_cnt+=1
            self.bike_heading_acceptance_rate_10 = acceptance_cnt / len(data_list)
    #计算速度朝向合格率
    def calculate_bike_velocity_heading_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.bike_velocity_heading_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=10:
                    acceptance_cnt+=1
            self.bike_velocity_heading_acceptance_rate = acceptance_cnt / len(data_list)
    #计算box宽度合格率
    def calculate_bike_box_width_acceptance_rate (self,data_list): 
        if len(data_list)==0:
            self.bike_box_width_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.15:
                    acceptance_cnt+=1
            self.bike_box_width_acceptance_rate = acceptance_cnt / len(data_list)
    #计算box长度合格率
    def calculate_bike_box_length_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.bike_box_length_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.15:
                    acceptance_cnt+=1
            self.bike_box_length_acceptance_rate = acceptance_cnt / len(data_list)
   
    """
    vehicle
    """ #输出置信度合格率
    def calculate_vehicle_status_matching_success_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.vehicle_status_matching_success_acceptance_rate = -1
        else:
            acceptance_cnt =0
            for data in data_list:
                if data :
                    acceptance_cnt+=1
            self.vehicle_status_matching_success_acceptance_rate = acceptance_cnt / len(data_list)
    #计算几何中心<0.5的合格率
    def calculate_vehicle_position_acceptance_rate(self,data_list):
        if len(data_list)==0:
            self.vehicle_position_acctptance_rate = -1
        else:
            acceptance_cnt =0
            for data in data_list:
                if data<= 0.5:
                    acceptance_cnt+=1
            self.vehicle_position_acctptance_rate = acceptance_cnt / len(data_list)
    #计算速度大小合格率
    def calculate_vehicle_velocity_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.vehicle_velocity_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.2:
                    acceptance_cnt+=1
            self.vehicle_velocity_acceptance_rate = acceptance_cnt / len(data_list)
    #计算障碍物方向偏差合格率5度
    def calculate_vehicle_heading_acceptance_rate_5 (self,data_list):
        if len(data_list)==0:
            self.vehicle_heading_acceptance_rate_5 = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=5:
                    acceptance_cnt+=1
            self.vehicle_heading_acceptance_rate_5 = acceptance_cnt / len(data_list)
    #计算障碍物方向偏差合格率10度
    def calculate_vehicle_heading_acceptance_rate_10 (self,data_list):
        if len(data_list)==0:
            self.vehicle_heading_acceptance_rate_10 = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=10:
                    acceptance_cnt+=1
            self.vehicle_heading_acceptance_rate_10 = acceptance_cnt / len(data_list)
    #计算速度朝向合格率
    def calculate_vehicle_velocity_heading_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.vehicle_velocity_heading_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=10:
                    acceptance_cnt+=1
            self.vehicle_velocity_heading_acceptance_rate = acceptance_cnt / len(data_list)
    #计算box宽度合格率
    def calculate_vehicle_box_width_acceptance_rate (self,data_list): 
        if len(data_list)==0:
            self.vehicle_box_width_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for i, data in enumerate( data_list):
                if data<=0.2:
                    acceptance_cnt+=1
                else:
                    box = boxs[self.frae_cnt]
                    bbox=box[i]
                    sim_box = bbox[3]
                    tracker_box = bbox[2]
                    iou = bbox[0]
                    sim_id  = bbox[4]
                    time = bbox[5]
                    width = bbox[6]
                    length = bbox[7]
                    print("sim_id",sim_id,"curr iou :",iou,"frame_cnt:",self.frae_cnt,"number: ",i,"time:",time,"width: ",width,"length: ",length)
                    # plot_polygon(tracker_box,sim_box)
            self.vehicle_box_width_acceptance_rate = acceptance_cnt / len(data_list)
    #计算box长度合格率
    def calculate_vehicle_box_length_acceptance_rate (self,data_list):
        if len(data_list)==0:
            self.vehicle_box_length_acceptance_rate = -1
        else:
            acceptance_cnt = 0
            for data in data_list:
                if data<=0.2:
                    acceptance_cnt+=1
            self.vehicle_box_length_acceptance_rate = acceptance_cnt / len(data_list)
    def get_walker_status_matching_success_acceptance_rate (self):
        return self.walker_status_matching_success_acceptance_rate
    def get_walker_position_acceptance_rate(self):
        return self.walker_position_acctptance_rate
    def get_walker_velocity_acceptance_rate (self):
        return self.walker_velocity_acceptance_rate
    def get_walker_heading_acceptance_rate_5 (self):
        return self.walker_heading_acceptance_rate_5
    def get_walker_heading_acceptance_rate_10 (self):
        return self.walker_heading_acceptance_rate_10
    def get_walker_velocity_heading_acceptance_rate (self):
        return self.walker_velocity_heading_acceptance_rate
    def get_walker_box_width_acceptance_rate (self):
        return self.walker_box_width_acceptance_rate
    def get_walker_box_length_acceptance_rate (self):
        return self.walker_box_length_acceptance_rate

    def get_bike_status_matching_success_acceptance_rate (self):
        return self.bike_status_matching_success_acceptance_rate
    def get_bike_velocity_acceptance_rate (self):
        return self.bike_velocity_acceptance_rate
    def get_bike_position_acceptance_rate(self):
        return self.bike_position_acctptance_rate
    def get_bike_heading_acceptance_rate_5 (self):
        return self.bike_heading_acceptance_rate_5
    def get_bike_heading_acceptance_rate_10 (self):
        return self.bike_heading_acceptance_rate_10
    def get_bike_velocity_heading_acceptance_rate (self):
        return self.bike_velocity_heading_acceptance_rate
    def get_bike_box_width_acceptance_rate (self):
        return self.bike_box_width_acceptance_rate
    def get_bike_box_length_acceptance_rate   (self):
        return self.bike_box_length_acceptance_rate

    def get_vehicle_status_matching_success_acceptance_rate (self):
        return self.vehicle_status_matching_success_acceptance_rate
    def get_vehicle_position_acceptance_rate(self):
        return self.vehicle_position_acctptance_rate
    def get_vehicle_velocity_acceptance_rate (self):
        return self.vehicle_velocity_acceptance_rate
    def get_vehicle_heading_acceptance_rate_5 (self):
        return self.vehicle_heading_acceptance_rate_5
    def get_vehicle_heading_acceptance_rate_10 (self):
        return self.vehicle_heading_acceptance_rate_10
    def get_vehicle_velocity_heading_acceptance_rate (self):
        return self.vehicle_velocity_heading_acceptance_rate
    def get_vehicle_box_width_acceptance_rate (self):
        return self.vehicle_box_width_acceptance_rate
    def get_vehicle_box_length_acceptance_rate (self):
        return self.vehicle_box_length_acceptance_rate


def CalculateVariance(average,data_list):
    print("--------------------------------------")
    print(data_list)
    variance = 0
    for data in data_list:
        variance+=(data - average)*(data - average)
        print("calculating... ",variance , data,average)
    
    variance = variance/len(data_list)
    return variance
# 误差计算类
class ErrorAnalysis:
    sim_position = Point()
    def __init__(self,sim_obj,tracker_obj) :
        self.sim_type_id = sim_obj.object_name
        self.sim_id = sim_obj.object_id
        self.tracker_obj_id = tracker_obj.id
        self.sim_position.x = round(sim_obj.world_position.x,2)
        self.sim_position.y = round(sim_obj.world_position.y,2)
        self.sim_position.z = round(sim_obj.world_position.z,2)
        tracker_obj.position.x=round(tracker_obj.position.x,2)
        tracker_obj.position.y=round(tracker_obj.position.y,2)
        tracker_obj.position.z=round(tracker_obj.position.z,2)
        self.tracker_position= tracker_obj.position
        self.position_error = 0

        self.is_simobj_static = sim_obj.is_static
        self.is_trackerobj_static = tracker_obj.is_static
        self.is_same_status = False
        self.sim_velocity=round(sim_obj.velocity,2)
        self.tracker_velocity=round(math .sqrt(tracker_obj.velocity.x*tracker_obj.velocity.x+tracker_obj.velocity.y*tracker_obj.velocity.y),2)
        self.velocity_error = 0
        sim_obj.velocity_vector.x=round( sim_obj.velocity_vector.x,2)
        sim_obj.velocity_vector.y=round( sim_obj.velocity_vector.y,2)
        sim_obj.velocity_vector.z=round( sim_obj.velocity_vector.z,2)
        self.sim_velocity_vector = sim_obj.velocity_vector
        tracker_obj.velocity.x=round(tracker_obj.velocity.x,2)
        tracker_obj.velocity.y=round(tracker_obj.velocity.y,2)
        tracker_obj.velocity.z=round(tracker_obj.velocity.z,2)
        self.tracker_velocity_vector = tracker_obj.velocity
        self.velocity_heading_error = 0
    
        self.sim_heading = round(sim_obj.world_heading,2)
        
        self.tracker_heading = round(tracker_obj.theta,2)
        self.heading_error = 0
    
        self.sim_box = sim_obj.polygon_point
        self.tracker_box = tracker_obj.polygon_point
        self.box_point_error = []

        self.sim_width  = round(sim_obj.width,2)
        self.tracker_width = round(tracker_obj.width,2)
        self.box_width_error = 0
        
        self.sim_length =  round(sim_obj.length,2)
        self.tracker_length = round(tracker_obj.length,2)
        self.box_legnth_error = 0
        
        self .sim_height = round(sim_obj.height,2)
        self.tracker_height = round(tracker_obj.height,2)
        self.box_height_error = 0

    def calculate_position_error(self):
        self.position_error = round(math.hypot(abs(self.tracker_position.x - self.sim_position.x),abs(self.tracker_position.y - self.sim_position.y)),2)
    def calculate_velocity_error(self):
        if round(self.sim_velocity)== 0:
            self.velocity_error = self.tracker_velocity/(1+self.sim_velocity)
        else:
            self.velocity_error = round(abs(self.tracker_velocity - self.sim_velocity)/abs(self.sim_velocity),2)
    def calculate_velocity_heading_error(self):
    #    velocity_heading_error = round( math.atan2(abs(self.sim_velocity_vector.y -self.tracker_velocity_vector.y ),abs(self.sim_velocity_vector.x -self.tracker_velocity_vector.x ))/math.pi*180,2)
        curr_sim_v = math.hypot(self.sim_velocity_vector.x,self.sim_velocity_vector.y)
        curr_tracker_v = math.hypot(self.tracker_velocity_vector.x, self.tracker_velocity_vector.y)
        
        if curr_sim_v<=0.1 and curr_tracker_v<0.1:
            self.velocity_heading_error = 0
        else:
            sim_vec_heding = round(math.atan2(self.sim_velocity_vector.y, self.sim_velocity_vector.x)/math.pi*180,2)        
            tracker_vec_heding = round(math.atan2(self.tracker_velocity_vector.y, self.tracker_velocity_vector.x)/math.pi*180,2)
            delta_heading  =abs(sim_vec_heding - tracker_vec_heding)
            if delta_heading<180:
                self.velocity_heading_error = delta_heading
            else:
                self.velocity_heading_error = 360 - delta_heading
    def calculate_heading_error(self):
        print("heading ", self.tracker_heading,self.sim_heading)
        temp_heading = round(abs(self.tracker_heading - self.sim_heading)/math.pi*180,2)
        if temp_heading >45 and temp_heading < 135:
            self.heading_error = abs(temp_heading - 90)
        if temp_heading>=135 and temp_heading<225:
            self.heading_error = abs(temp_heading -180)
        if temp_heading>=225 and temp_heading<315:
            self.heading_error = abs(temp_heading - 270)
    def calculate_box_error(self):
        final_err = 10000
        for i, sim_box_point in enumerate(self.sim_box):
            for tracker_box_point in self.tracker_box:
                tem_err = round(math.hypot(sim_box_point.x - tracker_box_point.x),2)
                if tem_err<final_err:
                    final_err = tem_err            
            self.box_point_error[i].append()

    def calculate_box_width_errror(self):
        self.box_width_error =round(abs(self.tracker_width - self.sim_width)/self.sim_width,2)
    def calculate_box_length_error(self):
        self.box_legnth_error = round(abs(self.tracker_length - self.sim_length)/self.sim_length,2)
    def calculate_box_height_errror(self):
        self.box_height_error = round(abs(self.tracker_height - self.sim_height)/self.sim_height,2)
    def compare_status_of_two_obj(self):
        if (self.is_trackerobj_static ==True and self.is_simobj_static == True)or(self.is_trackerobj_static ==False and self.is_simobj_static == False):
            self.is_same_status = True
        else:
            self.is_same_status = False
    def get_error_of_position(self):
        return self.position_error
    def get_error_of_heading(self):
        return self.heading_error
    def get_error_of_velocity(self):
        return self.velocity_error
    def get_error_of_velocity_heading(self):
        return self.velocity_heading_error
    def get_error_of_box(self):
        return self.box_point_error
    def is_static_matched(self):
        return self.is_same_status
    def get_box_width_error(self):
        return self.box_width_error
    def get_box_length_error(self):
        return self.box_legnth_error
    def get_box_height_error(self):
        return self.box_height_error

def calculate_bounding_box(w,l,yaw,pose):

    dx1 = math.cos(yaw) * l
    dy1 = math.sin(yaw) * l
    dx2 = math.sin(yaw) * w
    dy2 = -math.cos(yaw) * w
    corners = []
    corners.append(pose.x + dx1 + dx2 )
    corners.append(pose.y + dy1 + dy2)
    corners.append(pose.x + dx1 - dx2)
    corners.append( pose.y + dy1 - dy2)
    corners.append(pose.x - dx1 - dx2)
    corners.append( pose.y - dy1 - dy2)
    corners.append(pose.x - dx1 + dx2)
    corners.append( pose.y - dy1 + dy2)

    # left_up_x = pose.x-(l*math.cos(yaw)+w*math.sin(yaw))
    # left_up_y = pose.y-(l*math.sin(yaw)-w*math.cos(yaw))

    # left_down_x = pose.x-(l*math.cos(yaw)-w*math.sin(yaw))
    # left_down_y = pose.y-(l*math.sin(yaw)+w*math.cos(yaw))

    # right_up_x = pose.x+(l*math.cos(yaw)-w*math.sin(yaw))
    # right_up_y = pose.y+(l*math.sin(yaw)+w*math.cos(yaw))

    # right_down_x = pose.x + (l*math.cos(yaw)+w*math.sin(yaw))
    # right_down_y = pose.y + (l*math.sin(yaw)-w*math.cos(yaw))
    # print("l: ", l, " w: ", w, " yaw: ", yaw, " pose: ", pose)
    # box = []
    # box.append(left_up_x)
    # box.append(left_up_y)
    # box.append(left_down_x)
    # box.append(left_down_y)
    # box.append(right_down_x)
    # box.append(right_down_y)
    # box.append(right_up_x)
    # box.append(right_up_y)
    return corners

def main():
    try:
        client = carla.Client('localhost',2000)
        world = client.get_world()
        global curr_world
        curr_world = world
        rospy.init_node("Udi_evaluation_tool")
        obs_reader = obstacles()
        sleep(1)
        rate = rospy.Rate(20)
        frame_cnt = 200
        temp_cnt = 1
        finish = False
        sample_step = 2
        while not rospy.is_shutdown() and not finish :
            if len(obss)>0:
                if temp_cnt%sample_step==0:
                    global err_data 
                    snapshot = world.wait_for_tick()
                    actors =  world.get_actors()
                    objects = simObjects()
                    ego_location = carla.Location(0,0,0)
                    for actor in actors:
                        if re.match("vehicle.kuafumini",actor.type_id):
                            ego_location = actor.get_location()
                            ego_transform =actor.get_transform()
                            print("find ego!!")
                            break
                    for actor in actors:
                        actor_location = actor.get_location()
                        valid_dist = math.hypot(ego_location.x - actor_location.x,ego_location.y-actor_location.y)
                        obj = simObject() 
                        obj.is_bike = False
                        obj.is_walker = False
                        obj.is_vehicle = False
                        if (re.match("walker",actor.type_id) or re.match("vehicle",actor.type_id)) and not re.match("vehicle.kuafumini",actor.type_id) and valid_dist <= valid_distacnce:
                            bounding_box = actor.bounding_box.extent
                            print("--------------0-----------------")
                            if re.match("vehicle",actor.type_id):
                                number_of_wheels=actor.attributes['number_of_wheels']
                                if float(number_of_wheels) == 2 and float(bounding_box.y) <0.01:
                                    bounding_box.y = 0.435 
                                if float(number_of_wheels) == 2:
                                    obj.is_bike = True
                                else:
                                    obj.is_vehicle = True
                            else:
                                obj.is_walker = True
                                                               
                            print("box size: ",bounding_box.x,bounding_box.y,bounding_box.z,actor.type_id)
                            # print(actor.bounding_box.rotation.yaw,actor.get_transform().rotation.yaw)
                            
                            currac =  snapshot.find(actor.id) 
                            yaw = -actor.get_transform().rotation.yaw/180*math.pi

                            obj.object_name=actor .type_id
                            obj.object_id = actor.id
                            obj.world_position.x = actor.get_location().x
                            obj.world_position.y = -actor.get_location().y
                            obj.world_position.z = actor.get_location().z
                            obj.world_heading = yaw
                            obj.length = bounding_box.x*2
                            obj.width = bounding_box.y*2
                            obj.height = bounding_box.z*2
                            obj.velocity_vector.x = currac.get_velocity().x
                            obj.velocity_vector.y = -currac.get_velocity().y
                            obj.velocity_vector.z = currac.get_velocity().z
                            obj.velocity = math.sqrt( obj.velocity_vector.x* obj.velocity_vector.x+ obj.velocity_vector.y*obj.velocity_vector.y+ obj.velocity_vector.z* obj.velocity_vector.z)
                            if abs(obj.velocity) <0.05:
                                obj.is_static = True
                            else:
                                obj.is_static = False
                            delta_x = obj.world_position.x
                            delta_y = obj.world_position.y
                            l = bounding_box.x*2
                            w = bounding_box.y*2
                            print("l: ", l, " w: ", w, " yaw: ", yaw, " pose: ", obj.world_position)

                            # left_up_x = delta_x-(l*math.cos(yaw)+w*math.sin(yaw))
                            # left_up_y = delta_y-(l*math.sin(yaw)-w*math.cos(yaw))
                            # point_left_up = Point()
                            # point_left_up.x = left_up_x
                            # point_left_up.y = left_up_y
                            # point_left_up.z = 0
                            # obj.polygon_point.append(point_left_up)
                            # left_down_x = delta_x-(l*math.cos(yaw)-w*math.sin(yaw))
                            # left_down_y = delta_y-(l*math.sin(yaw)+w*math.cos(yaw))
                            # point_left_down = Point()
                            # point_left_down.x = left_down_x
                            # point_left_down.y = left_down_y
                            # point_left_down.z = 0.0
                            # obj.polygon_point.append(point_left_down)
                            # right_down_x =delta_x+(l*math.cos(yaw)+w*math.sin(yaw))
                            # right_down_y = delta_y+(l*math.sin(yaw)-w*math.cos(yaw))
                            # point_right_down = Point()
                            # point_right_down.x = right_down_x
                            # point_right_down.y = right_down_y
                            # point_right_down.z = 0.0
                            # obj.polygon_point.append(point_right_down)
                            # right_up_x = delta_x+(l*math.cos(yaw)-w*math.sin(yaw))
                            # right_up_y = delta_y+(l*math.sin(yaw)+w*math.cos(yaw))
                            # point_right_up = Point()
                            # point_right_up.x = right_up_x
                            # point_right_up.y = right_up_y
                            # point_right_up.z = 0.0
                            # obj.polygon_point.append(point_right_up)

                            
                            dx1 = math.cos(yaw) * l
                            dy1 = math.sin(yaw) * l
                            dx2 = math.sin(yaw) * w
                            dy2 = -math.cos(yaw) * w
                            corners = []
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

                            objects.simObjects.append(obj)
                    print("objects",objects)
                    data_error=calculate_error(obss,objects,ego_transform)
                    if temp_cnt ==frame_cnt:
                        export_data2csv(data_error,world)
                        finish = True
                        print("to_scv finished!")
                temp_cnt+=1
            else:
                print("there have no data, please start a scenario first!")
            rate.sleep()

    except KeyboardInterrupt:
        del client
  

if __name__ == '__main__':
    main()