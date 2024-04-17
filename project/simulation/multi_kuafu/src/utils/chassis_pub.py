#!/usr/bin/env python
#
# Copyright (c) 2022, Unity-Drive Inc. All rights reserved.
# Author: Zhenyu Li lizhenyu@unity-drive.com

import rospy
from std_msgs.msg import UInt8
from udi_msgs.msg import Chassis
from sys_eng_msgs.msg import CarData, DrivingCmd, DrivingStatus, ChassisStatus, CarDrivingFeedback
from sys_eng_msgs.msg import Uint8WithCheck, Float32WithCheck
from udi_msgs.msg import LocalizationEstimate

# default control mode: automatic
gl_CURRENT_CMD = 1
gl_CURRENT_SPEED = 0
gl_CURRENT_ODOM = 0
gl_prev_timestep = None
gl_downsampling = False

def process_msgs():
    global gl_CURRENT_CMD, gl_CURRENT_SPEED, gl_CURRENT_ODOM

    chassis_msg = Chassis()
    chassis_msg.header.frame_id = "chassis_pub"
    chassis_msg.header.timestamp_sec = rospy.get_time()
    chassis_msg.driving_mode = gl_CURRENT_CMD

    car_data_msg = CarData()
    car_data_msg.header.frame_id = "car_data_pub"
    car_data_msg.header.stamp = rospy.Time.now()

    car_driving_feedback_msg = CarDrivingFeedback()
    car_driving_feedback_msg.header.frame_id = "car_driving_feedback_pub"
    car_driving_feedback_msg.header.stamp = rospy.Time.now()

    driving_cmd = DrivingCmd()
    chassis_mode = Uint8WithCheck()
    chassis_mode2 = UInt8()
    simple_car_mode = 0
    # onsite mode
    if gl_CURRENT_CMD is 0:
        chassis_mode.data = 3
        chassis_mode2.data = 3
    # auto mode
    elif gl_CURRENT_CMD is 1:
        chassis_mode.data = 7
        chassis_mode2.data = 7
        simple_car_mode = 1
    # remote driving mode
    elif gl_CURRENT_CMD is 2:
        chassis_mode.data = 5
        chassis_mode2.data = 5
    # remote_escape mode 
    elif gl_CURRENT_CMD is 3:
        chassis_mode.data = 6
        chassis_mode2.data = 6
    # idle mode
    elif gl_CURRENT_CMD is 9:
        chassis_mode.data = 8
        chassis_mode2.data = 8
    else:
        print("Invalid control cmd: ", gl_CURRENT_CMD)
    chassis_mode.is_valid = True
    driving_cmd.chassis_control_mode_code = chassis_mode
    car_driving_feedback_msg.chassis_control_mode_code = chassis_mode.data

    driving_status = DrivingStatus()
    speed_feedback = Float32WithCheck()
    speed_feedback.data = gl_CURRENT_SPEED
    speed_feedback.is_valid = True
    driving_status.speed_feedback = speed_feedback

    chassis_status = ChassisStatus()
    odometer = Float32WithCheck()
    odometer.data = gl_CURRENT_ODOM
    odometer.is_valid = True
    chassis_status.odometer = odometer

    car_data_msg.driving_cmd = driving_cmd
    car_data_msg.driving_status = driving_status
    car_data_msg.chassis_status = chassis_status

    car_mode_msg = UInt8()
    car_mode_msg.data = simple_car_mode
    
    return chassis_msg, car_data_msg, car_mode_msg, car_driving_feedback_msg

def net_port_callback(data):
    global gl_CURRENT_CMD
    gl_CURRENT_CMD = data.data

def local_callback(data):
    global gl_CURRENT_SPEED, gl_CURRENT_ODOM, gl_prev_timestep, gl_downsampling
    gl_CURRENT_SPEED = data.pose.linear_velocity_vrf.x
    if gl_prev_timestep is not None:
        delta_t = data.header.timestamp_sec - gl_prev_timestep
        gl_CURRENT_ODOM += abs(delta_t * gl_CURRENT_SPEED)
    gl_prev_timestep = data.header.timestamp_sec

    if gl_downsampling:
        chassis_msg, car_data_msg, car_mode_msg, car_driving_feedback_msg = process_msgs()
        chassis_pub.publish(chassis_msg)
        car_data_pub.publish(car_data_msg)
        car_mode_pub.publish(car_mode_msg)
        car_driving_feedback_pub.publish(car_driving_feedback_msg)
    
    gl_downsampling = not gl_downsampling

if __name__ == '__main__':
    try:
        rospy.init_node('chassis_pub', anonymous=True)
        chassis_pub = rospy.Publisher('chassis', Chassis, queue_size=10)
        car_data_pub = rospy.Publisher('car_data/upload', CarData, queue_size=10)
        car_mode_pub = rospy.Publisher('lower_controller/car_modes', UInt8, queue_size=10)
        car_driving_feedback_pub = rospy.Publisher('sys_data_hub/car_driving_feedback', CarDrivingFeedback, queue_size=10)
        sub1 = rospy.Subscriber('remote_client/mode_switch_cmd', UInt8, net_port_callback)
        sub2 = rospy.Subscriber('localization_estimate', LocalizationEstimate, local_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass