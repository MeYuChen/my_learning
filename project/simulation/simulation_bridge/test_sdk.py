#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2023 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""

import os
import sys
import time
import etcd
import json
import redis
import carla
import loguru
import pathlib


def init_log(log_file: str, log_level="WARNING", log_size="100 MB", keep_log_days="7 days"):
    # sink = log_file + "_{time}.log"
    sink = log_file
    level = log_level
    rotation = log_size  # must be "10 MB" format
    retention = keep_log_days
    loguru.logger.add(sink=sink, level=level, rotation=rotation, retention=retention, enqueue=True)
    return loguru.logger


def demo():
    log = init_log("demo")
    log.debug('this is a debug message')
    log.info('this is another info message')
    log.warning('this is another warning message')
    log.error('this is another error message')
    log.success('this is success message!')
    log.critical('this is critical message!')

# oasis_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# print(oasis_path)
# sys.path.append(oasis_path)

import oasis
print(oasis.version)
# sys.exit(0)

root_dir = pathlib.Path(__file__).resolve().parent

NODE_IP = "127.0.0.1"  # 修改该 IP 为 oasis 部署机器的 IP 地址
NODE_NAME = "cluster_machine_1"
# 以下配置不需要修改
REDIS_HOST = NODE_IP
REDIS_PORT = 6379
REDIS_DB = 1
REDIS_PASSWORD = "123456"
ETCD_SERVER_IP = NODE_IP
ETCD_PORT = 2379


def get_task_info(task_id):
    pool = redis.ConnectionPool(host=REDIS_HOST,
                                port=REDIS_PORT,
                                db=REDIS_DB,
                                password=REDIS_PASSWORD,
                                decode_responses=True)
    r = redis.Redis(connection_pool=pool)
    task_info = r.hget("task_infos", task_id)
    task_info = json.loads(task_info)
    return task_info['task']


def print_lane_info(pnc):
    current_lane, left_lane, right_lane = pnc.get_lane_info()
    for lane_waypoint in current_lane:
        road_id = lane_waypoint.road_id
        lane_id = lane_waypoint.lane_id
        x = lane_waypoint.x
        y = lane_waypoint.y
        z = lane_waypoint.z
        roll = lane_waypoint.roll
        pitch = lane_waypoint.pitch
        yaw = lane_waypoint.yaw
        width = lane_waypoint.width
        left_lane_marking = lane_waypoint.left_lane_marking
        right_lane_marking = lane_waypoint.right_lane_marking

        oasis_sim.world.debug.draw_point(
            carla.Location(x=left_lane_marking.location.x,
                           y=left_lane_marking.location.y,
                           z=left_lane_marking.location.z),

            size=0.1,
            color=carla.Color(0, 255, 0), life_time=10
        )

        result = {
            "road_id": road_id,
            "lane_id": lane_id,
            "x": lane_waypoint.x,
            "y": lane_waypoint.y,
            "z": lane_waypoint.z,
            "pitch": lane_waypoint.pitch,
            "yaw": lane_waypoint.yaw,
            "width": lane_waypoint.width,
            "left_lane_marking": lane_waypoint.left_lane_marking,
            "right_lane_marking": lane_waypoint.right_lane_marking

        }
        print(f"road_id:{result}")


def print_object_detection_result():
    if object_detection:
        object_data_list = object_detection.get_obj_data()
        for object_data in object_data_list:
            if object_data.object_id != 0 and object_data.object_id is not None:
                log.info(f"object_data is: {vars(object_data)}")
                # log.info(f"world_x is: {object_data.world_x}")
                # log.info(f"world_y is: {object_data.world_y}")
                # log.info(f"world_z is: {object_data.world_z}")
                # log.info(f"velocity_x is: { object_data.velocity_x}")
                # log.info(f"velocity_y is: { object_data.velocity_y}")
                # log.info(f"velocity_z is: { object_data.velocity_z}")
                print("--------------------------------------")


        lane_data_list = object_detection.get_lane_data()
        for lane_data in lane_data_list:
            if lane_data.road_section_num != 0 and lane_data.road_section_num is not None:
                # log.info(f"lane_data is: {vars(lane_data)}")
                pass

        traffic_data_list = object_detection.get_traffic_light_data()
        for traffic_data in traffic_data_list:
            if traffic_data.traffic_light_num != 0 and traffic_data.traffic_light_num is not None:
                # log.info(f"traffic_data is: {vars(traffic_data)}")
                pass


if __name__ == '__main__':
    log_dir = str(root_dir.joinpath('log'))
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    log_file = os.path.join(log_dir, "oasis_bridge.log")
    if os.path.exists(log_file):
        os.remove(log_file)
    log = init_log(log_file, log_level="DEBUG")

    # test sim.py
    etcd_client = etcd.Client(host=ETCD_SERVER_IP, port=ETCD_PORT)
    etcd_key = f'/oasis/simulator/{NODE_NAME}/'
    tasks_key = etcd_key + 'tasks/'
    log.info("watch key is {}".format(tasks_key))

    for event in etcd_client.eternal_watch(tasks_key, recursive=True):
        event_key = event.key
        if event.action == 'set':
            task_id = event_key.split('/')[-2]
            event_value = event.value
            event_value = json.loads(event_value)

            action_type = event_value.get('status')  # init
            # action_type = "init"  # init
            carla_info = event_value.get('carla_info')
            # carla_info = {"ip": "172.16.19.104", "port": "2000"}
            log.info(f"task_id: {task_id} put changed, status is {action_type}, carla info is {carla_info}")
            if action_type == "init":  # 下发初始化动作
                task_info = get_task_info(task_id)
                oasis_sim = oasis.OasisSim(NODE_IP, 2000, task_info=task_info)
                oasis_sim.set_world_tick()
                ego = oasis_sim.get_ego_vehicle()
                log.info(f"ego_vehicle is {ego}")

                pnc = oasis.PNCInterfaces(oasis_sim)
                task = oasis.ScenarioInfoInterfaces(oasis_sim)

                # 主车加载完成后，加个延迟等待，否则传感器可能还没有加载上
                time.sleep(1)
                sensor = oasis.SensorInterfaces(oasis_sim)
                log.info(f"sensor init success")
                log.info(f"get_sensors: {task.get_sensors()}")
                log.info(f"get_task_id: {task.get_task_id()}")

                # test sensor_interface.py

                # rgb_sensor_name = "rgb-24t4QYaFCWpeMANNmizBiu"
                # image_path = os.path.join("/home/wohu/project/stand_alone/sdk/test_script/image/rgb", task_id)
                # rgb_camera = sensor.get_rgb_camera_sensor(rgb_sensor_name, image_path)

                # fish_sensor_name = ""
                # fish_image_path = os.path.join("/home/wohu/project/stand_alone/sdk/test_script/image/fish", task_id)
                # fisheye_camera = sensor.get_fisheye_camera_sensor(fish_sensor_name, fish_image_path)

                # depth_sensor_name = ""
                # depth_camera = sensor.get_depth_camera_sensor(depth_sensor_name)

                # semantic_camera = sensor.get_semantic_camera_sensor(rgb_sensor_name)
                # instance_camera = sensor.get_instance_camera_sensor(rgb_sensor_name)
                # dvs_camera = sensor.get_dvs_camera_sensor(rgb_sensor_name)

                # gnss_sensor_name = "gnss-DoyPDBMLqxcK8x2rstwNCh"
                # gnss_sensor = sensor.get_gnss_sensor(gnss_sensor_name)

                # imu_sensor_name = ""
                # imu_sensor = sensor.get_imu_sensor(imu_sensor_name)

                # radar_sensor_name = ""
                # radar_sensor = sensor.get_radar_sensor(radar_sensor_name)

                # ultrasonic_sensor_name = "ultrasonic-RSoBNr3d6DwtNAboEsX2X2"
                # ultrasonic_sensor = sensor.get_ultrasonic_sensor(ultrasonic_sensor_name)

                # ray_cast_lidar_sensor_name = "ray_cast-7T3V8qAgNLBNG699kGddFV"
                # ray_cast_lidar = sensor.get_ray_cast_lidar_sensor(ray_cast_lidar_sensor_name)
                # ray_cast_semantic_lidar = sensor.get_ray_cast_semantic_lidar_sensor(ray_cast_lidar_sensor_name)
                # ray_cast_mems_lidar = sensor.get_ray_cast_mems_lidar_sensor(ray_cast_lidar_sensor_name)

                object_sensor_name = "default"
                object_detection = sensor.get_object_detection_sensor(object_sensor_name)

                # test pnc_interface.py
                while True:
                    # oasis_sim.set_world_tick()
                    timestep = oasis_sim.get_world_timestep()
                    frame = oasis_sim.get_world_frame()
                    # log.info(f"timestep is {timestep}")
                    # log.info(f"frame is {frame}")
                    # log.info(f"{time.time()}")

                    # time.sleep(0.05)

                    # print_lane_info(pnc)

                    # object_info = pnc.get_object_info()
                    # odom_info = pnc.get_odom_info()

                    # traffic_lights_info = pnc.get_traffic_lights_info()
                    # log.info(f"lane_info is {lane_info}")
                    # log.info(f"object_info is {object_info}")
                    # log.info(f"odom_info is {odom_info}")
                    # log.info(f"traffic_lights_info is {traffic_lights_info}")

                    # log.debug(f"get_ego_control: {pnc.get_ego_control()}")
                    # log.debug(f"get_ego_physics_control: {pnc.get_ego_physics_control()}")
                    # log.debug(f"get_ego_vehicle_pose: {pnc.get_ego_vehicle_pose()}")
                    # log.debug(f"get_ego_geo_location: {pnc.get_ego_geo_location()}")
                    # log.debug(f"get_ego_current_gear: {pnc.get_ego_current_gear()}")
                    # log.debug(f"get_ego_current_throttle: {pnc.get_ego_current_throttle()}")
                    # log.debug(f"get_ego_vector_speed: {pnc.get_ego_vector_speed()}")
                    # log.debug(f"get_ego_brake_state: {pnc.get_ego_brake_state()}")
                    # log.debug(f"get_ego_brake_percent: {pnc.get_ego_brake_percent()}")
                    # log.debug(f"get_ego_hand_brake_state: {pnc.get_ego_hand_brake_state()}")
                    # log.debug(f"get_ego_wheel_steer_angle: {pnc.get_ego_wheel_steer_angle()}")
                    # log.debug(f"get_ego_light_state: {pnc.get_ego_light_state()}")
                    # log.debug(f"get_ego_left_light_st: {pnc.get_ego_left_light_st()}")
                    # log.debug(f"get_ego_left_right_st: {pnc.get_ego_left_right_st()}")
                    # log.debug(f"get_ego_acceleration: {pnc.get_ego_acceleration()}")
                    # log.debug(f"get_ego_steering_wheel_angle: {pnc.get_ego_steering_wheel_angle()}")

                    # log.debug(f"get_ego_engine_speed: {pnc.get_ego_engine_speed()}")
                    # log.debug(f"get_ego_no_vector_speed: {pnc.get_ego_no_vector_speed()}")
                    # log.debug(f"get_torque: {pnc.get_torque()}")
                    # log.debug(f"get_ego_angle_velocity: {pnc.get_ego_angle_velocity()}")
                    # log.debug(f"get_ego_yaw_rate: {pnc.get_ego_yaw_rate()}")
                    # log.debug(f"set_ego_control: {pnc.set_ego_control()}")
                    # log.debug(f"set_ego_ackermann_control: {pnc.set_ego_ackermann_control(speed=3.0, steer_angle=0.1)}")
                    # log.debug(f"set_ego_ackermann_settings: {pnc.set_ego_ackermann_settings()}")
                    # log.debug(f"set_ego_control_by_pose: {pnc.set_ego_control_by_pose(x=1.0, y=2.0, z=0.0)}")

                    # test task_info.py

                    # log.info(f"get_task_id: {task.get_task_id()}")
                    # log.info(f"is_data_record_enabled: {task.is_data_record_enabled()}")
                    # log.info(f"is_sensor_record_enabled: {task.is_sensor_record_enabled()}")
                    # log.info(f"get_fixed_delta_seconds: {task.get_fixed_delta_seconds()}")
                    # log.info(f"get_record_frequency: {task.get_record_frequency()}")
                    # log.info(f"get_controller: {task.get_controller()}")
                    # log.info(f"get_controller_version: {task.get_controller_version()}")
                    # log.info(f"get_controller_type: {task.get_controller_type()}")
                    # log.info(f"get_sensors: {task.get_sensors()}")
                    # log.info(f"get_render_mode: {task.get_render_mode()}")
                    # log.info(f"get_vehicle_physics_control: {task.get_vehicle_physics_control()}")
                    # log.info(f"get_vehicle_type: {task.get_vehicle_type()}")
                    # log.info(f"get_scenario_name: {task.get_scenario_name()}")
                    # log.info(f"get_stop_trigger: {task.get_stop_trigger()}")
                    # log.info(f"get_evaluation_standard: {task.get_evaluation_standard()}")

                    # start_position = task.get_start_position()
                    # end_position = task.get_end_position()
                    # start_location = oasis.Location(float(start_position.get("x")), -float(start_position.get("y")), float(start_position.get("z")))
                    # end_location = oasis.Location(float(end_position.get("x")), -float(end_position.get("y")), float(end_position.get("z")))
                    # log.info(f"get_start_position: {task.get_start_position()}")
                    # log.info(f"get_end_position: {task.get_end_position()}")

                    # if rgb_camera:
                    #     image_info = rgb_camera.get_sensor_data()
                    #     log.info(f"rgb_camera is: {image_info}")

                    # if fisheye_camera:
                    #     image_info = fisheye_camera.get_sensor_data()
                    #     log.info(f"fisheye_camera is: {image_info}")

                    # if depth_camera:
                    #     image_info = depth_camera.get_sensor_data()
                    #     log.info(f"depth_camera is: {image_info}")

                    # if semantic_camera:
                    #     image_info = semantic_camera.get_sensor_data()
                    #     log.info(f"image_info is: {image_info}")

                    # if instance_camera:
                    #     image_info = instance_camera.get_sensor_data()
                    #     log.info(f"instance_camera is: {image_info}")

                    # if dvs_camera:
                    #     image_info = dvs_camera.get_sensor_data()
                    #     log.info(f"image_info is: {image_info}")

                    # if gnss_sensor:
                    #     gnss_data = gnss_sensor.get_sensor_data()
                    #     log.info(f"gnss_data is: {gnss_data}")

                    # if imu_sensor:
                    #     imu_data = imu_sensor.get_sensor_data()
                    #     log.info(f"imu_data is: {imu_data}")

                    # if radar_sensor:
                    #     radar_data = radar_sensor.get_sensor_data()
                    #     log.info(f"radar_data is: {radar_data}")

                    # if ultrasonic_sensor:
                    #     ultrasonic_data = ultrasonic_sensor.get_sensor_data()
                    #     if ultrasonic_data:
                    #         log.info(f"ultrasonic_data altitude is: {ultrasonic_data[0].altitude}")
                    #         log.info(f"ultrasonic_data azimuth is: {ultrasonic_data[0].azimuth}")
                    #         log.info(f"ultrasonic_data depth is: {ultrasonic_data[0].depth}")
                    #         log.info(f"ultrasonic_data velocity is: {ultrasonic_data[0].velocity}")

                    # if ray_cast_lidar:
                    #     lidar_data = ray_cast_lidar.get_sensor_data()
                    #     log.info(f"ray_cast_lidar is: {lidar_data}")

                    # if ray_cast_semantic_lidar:
                    #     semantic_lidar_data = ray_cast_semantic_lidar.get_sensor_data()
                    #     log.info(f"semantic_lidar_data is: {semantic_lidar_data}")

                    # if ray_cast_mems_lidar:
                    #     mems_lidar = ray_cast_mems_lidar.get_sensor_data()
                    #     log.info(f"mems_lidar is: {mems_lidar}")

                    print_object_detection_result()

                    location = pnc.get_ego_vehicle_pose()[0]
                    if int(location.x) == 0 and int(location.y) == 0 and int(location.z) == 0:
                        sys.exit()
