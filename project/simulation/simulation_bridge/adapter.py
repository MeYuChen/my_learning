import sys
sys.path.append("/home/udi/x/modules")
import oasis
print(oasis.version)
# from oasis.sim import OasisSim
from oasis import OasisSim
# import cyber_py3.cyber_time as cyber_time
from cyber.python.cyber_py3 import cyber_time 
#writer type
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.localization.proto .localization_pb2 import LocalizationEstimate
from modules.localization.proto.localization_pb2 import LocalizationStatus
from modules.transform.proto.transform_pb2 import TransformStampeds
from modules.transform.proto.transform_pb2 import TransformStamped
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLight
from modules.perception.proto import traffic_light_detection_pb2 as tracffic_lights_info
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle
from modules.routing.proto import routing_pb2 as routing_info
from modules.routing.proto.routing_pb2 import RoutingRequest
from modules.planning.proto.planning_status_pb2 import LaneBorrowManual
from cyber.proto.run_mode_conf_pb2 import DebugMsg
import modules.map.proto.map_parking_space_pb2 as map_info
from modules.common.proto.map_name_pb2 import MapName
from modules.common.proto.geometry_pb2 import Point3D
import time
#reader type
from modules.control.proto.control_cmd_pb2 import ControlCommand 
import numpy as np

from udi_simulation_bridge.node import CyberNode

# pip install squaternion
from squaternion import Quaternion

import math
import numpy
from oasis import ScenarioInfoInterfaces
from oasis import PNCInterfaces
from oasis import CommonInterfaces
from oasis import ObjectDetectionSensor
from oasis import SensorInterfaces
# from oasis import VehicleControl
from threading import Thread

chassis_topic = "/apollo/canbus/chassis"
localization_estimate_topic ="/apollo/localization/pose"
localization_status_topic = "/apollo/localization/msf_status"
tf_topic = "/tf"
traffic_light_topic = "/apollo/perception/traffic_light"
perception_obstacles_topic = "/apollo/perception/obstacles"
routing_response_topic = "/apollo/routing_response"
routing_request_topic = "/apollo/routing_request"
lane_borrow_manual_topic = "lane_borrow_manual"
debug_msg_topic = "/apollo/cyber/debug_mode"
control_command_topic = "/apollo/control"
map_name_topic =  "/apollo/map_name"

PUBLISH_RATE_PERCEPTION = 10 #perception publish rate
PUBLISH_RATE_REQUEST =10 # routing request publish rate
PUBLISH_RATE_LOCALIZATION = 100
CHASSIS_PUBLISH_RATE = 100
TRAFFICLIGHT_PUBLISH_RATE = 100

class Adapter(CyberNode):
    def __init__(self, host='localhost', port=2000, task_info={}, goal=None, sync=True) -> None:
        super().__init__("adapter")
        self.task_info = task_info
        self.sim = oasis.OasisSim(host=host, port=port,task_info = task_info)
        self.sim.set_world_tick()
        self.ego_vehicle = self.sim.get_ego_vehicle()
        self.pnc = PNCInterfaces(self.sim)
        self.trans = CommonInterfaces(self.ego_vehicle)
        self.scenario = ScenarioInfoInterfaces(self.sim)
        time.sleep(1)
        self.sensor = oasis.SensorInterfaces(self.sim)
        self.object_detection = self.sensor.get_object_detection_sensor("target-Yf7jnVsXYpbV9NsVw8S4Gz")
        self.right_turn_ratio = 0.7
        self.left_turn_ratio = 0.85

        self.traffic_light_detection_writer = self.new_writer(traffic_light_topic,TrafficLightDetection,qos_depth=10)
        self.perception_obstacles_writer = self.new_writer(perception_obstacles_topic,PerceptionObstacles,qos_depth=10)
        self.map_name_writer = self.new_writer(map_name_topic,MapName,qos_depth=10)
        # self.routing_response_writer = self.new_writer(routing_response_topic,routing_info.RoutingResponse,qos_depth=10)
        self.routing_request_writer = self.new_writer(routing_request_topic,RoutingRequest,qos_depth=10)
        # self.lane_borrow_manual_writer = self.new_writer(lane_borrow_manual_topic,LaneBorrowManual,qos_depth=10)
        self.vehicle_chassis_writer = self.new_writer(chassis_topic,Chassis,qos_depth=10)
        self.vehicle_pose_writer = self.new_writer(localization_estimate_topic,LocalizationEstimate,qos_depth=10)
        self.localization_status_writer = self.new_writer(localization_status_topic,LocalizationStatus,qos_depth=10)
        self.tf_writer = self.new_writer(tf_topic, TransformStampeds)
       
        self.control_reader = self.new_reader(control_command_topic,ControlCommand,self.control_command_updated)

    def control_command_updated(self,cyber_vehicle_control):
        ego_throttle = cyber_vehicle_control.throttle / 100.0
        ego_brake = cyber_vehicle_control.brake / 100.0
        ego_rate = cyber_vehicle_control.steering_rate / 100.0

        if cyber_vehicle_control.steering_target < 0:
            cyber_vehicle_control.steering_target = cyber_vehicle_control.steering_target * self.right_turn_ratio
        else:
            cyber_vehicle_control.steering_target = cyber_vehicle_control.steering_target * self.left_turn_ratio

        ego_steer = -cyber_vehicle_control.steering_target / 100.0

        ego_hand_brake = cyber_vehicle_control.parking_brake
        ego_reverse = cyber_vehicle_control.gear_location == Chassis.GearPosition.GEAR_REVERSE
        if self.ego_vehicle:
            self.pnc.set_ego_control(ego_throttle,ego_steer,ego_brake,ego_hand_brake,ego_reverse,False,0)

    def send_chassis_msgs(self):
        """ 
        send messages related to vehicle status

        :return:
        """
        # print("task info ",len( self.task_info))
        # for info in self.task_info:
        #     print(info)
        rate = cyber_time.Rate(CHASSIS_PUBLISH_RATE)
        while not self.cyber.is_shutdown():
            print("chassis")
            self.write_chassis()           
            rate.sleep()

    def send_localization_msgs(self):
        """ 
        send messages related to vehicle status

        :return:
        """
        # print("task info ",len( self.task_info))
        # for info in self.task_info:
        #     print(info)

        # print("localization ")
        rate = cyber_time.Rate(PUBLISH_RATE_LOCALIZATION)
        while not self.cyber.is_shutdown():
            print("localization")
            self.write_localization()
            rate.sleep()

    def send_perception_msgs(self):
        """ 
        send messages related to vehicle status

        :return:
        """
        # print("task info ",len( self.task_info))
        # for info in self.task_info:
        #     print(info)
        num = 0
        rate = cyber_time.Rate(PUBLISH_RATE_PERCEPTION)
        while not self.cyber.is_shutdown():
            num+=1
            self.write_perception_obstacles_msg(num)
            # print("perception")
            rate.sleep()
            

    def send_traffic_light_msgs(self):
        """ 
        send messages related to vehicle status

        :return:
        """
        rate = cyber_time.Rate(TRAFFICLIGHT_PUBLISH_RATE)
        num = 0
        while not self.cyber.is_shutdown():
            self.write_traffic_light_msgs(num)
            print("traffic light")
            rate.sleep()

    def send_tf_msgs(self):
        """ 
        send messages related to vehicle status

        :return:
        """
        rate = cyber_time.Rate(20)
        while not self.cyber.is_shutdown():
            self.write_tf_msgs()
            rate.sleep()
    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        func = [
            self.write_routing_request,
            self.send_perception_msgs,
            self.send_chassis_msgs,
            self.send_localization_msgs,
            # self.send_traffic_light_msgs
            # self.send_tf_msgs,
        ]
        threads = []
        for f in func:
            t = Thread(target=f)
            threads.append(t)
            t.start()
        for t in threads:
            t.join()
        print("mission acomplished")
           

    def write_traffic_light_msgs(self,num):
        
        traffic_light_detection_msg = TrafficLightDetection()
        traffic_light_detection_msg.header.timestamp_sec = time.time()
        traffic_light_detection_msg.header.module_name = "traffic_light"
        traffic_light_detection_msg.header.sequence_num = num
        traffic_light_detection_msg.traffic_light.extend(self.get_traffic_light())
        # traffic_light_detection_msg.traffic_light_debug = self.get_traffic_light_debug()
        # traffic_light_detection_msg.contain_lights
        # traffic_light_detection_msg.camere_id
        self.traffic_light_detection_writer.write(traffic_light_detection_msg)
    
    def get_traffic_light_box(self):
        traffic_light_box = tracffic_lights_info.TrafficLightBox()
        return traffic_light_box
    def get_traffic_light_debug(self):
        traffic_light_debug = tracffic_lights_info.TrafficLightDebug()
        # traffic_light_debug.
        
        traffic_light_debug.cropbox = self.get_traffic_light_box()
        traffic_light_debug.box
        traffic_light_debug.signal_num
        traffic_light_debug.valid_pos
        traffic_light_debug.ts_diff_pos
        traffic_light_debug.ts_diff_sys
        traffic_light_debug.project_error
        traffic_light_debug.distance_to_stop_line
        traffic_light_debug.crop_roi
        traffic_light_debug.projected_roi
        traffic_light_debug.rectified_roi
        traffic_light_debug.debug_roi
        return traffic_light_debug
      
    def get_traffic_light(self):
        {
            'id': None, #int
            'state': None, #int
            'distance': None, #float
            'elapsed_time': None, #float
            'forward_angle': None, # float
            'traffic_light_num': None, #int
            'traffic_light_validity': None} #int
        traffic_data_list = self.object_detection.get_traffic_light_data()
        traffic_lights = []
        for traffic_data in traffic_data_list:
            traffic_light =  tracffic_lights_info.TrafficLight()
            if traffic_data.traffic_light_num != 0 and traffic_data.traffic_light_num is not None:
                print(f"traffic_data is: {vars(traffic_data)}")
                traffic_light.color 
                traffic_light.id  = traffic_data.id
                traffic_light.confidence
                traffic_light.tracing_time
                traffic_light.blink
                traffic_light.remaining_time
                traffic_lights.append(traffic_light)
        return traffic_lights
    
    def get_object_type(self,data_type,velocity):
        # enum Type {
        # UNKNOWN = 0;
        # UNKNOWN_MOVABLE = 1;
        # UNKNOWN_UNMOVABLE = 2;
        # PEDESTRIAN = 3;  // Pedestrian, usually determined by moving behavior.
        # BICYCLE = 4;     // bike, motor bike
        # VEHICLE = 5;     // Passenger car or truck.
        # };
        if "walker" in data_type:
            return 3
        elif "vehicle" in data_type:
            return 5
        elif velocity > 0.5:
            return 1
        else :
            return 2

    def get_percetion_obstacles(self):
        obstacles = []
        object_data_list = self.pnc.get_object_info()
        i = 0
        ego_pose = self.pnc.get_ego_vehicle_pose()
        for object_data in object_data_list:
            obstacle = PerceptionObstacle()
            probability = object_data.probability
            vertices_box = object_data.vertices_box
            center = object_data.center
            velocity = object_data.velocity
            rotation = object_data.rotation
            acceleration = object_data.acceleration
            size = object_data.size
            obstacle.id = int(object_data.carla_actor_id)
            i+=1
            data_type = object_data.name
            obstacle.type = self.get_object_type(data_type,velocity)
            obstacle.timestamp = time.time()
            obstacle.position.x = center.x
            obstacle.position.y = -center.y
            obstacle.position.z = 0
            obstacle.theta = -rotation.yaw/180*math.pi
            obstacle.velocity.x =velocity.x
            obstacle.velocity.y = -velocity.y
            obstacle.velocity.z = 0
            obstacle.length = size.x
            obstacle.width = size.y
            obstacle.height = size.z
            obstacle.acceleration.x = acceleration.x
            obstacle.acceleration.y = -acceleration.y
            obstacle.acceleration.z = acceleration.z

            curr_yaw = rotation.yaw/180*math.pi
            ego_yaw = ego_pose[1].yaw/180*math.pi
            l = obstacle.length
            w = obstacle.width
            #仿真器的vertices_box中的多边形定点有问题，所以自己构造多边形
            #多边形的构造需要在车辆坐标系下，所以yaw = 障碍物在世界坐标系下的yaw角+ego_yaw
            yaw  =  - curr_yaw           
            dx1 = math.cos(yaw) * l
            dy1 = math.sin(yaw) * l
            dx2 = math.sin(yaw) * w
            dy2 = -math.cos(yaw) * w
            polygon_points = []
            delta_x = obstacle.position.x
            delta_y = obstacle.position.y
            pose = Point3D()
            pose.x = delta_x + dx1 + dx2 
            pose.y = delta_y + dy1 + dy2
            pose.z = 0
            polygon_points.append(pose)

            pose = Point3D()
            pose.x = delta_x + dx1 - dx2 
            pose.y = delta_y + dy1 - dy2
            pose.z = 0
            polygon_points.append(pose)

            pose = Point3D()
            pose.x = delta_x - dx1 - dx2 
            pose.y = delta_y - dy1 - dy2
            pose.z = 0
            polygon_points.append(pose)

            pose = Point3D()
            pose.x = delta_x - dx1 + dx2 
            pose.y = delta_y - dy1 + dy2
            pose.z = 0
            polygon_points.append(pose)
            obstacle.polygon_point.extend(polygon_points)
            obstacles.append(obstacle)
        return obstacles

    def write_perception_obstacles_msg(self,num):
        perception_msg = PerceptionObstacles()
        perception_msg.perception_obstacle.extend(self.get_percetion_obstacles())
        perception_msg.header.timestamp_sec = time.time()
        perception_msg.header.module_name = "perception_obstacle"
        perception_msg.header.sequence_num = num
        # perception_msg.error_code
        # perception_msg.lane_marker
        # perception_msg.cipv_info
        self.perception_obstacles_writer.write(perception_msg)


    def transform_obstacle_to_car_coordinates(self,car_world, yaw_degrees,obstacle_world):
        """
        将障碍物坐标从世界坐标系变换到汽车坐标系。
        坐标系中的点不动则：
        右手坐标系绕z轴逆时针旋转变换矩阵： carla世界是右手坐标系
        
        [   cos(a)  sin(a)   0
            -sin(a) cos(a)   0
            0       0        1
        ]
        左手坐标系绕z轴顺时针旋转变换矩阵：
        
        [   cos(a)  sin(a)   0
            -sin(a) cos(a)   0
            0       0        1
        ]


        参数：
        - obstacle_world：障碍物在世界坐标系下的坐标（形如 [x, y, z]）
        - car_world：车辆在世界坐标系下的坐标（形如 [x, y, z]）
        - yaw_degrees：汽车坐标系下的yaw角度（单位：度）

        返回：
        - 障碍物在汽车坐标系下的坐标
        """
        # 将yaw角度转换为弧度
        yaw = np.deg2rad(yaw_degrees)

        # 计算旋转矩阵
        rotation_matrix = np.array([
            [np.cos(yaw), np.sin(yaw)],
            [-np.sin(yaw), np.cos(yaw)]
            ])

        # 变换障碍物坐标到世界坐标系下
        relative_position =   car_world -obstacle_world

        # 变换障碍物坐标到汽车坐标系下
        obstacle_car = np.dot(rotation_matrix, relative_position)

        return obstacle_car
    #get waypoint
    def get_misiion(self):

        mission_points = []
        start_point =self.scenario.get_start_position()
        lane_waypoint_start = routing_info.LaneWaypoint()
        # lane_waypoint_start.id = start_point.get('laneid')
        # lane_waypoint_start.s = start_point.get("s")
        lane_waypoint_start.pose.x  = float(start_point.get('x'))
        lane_waypoint_start.pose.y  = float( start_point.get('y'))
        # lane_waypoint_start.pose.z  = float( start_point.get('z'))
        lane_waypoint_start.heading = float(start_point.get('h'))
        mission_points.append(lane_waypoint_start)
        end_point = self.scenario.get_end_position()
        lane_waypoint_end = routing_info.LaneWaypoint()
        # lane_waypoint_end.id = end_point.get('laneid')
        # lane_waypoint_end.s = end_point.get("s")
        lane_waypoint_end.pose.x  = float(end_point.get('x'))
        lane_waypoint_end.pose.y  = float(end_point.get('y'))
        # lane_waypoint_end.pose.z  = float(end_point.get('z'))
        lane_waypoint_end.heading = float(end_point.get('h'))
        mission_points.append(lane_waypoint_end)
        return mission_points

    #get segment
    def get_lane_segments(self):
        lane_segment = routing_info.LaneSegment()
        lane_segment.id
        lane_segment.start_s
        lane_segment.end_s
        return lane_segment
    
    #get parking space
    def get_parking_space(self):
        parking_space = map_info.ParkingSpace()
        parking_space.id
        parking_space.polygon
        parking_space.overlap_id #repeated
        parking_space.heading
        return parking_space

    def get_parking_info(self):
        parking_info = routing_info.ParkingInfo()
        parking_info.parking_space_id
        parking_info.parking_point
        parking_info.parking_space_type
        parking_info.corner_point

    def write_routing_request_msg(self,num):
        routing_request_msg = RoutingRequest()
        routing_request_msg.header.timestamp_sec = time.time()
        routing_request_msg.header.module_name = "routing_request"
        routing_request_msg.header.sequence_num = num+1
        routing_request_msg.waypoint.extend(self.get_misiion())
        # routing_request_msg.blacklisted_lane = self.get_lane_segments()
        # routing_request_msg.blacklisted_road
        # routing_request_msg.broadcast = True
        # routing_request_msg.parking_info
        # routing_request_msg.dead_end_info 
        self.routing_request_writer.write(routing_request_msg)

    def write_lane_borrow_manual_msg():
        lane_borrow_manual_msg = LaneBorrowManual()
        
    def write_debug_msg_msg():
        debug_msg = DebugMsg()

    def write_map_name(self):
        name = MapName()
        map_name =  self.scenario.get_map_name()
        #自定义导入的地图含有.xodr 去掉
        if ".xodr" in map_name:
            map_name =map_name.replace(".xodr","")
        name.map_name = map_name
        self.map_name_writer.write(name)

    def write_routing_request(self):
        num = 0
        rate = cyber_time.Rate(PUBLISH_RATE_REQUEST)
        while not self.cyber.is_shutdown():
            num+=1
            self.write_routing_request_msg(num)
            self.write_map_name()
            print("mission")
            rate.sleep()

    def write_chassis(self):
        vehicle_chassis = Chassis()
        vehicle_chassis.header.timestamp_sec = time.time()
        # print("time ; ", vehicle_chassis.header.timestamp_sec)
        vehicle_chassis.header.frame_id = 'ego_vehicle'
        vehicle_chassis.engine_started = True
        vehicle_chassis.speed_mps = self.pnc.get_ego_no_vector_speed()
        vehicle_chassis.throttle_percentage = self.pnc.get_ego_control().throttle * 100.0
        vehicle_chassis.brake_percentage = self.pnc.get_ego_control().brake * 100.0
        vehicle_chassis.steering_percentage = -self.pnc.get_ego_control().steer * 100.0
        vehicle_chassis.parking_brake = self.pnc.get_ego_control().hand_brake
        vehicle_chassis.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        self.vehicle_chassis_writer.write(vehicle_chassis)

        localization_status = LocalizationStatus()
        localization_status.header.timestamp_sec = time.time()
        localization_status.fusion_status = 0  # OK = 0
        localization_status.measurement_time = time.time()
        localization_status.state_message = ""
        self.localization_status_writer.write(localization_status)
  
    def write_tf_msgs(self):
        # message TransformStampeds {
        # optional apollo.common.Header header = 1;
        # repeated TransformStamped transforms = 2;
        # }

        pose = self.pnc.get_ego_vehicle_pose()
        tf_msgs = TransformStampeds()
        tf_msg = TransformStamped()
        tf_msg.header.timestamp_sec = time.time()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'localization'

        tf_msg.transform.translation.x = pose[0].x
        tf_msg.transform.translation.y = -pose[0].y
        tf_msg.transform.translation.z = pose[0].z
        orientation = Quaternion.from_euler(pose[1].roll,pose[1].pitch,-pose[1].yaw) #(cyber_pose[1])
        # orientation = Quaternion.from_euler(*pose[1])

        tf_msg.transform.rotation.qx = orientation.x
        tf_msg.transform.rotation.qy = orientation.y
        tf_msg.transform.rotation.qz = orientation.z
        tf_msg.transform.rotation.qw = orientation.w
        tf_msgs.header.timestamp_sec = time.time()
        tf_msgs.header.frame_id = 'world'
        tf_msgs.transforms.append(tf_msg)
        self.tf_writer.write(tf_msgs)

    def write_localization(self):
        transform = self.pnc.get_ego_vehicle_pose()

        linear_vel = self.pnc.get_ego_vector_speed()
        angular_vel = self.pnc.get_ego_angle_velocity()
        accel = self.pnc.get_ego_acceleration()

        localization_estimate = LocalizationEstimate()
        localization_estimate.header.timestamp_sec = time.time()
        localization_estimate.header.frame_id = 'novatel'

        # cyber_pose = self.trans.carla_transform_to_cyber_pose(transform)
        cyber_pose = transform
        localization_estimate.pose.position.x = cyber_pose[0].x
        localization_estimate.pose.position.y = -cyber_pose[0].y
        localization_estimate.pose.position.z = cyber_pose[0].z
        # print("x",cyber_pose[0].x," y ",-cyber_pose[0].y," z  ")

        roll = cyber_pose[1].roll
        pitch = cyber_pose[1].pitch
        # 按理说这个yaw角度应该取反，但是取反会有问题，车无法正常行驶
        yaw = -cyber_pose[1].yaw
        orientation = Quaternion.from_euler(roll,pitch,yaw) #(cyber_pose[1])
        localization_estimate.pose.orientation.qx = orientation.x
        localization_estimate.pose.orientation.qy = orientation.y
        localization_estimate.pose.orientation.qz = orientation.z
        localization_estimate.pose.orientation.qw = orientation.w

        # cyber_twist = self.trans.carla_velocity_to_cyber_twist(linear_vel, angular_vel)
        localization_estimate.pose.linear_velocity.x = linear_vel.x
        localization_estimate.pose.linear_velocity.y = -linear_vel.y
        localization_estimate.pose.linear_velocity.z = linear_vel.z

        localization_estimate.pose.angular_velocity.x = angular_vel.x
        localization_estimate.pose.angular_velocity.y = -angular_vel.y
        localization_estimate.pose.angular_velocity.z = angular_vel.z

        # cyber_line_accel = self.trans.carla_acceleration_to_cyber_accel(accel)
        localization_estimate.pose.linear_acceleration.x = accel.x
        localization_estimate.pose.linear_acceleration.y = -accel.y
        localization_estimate.pose.linear_acceleration.z = accel.z
        # roll, pitch, yaw = self.trans.cyber_quaternion_to_cyber_euler(cyber_pose.orientation)

        enu_accel_velocity = self.n2b(pitch, roll, yaw, numpy.array([linear_vel.x,
                                                                  -linear_vel.y,
                                                                   linear_vel.z]))
        localization_estimate.pose.linear_acceleration_vrf.x = enu_accel_velocity[0, 0]
        localization_estimate.pose.linear_acceleration_vrf.y = enu_accel_velocity[0, 1]
        localization_estimate.pose.linear_acceleration_vrf.z = enu_accel_velocity[0, 2]

        enu_angular_velocity = self.n2b(pitch, roll, yaw, numpy.array([angular_vel.x,
                                                                     -angular_vel.y,
                                                                     angular_vel.z]))
        localization_estimate.pose.angular_velocity_vrf.x = enu_angular_velocity[0, 0]
        localization_estimate.pose.angular_velocity_vrf.y = enu_angular_velocity[0, 1]
        localization_estimate.pose.angular_velocity_vrf.z = enu_angular_velocity[0, 2]

        localization_estimate.pose.heading = math.radians(-transform[1].yaw)
        self.vehicle_pose_writer.write(localization_estimate)

        
    def n2b(self,x_radius, y_radius, z_radius, b):
        x_matrix = numpy.array(
            [
                [1, 0, 0],
                [0, numpy.cos(x_radius), -numpy.sin(x_radius)],
                [0, numpy.sin(x_radius), numpy.cos(x_radius)],
            ]
        )

        y_matrix = numpy.array(
            [
                [numpy.cos(y_radius), 0, numpy.sin(y_radius)],
                [0, 1, 0],
                [-numpy.sin(y_radius), 0, numpy.cos(y_radius)],
            ]
        )

        z_matrix = numpy.array(
            [
                [numpy.cos(z_radius), -numpy.sin(z_radius), 0],
                [numpy.sin(z_radius), numpy.cos(z_radius), 0],
                [0, 0, 1],
            ]
        )

        # conversion_matrix = numpy.matrix(z_matrix) * numpy.matrix(y_matrix) * numpy.matrix(x_matrix)
        # n = numpy.matrix(conversion_matrix.T) * numpy.matrix(numpy.array(b))

        n = (
            numpy.matrix(numpy.array(b))
            * numpy.matrix(x_matrix)
            * numpy.matrix(y_matrix)
            * numpy.matrix(z_matrix)
        )
        return n
    
    def print_object_detection_result(self,object_detection):
        if object_detection:
            object_data_list = object_detection.get_obj_data()
            for object_data in object_data_list:
                if object_data.object_id != 0 and object_data.object_id is not None:
                    print(f"object_data is: {vars(object_data)}")

            lane_data_list = object_detection.get_lane_data()
            for lane_data in lane_data_list:
                if lane_data.road_section_num != 0 and lane_data.road_section_num is not None:
                    print(f"lane_data is: {vars(lane_data)}")

            traffic_data_list = object_detection.get_traffic_light_data()
            for traffic_data in traffic_data_list:
                if traffic_data.traffic_light_num != 0 and traffic_data.traffic_light_num is not None:
                    print(f"traffic_data is: {vars(traffic_data)}")
    def run(self):

        self.update()

if __name__ == "__main__":
    # cyber.init('adaptor', anonymous=True)
    adapter = Adapter()
    adapter.run()
