// Copyright (c) 2022, Unity-Drive Inc. All rights reserved.
// Author: Zhenyu Li lizhenyu@unity-drive.com

#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <iostream>

#include "ackermann_msgs/AckermannDrive.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#include "carla_msgs/CarlaTrafficLightStatus.h"
#include "carla_msgs/CarlaTrafficLightStatusList.h"
#include "carla_msgs/CarlaWorldInfo.h"
#include "derived_object_msgs/ObjectArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "perception_msgs/PerceptionFrame.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Bool.h"
#include "udi_msgs/Chassis.h"
#include "udi_msgs/ControlCommand.h"
#include "udi_msgs/LocalizationEstimate.h"
#include "udi_msgs/PerceptionObject.h"
#include "udi_msgs/Point3D.h"
#include "udi_msgs/PredictionObstacle.h"
#include "udi_msgs/PredictionObstacles.h"
#include "udi_msgs/TrafficLightDetection.h"
#include "udrive_msgs/ControlCmd.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

ros::Subscriber obs_sub;
ros::Subscriber traffic_light_sub;
ros::Subscriber control_sub;
ros::Subscriber odom_sub;
// init. pub.
ros::Publisher localization_pub;
ros::Publisher localization_lost_pub;
ros::Publisher localization_2_pub;
// ros::Publisher chassis_pub;
ros::Publisher obs_pub;
ros::Publisher traffic_light_pub;
ros::Publisher control_pub;

// init. CARLA topic
const std::string CARLA_ODOMETRY_TOPIC;
const std::string CARLA_WAYPOINTS_TOPIC;
const std::string CARLA_VEHICLE_INFO_TOPIC;
const std::string CARLA_VEHICLE_STATUS_TOPIC;
const std::string CARLA_MAP_TOPIC;
const std::string CARLA_OBSTACLES_TOPIC;
const std::string CARLA_TRAFFIC_LIGHT_TOPIC;
const std::string CARLA_CONTROL_TOPIC;

const std::string UDI_LOCALIZATION_TOPIC;
const std::string UDI_LOCALIZATION_LOST_TOPIC;
const std::string UDI_LOCALIZATION_TOPIC_2;
const std::string UDI_CHASSIS_TOPIC;
const std::string UDI_OBSTACLES_TOPIC;
const std::string UDI_TRAFFIC_LIGHT_TOPIC;
const std::string UDI_CONTROL_TOPIC;

void odom_callback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  udi_msgs::LocalizationEstimate udi_local_msg;
  udi_local_msg.header.frame_id = "odometry";
  udi_local_msg.header.timestamp_sec = ros::Time::now().toSec();
  // local_msg.header.sequence_num =

  udi_local_msg.pose.position.x = odometry_msg->pose.pose.position.x;
  udi_local_msg.pose.position.y = odometry_msg->pose.pose.position.y;
  udi_local_msg.pose.position.z = odometry_msg->pose.pose.position.z;
  udi_local_msg.pose.orientation.qx = odometry_msg->pose.pose.orientation.x;
  udi_local_msg.pose.orientation.qy = odometry_msg->pose.pose.orientation.y;
  udi_local_msg.pose.orientation.qz = odometry_msg->pose.pose.orientation.z;
  udi_local_msg.pose.orientation.qw = odometry_msg->pose.pose.orientation.w;

  udi_local_msg.pose.linear_velocity_vrf.x = odometry_msg->twist.twist.linear.x;
  udi_local_msg.pose.linear_velocity_vrf.y = odometry_msg->twist.twist.linear.y;
  udi_local_msg.pose.linear_velocity_vrf.z = odometry_msg->twist.twist.linear.z;

  udi_local_msg.pose.angular_velocity_vrf.x =
      odometry_msg->twist.twist.angular.x;
  udi_local_msg.pose.angular_velocity_vrf.y =
      odometry_msg->twist.twist.angular.y;
  udi_local_msg.pose.angular_velocity_vrf.z =
      odometry_msg->twist.twist.angular.z;

  udi_local_msg.pose.linear_acceleration_vrf.x =
      odometry_msg->twist.twist.linear.x;
  udi_local_msg.pose.linear_acceleration_vrf.y =
      odometry_msg->twist.twist.linear.y;

  udi_local_msg.pose.heading =
      tf::getYaw(odometry_msg->pose.pose.orientation) * 0.175 * 9;

  // std::cout<<"h"
  // local_msg.Uncertainty
  // local_msg.LatencyStats
  // local_msg.ErrorCode
  localization_pub.publish(udi_local_msg);
  localization_2_pub.publish(*odometry_msg);
}

void tranformCarlaObject2UdiPredicionObstacle(
    const derived_object_msgs::ObjectArrayConstPtr& object_array_msg,
    udi_msgs::PredictionObstacles* udi_pred_obs_msg) {
  udi_pred_obs_msg->header.frame_id = "pred_obs";
  udi_pred_obs_msg->header.timestamp_sec = ros::Time::now().toSec();

  for (auto& item : object_array_msg->objects) {
    udi_msgs::PredictionObstacle pred_ob;
    auto& perc_obj = pred_ob.perception_object;
    perc_obj.timestamp = item.header.stamp.sec;
    perc_obj.id = item.id;
    perc_obj.position.x = item.pose.position.x;
    perc_obj.position.y = item.pose.position.y;
    perc_obj.position.z = item.pose.position.z;
    auto heading = tf::getYaw(item.pose.orientation);
    perc_obj.theta = heading;
    perc_obj.is_static = true;
    double obs_vel = std::hypot(item.twist.linear.x, item.twist.linear.y);
    if (fabs(obs_vel) > 0.2) {
      perc_obj.is_static = false;
    }
    perc_obj.type = 0;  // temporaly set TYPE_UNKNOWN_STATIC
    perc_obj.velocity.x = item.twist.linear.x;
    perc_obj.velocity.y = item.twist.linear.y;
    perc_obj.length = item.shape.dimensions[0];
    perc_obj.width = item.shape.dimensions[1];
    perc_obj.height = item.shape.dimensions[2];
    // transform shape into polygon_point
    if (item.shape.type == shape_msgs::SolidPrimitive::BOX) {
      for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
          udi_msgs::Point3D tmp;
          auto relative_x = pow(-1, i) * item.shape.dimensions[0];
          auto relative_y = pow(-1, i) * pow(-1, j) * item.shape.dimensions[1];
          tmp.x = relative_x * cos(heading) - relative_y * sin(heading) +
                  item.pose.position.x;
          tmp.y = relative_x * sin(heading) + relative_y * cos(heading) +
                  item.pose.position.y;
          perc_obj.polygon_point.push_back(tmp);
        }
      }
    } else {
      double radius;
      if (item.shape.type == shape_msgs::SolidPrimitive::SPHERE) {
        radius = item.shape.dimensions[0];
      } else {
        radius = item.shape.dimensions[1];
      }
      for (double i = 0.0; i < 2 * M_PI; i += M_PI_4) {
        udi_msgs::Point3D tmp;
        tmp.x = item.pose.position.x + radius * cos(i);
        tmp.y = item.pose.position.y + radius * sin(i);
        perc_obj.polygon_point.push_back(tmp);
      }
    }

    udi_pred_obs_msg->prediction_obstacle.push_back(pred_ob);
  }
}

void tranformCarlaObject2PerceptionFrame(
    const derived_object_msgs::ObjectArrayConstPtr& object_array_msg,
    perception_msgs::PerceptionFrame* percep_frame_msg) {
  // percep_frame_msg->header.frame_id = "percep_obs";
  // percep_frame_msg->header.timestamp_sec = ros::Time::now().toSec();

  // for (auto& item : object_array_msg->objects) {
  //   perception_msgs::Object udi_object;
  //   geometry_msgs::Point point()
  //   udi_object.polygon.append();
  // }
}

void obstacles_callback(
    const derived_object_msgs::ObjectArrayConstPtr& object_array_msg) {
  udi_msgs::PredictionObstacles udi_pred_obs_msg;
  tranformCarlaObject2UdiPredicionObstacle(object_array_msg, &udi_pred_obs_msg);
  obs_pub.publish(udi_pred_obs_msg);
}

void traffic_light_callback(
    const carla_msgs::CarlaTrafficLightStatusListConstPtr& carla_traffic_msg) {
  udi_msgs::TrafficLightDetection udi_traffic_msg;
  udi_traffic_msg.header.frame_id = "traffic_light";
  udi_traffic_msg.header.timestamp_sec = ros::Time::now().toSec();

  for (auto& item : carla_traffic_msg->traffic_lights) {
    udi_msgs::TrafficLight traffic_light;
    traffic_light.light_id = std::to_string(item.id);  // uint 32
    auto new_color = item.state + 1;
    if (new_color == 5) new_color = 0;
    traffic_light.color = new_color;
    udi_traffic_msg.traffic_light.push_back(traffic_light);
  }
  traffic_light_pub.publish(udi_traffic_msg);
}

const double deg_to_rad(const double deg) { return deg / 180.0 * 3.1415926; }

void ackermann_control_callback(const udrive_msgs::ControlCmdConstPtr& cmd) {
  ackermann_msgs::AckermannDrive ackermann_msg;
  ackermann_msg.steering_angle = deg_to_rad(cmd->front_wheel_angle);
  ackermann_msg.steering_angle_velocity = 0.0;
  ackermann_msg.speed = cmd->speed_mps;
  ackermann_msg.acceleration = 0.0;  // cmd->acceleration;
  ackermann_msg.jerk = 0.0;

  control_pub.publish(ackermann_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "remap_interface");
  ros::NodeHandle nh;
  std::string prefix;
  bool enable_perfect_perception;
  ros::param::get("tf_prefix", prefix);
  ros::param::get("enable_perfect_perception", enable_perfect_perception);
  if (prefix.size() == 0) prefix = "kuafu_0";

  const std::string CARLA_ODOMETRY_TOPIC = "/carla/" + prefix + "/odometry";
  const std::string CARLA_WAYPOINTS_TOPIC = "/carla/" + prefix + "/waypoints";
  const std::string CARLA_VEHICLE_INFO_TOPIC =
      "/carla/" + prefix + "/vehicle_info";
  const std::string CARLA_VEHICLE_STATUS_TOPIC =
      "/carla/" + prefix + "/vehicle_status";
  const std::string CARLA_MAP_TOPIC = "/carla/world_info";
  const std::string CARLA_OBSTACLES_TOPIC = "/carla/" + prefix + "/objects";
  const std::string CARLA_TRAFFIC_LIGHT_TOPIC = "/carla/traffic_lights/status";
  const std::string CARLA_CONTROL_TOPIC = "/carla/" + prefix + "/ackermann_cmd";

  const std::string UDI_LOCALIZATION_TOPIC = "localization_estimate";
  const std::string UDI_LOCALIZATION_LOST_TOPIC = "localization/lost";
  const std::string UDI_LOCALIZATION_2_TOPIC = "current_odom";
  const std::string UDI_CHASSIS_TOPIC = "chassis";
  const std::string UDI_OBSTACLES_TOPIC = "prediction_obstacles";
  const std::string UDI_TRAFFIC_LIGHT_TOPIC = "trafficlight_detection";
  const std::string UDI_CONTROL_TOPIC = "auto/cmd_vel";

  // chassis_pub = nh.advertise<udi_msgs::Chassis>(UDI_CHASSIS_TOPIC, 1);
  traffic_light_pub =
      nh.advertise<udi_msgs::TrafficLightDetection>(UDI_TRAFFIC_LIGHT_TOPIC, 1);
  control_pub =
      nh.advertise<ackermann_msgs::AckermannDrive>(CARLA_CONTROL_TOPIC, 1);

  odom_sub = nh.subscribe(CARLA_ODOMETRY_TOPIC, 1, odom_callback);
  if (enable_perfect_perception) {
    obs_pub =
        nh.advertise<udi_msgs::PredictionObstacles>(UDI_OBSTACLES_TOPIC, 1);
    obs_sub = nh.subscribe(CARLA_OBSTACLES_TOPIC, 1, obstacles_callback);
  }

  traffic_light_sub =
      nh.subscribe(CARLA_TRAFFIC_LIGHT_TOPIC, 1, traffic_light_callback);
  control_sub = nh.subscribe(UDI_CONTROL_TOPIC, 1, ackermann_control_callback);

  // init. pub.
  localization_pub =
      nh.advertise<udi_msgs::LocalizationEstimate>(UDI_LOCALIZATION_TOPIC, 1);
  localization_lost_pub =
      nh.advertise<std_msgs::Bool>(UDI_LOCALIZATION_LOST_TOPIC, 1);
  localization_2_pub =
      nh.advertise<nav_msgs::Odometry>(UDI_LOCALIZATION_2_TOPIC, 1);

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    std_msgs::Bool localization_lost_data;
    localization_lost_data.data = false;
    localization_lost_pub.publish(localization_lost_data);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ros::spin();

  return 0;
}