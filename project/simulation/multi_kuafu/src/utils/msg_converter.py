#!/usr/bin/env python

from matplotlib.font_manager import json_dump
import rospy
import json
from rospy_message_converter import json_message_converter
from rambot_msgs.msg import BehaviourArray
is_called = False
def callback(data):
    json_data = json_message_converter.convert_ros_message_to_json(data)
    with open("test.json", 'w') as f:
        f.write(json.dumps(json_data, indent=4))
    global is_called
    is_called = True

def main():
    rospy.init_node('ros2json', anonymous=True)
    rospy.Subscriber("/opendrive_behaviour_planner/behaviour_array", BehaviourArray, callback)
    global is_called
    while not is_called:
        rospy.sleep(0.1)
    rospy.signal_shutdown("no reason")


if __name__ == '__main__':
    main()