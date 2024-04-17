"""
    这个文件需要放到算法侧
"""
import sys
from oasis.agent.ad_agent import AdAgent
import rospy
import roslaunch

class MyAgent(AdAgent):
    def __init__(self, **kwargs):
        super(MyAgent, self).__init__(**kwargs)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.ad_launch_file_path = ["../launch/my_agent.launch", ] # launch 文件路径 字符串
        self.pure_pursuit_launch_file_path = [] # launch 文件的路径 字符串
        self.pure_pursuit_launch = None
        self.ad_launch = None

    def launch(self, *args):
        rospy.loginfo("Launch ad_agent")
        town = args[0] if len(args) > 0 else "Town01"
        cli_args = ["../launch/my_agent.launch", f'town:={town}']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(cli_args[0], roslaunch_args)]
        self.ad_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        if self.ad_launch is None:
            rospy.logerr("Failed to launch ad_agent")
            return False
        self.ad_launch.start()
        rospy.loginfo("Launch ad_agent")
        return True

    def launch_pure_pursuit(self, *args):
        rospy.loginfo("Launch pure_pursuit")
        self.pure_pursuit_launch = roslaunch.parent.ROSLaunchParent(self.uuid, self.pure_pursuit_launch_file_path)
        if self.pure_pursuit_launch is None:
            rospy.logerr("Failed to launch pure_pursuit")
            return False
        self.pure_pursuit_launch.start()
        rospy.loginfo("Launch pure_pursuit")
        return True

    def shutdown(self, *args):
        self.ad_launch.shutdown()
        rospy.loginfo("Shutdown ad_agent")
        return True

    def shutdown_pure_pursuit(self, *args):
        self.pure_pursuit_launch.shutdown()
        rospy.loginfo("Shutdown pure_pursuit")
        return True

if __name__ == '__main__':
    # 启动后, 在 OasisAgent 中使用 self.ad_agent_server.method() 可以调到这里的方法
    # 除了固定的 launch 和 shutdown, 还可以实现一些其他的方法, 在 OasisAgent 中可以调用到
    MyAgent(host="localhost", port=23456).run()