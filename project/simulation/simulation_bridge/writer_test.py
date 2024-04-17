from cyber.python.cyber_py3 import cyber

from modules.routing.proto.routing_pb2 import RoutingRequest
from modules.common.proto.map_name_pb2 import MapName
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
routing_request_topic = "/apollo/routing_request"
map_name_topic =  "/apollo/map_name"
chassis_topic = "/apollo/canbus/chassis"
localization_estimate_topic ="/apollo/localization/pose"
localization_status_topic = "/apollo/localization/msf_status"
tf_topic = "/tf"
traffic_light_topic = "/apollo/perception/traffic_light"
perception_obstacles_topic = "/apollo/perception/obstacles"
routing_response_topic = "/apollo/routing_response"
lane_borrow_manual_topic = "lane_borrow_manual"
debug_msg_topic = "/apollo/cyber/debug_mode"
control_command_topic = "/apollo/control"
class ApolloWriterTester:
    def __init__(self) -> None:
        self.node = cyber.Node("writer_tester")
        self.reouting_request_reader = self.node.create_reader(routing_request_topic,RoutingRequest,callback=self.routing_request_callback)
        self.map_name_reader = self.node.create_reader(map_name_topic,MapName,self.map_name_callback)
        self.perception_reader = self.node.create_reader(perception_obstacles_topic,PerceptionObstacles,self.perception_callback)
    def routing_request_callback(self, msg):
        # 处理接收到的 RoutingRequest 消息
        # print(f"Received RoutingRequest: {msg}")
        # module_name = str(msg.header.module_name)
        # name = module_name.split("in_",1)[1]
        # print("map name : ", module_name.split("in_",1)[1])
        # for point in msg.waypoint:
        #     print(point)
        pass
    def map_name_callback(self,msg):
        print("name: ",msg.map_name)
    
    def perception_callback(sef,obs):
        print("obs num : " ,len(obs.perception_obstacle) )
        for obj in obs.perception_obstacle:
            pose = obj.position
            print("x ",pose.x," y ",pose.y," z ",pose.z)

    def run(self):
        # self.node.spin()
        while True:
            pass

if __name__== "__main__":
    cyber.init()    
    tester = ApolloWriterTester()
    tester.run()
    cyber.shutdown()
