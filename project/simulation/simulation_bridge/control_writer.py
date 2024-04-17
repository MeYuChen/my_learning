import sys
from oasis import OasisSim
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.control.proto.control_cmd_pb2 import ControlCommand 
from udi_simulation_bridge.node import CyberNode

class cmd_writer(CyberNode):

    def __init__(self) -> None:
        super().__init__("concmd")
        # self.vehicle_info_writer = node.create_writer(
        #     "/apollo/vehicle_info",
        #     CarlaEgoVehicleInfo,
        #     qos_depth=10)
        self.control_writer = self.new_writer(
            "/apollo/control",
            ControlCommand,qos_depth=10)
    def write_control_test(self):
        vehicle = ControlCommand()
        vehicle.header.timestamp_sec = self.get_time()
        vehicle.header.frame_id = 'ego_vehicle_control'
        vehicle.throttle =100
        vehicle.brake = 0
        vehicle.steering_target = 0
        self.control_writer.write(vehicle)
    def run(self):
        rate = cyber_time.Rate(20)
        while not self.cyber.is_shutdown():
            self.write_control_test()
            rate.sleep()
if __name__ == "__main__":
    # cyber.init('adaptor', anonymous=True)
    controller = cmd_writer()
    controller.run()