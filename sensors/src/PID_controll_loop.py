#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool

next_state = 0
depth = 0
desire_state = 0
pid_enable = 0

class PID_Depth_Controll_Pub(Node):
    def __init__(self):
        super().__init__('PID_Depth_Controll_publisher')
        self.sub = self.create_subscription(Float64, 'depth', self.updatedepth, 10)
        self.sub1 = self.create_subscription(Float64, 'desire_state', self.updatedesirestate, 10)
        self.sub2 = self.create_subscription(Bool, 'PID_enable', self.undatepidtoggle, 10)
        self.pub = self.create_publisher(Float64, 'PID_Depth_Controll_Thrust', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = next_state
        self.pub.publish(msg)
        
    def updatedepth(self, msg):
        depth = msg.data
        
    def updatedesirestate(self, msg):
        desire_state = msg.data
        
    def undatepidtoggle(self, msg):
        pid_enable = msg.data
        
        
state = PID(1, 0.1, 0.05, setpoint=desire_state)

# Assume we have a system we want to control in controlled_system
#v = controlled_system.update(0)

while pid_enable:
    # Compute new output from the PID according to the systems current value
    next_state = state.__call__(depth)
    
    # Feed the PID output to the system and get its current value
    

def main(args=None):
    rclpy.init(args = args)
    publisher = PID_Depth_Controll_Pub()
    
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()	