#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool
import matplotlib.pyplot as plt

next_state = 0.0
depth = 0.0
desire_state = 0
pid_enable = 0
#depth_plot = []
#thrust_plot = []
time_counter = 1
state =PID(1, 0.1, 0.05, setpoint=0)

class PID_Depth_Controll_Pub(Node):
    def __init__(self):
        global state
        state.output_limits = (-1, 1)
        super().__init__('PID_Depth_Controll_publisher')
        self.sub = self.create_subscription(Float64, 'depth', self.updatedepth, 10)
        self.sub1 = self.create_subscription(Float64, 'desire_state', self.updatedesirestate, 10)
        self.sub2 = self.create_subscription(Bool, 'PID_enable', self.undatepidtoggle, 10)
        self.pub = self.create_publisher(Float64, 'PID_Depth_Controll_Thrust', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        global next_state
        global depth
        global pid_enable
        global state
        global time_counter
        
        print(f"next_state{next_state}, depth{depth}, pid_enable{pid_enable}, time_counter{time_counter}")

        # Assume we have a system we want to control in controlled_system
        #v = controlled_system.update(0)

        if pid_enable:
            #depth_plot.append(depth)
            # Compute new output from the PID according to the systems current value
            next_state = state.__call__(depth, time_counter)
            #thrust_plot.append(next_state)
            
            time_counter += 1
            # Feed the PID output to the system and get its current value

        """ if pid_enable: 
            plt.plot(depth_plot, color = 'r', label = 'depth')
            plt.plot(thrust_plot, color = 'g', label = 'thrust')
            plt.legend()
            plt.show """
        
        msg = Float64()
        msg.data = float(next_state)
        self.pub.publish(msg)
        
    def updatedepth(self, msg):
        global depth
        depth = msg.data
        
    def updatedesirestate(self, msg):
        global desire_state
        global state
        state.setpoint = desire_state
        desire_state = msg.data
        
    def undatepidtoggle(self, msg):
        global pid_enable
        pid_enable = msg.data
        
    
def main(args=None):
    rclpy.init(args = args)
    publisher = PID_Depth_Controll_Pub()
    
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()	