import rclpy
from rclpy.node import Node
from offboard_control_pkg.msg import Drone1Info, Drone2Info, Drone3Info
 
class ControlNode(Node):

    # Colours needed for the LED control
    '''
    color_dict = {
            "off": LEDControl.COLOR_OFF,
            "red": LEDControl.COLOR_RED,
            "green": LEDControl.COLOR_GREEN,
            "blue": LEDControl.COLOR_BLUE,
            "yellow": LEDControl.COLOR_YELLOW,
            "purple": LEDControl.COLOR_PURPLE,
            "amber": LEDControl.COLOR_AMBER,
            "cyan": LEDControl.COLOR_CYAN,
            "white": LEDControl.COLOR_WHITE,
        } '''
    
    def __init__(self):
        super().__init__('Control_Node')
        
        #--------------------------------------------
        # TO DO: ONLY THING TO CHANGE MANUALLY! 
        # Update the sequence 
        #--------------------------------------------
        self.sequence_number = 1  # Default sequence number
        
        # Use px4 namespace for the vehicles
        self.leader = None
        self.follower_1 = 'px4_2'
        self.follower_2 = 'px4_3'
        #--------------------------------------------
        # PUBLISHERS 

        # DRONE 1 /px4_1
        self.publisher_drone_1 = self.create_publisher(Drone1Info, 'drone1_info_topic', 10)

        # DRONE 2 /px4_2
        self.publisher_drone_2 = self.create_publisher(Drone2Info, 'drone2_info_topic', 10)

        # DRONE 3 /px4_3
        self.publisher_drone_3 = self.create_publisher(Drone1Info, 'drone1_info_topic', 10)
        
        #--------------------------------------------
        # Timer to tick the loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def sequence_one(self, leader, follower_1, follower_2):
        msg1 = Drone1Info()
        msg1.follower = self.leader
        msg1.follower_number = 1
        msg1.light_colour = "green"
        self.publisher_drone_1.publish(msg1)

        pass

    def sequence_two(self, leader, follower_1, follower_2):
        pass 

    def control_loop(self):
        if self.sequence_number == 1:
            self.sequence_one(self.leader, self.follower_1, self.follower_2)

        elif self.sequence_number == 2:
            self.sequence_two(self.leader, self.follower_1, self.follower_2)

        else:
            self.get_logger().info("Invalid sequence number")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()