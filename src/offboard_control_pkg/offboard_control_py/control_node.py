import rclpy
from rclpy.node import Node
from offboard_control_pkg.msg import Drone1Info, Drone2Info, Drone3Info
import numpy as np

class ControlNode(Node):
    def __init__(self):
        super().__init__('Control_Node')
        
        #--------------------------------------------
        # FOR THE USER
        # MANUALLY SET THE FOLLOWER AND LEADER DRONES
        #--------------------------------------------
        self.relationship_matrix = np.array([1, None, None], [2, 1, 1], [3, 2, 1])
        
        # self.relationship_matrix = np.array([1, None, None], [2, 1, 1], [3, 1, 2])
        #--------------------------------------------
        # PUBLISHERS 

        # DRONE 1 /px4_1
        self.publisher_drone_1 = self.create_publisher(Drone1Info, 'drone1_info_topic', 10)

        # DRONE 2 /px4_2
        self.publisher_drone_2 = self.create_publisher(Drone2Info, 'drone2_info_topic', 10)

        # DRONE 3 /px4_3
        self.publisher_drone_3 = self.create_publisher(Drone3Info, 'drone3_info_topic', 10)

        #--------------------------------------------
        self.namespace = ['px4_1', 'px4_2', 'px4_3']
        self.drone_msg_classes = [Drone1Info, Drone2Info, Drone3Info]
        # Timer to tick the loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def create_drone_msg(self, drone_index, matrix, seq_colours):
        MsgClass = self.drone_msg_classes[drone_index]
        msg = MsgClass()
        msg.follower_number = matrix[drone_index][2]
        
        # If the drone is a leader, it has no follower
        if matrix[drone_index][1] is None:
            msg.follower = None
            msg.light_colour = seq_colours[0]
        
        # If the drone is a follower, assign its leader & colour
        else:
            index = matrix[drone_index][1]
            msg.light_colour = seq_colours[index]
            msg.follower = self.namespace(index - 1)
        return msg

    def info_publisher(self, sequence, matrix):
        # Colours needed for the LED control
        ''' color_dict = {
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

        if matrix[0][2] != matrix[1][2] and matrix[0][2] != matrix[2][2] \
            and matrix[1][2] != matrix[2][2]:
            seq_colours = ['purple', 'amber', 'amber']
        else:
            seq_colours = ['cyan', 'green', 'red']

        for i in range(3):
            drone_msg = self.create_drone_msg(i, matrix, seq_colours)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()