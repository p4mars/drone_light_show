#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from offboard_control_interfaces.msg import Drone1Info, Drone2Info, Drone3Info

import numpy as np

class ControlNode(Node):
    def __init__(self):
        super().__init__('Control_Node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        #--------------------------------------------
        # FOR THE USER
        # MANUALLY SET THE FOLLOWER AND LEADER DRONES
        #--------------------------------------------
        # [Who is the leader, which follower, sequence number in the line]
        self.relationship_matrix = np.array([[0, 0, 1], [1, 1, 2], [2, 1, 3]])
        
        # self.relationship_matrix = np.array([1, None, None], [2, 1, 1], [3, 1, 2])
        #--------------------------------------------
        # PUBLISHERS 

        # DRONE 1 /px4_1
        self.publisher_drone_1 = self.create_publisher(Drone1Info, 'drone1_info_topic', qos_profile)

        # DRONE 2 /px4_2
        self.publisher_drone_2 = self.create_publisher(Drone2Info, 'drone2_info_topic', qos_profile)

        # DRONE 3 /px4_3
        self.publisher_drone_3 = self.create_publisher(Drone3Info, 'drone3_info_topic', qos_profile)

        #--------------------------------------------
        self.namespace = ['px4_1', 'px4_2', 'px4_3']

        self.drone_msg_classes = [Drone1Info, Drone2Info, Drone3Info]

        # Timer to tick the loop
        self.timer = self.create_timer(0.1, self.info_publisher)
        

    def create_drone_msg(self, drone_index, matrix, seq_colours):

        # Create the message for the drone

        #### This has to bedone manually, since there is no other way (?) to dynamically instatiate the message classes
        if drone_index == 0:
            MsgClass = Drone1Info()
        elif drone_index == 1:
            MsgClass = Drone2Info()
        elif drone_index == 2:
            MsgClass = Drone3Info()

        ### assign the instantiated message class to the universal variable
        msg = MsgClass    

        ### explicitly state the follower number as int in order to avoid an assertion error
        msg.follower_number = int(matrix[drone_index][1])

        ### Initialise all fields in the msg class
        msg.follower = ""
        msg.light_colour = seq_colours[0]

        
        # If the drone is a leader, it is not following anything
        if matrix[drone_index][0] == 0:
            msg.follower = "" ### Assign an empty value if the drone does not follow anything
            msg.light_colour = seq_colours[0]
        
        # If the drone is a follower, assign its leader & colour
        else:
            index = matrix[drone_index][0]
            msg.light_colour = seq_colours[matrix[drone_index][2]-1]
            msg.follower = self.namespace[index - 1] # Adjust for zero-based index

        # Publish the message to the corresponding drone topic
        if drone_index == 0:
            self.publisher_drone_1.publish(msg)
            self.get_logger().info(f"Publishing to Drone 1: {msg}")

        elif drone_index == 1:
            self.publisher_drone_2.publish(msg)
            self.get_logger().info("Publishing to Drone 2")

        elif drone_index == 2:
            self.publisher_drone_3.publish(msg)
            self.get_logger().info("Publishing to Drone 3")
        return 

    def info_publisher(self):
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
        matrix = self.relationship_matrix
        # Create a sequence of colours for the drone
        if matrix[0][2] != matrix[1][2] and matrix[0][2] != matrix[2][2] \
            and matrix[1][2] != matrix[2][2]:
            seq_colours = ['purple', 'amber', 'amber']
        else:
            seq_colours = ['cyan', 'green', 'red']

        for i in range(3):
            self.create_drone_msg(i, matrix, seq_colours)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()