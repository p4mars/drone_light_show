#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import \
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, \
          VehicleGlobalPosition, LedControl, VehicleStatus
from offboard_control_interfaces.msg import Drone3Info
import numpy as np 

class Drone_Three(Node):
    def __init__(self) -> None:
        super().__init__('Drone_Three_Node')

        self.leader = "" #px4_2, px4_3
        self.follower_number = None

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Needed for frame transformation 
        #if self.leader == 0:
        #    pass
        #else:
        #    self.leader_gps = self.create_subscription(
        #    VehicleGlobalPosition, f'{self.follower}/fmu/out/vehicle_global_position',
        #    self.global_position_callback, qos_profile)
#
        #    self.leader_vehicle_local_position_subscriber = self.create_subscription(
        #    VehicleLocalPosition, f'{self.follower}/fmu/out/vehicle_local_position',
        #    self.vehicle_local_position_callback, qos_profile)
#
        #    self.leader_vehicle_local_position = VehicleLocalPosition()

        #---------------------------------------
        # Publishers
        #---------------------------------------
        self.light_control_publisher = self.create_publisher(
            LedControl, 'px4_3/fmu/in/led_control', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 'px4_3/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 'px4_3/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 'px4_3/fmu/in/vehicle_command', qos_profile)

        #---------------------------------------
        # Subscribers
        #---------------------------------------
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, 'px4_3/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        
        self.global_pos_subscriber = self.create_subscription(
            VehicleGlobalPosition, 'px4_3/fmu/out/vehicle_global_position',
            self.global_position_callback, qos_profile)
        
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, 'px4_1/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        
        self.custom_subscriber = self.create_subscription(Drone3Info, 'drone3_info_topic', \
                                                           self.listener_callback, qos_profile)
        
        #---------------------------------------
        # State variables
        #---------------------------------------
        self.custom_msg = Drone3Info()
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.leader_vehicle_local_position = self.vehicle_local_position  # Use own local position as
        self.global_pos = VehicleGlobalPosition()
        self.coordinate_transform = [0,0]  # Placeholder for coordinate transformation offsets
        self.is_landing_triggered = False 
        self.is_disarmed = False
        
        #----------------------------------------
        # Initialise variables
        #----------------------------------------
        self.offboard_setpoint_counter = 0 # To count time passed
        self.takeoff_height = -2.0 # positive downward!
        self.theta = 0.0
        self.position_change = 0
        self.all_close_counter = 0

        #----------------------------------------
        # Create a timer to publish control commands (10Hz)
        #----------------------------------------
        self.dt = 0.1  # 10Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        flame_path = [[304.67,1259.00], [274.00,1160.33], [274.00,1160.33],
             [274.00,1160.33], [266.67,1037.33], [266.33,1037.33],
             [266.00,1037.33], [279.00,953.00], [279.00,953.00],
             [279.00,953.00], [309.00,875.00], [309.00,875.00],
             [309.00,875.00], [347.00,808.00], [347.00,808.00],
             [347.00,808.00], [389.00,757.00], [389.00,756.00],
             [389.00,755.00], [437.00,710.00], [437.00,710.00],
             [437.00,710.00], [490.00,668.00], [491.00,666.00],
             [492.00,664.00], [549.00,627.00], [549.00,627.00],
             [549.00,627.00], [607.00,593.00], [607.00,593.00],
             [607.00,593.00], [668.00,564.00], [668.00,564.00],
             [668.00,564.00], [725.00,538.00], [727.00,538.00],
             [729.00,538.00], [782.00,518.00], [784.00,517.00],
             [786.00,516.00], [843.00,501.00], [843.00,501.00],
             [843.00,501.00], [899.00,485.00], [899.00,485.00],
             [899.00,485.00], [961.00,469.00], [961.00,469.00],
             [961.00,469.00], [1032.00,449.00], [1032.00,449.00],
             [1032.00,449.00], [1117.00,421.00], [1117.00,421.00],
             [1117.00,421.00], [1180.00,397.00], [1180.00,397.00],
             [1180.00,397.00], [1239.00,360.00], [1239.00,360.00],
             [1239.00,360.00], [1281.00,329.00], [1281.00,329.00],
             [1281.00,329.00], [1313.00,323.00], [1313.00,323.00],
             [1313.00,323.00], [1322.00,347.00], [1322.00,348.00],
             [1322.00,349.00], [1307.00,376.00], [1306.00,378.00],
             [1305.00,380.00], [1274.00,406.00], [1272.00,408.00],
             [1270.00,410.00], [1226.00,441.00], [1225.00,441.00],
             [1224.00,441.00], [1174.00,478.00], [1174.00,480.00],
             [1174.00,482.00], [1110.00,525.00], [1106.00,527.00],
             [1102.00,529.00], [1039.00,575.00], [1037.00,578.00],
             [1035.00,581.00], [993.00,614.00], [991.00,616.00],
             [989.00,618.00], [951.00,657.00], [949.00,659.00],
             [947.00,661.00], [914.00,727.00], [914.00,727.00],
             [914.00,727.00], [905.00,777.00], [906.00,779.00],
             [907.00,781.00], [926.00,831.00], [928.00,834.00],
             [930.00,837.00], [969.00,855.00], [972.00,856.00],
             [975.00,857.00], [1030.00,865.00], [1037.00,862.00],
             [1044.00,859.00], [1094.00,838.00], [1095.00,838.00],
             [1096.00,838.00], [1152.00,806.00], [1152.00,805.00],
             [1152.00,804.00], [1186.00,791.00], [1186.00,791.00],
             [1186.00,791.00], [1202.00,815.00], [1203.00,816.00],
             [1204.00,817.00], [1200.00,849.00], [1197.00,854.00],
             [1194.00,859.00], [1171.00,895.00], [1169.00,898.00],
             [1167.00,901.00], [1145.00,947.00], [1145.00,948.00],
             [1145.00,949.00], [1094.00,1026.00], [1093.00,1028.00],
             [1092.00,1030.00], [1057.00,1080.00], [1053.00,1084.00],
             [1049.00,1088.00], [1006.00,1137.00], [1004.00,1141.00],
             [1002.00,1145.00], [962.00,1184.00], [959.00,1187.00],
             [956.00,1190.00], [893.00,1236.00], [891.00,1238.00],
             [889.00,1240.00], [840.00,1274.00], [836.00,1278.00],
             [832.00,1282.00], [760.00,1314.00], [759.00,1314.00],
             [758.00,1314.00], [693.00,1325.00], [691.00,1327.00],
             [689.00,1329.00], [647.00,1322.00], [647.00,1322.00],
             [647.00,1322.00], [642.00,1294.00], [642.00,1293.00],
             [642.00,1292.00], [670.00,1271.00], [672.00,1269.00],
             [674.00,1267.00], [708.00,1250.00], [712.00,1247.00],
             [716.00,1244.00], [745.00,1225.00], [748.00,1222.00],
             [751.00,1219.00], [784.00,1182.00], [785.00,1180.00],
             [786.00,1178.00], [814.00,1127.00], [815.00,1125.00],
             [816.00,1123.00], [839.00,1086.00], [841.00,1083.00],
             [843.00,1080.00], [858.00,1040.00], [859.00,1038.00],
             [860.00,1036.00], [861.00,989.00], [860.00,988.00],
             [859.00,987.00], [833.00,976.00], [827.00,976.00],
             [821.00,976.00], [790.00,1002.00], [788.00,1005.00],
             [786.00,1008.00], [760.00,1039.00], [754.00,1043.00],
             [748.00,1047.00], [698.00,1070.00], [691.00,1071.00],
             [684.00,1072.00], [620.00,1057.00], [620.00,1056.00],
             [620.00,1055.00], [590.00,1013.00], [590.00,1008.00],
             [590.00,1003.00], [597.00,957.00], [598.00,956.00],
             [599.00,955.00], [619.00,919.00], [620.00,918.00],
             [621.00,917.00], [658.00,874.00], [659.00,872.00],
             [660.00,870.00], [687.00,832.00], [690.00,829.00],
             [693.00,826.00], [720.00,775.00], [721.00,773.00],
             [722.00,771.00], [729.00,722.00], [729.00,722.00],
             [729.00,722.00], [706.00,703.00], [703.00,703.00],
             [700.00,703.00], [669.00,713.00], [668.00,714.00],
             [667.00,715.00], [641.00,750.00], [639.00,754.00],
             [637.00,758.00], [617.00,796.00], [615.00,798.00],
             [613.00,800.00], [577.00,829.00], [572.00,835.00],
             [567.00,841.00], [517.00,880.00], [515.00,881.00],
             [513.00,882.00], [477.00,913.00], [471.00,917.00],
             [465.00,921.00], [422.00,966.00], [420.00,969.00],
             [418.00,972.00], [371.00,1016.00], [368.00,1021.00],
             [365.00,1026.00], [336.00,1079.00], [334.00,1083.00],
             [332.00,1087.00], [319.00,1143.00], [319.00,1146.00],
             [319.00,1149.00], [321.00,1201.00], [321.00,1201.00]]
        
        flame_path = np.array(flame_path) # Converting to numpy array for easier manipulation
        # Flip the y-axis to make the origin (0,0) start at the bottom-left
        flame_path[:, 1] = np.max(flame_path[:, 1]) - flame_path[:, 1]
        flame_path[:,1] = - flame_path[:,1]  # Flip the y-axis for positive downward coordinate system (NED frame)
        flame_path[:, 0] = flame_path[:, 0] - flame_path[0][0] # Normalize to start from (0,0)
        flame_path = flame_path / np.max(flame_path)  # Scale down to fit in the local frame by scaling everything to 0-1 range
        flame_path = flame_path * 10.0  # Scale the maximum dimension of the flame to 10m for the trajectory
        flame_path = flame_path.tolist()
        self.flame_path = flame_path  # Store the flame path for later use
        
    def publish_vehicle_command(self, target, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = target
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    #Arming the vehicle by sending the command
    def arm(self):
        self.publish_vehicle_command(self.vehicle_status.system_id,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    # Disarming the vehicle by sending the command
    def disarm(self):
        self.publish_vehicle_command(self.vehicle_status.system_id,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    # Offboard mode vehicle command
    def engage_offboard_mode(self):
        self.publish_vehicle_command(self.vehicle_status.system_id,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    # Landing vehicle command
    def land(self):
        self.publish_vehicle_command(self.vehicle_status.system_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    #### Callback functions for the custom subscribers
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def global_position_callback(self, msg):
        self.global_pos = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
    
    def listener_callback(self, msg):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        """Handle incoming custom messages"""
        self.custom_msg = msg
        self.leader = msg.follower_of
        self.follower_number = msg.follower_number 
        self.get_logger().info(f"Received message: leader={self.custom_msg.leader}, color={self.custom_msg.light_colour}")
        self.drone_name = msg.drone_name

        # Needed for frame transformation 
        if self.leader == "":
            pass
        else:
            #self.leader_gps = self.create_subscription(
            #VehicleGlobalPosition, f'{self.leader}/fmu/out/vehicle_global_position',
            #self.global_position_callback, qos_profile)

            self.leader_vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.leader}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)

            self.leader_vehicle_local_position = VehicleLocalPosition()

    ##################################################################
    # Light control command 
    def light_control(self, funct: str, colour: str, num_blinks: int = 0, priority: int = 2):
        mode_dict = {
        "off": LedControl.MODE_OFF,
        "on": LedControl.MODE_ON,
        "disabled": LedControl.MODE_DISABLED,
        "blink_slow": LedControl.MODE_BLINK_SLOW,
        "blink_normal": LedControl.MODE_BLINK_NORMAL,
        "blink_fast": LedControl.MODE_BLINK_FAST,
        "breathe": LedControl.MODE_BREATHE,
        "flash": LedControl.MODE_FLASH,
        }

        color_dict = {
            "off": LedControl.COLOR_OFF,
            "red": LedControl.COLOR_RED,
            "green": LedControl.COLOR_GREEN,
            "blue": LedControl.COLOR_BLUE,
            "yellow": LedControl.COLOR_YELLOW,
            "purple": LedControl.COLOR_PURPLE,
            "amber": LedControl.COLOR_AMBER,
            "cyan": LedControl.COLOR_CYAN,
            "white": LedControl.COLOR_WHITE,
        }

        msg = LedControl()
        msg.led_mask = 0xff # All LEDs
        msg.mode = mode_dict[funct]
        msg.color = color_dict[colour]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.light_control_publisher.publish(msg)
        self.get_logger().info("Light control command sent")

    # Sending the messages to change into offboard mode
    # and to set the position setpoint
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # Publishing the position setpoints through TrajectorySetpoint
    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    # Calculating the position change in the local frame of the leader
    def follower_frame_transform(self):
        leader_latitude_origin = self.leader_vehicle_local_position.ref_lat
        leader_longitude_origin = self.leader_vehicle_local_position.ref_lon

        follower_latitude_origin = self.vehicle_local_position.ref_lat
        follower_longitude_origin = self.vehicle_local_position.ref_lon        
                
        def convert_coordinate_to_meters(latitude, longitude):
            #### This converts the coordinate degrees to meters as per wikipedia WGS84 conversions 
            # (accurate to a magnitude of cm)
            latitude = np.deg2rad(latitude)
            longitude = np.deg2rad(longitude)

            # COnversion according to the WGS84 ellipsoid model
            # https://en.wikipedia.org/wiki/Geographic_coordinate_system#Latitude_and_longitude
            m_latitude_per_deg = 111132.92 - 559.82*np.cos(2*latitude) + 1.175*np.cos(4*latitude) - 0.0023*np.cos(6*latitude)
            m_longitude_per_deg = 111412.84*np.cos(latitude) - 93.5*np.cos(3*latitude) + 0.118*np.cos(5*latitude)
            m_latitude = np.rad2deg(latitude) * m_latitude_per_deg
            m_longitude = np.rad2deg(longitude) * m_longitude_per_deg

            return m_latitude, m_longitude

        leader_latitude, leader_longitude = convert_coordinate_to_meters(leader_latitude_origin, leader_longitude_origin) 
        follower_latitude, follower_longitude = convert_coordinate_to_meters(follower_latitude_origin, follower_longitude_origin)
        
        # Calculate the difference in follower coordinates wrt the leader 
        # This is the follower position in the leader local frame
        origin_delta_latitude = follower_latitude - leader_latitude
        origin_delta_longitude = follower_longitude - leader_longitude

        self.get_logger().info(f"coordinate transform for follower: {[origin_delta_latitude, origin_delta_longitude]}") #, delta_altitude]}")
        
        # x offset - longitude, y offset - latitude
        self.coordinate_transform = [origin_delta_longitude, origin_delta_latitude] 

    # Update the trajectory for the follower drone
    def updated_trajectory(self, setpoints: list, follower_number: int, dt: float) -> list:
        # THREE SECOND DELAY
        extra_points = 3*follower_number/dt # 3 seconds of delay

        # Add extra points to the front of the setpoints to create a delay
        new_points = []
        for i in np.arange(extra_points):
            new_points.append([0.0, 0.0, 0.0])

        # Concatenate the new points to the front of the setpoints
        setpoints = new_points + setpoints
        return setpoints


    # CONTROL LOOP
    def timer_callback(self) -> None:
        # ----------------------------------------
        # TO DO!!!!!!!
        # Subscribe to the TC topic to see if you have a follower 
        # -----------------------------------------

        #self.get_logger().info(f"Received message: leader={self.leader}, color={msg.light_colour}")
        
        ######## Assign the leader and follower relationships and the light colour ##########
        self.leader = self.custom_msg.follower_of
        self.follower_number = self.custom_msg.follower_number
        colour = self.custom_msg.light_colour # light colour
        ###############


        print(f"Leader: {self.leader}, Follower number: {self.follower_number}, Colour: {colour}")
        print(colour)
        funct = "blink_slow" # light function

        self.publish_offboard_control_heartbeat_signal()
        
        # LIGHT FUNCTIONALITY 
        self.light_control(funct, colour)

        ## ---------------------------------------------------
        ## PHASE 1: Initialisation (first 5 seconds)
        ## Set the drone to offboard mode and arm it
        ## Will takeoff and maintain position for 5 seconds
        ## ---------------------------------------------------

        if self.offboard_setpoint_counter == 1:
            if self.leader != "":
                    self.follower_frame_transform()
            else:
                pass

        if self.offboard_setpoint_counter < 100:
            # ~10 seconds at 10Hz timer
            # Publish setpoint continuously
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        
            # Engage offboard after 2.5 second
            # Sending actual offboard command
            if self.offboard_setpoint_counter == 25:
                self.engage_offboard_mode()
                self.get_logger().info("Offboard mode requested")
                
            # Arm after 5 seconds
            # Sending arm command -> Will arm and take off
            elif self.offboard_setpoint_counter == 50:
                self.arm()
                self.get_logger().info("Arm command sent")
    
        
        ## ---------------------------------------------------
        ## PHASE 2.1: Frame transform example
        ## ---------------------------------------------------
        # Trajectory for an equilateral triangle in the leader frame
        #positions = [[-3.0,0.0,2.0], [0.0,5.196,2.0], [3.0,0.0,2.0], [-3.0,0.0,2.0], [0.0,5.196,2.0], [3.0,0.0,2.0], [-3.0,0.0,2.0], [0.0,5.196,2.0]] # an equilateral triangle example in the leader frame
        ###
        positions = self.flame_path # Use the flame path as the trajectory
        
        ### Margin for positional accuracy in order to proceed to the next point
        margin = 0.2 # 0.2m accuracy margin

        # ---------------------------------------------------
        # CONCATENATE LISTS IF FOLLOWER 
        # ---------------------------------------------------
        # !!!!TO DO!!!!!
        if self.leader != "":
            positions = self.updated_trajectory(positions, self.follower_number, self.dt)
        else:
            pass

        # Check if drone is within the positional accuracy margin of its target position
        if self.offboard_setpoint_counter >= 100:        
            if self.position_change < len(positions):    
                target_x = positions[self.position_change][0]
                target_y = positions[self.position_change][1]
                
                # Adding the offset if it is a follower drone 
                if self.leader != "":
                    offset_x = self.coordinate_transform[0] 
                    offset_y = self.coordinate_transform[1]
                else:
                    offset_x = 0.0
                    offset_y = 0.0

                # Telling to move to the next position after the first has been reached
                if self.vehicle_local_position.x - target_x < margin and \
                        self.vehicle_local_position.y - target_y < margin:
                    self.position_change += 1

                # Publishing ! :D
                self.publish_position_setpoint(target_x + offset_x, target_y + offset_y, self.takeoff_height)
            
            # Beginning landing sequence after all positions have been reached
            else:
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                current_x = self.vehicle_local_position.x
                current_y = self.vehicle_local_position.y
                dist = np.sqrt(current_x ** 2 + current_y ** 2)

                # Landing and disarming 
                if dist < margin:
                    self.land()
                    self.is_landing_triggered = True
                    if self.vehicle_local_position.z > -0.5:  # Within 0.5m of ground (NED frame)
                        self.get_logger().info("Landed successfully")
                        self.is_disarmed = True
                        self.disarm()

        if self.is_disarmed:
            rclpy.shutdown()
        
        # Increment counter 
        self.offboard_setpoint_counter += 1
                
def main(args=None) -> None:
    try:
        print('Starting offboard control node...')
        rclpy.init(args=args)
        offboard_control = Drone_Three()
        rclpy.spin(offboard_control)
        offboard_control.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"Exception occurred in Drone_Three_Node: {e}")
        rclpy.shutdown()    

#if __name__ == '__main__':
#    try:
#        main()
#    except Exception as e:
#        print(f"Exception occured in Drone_Three_Node: {e}")
