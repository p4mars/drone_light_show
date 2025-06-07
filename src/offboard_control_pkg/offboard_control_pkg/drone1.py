import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import \
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, \
          VehicleGlobalPosition, LEDControl
from offboard_control_interfaces.msg import Drone1Info


import numpy as np 

class Drone_One(Node):
    def __init__(self) -> None:
        super().__init__('Drone_One_Node')
        self.leader = None #px4_2, px4_3
        self.follower_number = None

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Needed for frame transformation 
        if self.leader == 0:
            pass
        else:
            self.leader_gps = self.create_subscription(
            VehicleGlobalPosition, f'{self.follower}/fmu/out/vehicle_global_position',
            self.global_position_callback, qos_profile)

            self.leader_vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.follower}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)

            self.leader_vehicle_local_position = VehicleLocalPosition()

        #---------------------------------------
        # Publishers
        #---------------------------------------
        self.light_control_publisher = self.create_publisher(
            LEDControl, 'px4_1/fmu/in/led_control', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 'px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 'px4_1/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 'px4_1/fmu/in/vehicle_command', qos_profile)

        #---------------------------------------
        # Subscribers
        #---------------------------------------
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, 'px4_1/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.global_pos_subscriber = self.create_subscription(
            VehicleGlobalPosition, 'px4_1/fmu/out/vehicle_global_position',
            self.global_position_callback, qos_profile)
        self.custom_subscriber = self.create_subscription(Drone1Info, 'drone_1_info', \
                                                          self.listener_callback, 10)

        #---------------------------------------
        # State variables
        #---------------------------------------
        self.custom_msg = Drone1Info()
        self.vehicle_local_position = VehicleLocalPosition()
        self.global_pos = VehicleGlobalPosition()
        self.coordinate_transform = []  
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
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    # Disarming the vehicle by sending the command
    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    # Offboard mode vehicle command
    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    # Landing vehicle command
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    # Light control command 
    def light_control(self, funct: str, colour: str, num_blinks: int = 5, priority: int = 2):
        mode_dict = {
        "off": LEDControl.MODE_OFF,
        "on": LEDControl.MODE_ON,
        "disabled": LEDControl.MODE_DISABLED,
        "blink_slow": LEDControl.MODE_BLINK_SLOW,
        "blink_normal": LEDControl.MODE_BLINK_NORMAL,
        "blink_fast": LEDControl.MODE_BLINK_FAST,
        "breathe": LEDControl.MODE_BREATHE,
        "flash": LEDControl.MODE_FLASH,
        }

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
        }

        msg = LEDControl()
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
        leader_longitude_origin = self.leader_vehicle_local_position_subscriber.ref_lon

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
    def updated_trajectory(setpoints: list, follower_number: int, dt: float) -> list:
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

        self.leader = self.custom_msg.follower
        self.follower_number = self.custom_msg.follower_number
        colour = self.custom_msg.light_colour # light colour
        funct = "blink_slow" # light function

        self.publish_offboard_control_heartbeat_signal()
        
        # LIGHT FUNCTIONALITY 
        self.light_control(self, funct, colour)

        ## ---------------------------------------------------
        ## PHASE 1: Initialisation (first 5 seconds)
        ## Set the drone to offboard mode and arm it
        ## Will takeoff and maintain position for 5 seconds
        ## ---------------------------------------------------

        if self.offboard_setpoint_counter == 1:
            if self.leader != 0:
                    self.follower_frame_transform(self.leader, self.follower_number, self.dt)
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
        positions = [[-3.0,0.0,2.0], [0.0,5.196,2.0], [3.0,0.0,2.0], [-3.0,0.0,2.0], [0.0,5.196,2.0], [3.0,0.0,2.0], [-3.0,0.0,2.0], [0.0,5.196,2.0]] # an equilateral triangle example in the leader frame
        
        margin = 0.2 # 0.2m accuracy margin

        # ---------------------------------------------------
        # CONCATENATE LISTS IF FOLLOWER 
        # ---------------------------------------------------
        # !!!!TO DO!!!!!
        if self.leader != 0:
            positions = self.updated_trajectory(positions, self.follower_number, self.dt)
        else:
            pass

        # Check if drone is within the positional accuracy margin of its target position
        if self.offboard_setpoint_counter >= 100:        
            if self.position_change < len(positions):    
                target_x = positions[self.position_change][0]
                target_y = positions[self.position_change][1]
                
                # Telling to move to the next position after the first has been reached
                if self.vehicle_local_position.x - positions[self.position_change][0] < margin and \
                        self.vehicle_local_position.y - positions[self.position_change][1] < margin:
                    self.position_change += 1

                # Adding the offset if it is a follower drone 
                if self.leader != 0:
                    offset_x = self.coordinate_transform[0] 
                    offset_y = self.coordinate_transform[1]
                else:
                    offset_x = 0.0
                    offset_y = 0.0

                # Publishing ! :D
                self.publish_position_setpoint(target_x+offset_x, target_y+offset_y, self.takeoff_height)
            
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
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = Drone_One()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
