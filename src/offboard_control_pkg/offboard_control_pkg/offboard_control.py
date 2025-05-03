import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock
import numpy as np

# Importing the PX4 topics 
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand 
from px4_msgs.msg import VehicleOdometry

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        
        # QoS settings for compatibility with PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
            )
        
        # SUBSCRIPTIONS
        # Vehicle Status
        self.subscriber_status = self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        self.subscriber_odometry = self.create_subscription(
            VehicleOdometry,  # Or VehicleLocalPosition (PX4 msgs vary by version)
            '/fmu/out/vehicle_odometry',  # Topic may vary (e.g., /fmu/out/vehicle_local_position)
            self.odometry_callback,
            qos_profile)
        
        # PUBLISHERS
        # Create publisher to send arm command
        self.publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # Control Variables
        self.arming_state = None
        self.nav_state = None
        self.current_z = None  # Track current altitude (NED frame: negative = up)
        self.altitude = 2.0  # Desired takeoff altitude (meters)
        self.radius = 3.0    # For circular trajectory
        self.theta = 0.0     # Starting angle for circular trajectory
        self.omega = 0.1     # Angular velocity (rad/s)
        self.dt = 0.1        # Time step

        # State Flags
        self.armed = False
        self.takeoff_complete = False

        # Main control timer (10Hz) Calling the control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Offboard control node initialised!')

    # SUBSCRIBER WILL INITIATE THIS FUNCTION RESULTING IN A PRINT STATEMENT
    def vehicle_status_callback(self, msg):
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def odometry_callback(self, msg):
        # NED frame: msg.position[2] is Z (negative = altitude above home)
        self.current_z = msg.position[2]  # Store current altitude

    def arm_drone_callback(self):
        # Create a vehicle command to arm the drone
        arm_cmd = VehicleCommand()
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0  # Arm the drone (1.0 to arm, 0.0 to disarm)

        arm_cmd.target_system = 1  # Target system (1 is usually the first drone in simulation)
        arm_cmd.target_component = 1  # Target component (1 for autopilot)
        arm_cmd.source_system = 1  # Source system (1 for the companion computer)
        arm_cmd.source_component = 1  # Source component

        self.publisher.publish(arm_cmd)
        self.get_logger().info('Arm command sent!')

    def set_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(msg)
        self.get_logger().info('Offboard command sent!')

    # def set_offboard_mode_callback(self):
    #     cmd = VehicleCommand()
    #     cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    #     cmd.param1 = 1.0  # PX4 custom mode (1 = offboard)
    #     cmd.param2 = 6.0  # Not used but should be 6 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    #     cmd.target_system = 1
    #     cmd.target_component = 1
    #     cmd.source_system = 1
    #     cmd.source_component = 1
    #     cmd.timestamp = int(Clock().now().nanoseconds / 1000)
    #     self.publisher.publish(cmd)
    
    def takeoff(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        cmd.param7 = self.altitude  # Takeoff altitude
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher.publish(cmd)
        self.get_logger().info(f'Takeoff command sent to {self.altitude}m')
    
    def publish_trajectory(self):
        self.get_logger().info('Trajectory setpoints sent')
        msg = TrajectorySetpoint()
        msg.position = [self.radius * np.cos(self.theta),  # x
                        self.radius * np.sin(self.theta),  # y
                        -self.altitude]                   # z (NED frame)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_trajectory.publish(msg)
        self.theta += self.omega * self.dt  # Update angle for next step

    def control_loop(self):
        self.set_offboard_mode()

        # Step 1: Arm if not armed
       
        if not self.armed:
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.set_offboard_mode_callback()
                    self.get_logger().info("Attempting to switch to offboard mode")
                return
                
            # Only arm if we're in offboard mode
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.arm_drone_callback()
                self.get_logger().info(f"ARMING = {self.arming_state}")
                self.get_logger().info(f"NAVIGATION_STATE = {self.nav_state}")
            else:
                self.armed = True
                self.takeoff()
                self.get_logger().info(
                    f"State: Armed={self.armed}, "
                    f"Offboard={self.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
                return 
            
        if not self.takeoff_complete:
            if self.current_z is not None and self.current_z <= -self.altitude * 0.95:
                self.takeoff_complete = True
                self.get_logger().info(f"Reached target altitude: {-self.current_z:.1f}m")
                return
        else:            
            self.publish_trajectory()
            return

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# # Importing the ROS2 topics
# import rclpy
# import numpy as np
# from rclpy.clock import Clock
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# # Importing the PX4 topics 
# from px4_msgs.msg import OffboardControlMode
# from px4_msgs.msg import TrajectorySetpoint
# from px4_msgs.msg import VehicleStatus
# from px4_msgs.msg import VehicleCommand 
# #############################################################
# # Node to start autonomous flight
# class OffboardControl(Node):
#     def __init__(self):
#         super().__init__('Testing_publisher')
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#             )

#         # SUBSCRIPTIONS
#         # Vehicle Status
#         self.subscriber_status = self.create_subscription(VehicleStatus,
#             '/fmu/out/vehicle_status_v1',
#             self.vehicle_status_callback,
#             qos_profile)
        
#         # PUBLISHERS
#         self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
#         self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
#         self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

#         # CREATING A TIMER 
#         timer_period = 0.02 # seconds 
#         self.timer = self.create_timer(timer_period, self.cmdloop_callback)
#         self.dt = timer_period

#         # USED TO GO IN A CIRCLE
#         self.declare_parameter('radius', 10.0)
#         self.declare_parameter('omega', 5.0)
#         self.declare_parameter('altitude', 5.0)

#         # DEFINING THE NAVIGATION STATE OF THE DRONE AND IF IT IS ARMED
#         self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
#         self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
#         # USED TO PREVENT SPAM 
#         self.takeoff_sent = False 
#         self.mode_sent = False
#         self.arm_sent = False

#         # INITIAL SETTINGS FOR MANEOUVRE
#         self.theta = 0.0
#         self.radius = self.get_parameter('radius').value
#         self.omega = self.get_parameter('omega').value
#         self.altitude = self.get_parameter('altitude').value
 
#     # SUBSCRIBER WILL INITIATE THIS FUNCTION RESULTING IN A PRINT STATEMENT
#     def vehicle_status_callback(self, msg):
#         print("NAV_STATUS: ", msg.nav_state)
#         print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
#         self.nav_state = msg.nav_state
#         self.arming_state = msg.arming_state
        
#     def publish_offboard_mode_command(self):
#         cmd = VehicleCommand()
#         cmd.timestamp = int(Clock().now().nanoseconds / 1000)
#         cmd.param1 = 1.0  # PX4 custom mode
#         cmd.param2 = 6.0  # OFFBOARD mode
#         cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
#         cmd.target_system = 1
#         cmd.target_component = 1
#         cmd.source_system = 1
#         cmd.source_component = 1
#         cmd.from_external = True

#         self.get_logger().info('Setting OFFBOARD mode')
#         self.publisher_vehicle_command.publish(cmd)

#     def publish_arm_command(self):
#         cmd = VehicleCommand()
#         cmd.timestamp = int(Clock().now().nanoseconds / 1000)
#         cmd.param1 = 1.0  # Arm
#         cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
#         cmd.target_system = 1
#         cmd.target_component = 1
#         cmd.source_system = 1
#         cmd.source_component = 1
#         cmd.from_external = True

#         self.get_logger().info('Sending ARM command')
#         self.publisher_vehicle_command.publish(cmd)


#     def cmdloop_callback(self):
#         # Publish offboard control modes
#         offboard_msg = OffboardControlMode()
#         offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)

#         # USING POSITION SET POINTS
#         offboard_msg.position = True
#         offboard_msg.velocity = False
#         offboard_msg.acceleration = False
#         self.publisher_offboard_mode.publish(offboard_msg)

#         # ONCE THE DRONE IS ARMED ALLOW IT TO TAKE OFF
#         if (self.arming_state == VehicleStatus.ARMING_STATE_ARMED) and not self.takeoff_sent:
#             self.publish_takeoff() # CALLING PUBLISH TAKEOFF DEFINITION
#             self.takeoff_sent = True

#         # Normal trajectory loop
#         # If the drone is 
#         if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
#             trajectory_msg = TrajectorySetpoint()
#             trajectory_msg.position[0] = self.radius * np.cos(self.theta)
#             trajectory_msg.position[1] = self.radius * np.sin(self.theta)
#             trajectory_msg.position[2] = -self.altitude
#             self.publisher_trajectory.publish(trajectory_msg)

#             self.theta = self.theta + self.omega * self.dt

#     def publish_takeoff(self):
#         takeoff_cmd = VehicleCommand()
#         takeoff_cmd.timestamp = int(Clock().now().nanoseconds / 1000)
#         takeoff_cmd.param1 = 2.5  # minimum pitch (not too important here)
#         takeoff_cmd.param7 = -self.altitude  # Altitude in meters
#         takeoff_cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF

#         takeoff_cmd.target_system = 1 # Drone 1
#         takeoff_cmd.target_component = 1 # Sent To Autopilot
#         takeoff_cmd.source_system = 1 # Companion Computer Sending the Command
#         takeoff_cmd.source_component = 1 # Default Component
#         takeoff_cmd.from_external = True # External source providing the command (ME)

#         self.get_logger().info('Sending takeoff command')
#         self.publisher_vehicle_command.publish(takeoff_cmd)

# def main(args=None):
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()