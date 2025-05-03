#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.is_landing_triggered = False  # Add to __init__

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Improved timer callback with reliable takeoff and landing sequence."""
        # Always maintain offboard heartbeat and setpoints
        self.publish_offboard_control_heartbeat_signal()
        
        # Current altitude (NED frame - negative = above ground)
        current_z = self.vehicle_local_position.z
        
        # PHASE 1: Initialization (first 5 seconds)
        if self.offboard_setpoint_counter < 50:  # ~5 seconds at 10Hz timer
            # Publish setpoint continuously
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
            # Engage offboard after 1 second
            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.get_logger().info("Offboard mode requested")
                
            # Arm after 2 seconds
            elif self.offboard_setpoint_counter == 20:
                self.arm()
                self.get_logger().info("Arm command sent")

        # PHASE 2: Takeoff monitoring
        elif not self.is_landing_triggered:
            # Check if we've reached target altitude (with 10% tolerance)
            altitude_error = abs(current_z - self.takeoff_height)
            
            if altitude_error < abs(self.takeoff_height * 0.1):  # Within 10% of target
                self.get_logger().info(f"Reached target altitude: {current_z:.2f}m")
                self.is_landing_triggered = True
                self.land()
                self.get_logger().info("Landing initiated")
            else:
                # Continue holding position
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        # PHASE 3: Landing monitoring
        else:
            # Check if we've landed (z position close to 0 + 10% tolerance)
            if current_z > -0.5:  # Within 0.5m of ground (NED frame)
                self.get_logger().info("Landed successfully")
                self.disarm()
                rclpy.shutdown()
            # Optional: Add timeout here if needed

        # Increment counter (but don't let it overflow)
        if self.offboard_setpoint_counter < 1000:  # Increased upper limit
            self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

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
#         # First, check if the drone is armed, if not, arm it
#         if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED and not self.arm_sent:
#             self.publish_arm_command()  # Arm the drone
#             self.arm_sent = True

#         # Once the drone is armed, send the offboard mode
#         if self.arming_state == VehicleStatus.ARMING_STATE_ARMED and not self.mode_sent:
#             self.publish_offboard_mode_command()  # Set to Offboard mode
#             self.mode_sent = True

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
#             self.publish_takeoff()  # CALLING PUBLISH TAKEOFF DEFINITION
#             self.takeoff_sent = True

#         # Normal trajectory loop
#         # If the drone is in offboard mode and armed
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

#         takeoff_cmd.target_system = 1  # Drone 1
#         takeoff_cmd.target_component = 1  # Sent To Autopilot
#         takeoff_cmd.source_system = 1  # Companion Computer Sending the Command
#         takeoff_cmd.source_component = 1  # Default Component
#         takeoff_cmd.from_external = True  # External source providing the command (ME)

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

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
# from rclpy.clock import Clock
# import numpy as np
# import time

# # PX4 messages
# from px4_msgs.msg import VehicleCommand, OffboardControlMode, \
#     TrajectorySetpoint, VehicleStatus, VehicleOdometry

# class OffboardControl(Node):
#     def __init__(self):
#         super().__init__('offboard_control_node')

#         # QoS settings
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # SUBSCRIBERS
#         self.sub_status = self.create_subscription(
#             VehicleStatus,
#             '/fmu/out/vehicle_status',
#             self.vehicle_status_callback,
#             qos_profile
#         )
#         self.sub_odom = self.create_subscription(
#             VehicleOdometry,
#             '/fmu/out/vehicle_odometry',
#             self.odometry_callback,
#             qos_profile
#         )

#         # PUBLISHERS
#         self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
#         self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
#         self.pub_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)


#         self.offboard_ready = False
#         self.nav_state = None
#         self.arming_state = None
#         self.current_z = None  # NED frame
#         self.armed = False
#         self.takeoff_complete = False

#         # Control params
#         self.altitude = 2.0  # meters
#         self.radius = 3.0    # meters
#         self.theta = 0.0
#         self.omega = 0.1     # rad/s
#         self.dt = 0.1

#         self.get_logger().info('Node initialized')

#         self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

#         # Instead of delayed_mode_timer
#         self.pre_stream_count = 0
#         self.pre_stream_timer = self.create_timer(0.1, self.pre_stream_callback)

#         # Start control loop timer
#         self.timer = self.create_timer(0.1, self.control_loop)

#     def pre_stream_callback(self):
#         if self.pre_stream_count < 10:
#             self.set_offboard_mode()
#             self.publish_takeoff_setpoint()
#             self.pre_stream_count += 1
#             self.get_logger().debug(f"Pre-streaming setpoint {self.pre_stream_count}")
#         else:
#             self.get_logger().info("Finished pre-streaming setpoints.")
#             self.send_offboard_request()
#             self.pre_stream_timer.cancel()  # stop the timer after 10 messages

#     def send_offboard_request(self):
#         self.set_offboard_mode_command()  # Send VehicleCommand to switch to OFFBOARD
#         self.offboard_requested = True
#         self.get_logger().info('Sent OFFBOARD mode command after pre-streaming.')

#     # Used to keep track of the drone's state
#     def vehicle_status_callback(self, msg):
#         self.nav_state = msg.nav_state
#         self.arming_state = msg.arming_state
        
#         # Debug
#         self.get_logger().info(f"nav_state: {self.nav_state}, arming_state: {self.arming_state}")


#     # Needs to have at least 2 setpoints before it can switch to offboard
#     def publish_takeoff_setpoint(self):
#         msg = TrajectorySetpoint()
#         msg.position = [0.0, 0.0, -self.altitude]  # NED: negative z is up
#         msg.yaw = 0.0
#         msg.timestamp = int(Clock().now().nanoseconds / 1000)
#         self.pub_trajectory.publish(msg)

#     # Updating the current z position of the drone
#     # This is used to check if the drone has taken off
#     # and to control the altitude during the circle maneuver
#     def odometry_callback(self, msg):
#         self.current_z = msg.position[2]

#     # OFFBOARD MODE

#     # This function sets the offboard mode for the drone
#     def set_offboard_mode(self):
#         msg = OffboardControlMode()
#         msg.position = True
#         msg.velocity = False
#         msg.acceleration = False

#         msg.timestamp = int(Clock().now().nanoseconds / 1000)
#         self.pub_offboard_mode.publish(msg)

#     # This function sends the command to set the offboard mode
#     def set_offboard_mode_command(self):
#         cmd = VehicleCommand()
#         cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
#         cmd.param1 = 1.0  # PX4 custom mode
#         cmd.param2 = 6.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
#         cmd.target_system = 1
#         cmd.target_component = 1
#         cmd.source_system = 1
#         cmd.source_component = 1
#         cmd.timestamp = int(Clock().now().nanoseconds / 1000)
#         self.pub_cmd.publish(cmd)
#         self.get_logger().info('Sent OFFBOARD mode command')

#     # ARMING THE DRONE
#     def arm_drone(self):
#         cmd = VehicleCommand()
#         cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
#         cmd.param1 = 1.0
#         cmd.target_system = 1
#         cmd.target_component = 1
#         cmd.source_system = 1
#         cmd.source_component = 1
#         cmd.timestamp = int(Clock().now().nanoseconds / 1000)
#         self.pub_cmd.publish(cmd)
#         self.get_logger().info('Sent ARM command')

#     # PUBLISHING THE CIRCLE SETPOINTS
#     def publish_circle_setpoint(self):
#         msg = TrajectorySetpoint()
#         msg.position = [
#             self.radius * np.cos(self.theta),
#             self.radius * np.sin(self.theta),
#             -self.altitude
#         ]
#         msg.yaw = self.theta + np.pi/2  # facing tangent to circle
#         msg.timestamp = int(Clock().now().nanoseconds / 1000)
#         self.pub_trajectory.publish(msg)
#         self.theta += self.omega * self.dt

#     # CONTROL LOOP
#     # This is where the magic happens
#     # All the functions are called in this loop to perform the maneuvers
#     def control_loop(self):
#         if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
#             self.get_logger().info('Waiting for OFFBOARD mode...')
#             return

#         self.get_logger().info('OFFBOARD mode active!')

#         if not self.armed:
#             if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
#                 self.arm_drone()
#             else:
#                 self.armed = True
#                 self.get_logger().info('Drone is armed!')

#         if not self.takeoff_complete:
#             if self.current_z is not None and self.current_z <= -self.altitude * 0.95:
#                 self.takeoff_complete = True
#                 self.get_logger().info(f"Reached target altitude: {-self.current_z:.1f}m")
#         else:
#             self.publish_circle_setpoint()  # <- use this instead of publish_trajectory


# def main(args=None):
#     rclpy.init(args=args)
#     node = OffboardControl()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



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