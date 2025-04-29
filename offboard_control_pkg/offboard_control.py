# Importing the ROS2 topics
import rclpy
import numpy as np
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Importing the PX4 topics 
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand 

# Node to start autonomous flight
class OffboardControl(Node):
    def __init__(self):
        super().__init__('Testing_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
            )


        # SUBSCRIPTIONS
        # Vehicle Status
        self.subscriber_status = self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        # PUBLISHERS
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # CREATING A TIMER 
        timer_period = 0.02 # seconds 
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        # USED TO GO IN A CIRCLE
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)

        # DEFINING THE NAVIGATION STATE OF THE DRONE AND IF IT IS ARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # USED TO PREVENT SPAM 
        self.takeoff_sent = False 

        # INITIAL SETTINGS FOR MANEOUVRE
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value
 
    # SUBSCRIBER WILL INITIATE THIS FUNCTION RESULTING IN A PRINT STATEMENT
    def vehicle_status_callback(self, msg):
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state


    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        # USING POSITION SET POINTS
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # ONCE THE DRONE IS ARMED ALLOW IT TO TAKE OFF
        if (self.arming_state == VehicleStatus.ARMING_STATE_ARMED) and not self.takeoff_sent:
            self.publish_takeoff() # CALLING PUBLISH TAKEOFF DEFINITION
            self.takeoff_sent = True

        # Normal trajectory loop
        # If the drone is 
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt

    def publish_takeoff(self):
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.timestamp = int(Clock().now().nanoseconds / 1000)
        takeoff_cmd.param1 = 2.5  # minimum pitch (not too important here)
        takeoff_cmd.param7 = -self.altitude  # Altitude in meters
        takeoff_cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF

        takeoff_cmd.target_system = 1 # Drone 1
        takeoff_cmd.target_component = 1 # Sent To Autopilot
        takeoff_cmd.source_system = 1 # Companion Computer Sending the Command
        takeoff_cmd.source_component = 1 # Default Component
        takeoff_cmd.from_external = True # External source providing the command (ME)

        self.get_logger().info('Sending takeoff command')
        self.publisher_vehicle_command.publish(takeoff_cmd)

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()