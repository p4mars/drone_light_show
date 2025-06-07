#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import \
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, SensorGps
import numpy as np 

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_node_OG')

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
        self.gps_position_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.gps_position_callback, qos_profile)

        # Initialise variables
        self.offboard_setpoint_counter = 0 # To count time passed
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -2.0
        self.radius = 1.0
        self.theta = 0
        self.is_landing_triggered = False  # Add to __init__

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    # Callback function for vehicle_local_position topic subscriber
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def gps_position_callback(self, gps_position):
        # Print GPS position data
        lat = gps_position.latitude_deg
        lon = gps_position.longitude_deg
        alt = gps_position.altitude_msl_m
        self.get_logger().info(f'Lat: {lat:.6f}, Lon: {lon:.6f}, Alt (MSL): {alt:.2f} m')

    # Callback function for vehicle_status topic subscriber.
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def publish_vehicle_command(self, command, **params) -> None:
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

    # Arming the vehicle by sending the command
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
        
    # CONTROL LOOP
    def timer_callback(self) -> None:
        # Always maintain offboard heartbeat 
        self.publish_offboard_control_heartbeat_signal()

        # Current altitude 
        current_z = self.vehicle_local_position.z
        
        # ---------------------------------------------------
        # PHASE 1: Initialisation (first 5 seconds)
        # Set the drone to offboard mode and arm it
        # Will takeoff and maintain position for 5 seconds
        # ---------------------------------------------------
        if self.offboard_setpoint_counter < 100:  # ~5 seconds at 10Hz timer
            # Publish setpoint continuously
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
            # Engage offboard after 1 second
            # Sending actual offboard command
            if self.offboard_setpoint_counter == 25:
                self.engage_offboard_mode()
                self.get_logger().info("Offboard mode requested")
                
            # Arm after 2 seconds
            # Sending arm command -> Will arm and take off
            elif self.offboard_setpoint_counter == 35:
                self.arm()
                self.get_logger().info("Arm command sent")
        
        # ---------------------------------------------------
        # PHASE 2: Circle trajectory
        # ---------------------------------------------------
        if self.offboard_setpoint_counter >= 100:
            # Check if the drone is armed and in offboard mode
            if self.theta < 2 * np.pi:

                # Publish circle trajectory
                self.publish_position_setpoint(
                    self.radius * np.cos(self.theta),
                    self.radius * np.sin(self.theta),
                    self.takeoff_height
                )
                self.theta += 0.1  # Increment theta for circular motion

            # If not, just hold position
            else:
                # ---------------------------------------------------
                # PHASE 3: Landing
                # ---------------------------------------------------
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
                

                #INITIATE LANDING
                if self.vehicle_local_position.x < 0.1 and self.vehicle_local_position.y < 0.1:
                    self.is_landing_triggered = True
                    self.land()
                    self.get_logger().info("Landing initiated")

                    # DISARM THE DRONE
                    if current_z > -0.5:  # Within 0.5m of ground (NED frame)
                        self.get_logger().info("Landed successfully")
                        self.disarm()
                        rclpy.shutdown()
    
        # Increment counter 
        if self.offboard_setpoint_counter < 1000:  
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
