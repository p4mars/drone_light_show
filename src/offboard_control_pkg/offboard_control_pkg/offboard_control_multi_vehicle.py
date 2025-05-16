#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import \
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, SensorGps
import numpy as np 

class Drone:
    def __init__(self, node, namespace, qos_profile):
        self.namespace = namespace
        self.node = node

        # Publishers
        self.offboard_control_mode_publisher = node.create_publisher(
            OffboardControlMode, f'{namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = node.create_publisher(
            TrajectorySetpoint, f'{namespace}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = node.create_publisher(
            VehicleCommand, f'{namespace}/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = node.create_subscription(
            VehicleLocalPosition, f'{namespace}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = node.create_subscription(
            VehicleStatus, f'{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.gps_position_subscriber = node.create_subscription(
            SensorGps, f'{namespace}/fmu/out/vehicle_gps_position',
            self.gps_position_callback, qos_profile)

        # State variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def gps_position_callback(self, msg):
        lat = msg.latitude_deg
        lon = msg.longitude_deg
        alt = msg.altitude_msl_m
        self.node.get_logger().info(f'[{self.namespace}] Lat: {lat:.6f}, Lon: {lon:.6f}, Alt (MSL): {alt:.2f} m')

class OffboardControl_MV(Node):

    def __init__(self) -> None:
        super().__init__('offboard_control_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        namespaces = ['/px4_1', '/px4_2'] #### Expand the list for the desired number of drones
        
        self.vehicles = [Drone(self, namespace, qos_profile) for namespace in namespaces]
        
        # Initialise variables
        self.offboard_setpoint_counter = 0 # To count time passed
        self.takeoff_height = -2.0 # positive downward!
        self.radius = 1.0
        self.theta = 0
        self.is_landing_triggered = False  # Add to __init__

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    
    def publish_vehicle_command(self, vehicle, command, **params) -> None:
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
        vehicle.vehicle_command_publisher.publish(msg)

    #Arming the vehicle by sending the command
    def arm(self, vehicle):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f'[{vehicle.namespace}] Arm command sent')

    # Disarming the vehicle by sending the command
    def disarm(self, vehicle):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f'[{vehicle.namespace}] Disarm command sent')

    # Offboard mode vehicle command
    def engage_offboard_mode(self, vehicle):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"[{vehicle.namespace}]Switching to offboard mode")

    # Landing vehicle command
    def land(self, vehicle):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info(f"[{vehicle.namespace}]Switching to land mode")

    # Sending the messages to change into offboard mode
    # and to set the position setpoint
    def publish_offboard_control_heartbeat_signal(self, vehicle):
        msg = OffboardControlMode()
        msg.position = True
        #msg.velocity = False
        #msg.acceleration = False
        #msg.attitude = False
        #msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vehicle.offboard_control_mode_publisher.publish(msg)

    # Publishing the position setpoints through TrajectorySetpoint
    def publish_position_setpoint(self, vehicle, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vehicle.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"[{vehicle.namespace}] Publishing position setpoint: {[x, y, z]}")
        
    # CONTROL LOOP
    def timer_callback(self) -> None:

        for vehicle in self.vehicles:
            self.publish_offboard_control_heartbeat_signal(vehicle)
            
        ## Always maintain offboard heartbeat 
        #self.publish_offboard_control_heartbeat_signal()
#
        ## Current altitude 
        #current_z = self.vehicle_local_position.z
        #
        ## ---------------------------------------------------
        ## PHASE 1: Initialisation (first 5 seconds)
        ## Set the drone to offboard mode and arm it
        ## Will takeoff and maintain position for 5 seconds
        ## ---------------------------------------------------
        if self.offboard_setpoint_counter < 100:  # ~5 seconds at 10Hz timer
            # Publish setpoint continuously
            self.publish_position_setpoint(vehicle, 0.0, 0.0, self.takeoff_height)
            
            # Engage offboard after 1 second
            # Sending actual offboard command
            if self.offboard_setpoint_counter == 25:
                self.engage_offboard_mode(vehicle)
                self.get_logger().info(f"[{vehicle.namespace}] Offboard mode requested")
                
            # Arm after 2 seconds
            # Sending arm command -> Will arm and take off
            elif self.offboard_setpoint_counter == 35:
                self.arm(vehicle)
                self.get_logger().info(f"[{vehicle.namespace}] Arm command sent")
        
        ## ---------------------------------------------------
        ## PHASE 2: Circle trajectory
        ## ---------------------------------------------------
        if self.offboard_setpoint_counter >= 100:
            # Check if the drone is armed and in offboard mode
            if self.theta < 2 * np.pi:

                # Publish circle trajectory
                x = self.radius * np.cos(self.theta)
                y = self.radius * np.sin(self.theta)

                self.publish_position_setpoint(vehicle, x, y, self.takeoff_height)
                self.theta += 0.1 # Increment theta for circular motion

            # If not, just hold position
            else:
                
                self.publish_position_setpoint(vehicle, 0.0, 0.0, self.takeoff_height)
                
            # ---------------------------------------------------
            # PHASE 3: Landing
            # ---------------------------------------------------
            #INITIATE LANDING
            if self.theta >= 2 * np.pi and not self.is_landing_triggered:
                self.is_landing_triggered = True
                self.land(vehicle)
                self.get_logger().info(f"[{vehicle.namespace}] Landing initiated")

            # Disarm the drone after landing
            if self.is_landing_triggered and vehicle.vehicle_local_position.z > -0.5:  # Within 0.5m of ground (NED frame)
                self.get_logger().info(f"[{vehicle.namespace}] Landed successfully")
                self.disarm(vehicle)
                rclpy.shutdown()
    
        # Increment counter 
        if self.offboard_setpoint_counter < 1000:  
            self.offboard_setpoint_counter += 1

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl_MV()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
