#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import \
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, SensorGps, VehicleGlobalPosition
import numpy as np 

class Drone:
    def __init__(self, node, namespace, qos_profile):
        self.namespace = namespace
        self.node = node
        self.theta = 0.0

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
        
        ######## Global position is not the raw GPS position and is instead the estimated global position based on the
        ######## drone control inputs and a Kalman filter in order to reduce noise
        self.global_pos_subscriber = node.create_subscription(
            VehicleGlobalPosition, f'{namespace}/fmu/out/vehicle_global_position',
            self.global_position_callback, qos_profile)

        # State variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.global_pos = VehicleGlobalPosition()
        self.coordinate_transform = []  
        self.target_system = self.vehicle_status.system_id
        self.is_landing_triggered = False 
        self.is_disarmed = False

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def gps_position_callback(self, msg):
        lat = msg.latitude_deg
        lon = msg.longitude_deg
        alt = msg.altitude_msl_m
        #self.node.get_logger().info(f'[{self.namespace}] Lat: {lat:.6f}, Lon: {lon:.6f}, Alt (MSL): {alt:.2f} m')
    
    def global_position_callback(self, msg):
        self.global_pos = msg
        #self.node.get_logger().info(f'[{self.namespace}] Global Position - Lat: {msg.lat}, Lon: {msg.lon}, Alt: {msg.alt} m')

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
         # Add to __init__

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.position_change = 0
        self.all_close_counter = 0
        

    def publish_vehicle_command(self, vehicle, target, command, **params) -> None:
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
        vehicle.vehicle_command_publisher.publish(msg)

    #Arming the vehicle by sending the command
    def arm(self, vehicle):
        #  vehicle, command, **params
        self.publish_vehicle_command(vehicle, vehicle.target_system,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f'[{vehicle.namespace}] Arm command sent')

    # Disarming the vehicle by sending the command
    def disarm(self, vehicle):
        self.publish_vehicle_command(vehicle, vehicle.target_system,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f'[{vehicle.namespace}] Disarm command sent')

    # Offboard mode vehicle command
    def engage_offboard_mode(self, vehicle):
        self.publish_vehicle_command(vehicle, vehicle.target_system,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"[{vehicle.namespace}]Switching to offboard mode")

    # Landing vehicle command
    def land(self, vehicle):
        self.publish_vehicle_command(vehicle, vehicle.target_system, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info(f"[{vehicle.namespace}]Switching to land mode")

    # Sending the messages to change into offboard mode
    # and to set the position setpoint
    def publish_offboard_control_heartbeat_signal(self, vehicle):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
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

    def follower_frame_transform(self, vehicle_leader, follower_number):
        leader_latitude = vehicle_leader.global_pos.lat
        leader_longitude = vehicle_leader.global_pos.lon
        #Leader_altitude = vehicle_leader.global_pos.alt

        follower_latitude = self.vehicles[follower_number].global_pos.lat
        follower_longitude = self.vehicles[follower_number].global_pos.lon
        #follower_altitude = self.vehicles[follower_number].global_pos.alt

        # Have to convert the degrees to meters:
        """#### We can assume a spherical Earth, in which case the following calculation can be used according to wiki:
        #  https://en.wikipedia.org/wiki/Geographic_coordinate_system#Latitude_and_longitude
        R_earth = 6367449 ## Earth's avergae meridional radius in meters
        conversion_coeff = np.pi/180*R_earth

        leader_longitude = conversion_coeff * np.cos(leader_latitude)*leader_longitude
        leader_latitude = conversion_coeff * leader_latitude

        follower_longitude = conversion_coeff * np.cos(follower_latitude)*follower_longitude
        follower_latitude = conversion_coeff * follower_latitude"""
        
        #print(f"Leader: {leader_latitude}, {leader_longitude}")
        def convert_coordinate_to_meters(latitude, longitude):
            #### This converts the coordinate degrees to meters as per wikipedia WGS84 conversions 
            # (accurate to a magnitude of cm)
            m_latitude_per_deg = 111132.92-559.82*np.cos(2*latitude) + 1.175*np.cos(4*latitude) - 0.0023*np.cos(6*latitude)
            m_longitude_per_deg = 111412.84*np.cos(latitude) - 93.5*np.cos(3*latitude) + 0.118*np.cos(5*latitude)
            m_latitude = latitude * m_latitude_per_deg
            m_longitude = longitude * m_longitude_per_deg
            return m_latitude, m_longitude

        leader_latitude, leader_longitude = convert_coordinate_to_meters(leader_latitude, leader_longitude) 
        follower_latitude, follower_longitude = convert_coordinate_to_meters(follower_latitude, follower_longitude)
        #calculate the difference in follower coordinates wrt the leader 
        # This is the follower position in the leader local frame
        delta_latitude = follower_latitude - leader_latitude
        delta_longitude = follower_longitude - leader_longitude
        #delta_altitude = follower_altitude - Leader_altitude

        self.vehicles[follower_number].coordinate_transform = [delta_latitude, delta_longitude] #, delta_altitude]

        
    # CONTROL LOOP
    def timer_callback(self) -> None:
        for vehicle in self.vehicles:
            self.publish_offboard_control_heartbeat_signal(vehicle)
            
        ## ---------------------------------------------------
        ## PHASE 1: Initialisation (first 5 seconds)
        ## Set the drone to offboard mode and arm it
        ## Will takeoff and maintain position for 5 seconds
        ## ---------------------------------------------------

        if self.offboard_setpoint_counter == 1:
            for i, vehicle in enumerate(self.vehicles):
                if i != 0:
                    #### Getting all of the coordinate transforms for the leader drone being drone 0
                    self.follower_frame_transform(self.vehicles[0], i)
                    #print(vehicle.coordinate_transform)

        if self.offboard_setpoint_counter < 100:
            # ~10 seconds at 10Hz timer
            # Publish setpoint continuously
            for vehicle in self.vehicles:
                self.publish_position_setpoint(vehicle, 0.0, 0.0, self.takeoff_height)
            
                # Engage offboard after 2.5 second
                # Sending actual offboard command
                if self.offboard_setpoint_counter == 25:
                    self.engage_offboard_mode(vehicle)
                    self.get_logger().info(f"[{vehicle.namespace}] Offboard mode requested")
                    
                # Arm after 5 seconds
                # Sending arm command -> Will arm and take off
                elif self.offboard_setpoint_counter == 50:
                    self.arm(vehicle)
                    self.get_logger().info(f"[{vehicle.namespace}] Arm command sent")
        
        ## ---------------------------------------------------
        ## PHASE 2: Circle trajectory example
        ## ---------------------------------------------------
        """
        for vehicle in self.vehicles:
            if vehicle.vehicle_local_position.z <= self.takeoff_height*0.95:
            # Check if the drone is armed and in offboard mode
                if vehicle.theta < 2 * np.pi:
                    # Publish circle trajectory
                    x = self.radius * np.cos(vehicle.theta)
                    y = self.radius * np.sin(vehicle.theta)

                    self.publish_position_setpoint(vehicle, x, y, self.takeoff_height)
                    vehicle.theta += 0.1 # Increment theta for circular motion

                # ---------------------------------------------------
                # PHASE 3: Landing
                # ---------------------------------------------------
                #INITIATE LANDING
                elif vehicle.theta >= 2 * np.pi and not vehicle.is_landing_triggered:
                    self.is_landing_triggered = True
                    self.land(vehicle)
                    self.get_logger().info(f"[{vehicle.namespace}] Landing initiated")

                    # Disarm the drone after landing
            if vehicle.is_landing_triggered and vehicle.vehicle_local_position.z > -0.5:  # Within 0.5m of ground (NED frame)
                self.get_logger().info(f"[{vehicle.namespace}] Landed successfully")
                self.disarm(vehicle)
                vehicle.is_disarmed = True
            """
        
        ## ---------------------------------------------------
        ## PHASE 2.1: Frame transform example
        ## ---------------------------------------------------
        Triangle_corner_positions = [[3.0,0.0,2.0], [1.5,2.6, 2.0], [0.0,0.0,2.0]] # an equilateral triangle example in the leader frame
        
        positional_accuracy_margin = 0.1 # 10cm accuracy margin

        ## publishing the leader position setpoint
        # Check if all drones are within the positional accuracy margin of their target positions
        if self.offboard_setpoint_counter >= 100:
            all_close = True
            if self.position_change < len(Triangle_corner_positions) - 1:
                all_close = True
                for i, vehicle in enumerate(self.vehicles):
                    if i == 0:
                        target_x = Triangle_corner_positions[self.position_change][0]
                        target_y = Triangle_corner_positions[self.position_change][1]
                    else:
                        # Check bounds before accessing lists
                        idx = self.position_change + i
                        #print(f"idx: {idx}", "i-1: ", i-1)
                        #print(f"coordinate_transforms[0]: {vehicle.coordinate_transform}")
                        target_x = Triangle_corner_positions[idx][0] - vehicle.coordinate_transform[0]
                        target_y = Triangle_corner_positions[idx][1] - vehicle.coordinate_transform[1]
                    current_x = vehicle.vehicle_local_position.x
                    current_y = vehicle.vehicle_local_position.y
                    self.publish_position_setpoint(vehicle, target_x, target_y, self.takeoff_height)

                    dist = np.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2)
                    if dist > positional_accuracy_margin:
                        all_close = False

            if all_close:
                self.get_logger().info("All drones are close to their target positions.")
                if self.all_close_counter < 50: # repeat the location check for 5 seconds
                    self.get_logger().info(f"All drones are close to their target positions. counter = [{self.all_close_counter}]")
                    self.all_close_counter += 1
                else:
                    self.get_logger().info(f"All drones are close to their target positions. Moving to next position. Changes: {self.position_change}")
                    self.all_close_counter = 0
                    self.position_change += 1


            if self.position_change >= len(Triangle_corner_positions) - 1:
                ### make all drones return to their original positions
                for i, vehicle in enumerate(self.vehicles):
                    self.publish_position_setpoint(vehicle, 0.0, 0.0, self.takeoff_height)
                    current_x = vehicle.vehicle_local_position.x
                    current_y = vehicle.vehicle_local_position.y
                    dist = np.sqrt(current_x ** 2 + current_y ** 2)
                    print(f"distance for drone [{i}]: {dist}")
                    if dist < positional_accuracy_margin:
                        self.get_logger().info(f"[{vehicle.namespace}] Landing initiated")
                        self.land(vehicle)
                        vehicle.is_landing_triggered = True
                        if vehicle.vehicle_local_position.z > -0.5:  # Within 0.5m of ground (NED frame)
                            self.get_logger().info("Landed successfully")
                            self.disarm(vehicle)
                        
            
        if self.vehicles[0].is_disarmed and self.vehicles[1].is_disarmed:
            rclpy.shutdown()
        
        # Increment counter 
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
