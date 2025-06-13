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
        #self.radius = 1.0
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
        leader_latitude_origin = vehicle_leader.vehicle_local_position.ref_lat
        leader_longitude_origin = vehicle_leader.vehicle_local_position.ref_lon
        #Leader_altitude = vehicle_leader.global_pos.alt

        follower_latitude_origin = self.vehicles[follower_number].vehicle_local_position.ref_lat
        follower_longitude_origin = self.vehicles[follower_number].vehicle_local_position.ref_lon        
        #follower_altitude = self.vehicles[follower_number].global_pos.alt
        
        #print(f"Leader: {leader_latitude}, {leader_longitude}")
        def convert_coordinate_to_meters(latitude, longitude):
            #### This converts the coordinate degrees to meters as per wikipedia WGS84 conversions 
            # (accurate to a magnitude of cm)
            latitude = np.deg2rad(latitude)
            longitude = np.deg2rad(longitude)
            # Conversion according to the WGS84 ellipsoid model
            # https://en.wikipedia.org/wiki/Geographic_coordinate_system#Latitude_and_longitude
            m_latitude_per_deg = 111132.92 - 559.82*np.cos(2*latitude) + 1.175*np.cos(4*latitude) - 0.0023*np.cos(6*latitude)
            m_longitude_per_deg = 111412.84*np.cos(latitude) - 93.5*np.cos(3*latitude) + 0.118*np.cos(5*latitude)
            m_latitude = np.rad2deg(latitude) * m_latitude_per_deg
            m_longitude = np.rad2deg(longitude) * m_longitude_per_deg
            return m_latitude, m_longitude

        leader_latitude, leader_longitude = convert_coordinate_to_meters(leader_latitude_origin, leader_longitude_origin) 
        follower_latitude, follower_longitude = convert_coordinate_to_meters(follower_latitude_origin, follower_longitude_origin)
        #calculate the difference in follower coordinates wrt the leader 
        # This is the follower position in the leader local frame
        origin_delta_latitude = follower_latitude - leader_latitude
        origin_delta_longitude = follower_longitude - leader_longitude
        #delta_altitude = follower_altitude - Leader_altitude
        self.get_logger().info(f"coordinate transform for follower {follower_number}: {[origin_delta_latitude, origin_delta_longitude]}") #, delta_altitude]}")
        self.vehicles[follower_number].coordinate_transform = [origin_delta_latitude, origin_delta_longitude] #, delta_altitude]

        
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
        ## PHASE 2: Circle tiangular trajectory example
        ## ---------------------------------------------------

        ##### Making a trajectory for an equilateral triangle in the leader frame with repeating verteces of [-3.0,0.0,2.0], [0.0,5.196,2.0], [3.0,0.0,2.0]
        Triangle_corner_positions = [[-3.0,0.0,2.0], [0.0,5.196,2.0], [3.0,0.0,2.0], [-3.0,0.0,2.0], [0.0,5.196,2.0], [3.0,0.0,2.0], [-3.0,0.0,2.0], [0.0,5.196,2.0]] # an equilateral triangle example in the leader frame
        
        positional_accuracy_margin = 0.2 # 0.2m accuracy margin

        ### Once the take-off procedure has ended after 10 seconds, the drones will start flying to their target positions:
        if self.offboard_setpoint_counter >= 100:

            ### If the drones are close to their target positions for sufficiently long, then move them to the next position
            ### The parameter that determines whether all drones are in fact at their positions is initialised as false for fail-safe purposes
            all_close = False

            # If the number of position changes has been less than the number of positions in the triangle, then make all drones fly to their target positions
            if self.position_change < len(Triangle_corner_positions) - 1:
                # the assumption is that all drones at their target positions
                all_close = True

                # To check if this is true, we can calculate the distance between the current position and the target position
                # and if the distance is greater than the positional accuracy margin, then we can set all_close to false:
                ## Start by going through all of the drones in the "vehicles" list
                for i, vehicle in enumerate(self.vehicles):

                    # For the leader drone, the calcualtion does not need a coordinate transformation
                    if i == 0:
                        target_x = Triangle_corner_positions[self.position_change][0]
                        target_y = Triangle_corner_positions[self.position_change][1]

                    # For all of the followers (if i != 0), however, the transformation has to be included in the calculation
                    else:
                        # make the second drone fly to the point preceding the leader drone (or the follower drone before it if i > 1)
                        # the index (idx) determines the target position in the list of triangle corner positions
                        idx = self.position_change + i
                        
                        # The coordinate transform is applied to the target position to convert it to the follower local frame
                        target_x = Triangle_corner_positions[idx][0] - vehicle.coordinate_transform[0]
                        target_y = Triangle_corner_positions[idx][1] - vehicle.coordinate_transform[1]
                    
                    # The current positions can be performed without the coordinate transformations for all drones since it is always given in the relevant local frame
                    current_x = vehicle.vehicle_local_position.x
                    current_y = vehicle.vehicle_local_position.y

                    ### Publish the position setpoint for the drone
                    self.publish_position_setpoint(vehicle, target_x, target_y, self.takeoff_height)


                    ## Calculate the distance between the target and currrent position to determine if the target has been reached
                    dist = np.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2)

                    ## If the distance is larger than the positional accuracy margin, then at least this drone is not at its target position and thus
                    ## not all drones are close to their target positions
                    if dist > positional_accuracy_margin:
                        all_close = False


            ### If the number of points in the path list has been exceeded, then make all drones return to their original positions:
            else:
            ### make all drones return to their original positions by cycling through all of the drones
                for i, vehicle in enumerate(self.vehicles):

                    ### Drones are once more assumed to be at their target positions unless proven otherwise using the distance calculation
                    all_close = True

                    # Each drone receives the same target position of [0.0, 0.0, self.takeoff_height] since that is the origin of their own ocal frames
                    # Therefore the drones will return to their starting locations and hover at the takeoff height
                    self.publish_position_setpoint(vehicle, 0.0, 0.0, self.takeoff_height)

                    ### The current position is determined in the local frame as was done before
                    current_x = vehicle.vehicle_local_position.x
                    current_y = vehicle.vehicle_local_position.y


                    #Since the target location is [0,0] for all drones, the distance can be calculated as follows:
                    dist = np.sqrt(current_x ** 2 + current_y ** 2)

                    ## We can mointor the distance from the landing location for all drones
                    print(f"distance for drone [{i}]: {dist}")

                    # If the distance is less than the positional accuracy margin, then the drone is close to its target position
                    if dist < positional_accuracy_margin:

                        #Log some information for the user to see the landing process.
                        self.get_logger().info(f"[{vehicle.namespace}] Drone is within positional accuracy margin. Hovering at position: {[0.0, 0.0, self.takeoff_height]}")
                        self.get_logger().info(f"[{vehicle.namespace}] hover counter: {self.all_close_counter}")

                        ## After hovering for 2 seconds, the drones will initiate landing
                        if self.all_close_counter > 20:  # 2 seconds at 10Hz
                            self.get_logger().info(f"[{vehicle.namespace}] Landing initiated")
                            self.land(vehicle)
                            vehicle.is_landing_triggered = True

                        # If the drone has landed (determined by its distance from the ground), then disarm it
                            if vehicle.vehicle_local_position.z > -0.5:  # Within 0.5m of ground (NED frame)
                                self.get_logger().info("Landed successfully")
                                self.disarm(vehicle)

                    ## If the distance is larger than the positional accuracy margin, then at least this drone is not at its target position and thus
                    ## not all drones are close to their target positions
                    else:
                        all_close = False

            ### For all instances where the closenes of all drones was checked, we can now determine the action to take if they are all close to their target positions
            if all_close:

                ## Publish a message to notify the user if all drones are close to their target positions
                #self.get_logger().info("All drones are close to their target positions.")

                ### If the drones are close to their target positions, then we can either hover for a while or move to the next position
                ### If the drones have been close to their positions for less than N seconds, we can keep hovering
                ### In this case N is 2.5 (25 iterations at 10Hz)

                if self.all_close_counter < 25: # repeat the location check for 2.5 seconds at 10Hz
                    self.get_logger().info(f"All drones are close to their target positions. counter = [{self.all_close_counter}]")
                    self.all_close_counter += 1

                ### If the drones have been close to their positions for more than N seconds, we can move to the next position
                ### This is done by increasing the position_change counter and resetting the all_close_counter to 0:
                else:
                    self.position_change += 1
                    self.get_logger().info(f"All drones are close to their target positions. Moving to next position. Changes: {self.position_change}")
                    self.all_close_counter = 0
                    


        
            
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
