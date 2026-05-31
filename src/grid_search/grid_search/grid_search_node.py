import rclpy
from rclpy.node import Node
import math
from enum import Enum
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, 
    VehicleStatus, VehicleLocalPosition
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class FlightState(Enum):
    IDLE = 0
    ARMING = 1
    TAKING_OFF = 2
    GRID_SEARCH = 3
    RETURNING_HOME = 4
    LANDING = 5
    LANDED = 6


class GridSearchNode(Node):
    def __init__(self):
        super().__init__('grid_search_node')

        # QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile)

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            qos_profile)

        # State variables
        self.vehicle_status = VehicleStatus()
        self.current_position = [0.0, 0.0, 0.0]
        self.flight_state = FlightState.IDLE
        self.state_timer = 0
        self.offboard_setpoint_counter = 0
        self.waypoint_index = 0

        # Grid parameters
        self.takeoff_altitude = -5.0  # NED: negative = up
        self.grid_width = 10.0        # meters (x direction)
        self.grid_height = 6.0        # meters (y direction) 
        self.line_spacing = 2.0       # meters (spacing between parallel lines)

        # Generate grid waypoints
        self.grid_waypoints = self.generate_grid_waypoints()

        self.get_logger().info("=" * 60)
        self.get_logger().info("GRID SEARCH NODE INITIALIZED")
        self.get_logger().info(f"Grid: {self.grid_width}m x {self.grid_height}m")
        self.get_logger().info(f"Line spacing: {self.line_spacing}m")
        self.get_logger().info(f"Total waypoints: {len(self.grid_waypoints)}")
        self.get_logger().info("=" * 60)

        # Timer at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.flight_state = FlightState.ARMING

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def local_position_callback(self, msg):
        self.current_position = [msg.x, msg.y, msg.z]

    def generate_grid_waypoints(self):
        """Generate grid waypoints for search pattern"""
        waypoints = []
        
        # Start at home altitude
        altitude = self.takeoff_altitude
        
        # Generate parallel lines
        going_right = True
        
        for y in range(0, int(self.grid_height) + 1, int(self.line_spacing)):
            if going_right:
                # Go right
                waypoints.append([0.0, float(y), altitude])
                waypoints.append([self.grid_width, float(y), altitude])
            else:
                # Go left
                waypoints.append([self.grid_width, float(y), altitude])
                waypoints.append([0.0, float(y), altitude])
            
            going_right = not going_right
        
        # Return to home
        waypoints.append([0.0, 0.0, altitude])
        
        return waypoints

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = -3.14
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("🚁 ARM command sent!")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("🛑 DISARM command sent!")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("📡 OFFBOARD mode engaged!")

    def distance_to_waypoint(self, waypoint):
        """Calculate distance to waypoint"""
        dx = waypoint[0] - self.current_position[0]
        dy = waypoint[1] - self.current_position[1]
        dz = waypoint[2] - self.current_position[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def timer_callback(self):
        # Always publish control mode
        self.publish_offboard_control_mode()

        # State machine
        if self.flight_state == FlightState.ARMING:
            if self.offboard_setpoint_counter == 0:
                self.get_logger().info("⏳ ARMING phase...")

            # Stream setpoints before arming
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)

            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()
                self.flight_state = FlightState.TAKING_OFF
                self.state_timer = 0
                self.get_logger().info("→ Transitioning to TAKEOFF")

            self.offboard_setpoint_counter += 1

        elif self.flight_state == FlightState.TAKING_OFF:
            # Climb to altitude
            target = self.grid_waypoints[0]
            self.publish_trajectory_setpoint(target[0], target[1], target[2])

            distance = math.sqrt(
                (target[0] - self.current_position[0])**2 +
                (target[1] - self.current_position[1])**2 +
                (target[2] - self.current_position[2])**2
            )

            if distance < 0.5:
                self.state_timer += 1
                if self.state_timer > 10:
                    self.flight_state = FlightState.GRID_SEARCH
                    self.waypoint_index = 0
                    self.state_timer = 0
                    self.get_logger().info("→ Transitioning to GRID SEARCH")
                    self.get_logger().info("📍 Starting grid pattern...")
            else:
                self.get_logger().info(f"⬆️  Taking off... Distance: {distance:.2f}m")
                self.state_timer = 0

        elif self.flight_state == FlightState.GRID_SEARCH:
            if self.waypoint_index < len(self.grid_waypoints):
                waypoint = self.grid_waypoints[self.waypoint_index]
                self.publish_trajectory_setpoint(waypoint[0], waypoint[1], waypoint[2])

                distance = self.distance_to_waypoint(waypoint)

                if distance < 0.5:
                    self.get_logger().info(
                        f"✓ Waypoint {self.waypoint_index} reached: "
                        f"({waypoint[0]:.1f}, {waypoint[1]:.1f}, {waypoint[2]:.1f})"
                    )
                    self.waypoint_index += 1
                else:
                    self.get_logger().info(
                        f"→ Moving to waypoint {self.waypoint_index}: "
                        f"({waypoint[0]:.1f}, {waypoint[1]:.1f}) - Distance: {distance:.2f}m"
                    )
            else:
                # Grid complete, return home and land
                self.flight_state = FlightState.LANDING
                self.state_timer = 0
                self.get_logger().info("→ Transitioning to LANDING")

        elif self.flight_state == FlightState.LANDING:
            # Descend to ground
            current_z = self.current_position[2]

            if abs(current_z) < 0.3:
                self.disarm()
                self.flight_state = FlightState.LANDED
                self.get_logger().info("=" * 60)
                self.get_logger().info("✅ MISSION COMPLETE!")
                self.get_logger().info("Grid search finished. Drone landed.")
                self.get_logger().info("=" * 60)
            else:
                # Descend at controlled rate
                descent_rate = 1.0  # m/s
                self.publish_trajectory_setpoint(0.0, 0.0, current_z + descent_rate * 0.1)
                self.get_logger().info(f"⬇️  Landing... Height: {current_z:.2f}m")

        elif self.flight_state == FlightState.LANDED:
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = GridSearchNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()