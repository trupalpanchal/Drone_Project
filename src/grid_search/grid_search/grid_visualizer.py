import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class GridVisualizer(Node):
    def __init__(self):
        super().__init__('grid_visualizer')

        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

        # Grid parameters (must match grid_search_node.py)
        self.grid_width = 10.0
        self.grid_height = 6.0
        self.line_spacing = 2.0
        self.altitude = -5.0

        # Generate grid waypoints (same logic as grid_search_node.py)
        self.grid_waypoints = self.generate_grid_waypoints()

        self.get_logger().info(f"Grid visualizer started! {len(self.grid_waypoints)} waypoints")

    def generate_grid_waypoints(self):
        """Generate grid waypoints for search pattern"""
        waypoints = []
        going_right = True

        for y in range(0, int(self.grid_height) + 1, int(self.line_spacing)):
            if going_right:
                waypoints.append([0.0, float(y), self.altitude])
                waypoints.append([self.grid_width, float(y), self.altitude])
            else:
                waypoints.append([self.grid_width, float(y), self.altitude])
                waypoints.append([0.0, float(y), self.altitude])

            going_right = not going_right

        waypoints.append([0.0, 0.0, self.altitude])
        return waypoints

    def publish_markers(self):
        marker_array = MarkerArray()

        # Draw line connecting all waypoints
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.3  # Line width
        line_marker.color.r = 1.0  # Red
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        for wp in self.grid_waypoints:
            p = Point()
            p.x = wp[0]
            p.y = wp[1]
            p.z = -wp[2]  # Flip Z
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)

        # Draw spheres at each waypoint
        for i, wp in enumerate(self.grid_waypoints):
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp[0]
            sphere.pose.position.y = wp[1]
            sphere.pose.position.z = -wp[2]
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.r = 0.0
            sphere.color.g = 1.0  # Green
            sphere.color.b = 0.0
            sphere.color.a = 0.7

            marker_array.markers.append(sphere)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GridVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()