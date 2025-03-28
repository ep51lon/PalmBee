#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Import Point message type
from pb_perception.palmbee_db import PalmTreeDB  # Import the database module

class RetrieveTreePositionNode(Node):
    def __init__(self):
        super().__init__('retrieve_tree_position_node')
        self.db = PalmTreeDB("/home/palmbee1/palmbee_ws/src/PalmBee/pb_perception/database/test.db")
        self.publisher = self.create_publisher(Point, 'tree_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second
        self.tag_id = "T001"  # Example tag_id, replace with dynamic input if needed

    def timer_callback(self):
        tree_data = self.db.get_tree(self.tag_id)
        if tree_data:
            point = Point()
            point.x = tree_data['tag_x']
            point.y = tree_data['tag_y']
            point.z = tree_data['tag_z']
            self.publisher.publish(point)
            self.get_logger().info(f"Published position for tag_id {self.tag_id}: ({point.x}, {point.y}, {point.z})")
        else:
            self.get_logger().warn(f"No data found for tag_id {self.tag_id}")

    def destroy_node(self):
        self.db.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RetrieveTreePositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()