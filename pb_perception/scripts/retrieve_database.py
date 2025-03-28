#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Import Point message type
from pb_perception.palmtree_db import PalmTreeDB  # Import the database module
from example_interfaces.srv import AddTwoInts  # Use AddTwoInts service to handle integer tag_id

class RetrieveTreePositionNode(Node):
    def __init__(self):
        super().__init__('retrieve_tree_position_node')
        self.db = PalmTreeDB("/home/palmbee1/palmbee_ws/src/PalmBee/pb_perception/database/test.db")
        self.publisher = self.create_publisher(Point, 'palmbee/tree/position', 10)
        self.service = self.create_service(AddTwoInts, 'palmbee/tree/request_position', self.handle_service_request)
        self.tag_id = None  # Initialize tag_id as None
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer to check and publish

    def handle_service_request(self, request, response):
        self.tag_id = request.a  # Use the first integer (a) as the tag_id
        response.sum = 0  # Placeholder response value
        self.get_logger().info(f"Tag ID {self.tag_id} set for publishing.")
        return response

    def timer_callback(self):
        if self.tag_id is not None:  # Only publish if a valid tag_id is set
            tree_data = self.db.get_tree(str(self.tag_id))
            if tree_data:
                point = Point()
                point.x = tree_data['tag_x']
                point.y = tree_data['tag_y']
                point.z = tree_data['tag_z']
                self.publisher.publish(point)
                self.get_logger().info(f"Position of tag_id {self.tag_id}: ({point.x}, {point.y}, {point.z})")
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