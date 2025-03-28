#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Import Point message type
from pb_perception.palmtree_db import PalmTreeDB  # Import the database module
from std_srvs.srv import SetBool  # Import service type for simplicity

class RetrieveTreePositionNode(Node):
    def __init__(self):
        super().__init__('retrieve_tree_position_node')
        self.db = PalmTreeDB("/home/palmbee1/palmbee_ws/src/PalmBee/pb_perception/database/test.db")
        self.publisher = self.create_publisher(Point, 'palmbee/tree/position', 10)
        self.service = self.create_service(SetBool, 'palmbee/tree/request_position', self.handle_service_request)
        self.tag_id = None  # Initialize tag_id as None
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer to check and publish

    def handle_service_request(self, request, response):
        if request.data:  # Assuming the tag_id is passed as a string in the request
            self.tag_id = str(request.data)  # Set the tag_id from the service request
            response.success = True
            response.message = f"Tag ID {self.tag_id} set for publishing."
            self.get_logger().info(response.message)
        else:
            self.tag_id = None  # Reset tag_id if no valid input
            response.success = False
            response.message = "Invalid tag ID provided."
            self.get_logger().warn(response.message)
        return response

    def timer_callback(self):
        if self.tag_id:  # Only publish if a valid tag_id is set
            tree_data = self.db.get_tree(self.tag_id)
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