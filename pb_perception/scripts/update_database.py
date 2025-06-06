#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
import math
from apriltag_msgs.msg import AprilTagDetectionArray  # Import the AprilTagDetectionArray message
import pandas as pd  # Import pandas
from sensor_msgs.msg import Image  # Import Image message
from cv_bridge import CvBridge, CvBridgeError  # Import CvBridge and CvBridgeError
import cv2  # Import OpenCV
import numpy as np  # Import NumPy
from pb_perception.palmtree_db import PalmTreeDB  # Import the database module
from collections import defaultdict  # Import defaultdict for storing tag positions

def calculate_distance(transform):
    translation = transform.transform.translation
    distance = math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)
    return distance

def print_transform(node, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    node.get_logger().info(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
    node.get_logger().info(f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

class CalculateDistanceNode(Node):
    def __init__(self):
        super().__init__('calculate_distance_node')
        self.initialize_components()

    def initialize_components(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.detections = []
        self.initialize_subscriptions()
        self.initialize_publishers()
        self.bridge = CvBridge()
        self.current_image = None
        self.df = pd.DataFrame(columns=['timestamp', 'tag_id', 'pos_x', 'pos_y', 'pos_z', 'distance'])
        self.last_append_time = self.get_clock().now().to_msg().sec
        self.tag_positions = defaultdict(list)
        self.db = PalmTreeDB("/home/palmbee1/palmbee_ws/src/PalmBee/pb_perception/database/test.db")

    def initialize_subscriptions(self):
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'apriltag/detections',
            self.detection_callback,
            10
        )
        self.image_subscription = self.create_subscription(
            Image,
            'zed/zed_node/left_gray/image_rect_gray',
            self.image_callback,
            10
        )

    def initialize_publishers(self):
        self.image_publisher = self.create_publisher(Image, 'palmbee/apriltag/annotated_image', 10)

    def detection_callback(self, msg):
        self.detections = [detection for detection in msg.detections]

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg().sec
        if self.current_image is not None:
            annotated_image = self.current_image.copy()
            self.process_detections(annotated_image)
            self.publish_annotated_image(annotated_image)
        else:
            self.get_logger().info(f"Waiting for tf...")

        if current_time - self.last_append_time >= 5:
            self.save_to_csv()
            self.last_append_time = current_time

    def process_detections(self, annotated_image):
        for detection in self.detections:
            tag_id = detection.id
            try:
                transform = self.tf_buffer.lookup_transform('map', f'tag36h11:{tag_id}', rclpy.time.Time())
                self.handle_detection(tag_id, transform, detection, annotated_image)
            except TransformException as ex:
                self.get_logger().warn(f"Transform for marker {tag_id} not available")

    def handle_detection(self, tag_id, transform, detection, annotated_image):
        print_transform(self, transform)
        distance = calculate_distance(transform)
        self.get_logger().info(f"Marker {tag_id} -> Distance: {distance}")
        self.annotate_image(annotated_image, detection, transform, tag_id, distance)
        self.update_dataframe(tag_id, transform, distance)
        self.update_tag_positions(tag_id, transform)
        self.update_database(tag_id)

    def annotate_image(self, annotated_image, detection, transform, tag_id, distance):
        corners = [(int(point.x), int(point.y)) for point in detection.corners]
        cv2.polylines(annotated_image, [np.array(corners, dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
        translation = transform.transform.translation
        label = (f"ID: {tag_id},"
                 f"Pos: ({translation.x:.2f}, {translation.y:.2f}, {translation.z:.2f}),"
                 f"Dist: {distance:.2f}m")
        cv2.putText(annotated_image, label, (corners[0][0], corners[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    def update_dataframe(self, tag_id, transform, distance):
        timestamp = self.get_clock().now().to_msg().sec
        translation = transform.transform.translation
        new_row = pd.DataFrame([{
            'timestamp': timestamp,
            'tag_id': tag_id,
            'pos_x': translation.x,
            'pos_y': translation.y,
            'pos_z': translation.z,
            'distance': distance
        }])
        self.df = pd.concat([self.df, new_row], ignore_index=True)

    def update_tag_positions(self, tag_id, transform):
        translation = transform.transform.translation
        self.tag_positions[tag_id].append((translation.x, translation.y, translation.z))
        if len(self.tag_positions[tag_id]) > 20:
            self.tag_positions[tag_id].pop(0)

    def update_database(self, tag_id):
        mean_position = np.mean(self.tag_positions[tag_id], axis=0)
        tree_data = self.db.get_tree(str(tag_id))
        if tree_data:
            self.db.update_tree(
                str(tag_id),
                tag_position=(mean_position[0], mean_position[1], mean_position[2])
            )
        else:
            self.db.add_tree(
                str(tag_id),
                gps_coord=(0.0, 0.0, 0.0),  # Placeholder GPS coordinates
                tag_position=(mean_position[0], mean_position[1], mean_position[2]),
                center_pos=(0.0, 0.0, 0.0)  # Placeholder center position
            )

    def publish_annotated_image(self, annotated_image):
        try:
            annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.image_publisher.publish(annotated_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert annotated image: {e}")

    def save_to_csv(self):
        self.df.to_csv('detected_markers.csv', index=False)

    def destroy_node(self):
        self.db.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalculateDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()