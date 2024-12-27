import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/object_markers', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov5n.pt') 
        self.log_file = open("../detections/detected_objects.log", "w")

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #object detection
        results = self.model(cv_image)

        #visualization in RViz2
        markers = MarkerArray()
        for idx, detection in enumerate(results.xyxy[0]):
            x_min, y_min, x_max, y_max, conf, cls = detection.tolist()
            label = self.model.names[int(cls)]
            self.get_logger().info(f'Detected: {label} at [{x_min}, {y_min}, {x_max}, {y_max}]')

            self.log_file.write(f'Detected: {label} at [{x_min}, {y_min}, {x_max}, {y_max}]\n')

            #draw bounding boxes and labels on the image
            cv2.rectangle(cv_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
            cv2.putText(cv_image, f'{label} {conf:.2f}', (int(x_min), int(y_min) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            #create a marker for RViz2
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = (x_min + x_max) / 2.0
            marker.pose.position.y = (y_min + y_max) / 2.0
            marker.pose.position.z = 1.0  #depth
            marker.text = label
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.scale.z = 0.2 
            marker.id = idx
            markers.markers.append(marker)

        # Publishing processed image
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(processed_msg)

        # Publishing markers
        self.marker_publisher.publish(markers)

    def destroy(self):
        self.log_file.close()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
