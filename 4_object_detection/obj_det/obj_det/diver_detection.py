import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
from sensor_msgs.msg import Image
import cv2

model = YOLO('/home/ias0060/dev_ws/src/obj_det/obj_det/best.pt') 

class DiverDetection(Node):

    def __init__(self):
        super().__init__('image_forwarder')
        self.sub_topic = self.declare_parameter('subscriber_topic', '/camera/image_raw').value
        self.pub_topic = self.declare_parameter('publisher_topic', '/output_image').value
        self.subscription = self.create_subscription(
            Image,
            self.sub_topic,
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Image, self.pub_topic, 10)
        self.br = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = model(cv_image)

        if results is not None and len(results[0].boxes.conf) > 0:
            print(results[0].boxes)
            conf = results[0].boxes.conf[0].item()
            #round conf to 2 decimal places
            #conf = round(conf, 2)
            box = results[0].boxes.data[0].tolist()[0:4]

            cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            #cv2.putText(cv_image, str(conf), (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Convert OpenCV image to ROS Image message
            msg = self.br.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = DiverDetection()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
