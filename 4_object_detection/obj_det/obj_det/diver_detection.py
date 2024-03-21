import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
from sensor_msgs.msg import Image

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

        img = results[0].plot()

        # Convert OpenCV image back to ROS Image message
        ros_image = self.br.cv2_to_imgmsg(img, encoding="bgr8")

        # Publish the image to the new topic
        self.publisher.publish(ros_image)

def main(args=None):
  rclpy.init(args=args)
  node = DiverDetection()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
