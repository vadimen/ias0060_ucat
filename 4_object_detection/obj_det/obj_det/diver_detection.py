#i tested pid class by setting robot to some depth
#it works
#the next step: make robot center the line by using PID
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
import cv2

from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import WrenchStamped

model = YOLO('/home/ias0060/dev_ws/src/obj_det/obj_det/best.pt') 

class PID:
    #tutorial for myself
    #1. create class with kp, ki, kd
    #2. 
    def __init__(self, kp, ki, kd):
        self.dt = 0.1

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.desired_value = None
        self.last_errors = []
        self.previous_error = 0
        self.control_value = None 
        self.current_value = None

    def generate_control_value(self, current_value):
        self.current_value = current_value
        if self.desired_value is None:
            print("Desired value not set")
            return 0

        error = self.desired_value - current_value
        self.last_errors.append(error * self.dt)

        self.control_value = self.kp * error + self.ki * sum(self.last_errors[-50:]) + \
            self.kd * (error - self.previous_error) / self.dt

        self.previous_error = error

    def get_control_value(self):
        return self.control_value

    def set_desired_value(self, value):
        self.desired_value = value

    def get_desired_value(self):
        return self.desired_value

    def get_current_value(self):
        return self.current_value 

class DiverDetection(Node):

    def __init__(self):
        super().__init__('image_forwarder')
        self.sub_topic = self.declare_parameter('subscriber_topic', '/camera/image_raw').value
        self.pub_topic = self.declare_parameter('publisher_topic', '/output_image').value
        # self.subscription = self.create_subscription(
        #     Image,
        #     self.sub_topic,
        #     self.image_callback,
        #     10)
        # self.subscription  # prevent unused variable warning
        # self.publisher = self.create_publisher(Image, self.pub_topic, 10)
        self.br = CvBridge()
        self.dt = 0.1

        self.xl = None
        self.yl = None
        self.line_len = 100
        self.half_line_len = self.line_len // 2

        self.xPID = PID(kp=0.1, ki=0.01, kd=0.01)
        self.yPID = PID(kp=0.1, ki=0.01, kd=0.01)

        self.lenPID = PID(kp=0.1, ki=0.01, kd=0.01)
        #self.lenPID.set_desired_value(self.line_len)

        #testing pid on depth
        #
        #
    #     self.depthPID = PID(kp=1, ki=0.5, kd=0.8)

    #     self.depth_subscription = self.create_subscription(
    #         FluidPressure,
    #         '/pressure',
    #         self.depth_value_receiver,
    #         10)

    #     self.depth_publisher = self.create_publisher(
    #         WrenchStamped,
    #         '/ucat/force_req',
    #         10)

    #     self.depth_timer = self.create_timer(self.dt, self.depth_value_sender)
    
    # def depth_value_sender(self):
    #     if self.depthPID.get_desired_value() is not None:# after first init
    #         print("Sending depth value")
    #         control_value = self.depthPID.get_control_value()
    #         # Create message
    #         msg = WrenchStamped()
    #         msg.wrench.force.z = control_value

    #         print("Publishing: ", control_value)

    #         self.depth_publisher.publish(msg)

    #         # Write the current_depth and the error to a CSV file
    #         self.get_logger().info(f"Current depth: {self.depthPID.get_current_value()}")

    # def depth_value_receiver(self, pressure):
    #     current_depth = (pressure.fluid_pressure - 101325) / 9.80665 / 1023.6
    #     print("We are now at depth: ", current_depth)
    #     if self.depthPID.get_desired_value() is None:  # first init
    #         self.depthPID.set_desired_value(current_depth - 2)
    #     self.depthPID.generate_control_value(current_depth)

    # def image_callback(self, msg):
    #     return
    #     # Convert ROS Image message to OpenCV image
    #     cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    #     if self.xl is None:
    #         self.xl = cv_image.shape[1] // 2
    #         self.yl = cv_image.shape[0] // 2

    #     results = model(cv_image)

    #     #draw a line in the middle of the image
    #     cv2.line(cv_image, (self.xl - self.half_line_len, self.yl), 
    #                 (self.xl + self.half_line_len, self.yl), (0, 0, 255), 2)

    #     if results is not None and len(results[0].boxes.conf) > 0:
    #         print(results[0].boxes)
    #         conf = results[0].boxes.conf[0].item()
    #         box = results[0].boxes.data[0].tolist()[0:4]

    #         cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)

    #         #draw circle in middle of the bounding box
    #         x = int((box[0] + box[2]) / 2)
    #         y = int((box[1] + box[3]) / 2)
    #         cv2.circle(cv_image, (x, y), 5, (0, 255, 0), -1)

    #         # Convert OpenCV image to ROS Image message
    #         msg = self.br.cv2_to_imgmsg(cv_image, encoding="bgr8")

    #         # get bounding box length
    #         len = box[2] - box[0]
    #         # update PID controllers
    #         desired_distance = lenPID.update(len)


    #     self.publisher.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = DiverDetection()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
