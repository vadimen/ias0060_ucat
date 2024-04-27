import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import WrenchStamped
import csv
import os
from geometry_msgs.msg import Twist 


class DepthControl(Node):
    def __init__(self):
        super().__init__('depthControl')
        self.nr_samples = 500

        self.desired_depth = None
        self.current_depth = None
        self.dt = 0.1
        self.last_errors = []
        self.rms_errors = []
        self.previous_error = 0

        self.P = 1
        self.I = 0.5
        self.D = 0.8

        if not os.path.exists('csv'):
            os.makedirs('csv')

        self.csv_file_name = f'csv/pid_{self.P}_{self.I}_{self.D}.csv'
        self.csv_file = open(self.csv_file_name, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow(['Current Depth', 'Error', 'RMS', 'Signal'])

        self.row_count = 0  # Initialize row count

        self.subscription = self.create_subscription(
            FluidPressure,
            '/pressure',
            self.depthCallback,
            10)

        self.publisher = self.create_publisher(
            WrenchStamped,
            '/ucat/force_req',
            10)

        self.timer = self.create_timer(self.dt, self.pid_callback)
        self.subscription  # prevent unused variable warning

    def depthCallback(self, pressure):
        self.current_depth = (pressure.fluid_pressure - 101325) / 9.80665 / 1023.6
        if self.desired_depth is None:  # first init
            self.desired_depth = self.current_depth - 2
        #self.get_logger().info('Current depth: "%s"' % self.current_depth)

    def pid_callback(self):
        if self.current_depth is not None:  # after first init
            error = self.desired_depth - self.current_depth
            self.rms_errors.append(error ** 2)
            self.last_errors.append(error * self.dt)            

            desired_value = self.P * error + self.I * sum(self.last_errors[-50:]) + \
                                        self.D * (error - self.previous_error) / self.dt

            # Create message
            msg = WrenchStamped()
            msg.wrench.force.z = desired_value

            print("Publishing: ", desired_value)

            self.publisher.publish(msg)

            # Write the current_depth and the error to a CSV file
            rms = (sum(self.rms_errors) / len(self.rms_errors)) ** 0.5
            self.csv_writer.writerow([self.current_depth, error, rms, desired_value])
            self.get_logger().info(f"Current depth: {self.current_depth}, Error: {error}, RMS: {rms}")
            self.row_count += 1

            self.previous_error = error

            if self.row_count >= self.nr_samples:
                self.csv_file.close()
                self.get_logger().info(f'CSV file {self.csv_file_name} closed')
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    subscriber = DepthControl()
    rclpy.spin(subscriber)
    # Removed subscriber.destroy_node() as shutdown is called in pid_callback

if __name__ == '__main__':
    main()