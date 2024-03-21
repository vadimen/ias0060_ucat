# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Vector3

from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench


class KeyPressPublisher(Node):

    def __init__(self):
        super().__init__('key_press_publisher')
        self.pub_vel = self.create_publisher(WrenchStamped, '/ucat/force_fins', 10)  # Adjust queue size as needed
        self.get_logger().info("Keyboard node started")
        self.key_map = {
            'w': WrenchStamped(wrench=Wrench(force=Vector3(x=50.0))),
            's': WrenchStamped(wrench=Wrench(force=Vector3(x=-50.0))),
            'a': WrenchStamped(wrench=Wrench(force=Vector3(y=-50.0))),
            'd': WrenchStamped(wrench=Wrench(force=Vector3(y=50.0))),
            'q': WrenchStamped(wrench=Wrench(force=Vector3(z=50.0))),
            'e': WrenchStamped(wrench=Wrench(force=Vector3(z=-50.0))),
            'z': WrenchStamped(wrench=Wrench(torque=Vector3(z=-25.0))),
            'c': WrenchStamped(wrench=Wrench(torque=Vector3(z=25.0))),
        }

    def listen_for_key(self):
        try:
            from pynput import keyboard
            def on_press(key):
                try:
                    msg = self.key_map.get(key.char.lower())
                    if msg:
                        self.pub_vel.publish(msg)
                        self.get_logger().info(f"Key pressed: {key.char}")
                except AttributeError:
                    pass  # Ignore non-character keys

            listener = keyboard.Listener(on_press=on_press)
            listener.start()
            listener.join()
        except ImportError:
            self.get_logger().error("Failed to import pynput. Install it using 'pip install pynput'")

def main():
    rclpy.init()
    node = KeyPressPublisher()
    node.listen_for_key()
    rclpy.shutdown()

if __name__ == '__main__':
    main()