import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32

from std_msgs.msg import String, Int16
import subprocess

from geometry_msgs.msg import Pose
import time
from rclpy.executors import SingleThreadedExecutor
import threading
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from std_msgs.msg import Empty



class RoboTaxiController(Node):

    def __init__(self):
        super().__init__('robotaxi_controller')
        self.create_subscription(String,'/voice_cmd',self.voice_cmd_callback,10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.distance_subscriber = self.create_subscription(Float32, "/distance", self.distance_callback, 10)
        self.x = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.curr_dist = 0
        
        self.thread_executor = ThreadPoolExecutor(max_workers=1)

        self.move_executor = SingleThreadedExecutor()
        move_thread = threading.Thread(target=self.move_executor.spin)
        move_thread.start()
        print('ROSGPT Robo RoboTaxi Controller Started. Waiting for input commands ...')
    

    def distance_callback(self, msg):
        if isinstance(msg, Float32):
            self.curr_dist = msg.data
        elif isinstance(msg, Int16):
            self.curr_dist = float(msg.data)  # Convert Int16 to float for consistency
        else:
            print('Received an unexpected message type for the /distance topic.')
            return

        # print(f'Updated Distance: {self.curr_dist}')

    def stop(self):
        print('Stopping the robotaxi ...')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        self.velocity_publisher.publish(twist_msg)
        print('The Robot has stopped...')

    #this callback represents the ROSGPTParser. It takes a JSON, parses it, and converts it to a ROS 2 command
    def voice_cmd_callback(self, msg):
        #print(msg.data)
        try:
            cmd = json.loads(msg.data)
            cmd = json.loads(cmd['json']) #we only consider the pure json message. cmd['text'] contains a mix of text and json
            print('JSON command received: \n',cmd,'\n')

            if cmd['action'] == 'move':
                linear_speed = cmd['params'].get('linear_speed', 0.2)
                distance = cmd['params'].get('distance', 1.0)
                direction = cmd['params'].get('direction', "forward")
                print(f'linear_speed: {linear_speed}, distance: {distance}, direction: {direction}')

                self.thread_executor.submit(self.move, linear_speed, distance, direction)

            elif cmd['action'] == 'stop':
                self.thread_executor.submit(self.stop)

            elif cmd['action'] == 'self_drive':
                self.thread_executor.submit(self.self_drive)

            elif cmd['action'] == 'rotate':
                angular_speed = cmd['params'].get('angular_speed', 0.2)
                angle = float(cmd['params'].get('angle', 1.0))
                direction = cmd['params'].get('direction', "clockwise")

                print(f'angular_speed: {angular_speed}, angle: {angle}, direction: {direction}')
                self.thread_executor.submit(self.rotate, angular_speed, angle, direction)
 
        except json.JSONDecodeError:
            print('[json.JSONDecodeError] Invalid or empty JSON string received:', msg.data)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))   


    def get_distance(self):
        return copy.copy(self.curr_dist)
    
    def self_drive(self):
        try:
            # Start the computer_vision_node
            subprocess.Popen(["ros2", "run", "self_driving_car_pkg", "computer_vision_node"])
            print("Successfully started computer_vision_node.")
        except Exception as e:
            print(f"Failed to start computer_vision_node: {str(e)}")
    

    def move(self, linear_speed, distance, direction): 
        print(f'Start moving the robotaxi {direction} at {linear_speed} m/s for a distance of {distance} meters')

        linear_vector = Vector3()

        if direction == "forward":
            linear_vector.x = linear_speed
        elif direction == "backward":
            linear_vector.x = -linear_speed
        else:
            print('Invalid direction provided')
            return

        twist_msg = Twist()
        twist_msg.linear = linear_vector

        # Reset distance traveled
        traveled_distance = 0
        initial_distance = self.get_distance() 

        try:
            while traveled_distance < distance:
                self.velocity_publisher.publish(twist_msg)
                self.move_executor.spin_once(timeout_sec=0.1)
                current_distance = self.get_distance()
                traveled_distance = abs(current_distance - initial_distance)
                print(traveled_distance)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))
            

        print("stop here")
        # Stop the movement after reaching the target distance
        self.stop()

        print("Stopping the robotaxi ...")


    def rotate(self, angular_speed, angle, direction): 
        print(f'Start rotating the robotaxi {direction} at {angular_speed} rad/s for 5 seconds.')

        angular_vector = Vector3()
        linear_vector = Vector3()
        linear_vector.x = 2.0

        if direction == "clockwise":
            angular_vector.z = -angular_speed
        elif direction == "anticlockwise":
            angular_vector.z = angular_speed
        else:
            print('Invalid direction provided')
            return

        twist_msg = Twist()
        twist_msg.angular = angular_vector
        twist_msg.linear = linear_vector

        try:
            start_time = time.time()
            while time.time() - start_time < 5:  # Loop for 5 seconds
                self.velocity_publisher.publish(twist_msg)
                print("publish")
                self.move_executor.spin_once(timeout_sec=0.1)

        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))

        # Stop the rotation after 5 seconds
        self.stop()

        print("Stopping the robotaxi ...")


    
def main(args=None):
    rclpy.init(args=args)
    node = RoboTaxiController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()