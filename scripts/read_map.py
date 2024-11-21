#!/usr/bin/env python3

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
# import turtlebot3_navigation2
# import wall_follower.robot_navigator
import os 
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class ReadMap(Node):


    def __init__(self):
        super().__init__('read_map')
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )

        # path to map file

        map_path = os.path.join(os.path.dirname(__file__), '/home/rsa/colcon_ws/out_wall.csv')
        self.data = []
        try:
            self.data = self.load_csv_data(map_path)
            self.get_logger().info("CSV file loaded")

            for entry in self.data:
                # self.get_logger().info(f"x:{entry[0]}, Value:{entry[1]}, Type:{entry[2]}")
                self.get_logger().info(f"data: {entry}")
                go_to = PoseStamped()
                go_to.pose.position.x = entry[0]
                go_to.pose.position.y = entry[1]
                self.goal_pub.publish(go_to)

        except FileNotFoundError:
            self.get_logger().error("CSV file not found")
        except csv.Error:
            self.get_logger().error("Error reading csv file")

        start_at = PoseWithCovarianceStamped()
        start_at.pose.pose.position.x = self.data[0][0]
        start_at.pose.pose.position.y = self.data[0][1]
        start_at.pose.pose.orientation.z = self.data[0][2]
        self.init_pose_pub.publish(start_at)
    
        
        
    def listener_callback(self, data):
        pass 

    def load_csv_data(self, csv_path):
        data = []

        with open(csv_path, mode='r') as file:
            csv_reader = csv.reader(file)
            # read in top row as the starting 

            for i, row in enumerate(csv_reader):
                try:
                    if i == 0:
                        startx = float(row[0])
                        starty = float(row[1])
                        heading = float(row[2])
                        data.append((startx, starty, heading))
                    else:
                        x = float(row[0])
                        y = float(row[1])
                        markerType = row[2]
                        data.append((x, y, markerType))
                    # self.get_logger().info(f"data: {row[0]}, {row[1]}, {row[2]}")
                except ValueError as e:
                    self.get_logger().info("error parsing")
        return data
    

def main(args=None):
    rclpy.init(args=args)
    readmapnode = ReadMap()
    # nav = BasicNavigator()
    # nav.setInitialPose(init_pose)
    rclpy.spin(readmapnode)
    readmapnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()