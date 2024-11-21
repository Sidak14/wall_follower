#!/usr/bin/env python3

# ROS 2 program to subscribe to real-time streaming 
# video from TurtleBot3 Pi camera and find coloured landmarks.
# Author: Claude Sammut

# Colour segementation and connected components example added by Claude sSammut
	
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import math

from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import LaserScan # Laser scan message type
from sensor_msgs.msg import CameraInfo # Need to know camera frame
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped

import wall_follower.landmark
from wall_follower.landmark import marker_type


field_of_view_h = 62.2
field_of_view_v = 48.8
markers = {}
not_logged = True


class SeeMarker(Node):
	"""
	Create an ImageSubscriber class, which is a subclass of the Node class.
	"""
	def __init__(self):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a name
		super().__init__('see_marker')

		self.range = 0
		self.prev_h = 0
		self.prev_r = 0

			
		# Create the subscriber. This subscriber will receive an Image
		# from the video_frames topic. The queue size is 10 messages.
		self.subscription = self.create_subscription(
			Image,
			'/camera/image_raw/uncompressed', 
			self.listener_callback, 
			10)
		self.subscription # prevent unused variable warning

		self.should_print = self.create_subscription(
			String,
			'/n_start',
			self.stop_now,
			10
		)
		self.should_print
			
		# Used to convert between ROS and OpenCV images
		self.br = CvBridge()

		self.point_publisher = self.create_publisher(PointStamped, '/marker_position', 10)

	def stop_now(self, data):
		global not_logged
		if not_logged:
			self.get_logger().info("Logging Markers")
			f = open("./out_wall_2.csv", "a")
			for marker in markers.keys():
				f.write(f"{markers[marker][0]}, {markers[marker][1]}, {marker}\n")
				self.get_logger().info(f"{markers[marker][0]}, {markers[marker][1]}, {marker}")
			f.close()
			not_logged = False
			

	def listener_callback(self, data):
		"""
		Callback function.
		"""
		# Display the message on the console
		# self.get_logger().info('Receiving video frame')
 
		# Convert ROS Image message to OpenCV image
		current_frame = self.br.imgmsg_to_cv2(data, 'bgra8')

		# The following code is a simple example of colour segmentation
		# and connected components analysis
		
		# Convert BGR image to HSV
		hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

		# Find pink blob
		pink_blob = segment(current_frame, hsv_frame, "pink")
		if pink_blob:
			(pink_x, pink_y, pink_h, p_d, p_a) = pink_blob

			for c in ["blue", "green", "yellow"]:
				blob = segment(current_frame, hsv_frame, c)
				if blob:
					(c_x, c_y, c_h, c_d, c_a) = blob

					# Check to see if the blobsa are verically aligned
					if abs(pink_x - c_x) > pink_h:
#						print(f'pink_x = {pink_x}, pink_y = {pink_y}, h = {pink_h}')
						continue

					marker_at = PointStamped()
					marker_at.header.stamp = self.get_clock().now().to_msg()
					marker_at.header.frame_id = 'camera_link'

					if c_y < pink_y:	# +y is down
						marker_at.point.z = float(marker_type.index(c + '/pink'))
					else:
						marker_at.point.z = float(marker_type.index('pink/' + c))
					
					#x, y = polar_to_cartesian(c_d, c_a)

					#marker_at.point.x = x
					#marker_at.point.y = y
					
					total_height = c_h + pink_h
					
					measured_d = 130.45 * math.exp(-0.017 * total_height)
					
					x, y = polar_to_cartesian(measured_d, c_a - 30)
					
					marker_at.point.x = x / 100
					marker_at.point.y = y / 100

#					print(f'Camera coordinates: {x}, {y}')
					self.point_publisher.publish(marker_at)
					
					self.get_logger().info('Published Point: x=%f, y=%f, z=%f colour=%s distance=%f' %
						(marker_at.point.x, marker_at.point.y, marker_at.point.z, c, measured_d))

					markers.update({marker_type[int(marker_at.point.z)]: [marker_at.point.x, marker_at.point.y]})
					self.get_logger().info(str(markers))

		# Display camera image
		cv2.imshow("camera", current_frame)	
		cv2.waitKey(1)


colours = {
	"pink":	 	((140,50,50), (170, 255, 255)),
	"blue":		((95,194,112), (140, 207, 165)),
	"green":	((75,142,79), (82, 210, 98)),
	"yellow":	((24,229,140), (27, 246, 200))
}


def segment(current_frame, hsv_frame, colour):
	"""
	Mask out everything except the specified colour
	Connect pixels to form a blob
	"""

	(lower, upper) = colours[colour]

	# Mask out everything except pink pixels
	mask = cv2.inRange(hsv_frame, lower, upper)
	result = cv2.bitwise_and(current_frame, current_frame, mask=mask)

	# Run 4-way connected components, with statistics
	blobs = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)

	# Display masked image
#	cv2.imshow("result", result)
 
	# Print statistics for each blob (connected component)
	return get_stats(blobs, colour)


def get_stats(blobs, colour):
	"""
	Print statistics for each blob (connected component)
	of the specified colour
	Return the centroid and height of the largest blob, if there is one.
	"""

	(numLabels, labels, stats, centroids) = blobs
	if numLabels == 0:
		return None

	largest = 0
	rval = None
	centre = 320 # 640/2

	for i in range(1, numLabels):
		x = stats[i, cv2.CC_STAT_LEFT]
		y = stats[i, cv2.CC_STAT_TOP]
		w = stats[i, cv2.CC_STAT_WIDTH]
		h = stats[i, cv2.CC_STAT_HEIGHT]
		area = stats[i, cv2.CC_STAT_AREA]
		(cx, cy) = centroids[i]
#		print(colour, x, y, w, h, area, cx, cy)

		if area > largest:
			largest = area
			distance = 35.772 * pow(h, -0.859) # obtained experimentally
			aspect_ratio = h/w
			if aspect_ratio < 0.8:
				if cx < centre:
					cx += h-w
				else:
					cx -= h-w
			angle = (centre - cx) * field_of_view_h / 480
			if angle < 0:
				angle += 360
			rval = (cx, cy, h, distance, angle)

	return rval


def polar_to_cartesian(distance, angle):
	# Convert angle from degrees to radians
	angle_rad = math.radians(angle)

	# Calculate x and y coordinates
	x = distance * math.cos(angle_rad)
	y = distance * math.sin(angle_rad)

	return x, y


def main(args=None):
	
	# Initialize the rclpy library
	rclpy.init(args=args)
	
	# Create the node
	see_marker = SeeMarker()
	
	# Spin the node so the callback function is called.
	rclpy.spin(see_marker)
	
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	see_marker.destroy_node()
	
	# Shutdown the ROS client library for Python
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()