#!/usr/bin/env python

import numpy as np
import rospy
import pdb

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

class LaneDetector():
	"""
	A class for applying your cone detection algorithms to the real robot.
	Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
	Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
	"""
	def __init__(self):
		# toggle line follower vs cone parker
		self.LineFollower = False

		# Subscribe to ZED camera RGB frames
		self.cone_pub = rospy.Publisher("/relative_center_px", ConeLocationPixel, queue_size=10)
		self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
		self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
		self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

	def image_callback(self, image_msg):
		# Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
		# From your bounding box, take the center pixel on the bottom
		# (We know this pixel corresponds to a point on the ground plane)
		# publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
		# convert it to the car frame.

		#################################
		# detect the cone and publish its
		# pixel location in the image.
		# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
		#################################

		image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
		# flip camera image if upsidedown
		ZED_UPSIDEDOWN = True
		LINE_FOLLOWER = False
		rotated = image
		if (ZED_UPSIDEDOWN):
			rotated = cv2.rotate(image, cv2.ROTATE_180)

		if (LINE_FOLLOWER):
			h = image_msg.height
			w = image_msg.width
			rotated = rotated[int(3*h/4.0):, 0:]
		debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
		

		center_pixel = self.cd_color_segmentation(rotated, "no_template")
		pixel_msg = ConeLocationPixel()
		pixel_msg.u = center_pixel[0]
		pixel_msg.v = center_pixel[1]
		#self.debug_pub.publish(image)
		self.cone_pub.publish(pixel_msg)


	def image_print(self, img):
		"""
		Helper function to print out images, for debugging. Pass them in as a list.
		Press any key to continue.
		"""
		cv2.imshow("image", img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	def merge_lines(self, lines, slope_filter):
		"""
		Filters lines using slope_filter condition returns the slope 
		and intercept that is the average of those lines.

		NOTE: slopes are flipped because y is flipped. So positive slope -> right side, negative -> left.
		Returns None if slope range is empty.
		"""
		sum_slope = 0
		sum_intercept = 0
		num_lines = 0
		for line in lines:
			for x1,y1,x2,y2 in line:
				line_slope = (y2 - y1) / (x2 - x1)
				if (slope_filter(line_slope)):
					num_lines += 1
					intercept = y1/line_slope - x1
					sum_slope += line_slope
					sum_intercept += intercept
		print(num_lines)
		
		if (num_lines > 0):
			return ((sum_slope/num_lines), (sum_intercept/num_lines))
		print("No lines in slope range")
		return None

	def line_in_image(self, slope, intercept, xmax, ymax):
		ymin = 0
		xmin = 0
		# calculate x-coordinate of entry point
		entry_x = xmin
		entry_y = slope * entry_x + intercept

		# check if entry point is out of bounds
		if entry_y < ymin:
			entry_y = ymin
			entry_x = (entry_y - intercept) / slope
		elif entry_y > ymax:
			entry_y = ymax
			entry_x = (entry_y - intercept) / slope

		# calculate x-coordinate of exit point
		exit_x_top = (ymax - intercept) / slope
		exit_x_bottom = (ymin - intercept) / slope

		if ymin <= exit_x_top <= ymax:
			exit_x = exit_x_top
			exit_y = ymax
		elif ymin <= exit_x_bottom <= ymax:
			exit_x = exit_x_bottom
			exit_y = ymin
		else:
			exit_y = slope * xmax + intercept
			if exit_y < ymin:
				exit_y = ymin
				exit_x = (exit_y - intercept) / slope
			elif exit_y > ymax:
				exit_y = ymax
				exit_x = (exit_y - intercept) / slope
			else:
				exit_x = xmax

		# check if exit point is out of bounds in x-direction
		if exit_x < xmin:
			exit_x = xmin
			exit_y = slope * exit_x + intercept
		elif exit_x > xmax:
			exit_x = xmax
			exit_y = slope * exit_x + intercept

		return (int(entry_x), int(entry_y), int(exit_x), int(exit_y))

		#  # calculate x-coordinate of entry point
		# entry_x = 0
		# entry_y = slope * entry_x + intercept

		# # check if entry point is out of bounds
		# if entry_y < 0:
		#     entry_y = 0
		#     entry_x = (entry_y - intercept) / slope
		# # calculate x-coordinate of exit point
		# exit_x = x_max
		# exit_y = slope * exit_x + intercept
		# # check if exit point is out of bounds
		# if exit_y > y_max:
		#     exit_y = y_max
		#     exit_x = (exit_y - intercept) / slope
		# return (int(entry_x), int(entry_y), int(exit_x), int(exit_y))
	
	def select_longest(self, lines, slope_filter):
		"""
		Finds longest line that matches slope_filter condition.
		Returns 'None' if no line found.
		"""
		max_line = [None]
		max = 0
		for line in lines:
			for x1,y1,x2,y2 in line:
				line_slope = float(y2 - y1) / (x2 - x1)
				if (slope_filter(line_slope, x1, y1, x2, y2)):
					length = ((x2-x1)**2 + (y2-y1)**2)**(1/2)
					if (length > max):
						max = length
						max_line = line[0]
		return max_line

	def cd_color_segmentation(self, img, template):
		"""
		Implement the cone detection using color segmentation algorithm
		Input:
			img: np.3darray; the input image with a cone to be detected. BGR.
			template_file_path; Not required, but can optionally be used to automate setting hue filter values.
		Return:
			bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
					(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
		"""
		LANE_SLOPE_MIN = 0.1
		LANE_SLOPE_MAX = 1
		LANE_Y_THRESHOLD = 220
		########## YOUR CODE STARTS HERE ##########
		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
		#image_print(hsv_img)
		# orange ranges in hues from 30-45
		# settings for good performance on testing: (0, 220, 90), (45, 255, 255)
		mask = cv2.inRange(img, (170, 170, 170), (255, 255, 255))
		#image_print(mask)
		kernel = np.ones((3,3), np.uint8)
		eroded = cv2.erode(mask, kernel)
		dilated = cv2.dilate(eroded, kernel)
		#image_print(eroded)
		#image_print(dilated)
		# gray
		#gray = cv2.cvtColor(mask ,cv2.COLOR_BGR2GRAY)
		# blur
		kernel_size = 5
		blur_gray = cv2.GaussianBlur(mask,(kernel_size, kernel_size),0)
		# canny edge detection
		low_threshold = 50
		high_threshold = 150
		edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
		# hough for lines
		rho = 1  # distance resolution in pixels of the Hough grid
		theta = np.pi / 180  # angular resolution in radians of the Hough grid
		threshold = 70  # minimum number of votes (intersections in Hough grid cell)
		min_line_length = 70 # minimum number of pixels making up a line
		max_line_gap = 20  # maximum gap in pixels between connectable line segments
		line_image = np.copy(img) * 0  # creating a blank to draw lines on

		# Run Hough on edge detected image
		# Output "lines" is an array containing endpoints of detected line segments
		lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
							min_line_length, max_line_gap)
		# print(lines.shape)
		# slopes = (lines[:,:, 3] - lines[:,:, 2] / lines[:,:, 1] - lines[:,:, 0])
		# THRESHOLD = 0.1
		# dup_ids = set()
		# for i in range(len(slopes)):
		# 	for j in range(i+1, len(slopes)):
		# 		if abs(slopes[i] - slopes[j]) < THRESHOLD:
		# 			dup_ids.add(j)
		# print(dup_ids)
		# singleton_ids = [i for i in range(len(slopes)) if i not in dup_ids]
		# lines = lines[singleton_ids]
		
		# filter out lines that are not in correct slope range (too flat)
		# 	or that are too high in the image (y too low)
		filtered_lines = []
		if lines is not None:
			for line in lines:
				for x1,y1,x2,y2 in line:
					line_slope = abs(float(y2 - y1) / (x2 - x1))
					# print(line_slope)
					if (line_slope > LANE_SLOPE_MIN and max(y2, y1) > LANE_Y_THRESHOLD):
						#print(line)
						filtered_lines.append(line)
						cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
		if (len(filtered_lines) == 0):
			print('no good lines found')
			return
		filtered_lines = np.array(filtered_lines)
		# right_line = merge_lines(lines, lambda slope: slope > LANE_SLOPE_MIN)
		# left_line = merge_lines(filtered_lines, lambda slope: slope < -1 * LANE_SLOPE_MIN)
		# print(left_line)
		# image_height = img.shape[0]
		# image_width = img.shape[1]
		# #RIGHT LINE
		# right_line_frame = line_in_image(right_line[0], right_line[1], image_width, image_height)
		# left_line_frame = line_in_image(left_line[0], left_line[1], image_width, image_height)

		# cv2.line(line_image,(right_line_frame[0],right_line_frame[1]),(right_line_frame[2],right_line_frame[3]),(0,255,0),5)
		# cv2.line(line_image,(left_line_frame[0],left_line_frame[1]),(left_line_frame[2],left_line_frame[3]),(0,255,0),5)
		# #cv2.line(line_image, (0, 0), (10, 1000), 5)
		print(filtered_lines.shape)
		width = img.shape[1]
		def left_filter(slope, x1, y1, x2, y2):
			if slope < -1 * LANE_SLOPE_MIN:
				# if y2 is higher than y1 (less is higher)
				if (y1 > y2):
					# check if higher x is in right 1/3
					return x2 < (width * 2/3)
				else:
					return x1 < (width * 2/3)
			return False
		
		def right_filter(slope, x1, y1, x2, y2):
			if slope > LANE_SLOPE_MIN:
				# if y2 is higher than y1 (less is higher)
				if (y1 > y2):
					# check if higher x is in right 1/3
					return x2 > (width * 1/3)
				else:
					return x1 > (width * 1/3)
			return False
					
		longest_right = self.select_longest(filtered_lines, right_filter)
		longest_left = self.select_longest(filtered_lines, left_filter)
		print(lines.shape)
		# Draw the lines on the  image
		"""
                if(longest_right[0] == None):
			print("no longest right found")
		else:
			cv2.line(line_image,(longest_right[0],longest_right[1]),(longest_right[2],longest_right[3]),(0,0,255),5)
		if(longest_left[0] == None):  
			print("no longest left found")
		else:
			cv2.line(line_image,(longest_left[0],longest_left[1]),(longest_left[2],longest_left[3]),(0,0,255),5)
		"""
        #lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

		# Draw center pixel on the image
		lookahead = 150 # distance from bottom edge
		n = img.shape[0]
		print(img.shape)
		midy = n - lookahead
		one_lane_offset = 10
		if (longest_right[0] == None and longest_left[0] == None):
			print("no lines!")
			mid_x = image.shape[1]/2
			return (mid_x, lookahead)
		elif (longest_right[0] == None):
			print("no longest right")
			slope1 = -(longest_left[3]-longest_left[1])/(longest_left[2]-longest_left[0])
			midx1 = longest_left[0] + (longest_left[1]-midy)//slope1
			pixel = (midx1 + 300, midy)
			print(pixel)
			return pixel
		elif (longest_left[0] == None):
			print("no longest left")
			slope2 = -(longest_right[3]-longest_right[1])/(longest_right[2]-longest_right[0])
			midx2 = longest_right[0] + (longest_right[1]-midy)//slope2
			pixel = (midx2 + one_lane_offset, midy)
			print(pixel)
			return pixel
		x11 = longest_left[0]
		y11 = longest_left[1]
		x22 = longest_right[2]
		y22 = longest_right[3]
		# slope calcs assume convention ind 0, 1, 2, 3 = x1, y1, x2, y2
		slope1 = -(longest_left[3]-longest_left[1])/(longest_left[2]-longest_left[0])
		#print(slope1)
		slope2 = -(longest_right[3]-longest_right[1])/(longest_right[2]-longest_right[0])
		#print(slope2)

		midx1 = x11 + (y11-midy)//slope1
		#print('midx1',midx1)
		midx2 = x22 + (y22-midy)//slope2
		#print('midx2',midx2)
		midx = (midx1+midx2)/2

		center_pixel = (midx,midy)

		# draws intermediate and center pixel locations
		# lines_edges_center = cv2.circle(lines_edges,(int(center_pixel[0]),int(center_pixel[1])) , 5, (0, 255, 0), 2) # green
		# lines_edges_midx1 = cv2.circle(lines_edges_center,(int(midx1),int(midy)) , 5, (255, 255, 0), 2) # cyan
		# lines_edges_midx2 = cv2.circle(lines_edges_midx1,(int(midx2),int(midy)) , 5, (255, 0, 255), 2) # purple

		# lines_edges_center = cv2.addWeighted(lines_edges, 0.8, target_pixel, 1, 0)

		# cv2.imshow('test',lines_edges)
		# cv2.imshow('test',lines_edges_center)
		# waits for user to press any key
		# (this is necessary to avoid Python kernel form crashing)
		# cv2.waitKey(0) 
		# closing all open windows
		# cv2.destroyAllWindows()

		return center_pixel


### status checks/safety controller

""" slope_nomimal = np.tan(24.5*np.pi/180)
arg_line1 = np.arctan(slope1)
arg_line2 = np.arctan(slope2)
arg_sum = (arg_line1 + arg_line2)
kp = 1
if abs(arg_sum) > (45*np.pi/180)
	drive.steering_angle = kp*arg_sum # assuming positive steering is right, check this """

if __name__ == '__main__':

    try:
        rospy.init_node('LaneDetector', anonymous=True)
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

		
