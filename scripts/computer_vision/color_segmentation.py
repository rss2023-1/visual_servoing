import cv2
import numpy as np
import pdb

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

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
	# orange ranges in hues from 30-45
	# settings for good performance on testing: (0, 220, 90), (45, 255, 255)
	mask = cv2.inRange(hsv_img, (0, 150, 90), (45, 255, 255))
	kernel = np.ones((3,3), np.uint8)
	eroded = cv2.erode(mask, kernel)
	dilated = cv2.dilate(eroded, kernel)
	# image_print(eroded)
	# image_print(dilated)
	# Extract contours
	contour_results = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	# 0 for python3, 1 for python 2
	contours = contour_results[1]
	outlined = cv2.drawContours(img, contours, -1, (0, 255, 255), 3)
	#print(type(contours)
	# image_print(outlined)
	sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)
	if (len(contours) > 0):
		x,y,w,h = cv2.boundingRect(sortedContours[0])
	else:
		x,y,w,h = (0,0,0,0)
		# print("missed bb")
		# image_print(img)
	bounding_box = ((x,y),(x+w,y+h))
	#bounding_img = cv2.rectangle(img, bounding_box[0], bounding_box[1], (0,255,0),2)
	#image_print(bounding_img)

	########### YOUR CODE ENDS HERE ###########
	# Return bounding box
	return bounding_box

if __name__ == "__main__":
	img = cv2.imread("./test_images_cone/test9.jpg")
	cd_color_segmentation(img, "template holder")
