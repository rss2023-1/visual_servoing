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

	bounding_box = None

	light_orange = (8, 50, 50)
	dark_orange = (15, 255, 255)

	# Convert to HSV
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# Make a mask from HSV
	mask = cv2.inRange(hsv_img, light_orange, dark_orange)

	# Find all the contours in the mask
	_, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	if len(contours) != 0:
		# Find the biggest countour (c)
		c = max(contours, key=cv2.contourArea)
		x,y,w,h = cv2.boundingRect(c)

		bounding_box = ((x,y),(x+w,y+h))

	# Return bounding box
	return bounding_box



if __name__ == "__main__":
	# Read in image
	img = cv2.imread("test_images_cone/test15.jpg")

	# Run your algorithm
	bbox = cd_color_segmentation(img, None)

	# Draw bounding box
	cv2.rectangle(img, bbox[0], bbox[1], (0, 255, 0), 2)

	# Display image
	image_print(img)
	print("Bounding box: ", bbox)
	print("Done")
