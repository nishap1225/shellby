#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import *

# SOURCE: LAB 4, IMAGE_PROCESS.PY
# Create a CvBridge to convert ROS messages to OpenCV images
bridge = CvBridge()

base_pos = [] # list of valid positions 
curr_pos = [] # list of current centroid positions 
curr_order = [] # current positions of cup
cup1 = -1 
cup2 = -1 

NUM_CUPS = 3
EPSILON = 5
BALL = -1 

# base_pos = [(0, 100), (20, 100), (40, 100)] (L -> R) 
# curr_pos = [0, 1, 2] # 0'th index is the cup in the 0'th base position  

# next iter 
# calculate centroids -> curr_pos 
# see if the y value of any differs from base_pos -> record which cup is moving (find which two cups haven't moved w/ euclidian distance)


# Converts a ROS Image message to a NumPy array to be displayed by OpenCV
def ros_to_np_img(ros_img_msg):
  return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

def np_img_to_ros(cv2_array):
	return bridge.cv2_to_imgmsg(cv2_array, encoding='passthrough')

def do_kmeans(data, n_clusters):
    """Uses opencv to perform k-means clustering on the data given. Clusters it into
       n_clusters clusters.

       Args:
         data: ndarray of shape (n_datapoints, dim)
         n_clusters: int, number of clusters to divide into.

       Returns:
         clusters: integer array of length n_datapoints. clusters[i] is
         a number in range(n_clusters) specifying which cluster data[i]
         was assigned to. 
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    _, clusters, centers = kmeans = cv2.kmeans(data.astype(np.float32), n_clusters, bestLabels=None, criteria=criteria, attempts=1, flags=cv2.KMEANS_RANDOM_CENTERS)

    return clusters

def euclidian(p1, p2): 
	return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def callback(img): 
	img_array = ros_to_np_img(img)
	img_d = cv2.resize(img_array, dsize=(int(img_array.shape[1]/4), int(img_array.shape[0]/4)), interpolation=cv2.INTER_NEAREST)
	img_blur = cv2.GaussianBlur(img_d, (21,21), 0)

	img_mask = cv2.inRange(img_blur, np.array([0, 0, 70]), np.array([63, 63, 150]))
	img_u = cv2.resize(src=img_mask, dsize=(img_array.shape[1], img_array.shape[0]), interpolation=cv2.INTER_NEAREST)
	img_message = np_img_to_ros(img_u.astype(np.uint8))

	try:
		pub_mask.publish(img_message)
	except rospy.ROSInterruptException: pass

	# https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
	contours, hierarchy = cv2.findContours(img_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	new_centroids = [] 

	for c in contours:
		M = cv2.moments(c)
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])

			new_centroids.append([cX, cY])

			# put text and highlight the center
			cv2.circle(img_mask, (cX, cY), 5, (0, 0, 0), -1)

	if len(base_pos) == 0: 
		curr_pos = new_centroids 
		base_pos = new_centroids 
		curr_order = list(range(NUM_CUPS))
	else: 
		for n in new_centroids: 
			dist = np.array(list(map(lambda c: euclidian(n, c), list(base_pos))))
			away_from_base = np.min(dist) > EPSILON 

			dist = np.array(list(map(lambda c: euclidian(n, c), list(cur_pos))))
			moving = np.min(dist) > EPSILON 

			if away_from_base and moving: 
				if cup1 == -1: 
					cup1 = # cup 
				else: 
					cup2 = # cup 
			elif not away_from_base and not moving: 
				assert (cup1 == -1 and cup2 == -1) or (cup1 != -1 and cup2 != -1)
				if cup1 != -1 and cup2 != -1: 
					i1 = curr_order.index(cup1) 
					i2 = curr_order.index(cup2) 
					curr_order[i1] = cup2 
					curr_order[i2] = cup1 

				# update base pos? 

		# ? 
		curr_pos = new_centroids 

	a_img_u = cv2.resize(src=img_mask, dsize=(img_array.shape[1], img_array.shape[0]), interpolation=cv2.INTER_NEAREST)
	a_img_message = np_img_to_ros(a_img_u.astype(np.uint8))

	assert len(curr_pos) == len(base_pos) == len(curr_order) == NUM_CUPS 

	try:
		pub_annotated.publish(a_img_message)
		print(base_pos)
		print(curr_pos)
		print(curr_order)
		print("__________________________________________")
	except rospy.ROSInterruptException: pass
	

def observe(): 
	rospy.Subscriber("/usb_cam/image_raw", Image, callback)
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('guesser', anonymous = 'True')
	pub_mask = rospy.Publisher("/guesser/image_mask", Image, queue_size=10)
	pub_bin = rospy.Publisher("guesser/image_bin", Image, queue_size=10)
	pub_annotated = rospy.Publisher("guesser/image_centroid", Image, queue_size=10)

	print("How many cups are in the game?")
	NUM_CUPS = int(input())
	assert NUM_CUPS > 0 

	print("Which cup is the ping pong ball under?")
	BALL = int(input())
	assert BALL < NUM_CUPS

	observe() 