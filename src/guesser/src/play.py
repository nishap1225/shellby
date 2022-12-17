#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from numpy.linalg import *
import signal

# SOURCE: LAB 4, IMAGE_PROCESS.PY
# Create a CvBridge to convert ROS messages to OpenCV images
bridge = CvBridge()

NUM_CUPS = 3
EPSILON = 5
BALL = -1 

centroids = [] 

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
	# Guassian Blur
	img_array = ros_to_np_img(img)
	img_d = cv2.resize(img_array, dsize=(int(img_array.shape[1]/4), int(img_array.shape[0]/4)), interpolation=cv2.INTER_NEAREST)
	img_blur = cv2.GaussianBlur(img_d, (21,21), 0)

	# Select out RED items 
	img_mask = cv2.inRange(img_blur, np.array([0, 0, 70]), np.array([63, 63, 150]))
	img_u = cv2.resize(src=img_mask, dsize=(img_array.shape[1], img_array.shape[0]), interpolation=cv2.INTER_NEAREST)
	img_message = np_img_to_ros(img_u.astype(np.uint8))

	# publish mask 
	try:
		pub_mask.publish(img_message)
	except rospy.ROSInterruptException: pass

	# Find centers of mass 
	# https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
	contours, hierarchy = cv2.findContours(img_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	new_centroids = [] 

	for c in contours:
		M = cv2.moments(c)
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])


			# Rule out centroids which are too close to others 
			far = True 
			for centroid in new_centroids: 
				if euclidian([cX, cY], centroid) < EPSILON: 
					far = False 

			if far: 
				new_centroids.append([cX, cY])

			# put text and highlight the center
			cv2.circle(img_mask, (cX, cY), 5, (0, 0, 0), -1)

	global centroids 
	if len(centroids) == 0: 
		new_centroids.sort(key = lambda c: c[0])
		centroids = new_centroids 
		print("First iteration: " + str(centroids))
		return 
	
	a_img_u = cv2.resize(src=img_mask, dsize=(img_array.shape[1], img_array.shape[0]), interpolation=cv2.INTER_NEAREST)
	a_img_message = np_img_to_ros(a_img_u.astype(np.uint8))

	# publish centroids
	try:
		pub_annotated.publish(a_img_message)
	except rospy.ROSInterruptException: pass
	
	dist = np.full((len(new_centroids), NUM_CUPS), float('inf'))

	for i in range(len(new_centroids)): 
		for j in range(NUM_CUPS): 
			dist[i][j] = euclidian(new_centroids[i], centroids[j])

	for n in range(NUM_CUPS): 
		min_index = np.argmin(dist)
		min_i, min_j = min_index // NUM_CUPS, min_index % NUM_CUPS 
		centroids[min_j] = new_centroids[min_i] 
		dist[:, min_j] = np.full((1, len(new_centroids)), float('inf'))
		dist[min_i, :] = np.full(NUM_CUPS, float('inf'))

	assert len(centroids) == NUM_CUPS 
	assert max(max(centroids)) < float('inf')

	print(centroids)

def observe(): 
	rospy.Subscriber("/usb_cam/image_raw", Image, callback)
	rospy.spin()

def handler(signum, frame):
	x_centroids = [c[0] for c in centroids]
	order = np.argsort(x_centroids)
	ball_pos = np.where(order == BALL)[0][0]
	print("The ball is at position " + str(ball_pos) + " from the left.")
	print("The ball is at " + str(centroids[BALL]))  
	exit(1)
 
signal.signal(signal.SIGINT, handler)

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