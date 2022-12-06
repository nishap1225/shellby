#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy

# Import the String message type from the /msg directory of the std_msgs package.
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage 
import tf2_ros 
import numpy as np

# Define the method which contains the node's main functionality
def tf_publish():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/String to the topic /chatter_talk
    pub = rospy.Publisher('tf', TFMessage, queue_size=10)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(5) # 10hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # Construct a string that we want to publish (in Python, the "%"
        # operator functions similarly to sprintf in C or MATLAB)
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        
        error = True 
        # while not rospy.is_shutdown() and error: 
        #     try: 
        #         t = tfBuffer.lookup_transform('ar_marker_4', 'usb_cam', rospy.Time())
        #         error = False 
        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #         rospy.sleep(1) 
        #         error = True 
        #         continue 

        # print(t)
        # assert t.child_frame_id == 'usb_cam'
        # t.header.frame_id = 'base'
        t = TransformStamped() 
        t.header.frame_id = 'ar_marker_4'
        t.child_frame_id = 'base'


        t.transform.rotation.w = 0 
        t.transform.rotation.x = np.sqrt(2)/2 
        t.transform.rotation.y = 0 
        t.transform.rotation.z = np.sqrt(2)/2  


        t.header.stamp = rospy.get_rostime()

        print(t)
        # print(rospy.Time())
        message = TFMessage() 
        message.transforms = [t]
        
        # # Publish our string to the 'chatter_talk' topic
        pub.publish(message)

        t.child_frame_id = 'reference/base'
        print("_")
        print(t)
        pub.publish(message)
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        tf_publish()
    except rospy.ROSInterruptException: pass
