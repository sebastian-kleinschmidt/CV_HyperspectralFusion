import time
import message_filters
import cv2 as cv
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def callback(image_wv1, image_wv2, image_wv3, image_wv4, image_wv5):
    stamp = image_wv1.header.stamp

    cv_img_wv1 = bridge.imgmsg_to_cv2(image_wv1, desired_encoding="bgr8")
    cv_img_wv1 = cv.cvtColor(cv_img_wv1, cv.COLOR_BGR2GRAY)
    cv_img_wv2 = bridge.imgmsg_to_cv2(image_wv2, desired_encoding="bgr8")
    cv_img_wv2 = cv.cvtColor(cv_img_wv2, cv.COLOR_BGR2GRAY)
    cv_img_wv3 = bridge.imgmsg_to_cv2(image_wv3, desired_encoding="bgr8")
    cv_img_wv3 = cv.cvtColor(cv_img_wv3, cv.COLOR_BGR2GRAY)
    cv_img_wv4 = bridge.imgmsg_to_cv2(image_wv4, desired_encoding="bgr8")
    cv_img_wv4 = cv.cvtColor(cv_img_wv4, cv.COLOR_BGR2GRAY)
    cv_img_wv5 = bridge.imgmsg_to_cv2(image_wv5, desired_encoding="bgr8")
    cv_img_wv5 = cv.cvtColor(cv_img_wv5, cv.COLOR_BGR2GRAY)

    #Fusion
    cv_img_merged = np.zeros(cv_img_wv1.shape)
    cv.accumulate(cv_img_wv1, cv_img_merged);
    cv.accumulate(cv_img_wv2, cv_img_merged);
    cv.accumulate(cv_img_wv3, cv_img_merged);
    cv.accumulate(cv_img_wv4, cv_img_merged);
    cv.accumulate(cv_img_wv5, cv_img_merged);
    cv_img_merged = cv_img_merged/5
    cv_img_merged = cv_img_merged.astype(np.uint8)

    #Publish
    message = bridge.cv2_to_imgmsg(cv_img_merged, encoding="mono8")
    message.header.stamp = stamp

    pub.publish(message)

image_sub_wv1 = message_filters.Subscriber('/camera/image_wavelength_0', Image)
image_sub_wv2 = message_filters.Subscriber('/camera/image_wavelength_4', Image)
image_sub_wv3 = message_filters.Subscriber('/camera/image_wavelength_5', Image)
image_sub_wv4 = message_filters.Subscriber('/camera/image_wavelength_6', Image)
image_sub_wv5 = message_filters.Subscriber('/camera/image_wavelength_9', Image)

print('initialized')
rospy.init_node('HyperspectralFusion')
pub = rospy.Publisher('/camera/image_hyperspec', Image, queue_size=10)
ts = message_filters.ApproximateTimeSynchronizer([image_sub_wv1, image_sub_wv2, image_sub_wv3, image_sub_wv4, image_sub_wv5], 10, 0.1, allow_headerless=False)
ts.registerCallback(callback)
rospy.spin()
