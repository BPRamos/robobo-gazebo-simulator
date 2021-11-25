#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image 
from aruco_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovariance, Quaternion

from cv_bridge import CvBridge

import cv2
import numpy as np
from cv2 import aruco

import tf


class ROBOBO_ARUCOS(object):
    def __init__(self, robobo_name='robobo', calibration_image_path=None,dictionary=aruco.DICT_4X4_100, marker_length=100):
        self.camera_matrix = np.array([[53.31260399, 0., 337.47022316],[0., 207.91343066, 242.31044794],[0., 0.,1.]])
        self.dist_coeff = np.array([[-3.22032638e-03, -1.47422975e-03, -2.45554419e-03, 1.42331365e-02, 4.98036513e-05]])

        self.robobo_name = robobo_name
        rospy.init_node('RoboboArucos', anonymous=True)

        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(dictionary)
        self.parameters = aruco.DetectorParameters_create()
        self.parameters.minDistanceToBorder = 3
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.marker_length = marker_length

        # Topics
        rospy.Subscriber("camera/image", Image, self.camera_callback)
        self.aruco_pub = rospy.Publisher("tag", Marker, queue_size=10)

    def camera_callback(self, data):
        # Use the bridge to convert the ROS Image message to OpenCV message
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Arucos detection
        #                               cv2.aruc.detectMarkers(mat, Aruco.getPredefinedDictionary(currentTagDict), markerCorners, markerIds, parameters, rejectedCandidates, calibrationData.getCameraMatrixMat(), calibrationData.getDistCoeffsMat());
        (corners_list, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters, cameraMatrix = self.camera_matrix, distCoeff=self.dist_coeff)

        if(ids is not None):
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners_list, self.marker_length, self.camera_matrix, self.dist_coeff)
            rotation_matrix = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]], dtype=float)   #Reused

            for (id, rvec, tvec) in zip(ids, rvecs, tvecs):
                
                # To quaternion (We do it here to avoid creating arrays over and over)
                rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

                # convert the matrix to a quaternion
                quaternion= tf.transformations.quaternion_from_matrix(rotation_matrix)

                msg = Marker()
                msg.confidence = 1.0
                msg.id = id[0]
                
                msg.pose = PoseWithCovariance()
                msg.pose.pose.position.x = tvec[0][0]
                msg.pose.pose.position.y = tvec[0][1]
                msg.pose.pose.position.z = tvec[0][2]
                
                msg.pose.pose.orientation = Quaternion()
                msg.pose.pose.orientation.x = quaternion[0]
                msg.pose.pose.orientation.y = quaternion[1]
                msg.pose.pose.orientation.z = quaternion[2]
                msg.pose.pose.orientation.w = quaternion[3]

                msg.header = Header()
                msg.header.stamp = rospy.Time.now()

                self.aruco_pub.publish(msg)
        
    def set_dictionary(self, dictionary):
        self.aruco_dict = aruco.Dictionary_get(dictionary)

    def run(self):
        rospy.spin()



def main():
    instancia = ROBOBO_ARUCOS()
    try:
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        print("Shutting down Robobo Arucos module")
        
if __name__ == '__main__':
    main()
