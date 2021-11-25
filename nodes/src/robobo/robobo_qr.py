#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image 
from robobo_msgs.msg import QrCode, QrCodeChange
from opencv_apps.msg import Point2D

from cv_bridge import CvBridge
from pyzbar import pyzbar

import numpy as np
 
class ROBOBO_QR():
    def __init__(self, robobo_name='robobo'):
        self.robobo_name = robobo_name
        rospy.init_node('RoboboQR', anonymous=True)

        self.bridge = CvBridge()

        self.qrDictionary = dict()
        self.currentIDs = []

        # Topics
        rospy.Subscriber("camera/image", Image, self.camera_callback)
        self.qr_code = rospy.Publisher("QR/code", QrCode, queue_size=10)
        self.qr_change = rospy.Publisher("QR/change", QrCodeChange, queue_size=10)
        

    def message_new_qr(self, qr):
        """
        # Metadata about the frame
        Header header
        # Text contained on the QR Code
        string text
        # Distance between the center of the code and
        # the first result point
        float32 distance
        # Coordinate of the center of the code
        opencv_apps/Point2D center
        # Vector of coordinates of the result points
        opencv_apps/Point2D[] result_points 
        """

        polygon = qr.polygon
        (x, y, w, h) = qr.rect

        cx = x + w / 2
        cy = y + h / 2

        msg = QrCode()

        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.text = qr.data.decode()
        msg.distance = np.sqrt(np.power(cx-polygon[1][0],2) + np.power(cy - polygon[1][1],2))
        msg.center = Point2D(cx,cy)
        msg.result_points[0] = Point2D(polygon[1])     #Assuming pyZBar returns QR with points in-order
        msg.result_points[1] = Point2D(polygon[0])
        msg.result_points[2] = Point2D(polygon[3])

        self.qr_code.publish(msg)
    
    def message_qr_change(self, qr):
        """
        # Represents a change (appearance and disappearance) of 
        # a QR Code
        #
        # Text of the QR Code
        string id
        # Distance between the first result point and the center
        # of the code 
        float32 distance
        # Coordinate of the center of the code
        opencv_apps/Point2D center
        """

        polygon = qr.polygon
        (x, y, w, h) = qr.rect

        cx = x + w / 2
        cy = y + h / 2

        msg = QrCodeChange()

        msg.id = qr.data.decode()
        msg.distance = np.sqrt(np.power(cx-polygon[1][0],2) + np.power(cy - polygon[1][1],2))
        msg.center = Point2D(cx,cy)
        self.qr_change.publish(msg)


    def camera_callback(self, data):
        # Use the bridge to convert the ROS Image message to OpenCV message
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        decodedCodes = pyzbar.decode(cv_image)
        
        newCurrIDs = []
        # fill the current QRs list, messaging new ones, adding new ones
        for code in decodedCodes:
            qrID = code.data.decode()
            if(qrID not in self.currentIDs):
                if(qrID not in self.qrDictionary.keys()):
                    self.message_new_qr(code)
                    self.qrDictionary[qrID] = code
                self.message_qr_change(code)
            newCurrIDs.append(qrID)
        
        # message the missing ones
        for oldID in self.currentIDs:
            if(oldID not in newCurrIDs):
                self.message_qr_change(qrDictionary[oldID])
        self.currentIDs = newCurrIDs

    def run(self):
        rospy.spin()

def main():
    instancia = ROBOBO_QR()
    try:
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        print "Shutting down Robobo QR module"
        
if __name__ == '__main__':
    main()
