#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

import cv2
from cv_bridge import CvBridge

import tensorflow as tf
import numpy as np

#import tflite_runtime.interpreter as tflite

def resize_and_crop(img, size):
    height, width = size
    in_height, in_width, _ = img.shape

    rospy.loginfo("Image Shape: %s", str((in_height, in_width)))
    rospy.loginfo("Self size: %s", str((height, width)))

    if ((in_width != width) or (in_height != height)):
        scale_X = width / float(in_width)
        scale_Y = height / float(in_height)


        if(scale_X > scale_Y):
            #width, scale_X
            new_height = int(in_height*scale_X)
            img_resized = cv2.resize(img, (width, new_height))
            center_y = new_height//2
            return img_resized[(center_y-(height//2)):(center_y+(height//2)), 0:width]   #rows(height), columns(width)
        else:
            #height
            new_width = int(in_width*scale_Y)
            img_resized = cv2.resize(img, (new_width, height))
            center_x = new_width//2
            return img_resized[0:height, (center_x-(width//2)):(center_x+(width//2))]   #rows(height), columns(width)

class ROBOBO_TENSORFLOW(object):
    def __init__(self, robobo_name='robobo', file_path="./detect.tflite"):

        self.robobo_name = robobo_name
        self.file_path = file_path
        rospy.init_node('RoboboObjects', anonymous=True)

        self.bridge = CvBridge()

        self.size = (300, 300)
        """
        with(open("./labelmap.txt")) as f:
            self.labels = [line.rstrip() for line in f.readlines()]
        """

        #self.interpreter = tflite.Interpreter(model_path="./detect.tflite")
        self.interpreter = tf.lite.Interpreter(model_path=self.file_path)
        self.interpreter.allocate_tensors()

        self.inputDetails = self.interpreter.get_input_details()
        self.outputDetails = self.interpreter.get_output_details()

        # Topics
        rospy.Subscriber("camera/image", Image, self.camera_callback)
        self.tf_pub = rospy.Publisher("detected_object", Detection2DArray, queue_size=10)     #Correct publisher?
        self.tf_pub_debug = rospy.Publisher("debug_detected_object_img", Image, queue_size=10)

    def camera_callback(self, data):
        # Use the bridge to convert the ROS Image message to OpenCV message
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        #inp = cv2.resize(cv_image, self.size)
        inp = resize_and_crop(cv_image, self.size)

        norm_image = cv2.normalize(inp, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        norm_rgb = cv2.cvtColor(norm_image, cv2.COLOR_BGR2RGB)

        #rgb = cv2.cvtColor(norm_image, cv2.COLOR_BGR2RGB)
        rgb = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB)
        
        #debug
        img_message = self.bridge.cv2_to_imgmsg(norm_rgb, encoding="passthrough")
        self.tf_pub_debug.publish(img_message)
        
        rgb_tensor = tf.convert_to_tensor(norm_rgb, dtype=tf.float32)

        input_data = np.expand_dims(rgb_tensor,axis=0)

        self.interpreter.set_tensor(self.inputDetails[0]['index'], input_data)
        self.interpreter.invoke()

        detection_boxes = self.interpreter.get_tensor(self.outputDetails[0]['index'])
        detection_classes = self.interpreter.get_tensor(self.outputDetails[1]['index'])
        detection_scores = self.interpreter.get_tensor(self.outputDetails[2]['index'])

        num_boxes = self.interpreter.get_tensor(self.outputDetails[3]['index'])

        detections = Detection2DArray()
        detections.header.frame_id = data.header.frame_id       #frame
        detections.header.stamp = data.header.stamp             #time

        frameSize = np.array([data.height, data.width, data.height, data.width])

        for i in range(int(num_boxes[0])):
            class_id = int(detection_classes[0, i])

            detection = Detection2D()
            detection.header.frame_id = detections.header.frame_id
            detection.header.stamp = detections.header.stamp
            
            detection.bbox = BoundingBox2D()
            detection.bbox.center = Pose2D()

            # TFlite returns bounding boxes in (ymin, xmin, ymax, xmax) format, in the [0,1] range.
            # We use the size of the frame image as a range instead. (Check if that is consistent!)
            bbox = detection_boxes[0][i] * frameSize

            detection.bbox.center.x = np.divide((bbox[1] + bbox[3]), 2.0)
            detection.bbox.center.y = np.divide((bbox[0] + bbox[2]), 2.0)
            detection.bbox.size_x = (bbox[3] - bbox[1])
            detection.bbox.size_y = (bbox[2] - bbox[0])

            detection.results.append(ObjectHypothesisWithPose())
            detection.results[0].score = detection_scores[0, i]
            detection.results[0].id = class_id

            detections.detections.append(detection)

        self.tf_pub.publish(detections)

    def run(self):
        rospy.spin()


def main():
    instancia = ROBOBO_TENSORFLOW()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Robobo Object Detection module"
        
if __name__ == '__main__':
    main()
