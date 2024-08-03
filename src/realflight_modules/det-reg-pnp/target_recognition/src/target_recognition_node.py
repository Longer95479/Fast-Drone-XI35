#!/usr/bin/env python


import os
import rospy
from target_recognition.srv import ProcessImage, ProcessImageResponse
from sensor_msgs.msg import Image
import cv2
import numpy as np
import onnxruntime
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def preprocess_image_opencv(cv_image):
    image = cv2.resize(cv_image, (28, 28))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = image.astype(np.float32)
    image = np.expand_dims(image, axis=-1)
    image = (image / 255.0 - 0.5) / 0.5
    image = np.expand_dims(image, axis=0)
    image = np.transpose(image, (0, 3, 1, 2))
    return image

def handle_process_image(req):
    try:
        script_directory = os.path.dirname(os.path.abspath(__file__))
        onnx_model_path = os.path.dirname(script_directory)
        onnx_model_path = onnx_model_path + "/model.onnx"
        cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")

        net_session = onnxruntime.InferenceSession(onnx_model_path)
        inputs = {net_session.get_inputs()[0].name: preprocess_image_opencv(cv_image)}
        outs = net_session.run(None, inputs)[0]

        #rospy.loginfo("onnx weights: %s", outs)
        prediction = outs.argmax(axis=1)[0]
        weight = np.max(outs)
        #rospy.loginfo("onnx prediction: %d", prediction)

        return ProcessImageResponse(weight, prediction)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return ProcessImageResponse(-1)
    except Exception as e:
        rospy.logerr("Error processing image: %s", e)
        return ProcessImageResponse(-1)

def target_recognition_node():
    rospy.init_node('target_recognition_node')
    s = rospy.Service('process_image', ProcessImage, handle_process_image)
    rospy.loginfo("Ready to process images.")
    rospy.spin()

if __name__ == "__main__":
    target_recognition_node()

