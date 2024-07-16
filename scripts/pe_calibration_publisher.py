#!/usr/bin/env python
from __future__ import print_function

import rospy
import roslib
import yaml
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

roslib.load_manifest('pe_camerainfo_publisher')


class PeCameraInfoPublisher:
    def __init__(self):
        rospy.init_node("pe_camerainfo_publisher", anonymous=True)

        
        self.__input_topic = rospy.get_param('~input_topic', '/')
        self.__calibration_yaml = rospy.get_param('~calibration_yaml', '/')
        self.__is_fish_eye = rospy.get_param('~fisheye', False)
        self.__alpha = rospy.get_param('~alpha', 0.0)

        self.__camera_info_msg = self.__parse_yaml(self.__calibration_yaml)
        rospy.loginfo("YAML Parsed correctly: {}".format(self.__calibration_yaml))

        rospy.loginfo("Subscribed to: {} [sensor_msgs/Image]".format(self.__input_topic))
        rospy.Subscriber(self.__input_topic, Image, self.__image_callback)

        rospy.loginfo("Publishing to: {} [sensor_msgs/CameraInfo]".format("camera_info"))
        self.__camera_info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=1)

        rospy.spin()

    def __dynamic_config_callback(self, config, level):

        return config

    def __image_callback(self, image_msg):
        self.__camera_info_msg.header = image_msg.header
        self.__camera_info_pub.publish(self.__camera_info_msg)

    def __parse_yaml(self, yaml_fname):
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.safe_load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        if "projection_matrix" in calib_data:
            camera_info_msg.P = calib_data["projection_matrix"]["data"]
        else:
            P = np.zeros((3, 4), dtype=np.float64)
            intrinsics = np.array(camera_info_msg.K, dtype=np.float64, copy=True).reshape((3, 3))
            distortion = np.array(camera_info_msg.D, dtype=np.float64, copy=True).reshape((len(camera_info_msg.D), 1))
            if self.__is_fish_eye:
                print("Missing Projection Matrix - Fisheye - Calculating ...")
                ncm = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(intrinsics,
                                                                             distortion,
                                                                             (camera_info_msg.width,
                                                                              camera_info_msg.height),
                                                                             np.eye(3), 
                                                                             balance=self.__alpha)
            else:
                print("Missing Projection Matrix - Pinhole - Calculating ...")
                ncm, _ = cv2.getOptimalNewCameraMatrix(intrinsics,
                                                       distortion,
                                                       (camera_info_msg.width, camera_info_msg.height),
                                                       self.__alpha)
            for j in range(3):
                for i in range(3):
                    P[j, i] = ncm[j, i]

            camera_info_msg.P = P.flatten().tolist()
        print(camera_info_msg.P)

        if "camera_model" in calib_data:
            camera_info_msg.distortion_model = calib_data["camera_model"]
        elif "distortion_model" in calib_data:
            camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg


if __name__ == "__main__":
    campub = PeCameraInfoPublisher()