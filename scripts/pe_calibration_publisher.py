#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('pe_camerainfo_publisher')

import rospy
import yaml
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class PeCameraInfoPublisher:
    def __init__(self):
        rospy.init_node("pe_camerainfo_publisher", anonymous=True)

        self.__namespace = rospy.get_param('~namespace', '/')
        self.__input_topic = rospy.get_param('~input_topic', '/')
        self.__calibration_yaml = rospy.get_param('~calibration_yaml', '/')

        self.__camera_info_msg = self.__parse_yaml(self.__calibration_yaml)
        rospy.loginfo("YAML Parsed correctly: {}".format(self.__calibration_yaml))

        rospy.loginfo("Subscribed to: {}/{} [sensor_msgs/Image]".format(self.__namespace, self.__input_topic))
        rospy.Subscriber(self.__namespace + "/" + self.__input_topic, Image, self.__image_callback)

        rospy.loginfo("Publishing to: {} [sensor_msgs/CameraInfo]".format(self.__namespace + "/camera_info"))
        self.__camera_info_pub = rospy.Publisher(self.__namespace + "/camera_info", CameraInfo, queue_size=1)

        rospy.spin()

    def __dynamic_config_callback(self, config, level):

        return config

    def __image_callback(self, image_msg):
        self.__camera_info_msg.header = image_msg.header
        self.__camera_info_pub.publish(self.__camera_info_msg)

    def __parse_yaml(self, yaml_fname):
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
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
            ncm, _ = cv2.getOptimalNewCameraMatrix(intrinsics,
                                                   distortion,
                                                   (camera_info_msg.width, camera_info_msg.height),
                                                   0.0)
            for j in range(3):
                for i in range(3):
                    P[j, i] = ncm[j, i]

            camera_info_msg.P = P.flatten().tolist()
        print(camera_info_msg.P)

        camera_info_msg.distortion_model = calib_data["camera_model"]
        return camera_info_msg


if __name__ == "__main__":
    campub = PeCameraInfoPublisher()