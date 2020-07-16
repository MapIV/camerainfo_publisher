
import rospy
import yaml
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class PeCameraInfoPublisher:
    def __init__(self, namespace, input_topic, calibration_yaml):
        
        rospy.init_node("pe_camera_info_publisher", anonymous=True)

        self.__camera_info_msg = self.__parse_yaml(calibration_yaml)
        rospy.loginfo("YAML Parsed correctly: {}".format(calibration_yaml))

        rospy.loginfo("Subscribed to: {}/{} [sensor_msgs/Image]".format(namespace, image_topic))
        rospy.Subscriber(namespace + "/" + input_topic, Image, self.__image_callback)

        rospy.loginfo("Publishing to: {} [sensor_msgs/CameraInfo]".format(namespace + "/camera_info"))
        self.__camera_info_pub = rospy.Publisher(namespace + "/camera_info", CameraInfo, queue_size=1)

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
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["camera_model"]
        return camera_info_msg

    def calculate_omc(self, camera_info_msg):
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
                # P[j, i] = intrinsics[j, i]
        print(P.flatten().tolist())

        

if __name__ == "__main__":
    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("filename", help="Path to yaml file containing camera calibration data")
    arg_parser.add_argument("namespace", help="Camera namespace")
    arg_parser.add_argument("image_topic", help="Image topic to subscribe")
    args = arg_parser.parse_args()

    filename = args.filename
    namespace = args.namespace
    image_topic = args.image_topic

    campub = PeCameraInfoPublisher(namespace, image_topic, filename)