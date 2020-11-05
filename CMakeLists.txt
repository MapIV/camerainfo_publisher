cmake_minimum_required(VERSION 3.5)
project(pe_camerainfo_publisher)

find_package(catkin REQUIRED
        sensor_msgs
        rospy
        std_msgs
        )

catkin_package()

catkin_install_python(PROGRAMS
        scripts/pe_calibration_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )