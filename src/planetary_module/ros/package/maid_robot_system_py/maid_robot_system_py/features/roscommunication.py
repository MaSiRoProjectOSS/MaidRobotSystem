#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist


class RosCommunication:
    #####################################
    CAMERA_PIXEL_X = 800
    CAMERA_PIXEL_Y = 600
    CAMERA_ANGLE_X = -140.0
    CAMERA_ANGLE_Y = -140.0
    #####################################
    pub_Get_person_angle = rospy.Publisher(
        'debug/get_person_angle', Float32MultiArray, queue_size=10)
    pub_robot_left_view = rospy.Publisher(
        'robot_view_left', Image, queue_size=10)
    pub_robot_right_view = rospy.Publisher(
        'robot_view_right', Image, queue_size=10)
    pub_eye_target = rospy.Publisher('eye/target', Pose2D, queue_size=10)
    pub_eye_cmd_angle = rospy.Publisher(
        '/masiro/eye/cmd_angle', Twist, queue_size=10)
    pub_neck_cmd_angle = rospy.Publisher(
        '/ciro/neck/cmd_pose', Twist, queue_size=10)

    #####################################
    _data_neck_cmd_pose = Twist()
    _data_eye_cmd_angle = Twist()
    _data_eye_target = Pose2D(x=0, y=0)
    _data_person_angle = Float32MultiArray()
    _data_robot_view = Image(height=CAMERA_PIXEL_Y, width=CAMERA_PIXEL_X,
                             encoding="bgr8", is_bigendian=0, step=CAMERA_PIXEL_X * 3)
    #####################################
    _flag_send_neck = 0
    _flag_send_eye = 0
    #####################################
    param_device_id_left = -1
    param_device_id_right = -1
    param_device_name_left = ""
    param_device_name_right = ""
    param_device_path = "v4l"
    param_width = 960
    param_height = 540
    param_min_detection_confidence = 0.5
    param_min_tracking_confidence = 0.5

    param_upper_body_only = True
    param_box_rect = True
    #####################################

    def get_param(self, node_name):
        self.param_device_id_left = rospy.get_param(
            "/" + node_name + "/device/left/id", self.param_device_id_left)
        self.param_device_id_right = rospy.get_param(
            "/" + node_name + "/device/right/id", self.param_device_id_right)
        self.param_device_name_left = rospy.get_param(
            "/" + node_name + "/device/left/by_path", self.param_device_name_left)
        self.param_device_name_right = rospy.get_param(
            "/" + node_name + "/device/right/by_path", self.param_device_name_right)
        self.param_device_path = rospy.get_param(
            "/" + node_name + "/device/path", self.param_device_path)
        self.param_width = rospy.get_param(
            "/" + node_name + "/screen/width", self.param_width)
        self.param_height = rospy.get_param(
            "/" + node_name + "/screen/height", self.param_height)
        self.param_min_detection_confidence = rospy.get_param(
            "/" + node_name + "/confidence/min_detection", self.param_min_detection_confidence)
        self.param_min_tracking_confidence = rospy.get_param(
            "/" + node_name + "/confidence/min_tracking", self.param_min_tracking_confidence)

    def set_pose_clear(self):
        # set neck
        self._data_neck_cmd_pose.linear.x = 0
        self._flag_send_neck = 1
        # set eye
        self._data_eye_target.x = 0
        self._data_eye_target.y = 0
        self._flag_send_eye = 1

    def set_neck_confronted(self, target_y, target_z, target_roll):
        self._data_neck_cmd_pose.linear.x = 111
        self._data_neck_cmd_pose.angular.y = target_y / 2.5
        self._data_neck_cmd_pose.angular.z = target_z / -2.5
        self._data_neck_cmd_pose.angular.x = target_roll / 4.0
        self._flag_send_neck = 1

    def set_eye_angle(self, target_y, target_z):
        self._data_eye_cmd_angle.angular.y = target_y / -2.0
        self._data_eye_cmd_angle.angular.z = target_z / 1.5
        # eye_size
        self._data_eye_cmd_angle.linear.y = 0.98
        # _eye_cmd_angle.linear.z= 500 - 500* (abs(target_angle_z_L - target_angle_z_R) /10.0)

    def set_angle(self, pitch, roll, yaw, flag1, flag2):
        angle_data = []
        angle_data.append(yaw)
        angle_data.append(pitch)
        angle_data.append(roll)
        angle_data.append(flag1)
        angle_data.append(flag2)

        self._data_person_angle = Float32MultiArray(data=angle_data)
        self._flag_send_person_angle = 1

    def set_image(self, image):
        pub_data = np.reshape(
            image, (self.CAMERA_PIXEL_X * self.CAMERA_PIXEL_Y * 3))
        self._data_robot_view.data = pub_data.tolist()

    def send(self, mode):
        # publish eye angle
        self.pub_eye_cmd_angle.publish(self._data_eye_cmd_angle)
        # publish person angle
        if (0 != self._flag_send_person_angle):
            self._flag_send_person_angle = 0
            self.pub_Get_person_angle.publish(self._data_person_angle)
        # publish robot view
        if (mode == "left"):
            self.pub_robot_left_view.publish(self._data_robot_view)
        else:
            self.pub_robot_right_view.publish(self._data_robot_view)
        # publish neck position
        if (0 != self._flag_send_neck):
            self._flag_send_neck = 0
            self.pub_neck_cmd_angle.publish(self._data_neck_cmd_pose)
        # publish clear eye position
        if (0 != self._flag_send_eye):
            self._flag_send_eye = 0
            self.pub_eye_target.publish(self._data_eye_target)
