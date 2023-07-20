#!/usr/bin/env python3.10

import rclpy
import numpy as np

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class FaceRecognitionRos():
    #####################################
    CAMERA_PIXEL_X = 800
    CAMERA_PIXEL_Y = 600
    CAMERA_ANGLE_X = -140.0
    CAMERA_ANGLE_Y = -140.0
    #####################################
    _pub_person_angle = None
    _pub_robot_left_view = None
    _pub_robot_right_view = None
    _pub_eye_target = None
    _pub_eye_cmd_angle = None
    _pub_neck_cmd_angle = None
    #####################################
    param_device_left_id = -1
    param_device_left_by_path = '--'
    param_device_left_type = 'v4l'
    param_device_left_width = 960
    param_device_left_height = 540
    param_device_right_id = -1
    param_device_right_by_path = '--'
    param_device_right_type = 'v4l'
    param_device_right_width = 960
    param_device_right_height = 540
    param_confidence_min_detection = 0.5
    param_confidence_min_tracking = 0.5
    param_confidence_tracking_timeout = 3.0
    param_update = True

    param_upper_body_only = True
    param_box_rect = True
    #####################################
    _data_neck_cmd_pose = Twist()
    _data_eye_cmd_angle = Twist()
    _data_eye_target = Pose2D(x=0.0, y=0.0)
    _data_person_angle = Float32MultiArray()
    _data_robot_view_left = Image(height=CAMERA_PIXEL_Y, width=CAMERA_PIXEL_X,
                                  encoding='bgr8', is_bigendian=0, step=CAMERA_PIXEL_X * 3)
    _data_robot_view_right = Image(height=CAMERA_PIXEL_Y, width=CAMERA_PIXEL_X,
                                   encoding='bgr8', is_bigendian=0, step=CAMERA_PIXEL_X * 3)
    #####################################
    _flag_send_person_angle = 0
    _flag_send_eye = 0
    _flag_send_neck = 0
    #####################################

    def __init__(self, node):
        self._init_param(node)
        self._create_publisher(node)

    def _create_publisher(self, node):
        queue_size = 10
        self._pub_eye_cmd_angle = node.create_publisher(Twist,  'eye/cmd_angle', queue_size)
        self._pub_neck_cmd_angle = node.create_publisher(Twist,  'neck/cmd_pose', queue_size)
        self._pub_robot_left_view = node.create_publisher(Image,  'robot_view_left', queue_size)
        self._pub_robot_right_view = node.create_publisher(Image,  'robot_view_right', queue_size)
        self._pub_eye_target = node.create_publisher(Pose2D, 'gaze/target', queue_size)
        self._pub_person_angle = node.create_publisher(Float32MultiArray, 'gaze/get_person_angle', queue_size)

    def _init_param(self, node):
        node.declare_parameter('device/left/id', self.param_device_left_id)
        node.declare_parameter('device/left/by_path', self.param_device_left_by_path)
        node.declare_parameter('device/left/type', self.param_device_left_type)
        node.declare_parameter('device/left/width', self.param_device_left_width)
        node.declare_parameter('device/left/height', self.param_device_left_height)

        node.declare_parameter('device/right/id', self.param_device_right_id)
        node.declare_parameter('device/right/by_path', self.param_device_right_by_path)
        node.declare_parameter('device/right/type', self.param_device_right_type)
        node.declare_parameter('device/right/width', self.param_device_right_width)
        node.declare_parameter('device/right/height', self.param_device_right_height)

        node.declare_parameter('confidence/min_detection', self.param_confidence_min_detection)
        node.declare_parameter('confidence/min_tracking', self.param_confidence_min_tracking)
        node.declare_parameter('confidence/tracking_timeout', self.param_confidence_tracking_timeout)

        node.declare_parameter('update', self.param_update)

        param_update = rclpy.parameter.Parameter(
            'update',
            rclpy.Parameter.Type.BOOL,
            True
        )
        all_new_parameters = [param_update]
        node.set_parameters(all_new_parameters)
        self.get_parameter_update(node)
        self.get_parameter(node)

    def get_parameter_update(self, node):
        self.param_update = node.get_parameter_or('update', self.param_update).get_parameter_value().bool_value
        return self.param_update

    def get_parameter(self, node):
        if (True == self.param_update):
            self.param_device_left_id = node.get_parameter_or('device/left/id', self.param_device_left_id).get_parameter_value().integer_value
            self.param_device_left_by_path = node.get_parameter_or('device/left/by_path', self.param_device_left_by_path).get_parameter_value().string_value
            self.param_device_left_type = node.get_parameter_or('device/left/type', self.param_device_left_type).get_parameter_value().string_value
            self.param_device_left_width = node.get_parameter_or('device/left/width', self.param_device_left_width).get_parameter_value().integer_value
            self.param_device_left_height = node.get_parameter_or('device/left/height', self.param_device_left_height).get_parameter_value().integer_value

            self.param_device_right_id = node.get_parameter_or('device/right/id', self.param_device_right_id).get_parameter_value().integer_value
            self.param_device_right_by_path = node.get_parameter_or('device/right/by_path', self.param_device_right_by_path).get_parameter_value().string_value
            self.param_device_right_type = node.get_parameter_or('device/right/type', self.param_device_right_type).get_parameter_value().string_value
            self.param_device_right_width = node.get_parameter_or('device/right/width', self.param_device_right_width).get_parameter_value().integer_value
            self.param_device_right_height = node.get_parameter_or('device/right/height', self.param_device_right_height).get_parameter_value().integer_value

            self.param_confidence_min_detection = node.get_parameter_or('confidence/min_detection', self.param_confidence_min_detection).get_parameter_value().double_value
            self.param_confidence_min_tracking = node.get_parameter_or('confidence/min_tracking', self.param_confidence_min_tracking).get_parameter_value().double_value
            self.param_confidence_tracking_timeout = node.get_parameter_or('confidence/tracking_timeout', self.param_confidence_tracking_timeout).get_parameter_value().double_value

            self.print_parameter(node)
            self.param_update = False

    def print_parameter(self, node):
        node.get_logger().debug('Parameter: ')
        node.get_logger().debug(' device: ')
        node.get_logger().debug('  left: ')
        node.get_logger().debug('   type   : ' + str(self.param_device_left_type))
        node.get_logger().debug('   id     : ' + str(self.param_device_left_id))
        node.get_logger().debug('   by_path: ' + str(self.param_device_left_by_path))
        node.get_logger().debug('   width  : ' + str(self.param_device_left_width))
        node.get_logger().debug('   height : ' + str(self.param_device_left_height))
        node.get_logger().debug('  right: ')
        node.get_logger().debug('   type   : ' + str(self.param_device_right_type))
        node.get_logger().debug('   id     : ' + str(self.param_device_right_id))
        node.get_logger().debug('   by_path: ' + str(self.param_device_right_by_path))
        node.get_logger().debug('   width  : ' + str(self.param_device_right_width))
        node.get_logger().debug('   height : ' + str(self.param_device_right_height))
        node.get_logger().debug('  confidence: ')
        node.get_logger().debug('   min_detection : ' + str(self.param_confidence_min_detection))
        node.get_logger().debug('   min_tracking  : ' + str(self.param_confidence_min_tracking))
        node.get_logger().debug('  update  : ' + str(self.param_update))

    def set_pose_clear(self):
        # set neck
        self._data_neck_cmd_pose.linear.x = 0.0
        self._flag_send_neck = 1.0
        # set eye
        self._data_eye_target.x = 0.0
        self._data_eye_target.y = 0.0
        self._flag_send_eye = 1.0

    def set_neck_confronted(self, target_y, target_z, target_roll):
        self._data_neck_cmd_pose.linear.x = 111.0
        self._data_neck_cmd_pose.angular.y = target_y / 2.5
        self._data_neck_cmd_pose.angular.z = target_z / -2.5
        self._data_neck_cmd_pose.angular.x = target_roll / 4.0
        self._flag_send_neck = 1.0

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

    def set_image_left(self, image):
        pub_data = np.reshape(
            image, (self.CAMERA_PIXEL_X * self.CAMERA_PIXEL_Y * 3))
        self._data_robot_view_left.data = pub_data.tolist()

    def set_image_right(self, image):
        pub_data = np.reshape(
            image, (self.CAMERA_PIXEL_X * self.CAMERA_PIXEL_Y * 3))
        self._data_robot_view_right.data = pub_data.tolist()

    def send_info(self):
        # publish robot view
        self._pub_robot_left_view.publish(self._data_robot_view_left)
        self._pub_robot_right_view.publish(self._data_robot_view_right)

    def send(self):
        # publish eye angle
        self._pub_eye_cmd_angle.publish(self._data_eye_cmd_angle)
        # publish person angle
        if (0 != self._flag_send_person_angle):
            self._flag_send_person_angle = 0
            self._pub_person_angle.publish(self._data_person_angle)
        # publish neck position
        if (0 != self._flag_send_neck):
            self._flag_send_neck = 0
            self._pub_neck_cmd_angle.publish(self._data_neck_cmd_pose)
        # publish clear eye position
        if (0 != self._flag_send_eye):
            self._flag_send_eye = 0
            self._pub_eye_target.publish(self._data_eye_target)
