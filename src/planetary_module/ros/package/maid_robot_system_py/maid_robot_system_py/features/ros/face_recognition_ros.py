#!/usr/bin/env python3.10

import numpy as np
import copy

from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from maid_robot_system_interfaces.msg._pose_landmark_model import PoseLandmarkModel
from mediapipe.python.solutions.pose import PoseLandmark
from cv_bridge import CvBridge


class FaceRecognitionRos():
    _output_log = True
    #####################################
    _pub_image = None
    _pub_pose_landmark = None
    _bridge = None

    #####################################
    param_topic_sub_name = ''
    param_device_id = -1
    param_device_by_path = '--'
    param_device_type = 'v4l'
    param_video_width = 960
    param_video_height = 540
    param_video_area_x = 0
    param_video_area_y = 0
    param_video_area_width = 960
    param_video_area_height = 540
    param_video_angle = 90
    param_confidence_min_detection = 0.5
    param_confidence_min_tracking = 0.5
    param_confidence_visibility_th = 0.5
    param_image_width = 800
    param_image_height = 600
    _param_image_size = (800 * 600 * 3)
    param_image_overlay_information = False
    param_image_publish = False
    param_update = True
    param_info_verbose = False
    #####################################
    features_detect_markers = False

    #####################################
    person_data = PoseLandmarkModel()
    _data_image_scenery = None
    _data_image_overlay = None
    #####################################
    _all_new_parameters = None
    #####################################

    def __init__(self, node: Node):
        self._bridge = CvBridge()
        self._init_param(node)

    def create_publisher(self, node: Node, id: int):
        queue_size = 10
        if (len(self.param_topic_sub_name) == 0):
            sub_name = 'video' + id
        else:
            sub_name = self.param_topic_sub_name
        self._pub_image = node.create_publisher(
            Image, 'image/raw/' + sub_name, queue_size)
        self._pub_pose_landmark = node.create_publisher(
            PoseLandmarkModel, 'pose_landmark/' + sub_name, queue_size)

    def _init_param(self, node: Node):
        node.declare_parameter('topic_sub_name', self.param_topic_sub_name)
        node.declare_parameter('device/type', self.param_device_type)
        node.declare_parameter('device/id', self.param_device_id)
        node.declare_parameter('device/by_path', self.param_device_by_path)

        node.declare_parameter('video/settings/width', self.param_video_width)
        node.declare_parameter('video/settings/height', self.param_video_height)
        node.declare_parameter('video/settings/angle', self.param_video_angle)

        node.declare_parameter('video/area/x', self.param_video_area_x)
        node.declare_parameter('video/area/y', self.param_video_area_y)
        node.declare_parameter('video/area/width', self.param_video_area_width)
        node.declare_parameter('video/area/height', self.param_video_area_height)

        node.declare_parameter('confidence/min_detection', self.param_confidence_min_detection)
        node.declare_parameter('confidence/min_tracking', self.param_confidence_min_tracking)
        node.declare_parameter('confidence/visibility_th', self.param_confidence_visibility_th)

        node.declare_parameter('image/width', self.param_image_width)
        node.declare_parameter('image/height', self.param_image_height)
        node.declare_parameter('image/overlay_information', self.param_image_overlay_information)
        node.declare_parameter('image/publish', self.param_image_publish)

        node.declare_parameter('update', self.param_update)
        node.declare_parameter('info/verbose', self.param_info_verbose)

        node.declare_parameter('features/detect_markers', self.features_detect_markers)

        param_update = Parameter('update',
                                 Parameter.Type.BOOL,
                                 False)
        self._all_new_parameters = [param_update]
        self.param_update = True
        self.get_parameter(node, True)
        self._data_image_scenery = Image(height=self.param_image_height, width=self.param_image_width,
                                         encoding='bgr8', is_bigendian=0, step=self.param_image_width * 3)
        self._data_image_overlay = Image(height=self.param_image_height, width=self.param_image_width,
                                         encoding='bgr8', is_bigendian=0, step=self.param_image_width * 3)
        self._param_image_size = (self.param_image_width * self.param_image_height * 3)

    def get_parameter(self, node: Node, flag_initial):
        if (self.param_update is True):
            if (flag_initial is True):
                self.param_topic_sub_name = str(node.get_parameter_or(
                    'topic_sub_name', self.param_topic_sub_name).get_parameter_value().string_value)
                self.param_device_id = int(node.get_parameter_or(
                    'device/id', self.param_device_id).get_parameter_value().integer_value)
                self.param_device_by_path = str(node.get_parameter_or(
                    'device/by_path', self.param_device_by_path).get_parameter_value().string_value)
                self.param_device_type = str(node.get_parameter_or(
                    'device/type', self.param_device_type).get_parameter_value().string_value)
                self.param_video_width = int(node.get_parameter_or(
                    'video/settings/width', self.param_video_width).get_parameter_value().integer_value)
                self.param_video_height = int(node.get_parameter_or(
                    'video/settings/height', self.param_video_height).get_parameter_value().integer_value)
                self.param_video_angle = int(node.get_parameter_or(
                    'video/settings/angle', self.param_video_angle).get_parameter_value().integer_value)

                self.param_video_area_x = min(self.param_video_width, int(node.get_parameter_or(
                    'video/area/x', self.param_video_area_x).get_parameter_value().integer_value))
                self.param_video_area_y = min(self.param_video_height, int(node.get_parameter_or(
                    'video/area/y', self.param_video_area_y).get_parameter_value().integer_value))
                self.param_video_area_width = min(self.param_video_width, int(node.get_parameter_or(
                    'video/area/width', self.param_video_area_width).get_parameter_value().integer_value))
                self.param_video_area_height = min(self.param_video_height, int(node.get_parameter_or(
                    'video/area/height', self.param_video_area_height).get_parameter_value().integer_value))

                self.param_image_width = int(node.get_parameter_or(
                    'image/width', self.param_image_width).get_parameter_value().integer_value)
                self.param_image_height = int(node.get_parameter_or(
                    'image/height', self.param_image_height).get_parameter_value().integer_value)

            self.param_confidence_min_detection = float(node.get_parameter_or(
                'confidence/min_detection', self.param_confidence_min_detection).get_parameter_value().double_value)
            self.param_confidence_min_tracking = float(node.get_parameter_or(
                'confidence/min_tracking', self.param_confidence_min_tracking).get_parameter_value().double_value)
            self.param_confidence_visibility_th = float(node.get_parameter_or(
                'confidence/visibility_th', self.param_confidence_visibility_th).get_parameter_value().double_value)

            self.param_image_overlay_information = bool(node.get_parameter_or(
                'image/overlay_information', self.param_image_overlay_information).get_parameter_value().bool_value)
            self.param_image_publish = bool(node.get_parameter_or(
                'image/publish', self.param_image_publish).get_parameter_value().bool_value)
            self.param_info_verbose = bool(node.get_parameter_or(
                'info/verbose', self.param_info_verbose).get_parameter_value().bool_value)

            self.features_detect_markers = bool(node.get_parameter_or(
                'features/detect_markers', self.features_detect_markers).get_parameter_value().bool_value)

            self.print_parameter(node)
            self.param_update = False
            node.set_parameters(self._all_new_parameters)

    def get_parameter_update(self, node: Node):
        self.param_update = node.get_parameter_or(
            'update', self.param_update).get_parameter_value().bool_value
        return self.param_update

    def print_parameter(self, node: Node):
        if (self._output_log is True):
            node.get_logger().info('Parameter: ')
            node.get_logger().debug(' device: ')
            node.get_logger().debug('   type   : ' + str(self.param_device_type))
            node.get_logger().debug('   id     : ' + str(self.param_device_id))
            node.get_logger().debug('   by_path: ' + str(self.param_device_by_path))
            node.get_logger().debug(' video: ')
            node.get_logger().debug('   width  : ' + str(self.param_video_width))
            node.get_logger().debug('   height : ' + str(self.param_video_height))
            node.get_logger().debug('   angle  : ' + str(self.param_video_angle))
            node.get_logger().debug(' confidence: ')
            node.get_logger().debug('  min_detection : '
                                    + str(self.param_confidence_min_detection))
            node.get_logger().debug('  min_tracking  : '
                                    + str(self.param_confidence_min_tracking))
            node.get_logger().debug('  visibility_th : '
                                    + str(self.param_confidence_visibility_th))
            node.get_logger().debug(' image: ')
            node.get_logger().debug('  width   : ' + str(self.param_image_width))
            node.get_logger().debug('  height  : ' + str(self.param_image_height))
            node.get_logger().debug('  overlay : ' + str(self.param_image_overlay_information))
            node.get_logger().debug('  publish : ' + str(self.param_image_publish))
            node.get_logger().info(' update   : ' + str(self.param_update))
            node.get_logger().debug(' info: ')
            node.get_logger().debug('  verbose : ' + str(self.param_info_verbose))
            node.get_logger().debug(' features: ')
            node.get_logger().debug('  detect_markers : ' + str(self.features_detect_markers))

    def repackaging(self, landmarks: PoseLandmarkModel):
        human_detected = False
        box_x_min = 1000
        box_y_min = 1000
        box_x_max = -1000
        box_y_max = -1000
        for index, landmark in enumerate(landmarks.landmark):
            landmark_x = float(landmark.x)
            landmark_y = float(landmark.y)
            landmark_z = float(landmark.z)
            landmark_exist = True
            landmark_visibility = float(landmark.visibility)
            if landmark_visibility < self.param_confidence_visibility_th:
                landmark_exist = False
            else:
                human_detected = True
                box_x_min = min(box_x_min, float(landmark_x))
                box_y_min = min(box_y_min, float(landmark_y))
                box_x_max = max(box_x_max, float(landmark_x))
                box_y_max = max(box_y_max, float(landmark_y))

            if PoseLandmark.NOSE == index:
                self.person_data.nose.x = landmark_x
                self.person_data.nose.y = landmark_y
                self.person_data.nose.z = landmark_z
                self.person_data.nose.exist = landmark_exist
                self.person_data.nose.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_INNER == index:
                self.person_data.left.eye_inner.x = landmark_x
                self.person_data.left.eye_inner.y = landmark_y
                self.person_data.left.eye_inner.z = landmark_z
                self.person_data.left.eye_inner.exist = landmark_exist
                self.person_data.left.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE == index:
                self.person_data.left.eye.x = landmark_x
                self.person_data.left.eye.y = landmark_y
                self.person_data.left.eye.z = landmark_z
                self.person_data.left.eye.exist = landmark_exist
                self.person_data.left.eye.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_OUTER == index:
                self.person_data.left.eye_outer.x = landmark_x
                self.person_data.left.eye_outer.y = landmark_y
                self.person_data.left.eye_outer.z = landmark_z
                self.person_data.left.eye_outer.exist = landmark_exist
                self.person_data.left.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_INNER == index:
                self.person_data.right.eye_inner.x = landmark_x
                self.person_data.right.eye_inner.y = landmark_y
                self.person_data.right.eye_inner.z = landmark_z
                self.person_data.right.eye_inner.exist = landmark_exist
                self.person_data.right.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE == index:
                self.person_data.right.eye.x = landmark_x
                self.person_data.right.eye.y = landmark_y
                self.person_data.right.eye.z = landmark_z
                self.person_data.right.eye.exist = landmark_exist
                self.person_data.right.eye.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_OUTER == index:
                self.person_data.right.eye_outer.x = landmark_x
                self.person_data.right.eye_outer.y = landmark_y
                self.person_data.right.eye_outer.z = landmark_z
                self.person_data.right.eye_outer.exist = landmark_exist
                self.person_data.right.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EAR == index:
                self.person_data.left.ear.x = landmark_x
                self.person_data.left.ear.y = landmark_y
                self.person_data.left.ear.z = landmark_z
                self.person_data.left.ear.exist = landmark_exist
                self.person_data.left.ear.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EAR == index:
                self.person_data.right.ear.x = landmark_x
                self.person_data.right.ear.y = landmark_y
                self.person_data.right.ear.z = landmark_z
                self.person_data.right.ear.exist = landmark_exist
                self.person_data.right.ear.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_LEFT == index:
                self.person_data.left.mouth.x = landmark_x
                self.person_data.left.mouth.y = landmark_y
                self.person_data.left.mouth.z = landmark_z
                self.person_data.left.mouth.exist = landmark_exist
                self.person_data.left.mouth.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_RIGHT == index:
                self.person_data.right.mouth.x = landmark_x
                self.person_data.right.mouth.y = landmark_y
                self.person_data.right.mouth.z = landmark_z
                self.person_data.right.mouth.exist = landmark_exist
                self.person_data.right.mouth.visibility = landmark_visibility
            elif PoseLandmark.LEFT_SHOULDER == index:
                self.person_data.left.shoulder.x = landmark_x
                self.person_data.left.shoulder.y = landmark_y
                self.person_data.left.shoulder.z = landmark_z
                self.person_data.left.shoulder.exist = landmark_exist
                self.person_data.left.shoulder.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_SHOULDER == index:
                self.person_data.right.shoulder.x = landmark_x
                self.person_data.right.shoulder.y = landmark_y
                self.person_data.right.shoulder.z = landmark_z
                self.person_data.right.shoulder.exist = landmark_exist
                self.person_data.right.shoulder.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ELBOW == index:
                self.person_data.left.elbow.x = landmark_x
                self.person_data.left.elbow.y = landmark_y
                self.person_data.left.elbow.z = landmark_z
                self.person_data.left.elbow.exist = landmark_exist
                self.person_data.left.elbow.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ELBOW == index:
                self.person_data.right.elbow.x = landmark_x
                self.person_data.right.elbow.y = landmark_y
                self.person_data.right.elbow.z = landmark_z
                self.person_data.right.elbow.exist = landmark_exist
                self.person_data.right.elbow.visibility = landmark_visibility
            elif PoseLandmark.LEFT_WRIST == index:
                self.person_data.left.wrist.x = landmark_x
                self.person_data.left.wrist.y = landmark_y
                self.person_data.left.wrist.z = landmark_z
                self.person_data.left.wrist.exist = landmark_exist
                self.person_data.left.wrist.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_WRIST == index:
                self.person_data.right.wrist.x = landmark_x
                self.person_data.right.wrist.y = landmark_y
                self.person_data.right.wrist.z = landmark_z
                self.person_data.right.wrist.exist = landmark_exist
                self.person_data.right.wrist.visibility = landmark_visibility
            elif PoseLandmark.LEFT_PINKY == index:
                self.person_data.left.pinky.x = landmark_x
                self.person_data.left.pinky.y = landmark_y
                self.person_data.left.pinky.z = landmark_z
                self.person_data.left.pinky.exist = landmark_exist
                self.person_data.left.pinky.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_PINKY == index:
                self.person_data.right.pinky.x = landmark_x
                self.person_data.right.pinky.y = landmark_y
                self.person_data.right.pinky.z = landmark_z
                self.person_data.right.pinky.exist = landmark_exist
                self.person_data.right.pinky.visibility = landmark_visibility
            elif PoseLandmark.LEFT_INDEX == index:
                self.person_data.left.index.x = landmark_x
                self.person_data.left.index.y = landmark_y
                self.person_data.left.index.z = landmark_z
                self.person_data.left.index.exist = landmark_exist
                self.person_data.left.index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_INDEX == index:
                self.person_data.right.index.x = landmark_x
                self.person_data.right.index.y = landmark_y
                self.person_data.right.index.z = landmark_z
                self.person_data.right.index.exist = landmark_exist
                self.person_data.right.index.visibility = landmark_visibility
            elif PoseLandmark.LEFT_THUMB == index:
                self.person_data.left.thumb.x = landmark_x
                self.person_data.left.thumb.y = landmark_y
                self.person_data.left.thumb.z = landmark_z
                self.person_data.left.thumb.exist = landmark_exist
                self.person_data.left.thumb.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_THUMB == index:
                self.person_data.right.thumb.x = landmark_x
                self.person_data.right.thumb.y = landmark_y
                self.person_data.right.thumb.z = landmark_z
                self.person_data.right.thumb.exist = landmark_exist
                self.person_data.right.thumb.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HIP == index:
                self.person_data.left.hip.x = landmark_x
                self.person_data.left.hip.y = landmark_y
                self.person_data.left.hip.z = landmark_z
                self.person_data.left.hip.exist = landmark_exist
                self.person_data.left.hip.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HIP == index:
                self.person_data.right.hip.x = landmark_x
                self.person_data.right.hip.y = landmark_y
                self.person_data.right.hip.z = landmark_z
                self.person_data.right.hip.exist = landmark_exist
                self.person_data.right.hip.visibility = landmark_visibility
            elif PoseLandmark.LEFT_KNEE == index:
                self.person_data.left.knee.x = landmark_x
                self.person_data.left.knee.y = landmark_y
                self.person_data.left.knee.z = landmark_z
                self.person_data.left.knee.exist = landmark_exist
                self.person_data.left.knee.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_KNEE == index:
                self.person_data.right.knee.x = landmark_x
                self.person_data.right.knee.y = landmark_y
                self.person_data.right.knee.z = landmark_z
                self.person_data.right.knee.exist = landmark_exist
                self.person_data.right.knee.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ANKLE == index:
                self.person_data.left.ankle.x = landmark_x
                self.person_data.left.ankle.y = landmark_y
                self.person_data.left.ankle.z = landmark_z
                self.person_data.left.ankle.exist = landmark_exist
                self.person_data.left.ankle.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ANKLE == index:
                self.person_data.right.ankle.x = landmark_x
                self.person_data.right.ankle.y = landmark_y
                self.person_data.right.ankle.z = landmark_z
                self.person_data.right.ankle.exist = landmark_exist
                self.person_data.right.ankle.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HEEL == index:
                self.person_data.left.heel.x = landmark_x
                self.person_data.left.heel.y = landmark_y
                self.person_data.left.heel.z = landmark_z
                self.person_data.left.heel.exist = landmark_exist
                self.person_data.left.heel.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HEEL == index:
                self.person_data.right.heel.x = landmark_x
                self.person_data.right.heel.y = landmark_y
                self.person_data.right.heel.z = landmark_z
                self.person_data.right.heel.exist = landmark_exist
                self.person_data.right.heel.visibility = landmark_visibility
            elif PoseLandmark.LEFT_FOOT_INDEX == index:
                self.person_data.left.foot_index.x = landmark_x
                self.person_data.left.foot_index.y = landmark_y
                self.person_data.left.foot_index.z = landmark_z
                self.person_data.left.foot_index.exist = landmark_exist
                self.person_data.left.foot_index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_FOOT_INDEX == index:
                self.person_data.right.foot_index.x = landmark_x
                self.person_data.right.foot_index.y = landmark_y
                self.person_data.right.foot_index.z = landmark_z
                self.person_data.right.foot_index.exist = landmark_exist
                self.person_data.right.foot_index.visibility = landmark_visibility
        ##############################################
        if (human_detected is False):
            box_x_min = 0
            box_y_min = 0
            box_x_max = 0
            box_y_max = 0
        self.person_data.area.x = box_x_min
        self.person_data.area.y = box_y_min
        self.person_data.area.width = box_x_max - box_x_min
        self.person_data.area.height = box_y_max - box_y_min
        self.person_data.human_detected = human_detected

    def set_image(self, image):
        if (self.param_image_publish is True):
            self._data_image_scenery = self._bridge.cv2_to_imgmsg(np.array(image), "bgr8")

    def set_image_overlay(self, image):
        if (self.param_image_publish is True):
            self._data_image_overlay = self._bridge.cv2_to_imgmsg(np.array(image), "bgr8")

    def send_image(self):
        if (self.param_image_publish is True):
            if (self.param_image_overlay_information is True):
                self._pub_image.publish(self._data_image_overlay)
            else:
                self._pub_image.publish(self._data_image_scenery)

    def send_recognition(self):
        if (self.person_data.human_detected is True):
            self._pub_pose_landmark.publish(self.person_data)
