#!/usr/bin/env python3.10

import numpy as np
import copy

from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from maid_robot_system_interfaces.msg._pose_detection import PoseDetection
from maid_robot_system_interfaces.msg._pose_landmark_model import PoseLandmarkModel
from mediapipe.python.solutions.pose import PoseLandmark
from cv_bridge import CvBridge


class RosCommRecognition():
    _output_log = True
    #####################################
    _pub_image = None
    _pub_pose_detection = None
    _bridge = None

    #####################################
    param_topic_sub_name = ''
    param_device_id = -1
    param_device_by_path = '--'
    param_device_type = 'v4l'
    param_video_width = 960
    param_video_height = 540
    param_video_area_start_x = 0
    param_video_area_start_y = 0
    param_video_area_end_x = 960
    param_video_area_end_y = 540
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
    pose_detection = PoseDetection()
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
        self._pub_pose_detection = node.create_publisher(
            PoseDetection, 'pose_detection/' + sub_name, queue_size)

    def _init_param(self, node: Node):
        node.declare_parameter('topic_sub_name', self.param_topic_sub_name)
        node.declare_parameter('device/type', self.param_device_type)
        node.declare_parameter('device/id', self.param_device_id)
        node.declare_parameter('device/by_path', self.param_device_by_path)

        node.declare_parameter('video/settings/width', self.param_video_width)
        node.declare_parameter('video/settings/height', self.param_video_height)
        node.declare_parameter('video/settings/angle', self.param_video_angle)

        node.declare_parameter('video/area/center_x', self.param_video_area_start_x)
        node.declare_parameter('video/area/center_y', self.param_video_area_start_y)
        node.declare_parameter('video/area/width', int(self.param_video_area_end_x - self.param_video_area_start_x))
        node.declare_parameter('video/area/height', int(self.param_video_area_end_y - self.param_video_area_start_y))

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

    def _calc_area(self, node: Node):
        area_center_x = int(node.get_parameter_or(
            'video/area/center_x', self.param_video_area_start_x).get_parameter_value().integer_value)
        area_center_y = int(node.get_parameter_or(
            'video/area/center_y', self.param_video_area_start_y).get_parameter_value().integer_value)
        area_width = int(node.get_parameter_or(
            'video/area/width', int(self.param_video_area_end_x - self.param_video_area_start_x)).get_parameter_value().integer_value)
        area_height = int(node.get_parameter_or(
            'video/area/height', int(self.param_video_area_end_y - self.param_video_area_start_y)).get_parameter_value().integer_value)

        self.pose_detection.video_info.offset_x = int(max(0, min(area_center_x, self.param_video_width)))
        self.pose_detection.video_info.offset_y = int(max(0, min(area_center_y, self.param_video_height)))
        area_center_x = int((self.param_video_width / 2) - (area_width / 2) + area_center_x)
        area_center_y = int((self.param_video_height / 2) - (area_height / 2) + area_center_y)
        self.param_video_area_end_x = int(max(0, min((area_width + area_center_x), self.param_video_width)))
        self.param_video_area_end_y = int(max(0, min((area_height + area_center_y), self.param_video_height)))
        self.param_video_area_start_x = int(min(max(area_center_x, 0), self.param_video_width))
        self.param_video_area_start_y = int(min(max(area_center_y, 0), self.param_video_height))

        if ((self.param_video_area_end_x - self.param_video_area_start_x) <= 0):
            self.param_video_area_start_x = 0
            self.param_video_area_end_x = self.param_video_width
        if ((self.param_video_area_end_y - self.param_video_area_start_y) <= 0):
            self.param_video_area_start_y = 0
            self.param_video_area_end_y = self.param_video_height

        self.pose_detection.video_info.cutting_width = (self.param_video_area_end_x - self.param_video_area_start_x)
        self.pose_detection.video_info.cutting_height = (self.param_video_area_end_x - self.param_video_area_start_x)

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
                self.pose_detection.video_info.width = self.param_video_width
                self.pose_detection.video_info.height = self.param_video_height
                self.pose_detection.video_info.angle = self.param_video_angle

                self.param_image_width = int(node.get_parameter_or(
                    'image/width', self.param_image_width).get_parameter_value().integer_value)
                self.param_image_height = int(node.get_parameter_or(
                    'image/height', self.param_image_height).get_parameter_value().integer_value)

            self._calc_area(node)

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
            node.get_logger().info(' area: ')
            node.get_logger().info('   start  : [{}, {}]'.format(self.param_video_area_start_x, self.param_video_area_start_y))
            node.get_logger().info('   end    : [{}, {}]'.format(self.param_video_area_end_x, self.param_video_area_end_y))
            node.get_logger().debug(' confidence: ')
            node.get_logger().debug('  min_detection : ' + str(self.param_confidence_min_detection))
            node.get_logger().debug('  min_tracking  : ' + str(self.param_confidence_min_tracking))
            node.get_logger().debug('  visibility_th : ' + str(self.param_confidence_visibility_th))
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
                self.pose_detection.landmark.nose.x = landmark_x
                self.pose_detection.landmark.nose.y = landmark_y
                self.pose_detection.landmark.nose.z = landmark_z
                self.pose_detection.landmark.nose.exist = landmark_exist
                self.pose_detection.landmark.nose.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_INNER == index:
                self.pose_detection.landmark.left.eye_inner.x = landmark_x
                self.pose_detection.landmark.left.eye_inner.y = landmark_y
                self.pose_detection.landmark.left.eye_inner.z = landmark_z
                self.pose_detection.landmark.left.eye_inner.exist = landmark_exist
                self.pose_detection.landmark.left.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE == index:
                self.pose_detection.landmark.left.eye.x = landmark_x
                self.pose_detection.landmark.left.eye.y = landmark_y
                self.pose_detection.landmark.left.eye.z = landmark_z
                self.pose_detection.landmark.left.eye.exist = landmark_exist
                self.pose_detection.landmark.left.eye.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_OUTER == index:
                self.pose_detection.landmark.left.eye_outer.x = landmark_x
                self.pose_detection.landmark.left.eye_outer.y = landmark_y
                self.pose_detection.landmark.left.eye_outer.z = landmark_z
                self.pose_detection.landmark.left.eye_outer.exist = landmark_exist
                self.pose_detection.landmark.left.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_INNER == index:
                self.pose_detection.landmark.right.eye_inner.x = landmark_x
                self.pose_detection.landmark.right.eye_inner.y = landmark_y
                self.pose_detection.landmark.right.eye_inner.z = landmark_z
                self.pose_detection.landmark.right.eye_inner.exist = landmark_exist
                self.pose_detection.landmark.right.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE == index:
                self.pose_detection.landmark.right.eye.x = landmark_x
                self.pose_detection.landmark.right.eye.y = landmark_y
                self.pose_detection.landmark.right.eye.z = landmark_z
                self.pose_detection.landmark.right.eye.exist = landmark_exist
                self.pose_detection.landmark.right.eye.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_OUTER == index:
                self.pose_detection.landmark.right.eye_outer.x = landmark_x
                self.pose_detection.landmark.right.eye_outer.y = landmark_y
                self.pose_detection.landmark.right.eye_outer.z = landmark_z
                self.pose_detection.landmark.right.eye_outer.exist = landmark_exist
                self.pose_detection.landmark.right.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EAR == index:
                self.pose_detection.landmark.left.ear.x = landmark_x
                self.pose_detection.landmark.left.ear.y = landmark_y
                self.pose_detection.landmark.left.ear.z = landmark_z
                self.pose_detection.landmark.left.ear.exist = landmark_exist
                self.pose_detection.landmark.left.ear.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EAR == index:
                self.pose_detection.landmark.right.ear.x = landmark_x
                self.pose_detection.landmark.right.ear.y = landmark_y
                self.pose_detection.landmark.right.ear.z = landmark_z
                self.pose_detection.landmark.right.ear.exist = landmark_exist
                self.pose_detection.landmark.right.ear.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_LEFT == index:
                self.pose_detection.landmark.left.mouth.x = landmark_x
                self.pose_detection.landmark.left.mouth.y = landmark_y
                self.pose_detection.landmark.left.mouth.z = landmark_z
                self.pose_detection.landmark.left.mouth.exist = landmark_exist
                self.pose_detection.landmark.left.mouth.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_RIGHT == index:
                self.pose_detection.landmark.right.mouth.x = landmark_x
                self.pose_detection.landmark.right.mouth.y = landmark_y
                self.pose_detection.landmark.right.mouth.z = landmark_z
                self.pose_detection.landmark.right.mouth.exist = landmark_exist
                self.pose_detection.landmark.right.mouth.visibility = landmark_visibility
            elif PoseLandmark.LEFT_SHOULDER == index:
                self.pose_detection.landmark.left.shoulder.x = landmark_x
                self.pose_detection.landmark.left.shoulder.y = landmark_y
                self.pose_detection.landmark.left.shoulder.z = landmark_z
                self.pose_detection.landmark.left.shoulder.exist = landmark_exist
                self.pose_detection.landmark.left.shoulder.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_SHOULDER == index:
                self.pose_detection.landmark.right.shoulder.x = landmark_x
                self.pose_detection.landmark.right.shoulder.y = landmark_y
                self.pose_detection.landmark.right.shoulder.z = landmark_z
                self.pose_detection.landmark.right.shoulder.exist = landmark_exist
                self.pose_detection.landmark.right.shoulder.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ELBOW == index:
                self.pose_detection.landmark.left.elbow.x = landmark_x
                self.pose_detection.landmark.left.elbow.y = landmark_y
                self.pose_detection.landmark.left.elbow.z = landmark_z
                self.pose_detection.landmark.left.elbow.exist = landmark_exist
                self.pose_detection.landmark.left.elbow.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ELBOW == index:
                self.pose_detection.landmark.right.elbow.x = landmark_x
                self.pose_detection.landmark.right.elbow.y = landmark_y
                self.pose_detection.landmark.right.elbow.z = landmark_z
                self.pose_detection.landmark.right.elbow.exist = landmark_exist
                self.pose_detection.landmark.right.elbow.visibility = landmark_visibility
            elif PoseLandmark.LEFT_WRIST == index:
                self.pose_detection.landmark.left.wrist.x = landmark_x
                self.pose_detection.landmark.left.wrist.y = landmark_y
                self.pose_detection.landmark.left.wrist.z = landmark_z
                self.pose_detection.landmark.left.wrist.exist = landmark_exist
                self.pose_detection.landmark.left.wrist.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_WRIST == index:
                self.pose_detection.landmark.right.wrist.x = landmark_x
                self.pose_detection.landmark.right.wrist.y = landmark_y
                self.pose_detection.landmark.right.wrist.z = landmark_z
                self.pose_detection.landmark.right.wrist.exist = landmark_exist
                self.pose_detection.landmark.right.wrist.visibility = landmark_visibility
            elif PoseLandmark.LEFT_PINKY == index:
                self.pose_detection.landmark.left.pinky.x = landmark_x
                self.pose_detection.landmark.left.pinky.y = landmark_y
                self.pose_detection.landmark.left.pinky.z = landmark_z
                self.pose_detection.landmark.left.pinky.exist = landmark_exist
                self.pose_detection.landmark.left.pinky.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_PINKY == index:
                self.pose_detection.landmark.right.pinky.x = landmark_x
                self.pose_detection.landmark.right.pinky.y = landmark_y
                self.pose_detection.landmark.right.pinky.z = landmark_z
                self.pose_detection.landmark.right.pinky.exist = landmark_exist
                self.pose_detection.landmark.right.pinky.visibility = landmark_visibility
            elif PoseLandmark.LEFT_INDEX == index:
                self.pose_detection.landmark.left.index.x = landmark_x
                self.pose_detection.landmark.left.index.y = landmark_y
                self.pose_detection.landmark.left.index.z = landmark_z
                self.pose_detection.landmark.left.index.exist = landmark_exist
                self.pose_detection.landmark.left.index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_INDEX == index:
                self.pose_detection.landmark.right.index.x = landmark_x
                self.pose_detection.landmark.right.index.y = landmark_y
                self.pose_detection.landmark.right.index.z = landmark_z
                self.pose_detection.landmark.right.index.exist = landmark_exist
                self.pose_detection.landmark.right.index.visibility = landmark_visibility
            elif PoseLandmark.LEFT_THUMB == index:
                self.pose_detection.landmark.left.thumb.x = landmark_x
                self.pose_detection.landmark.left.thumb.y = landmark_y
                self.pose_detection.landmark.left.thumb.z = landmark_z
                self.pose_detection.landmark.left.thumb.exist = landmark_exist
                self.pose_detection.landmark.left.thumb.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_THUMB == index:
                self.pose_detection.landmark.right.thumb.x = landmark_x
                self.pose_detection.landmark.right.thumb.y = landmark_y
                self.pose_detection.landmark.right.thumb.z = landmark_z
                self.pose_detection.landmark.right.thumb.exist = landmark_exist
                self.pose_detection.landmark.right.thumb.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HIP == index:
                self.pose_detection.landmark.left.hip.x = landmark_x
                self.pose_detection.landmark.left.hip.y = landmark_y
                self.pose_detection.landmark.left.hip.z = landmark_z
                self.pose_detection.landmark.left.hip.exist = landmark_exist
                self.pose_detection.landmark.left.hip.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HIP == index:
                self.pose_detection.landmark.right.hip.x = landmark_x
                self.pose_detection.landmark.right.hip.y = landmark_y
                self.pose_detection.landmark.right.hip.z = landmark_z
                self.pose_detection.landmark.right.hip.exist = landmark_exist
                self.pose_detection.landmark.right.hip.visibility = landmark_visibility
            elif PoseLandmark.LEFT_KNEE == index:
                self.pose_detection.landmark.left.knee.x = landmark_x
                self.pose_detection.landmark.left.knee.y = landmark_y
                self.pose_detection.landmark.left.knee.z = landmark_z
                self.pose_detection.landmark.left.knee.exist = landmark_exist
                self.pose_detection.landmark.left.knee.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_KNEE == index:
                self.pose_detection.landmark.right.knee.x = landmark_x
                self.pose_detection.landmark.right.knee.y = landmark_y
                self.pose_detection.landmark.right.knee.z = landmark_z
                self.pose_detection.landmark.right.knee.exist = landmark_exist
                self.pose_detection.landmark.right.knee.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ANKLE == index:
                self.pose_detection.landmark.left.ankle.x = landmark_x
                self.pose_detection.landmark.left.ankle.y = landmark_y
                self.pose_detection.landmark.left.ankle.z = landmark_z
                self.pose_detection.landmark.left.ankle.exist = landmark_exist
                self.pose_detection.landmark.left.ankle.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ANKLE == index:
                self.pose_detection.landmark.right.ankle.x = landmark_x
                self.pose_detection.landmark.right.ankle.y = landmark_y
                self.pose_detection.landmark.right.ankle.z = landmark_z
                self.pose_detection.landmark.right.ankle.exist = landmark_exist
                self.pose_detection.landmark.right.ankle.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HEEL == index:
                self.pose_detection.landmark.left.heel.x = landmark_x
                self.pose_detection.landmark.left.heel.y = landmark_y
                self.pose_detection.landmark.left.heel.z = landmark_z
                self.pose_detection.landmark.left.heel.exist = landmark_exist
                self.pose_detection.landmark.left.heel.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HEEL == index:
                self.pose_detection.landmark.right.heel.x = landmark_x
                self.pose_detection.landmark.right.heel.y = landmark_y
                self.pose_detection.landmark.right.heel.z = landmark_z
                self.pose_detection.landmark.right.heel.exist = landmark_exist
                self.pose_detection.landmark.right.heel.visibility = landmark_visibility
            elif PoseLandmark.LEFT_FOOT_INDEX == index:
                self.pose_detection.landmark.left.foot_index.x = landmark_x
                self.pose_detection.landmark.left.foot_index.y = landmark_y
                self.pose_detection.landmark.left.foot_index.z = landmark_z
                self.pose_detection.landmark.left.foot_index.exist = landmark_exist
                self.pose_detection.landmark.left.foot_index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_FOOT_INDEX == index:
                self.pose_detection.landmark.right.foot_index.x = landmark_x
                self.pose_detection.landmark.right.foot_index.y = landmark_y
                self.pose_detection.landmark.right.foot_index.z = landmark_z
                self.pose_detection.landmark.right.foot_index.exist = landmark_exist
                self.pose_detection.landmark.right.foot_index.visibility = landmark_visibility
        ##############################################
        if (human_detected is False):
            box_x_min = 0
            box_y_min = 0
            box_x_max = 0
            box_y_max = 0
        self.pose_detection.detected_area.x = box_x_min
        self.pose_detection.detected_area.y = box_y_min
        self.pose_detection.detected_area.width = box_x_max - box_x_min
        self.pose_detection.detected_area.height = box_y_max - box_y_min
        self.pose_detection.human_detected = human_detected

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
        if (self.pose_detection.human_detected is True):
            self._pub_pose_detection.publish(self.pose_detection)
