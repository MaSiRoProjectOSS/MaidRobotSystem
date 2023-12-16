#!/usr/bin/env python3.10

import rclpy
import traceback
import queue
import numpy as np
import cv2 as cv
import copy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from utils.cv_fps_calc import CvFpsCalc
from mediapipe.python.solutions.pose import PoseLandmark
from mediapipe.python.solutions.holistic import Holistic as mp_holistic
import maid_robot_system_interfaces.srv as MrsSrv
import maid_robot_system_interfaces.msg as MrsMsg
from collections import deque


class ImageAnalysis:
    _holistic = None
    _current_min_detection = 0.5
    _current_min_tracking = 0.5

    def __init__(self, confidence_min_detection: float, confidence_min_tracking: float):
        self._load_model(confidence_min_detection, confidence_min_tracking, True)

    def _load_model(self, confidence_min_detection: float, confidence_min_tracking: float, force: bool):
        if ((force is True)
           or (self._current_min_detection != confidence_min_detection)
           or (self._current_min_tracking != confidence_min_tracking)):
            self._current_min_detection = confidence_min_detection
            self._current_min_tracking = confidence_min_tracking
            self._holistic = mp_holistic(
                min_detection_confidence=self._current_min_detection,
                min_tracking_confidence=self._current_min_tracking,
            )

    def detect_holistic(self, image, confidence_min_detection: float, confidence_min_tracking: float):
        landmarks = None
        self._load_model(confidence_min_detection, confidence_min_tracking, False)
        if self._holistic is not None:
            image.flags.writeable = False
            results = self._holistic.process(image)
            image.flags.writeable = True
            if results is not None:
                landmarks = results.pose_landmarks
        return landmarks


class MediapipeNodeParam():

    def update_parameters(self, node: Node):
        new_parameters = [Parameter("confidence/min_detection", Parameter.Type.DOUBLE, self.confidence_min_detection),
                          Parameter("confidence/min_tracking", Parameter.Type.DOUBLE, self.confidence_min_tracking),
                          Parameter("confidence/visibility_th", Parameter.Type.DOUBLE, self.confidence_visibility_th),
                          Parameter("area/center_x", Parameter.Type.INTEGER, self.area_center_x),
                          Parameter("area/center_y", Parameter.Type.INTEGER, self.area_center_y),
                          Parameter("area/width", Parameter.Type.INTEGER, self.area_width),
                          Parameter("area/height", Parameter.Type.INTEGER, self.area_height),
                          Parameter("mirror", Parameter.Type.BOOL, self.mirror),
                          Parameter("upside_down", Parameter.Type.BOOL, self.upside_down),
                          Parameter("clockwise", Parameter.Type.INTEGER, self.clockwise)
                          ]
        node.set_parameters(new_parameters)

    def _callback_on_params(self, parameter_list):
        result = False
        for parameter in parameter_list:
            if (parameter.name == 'INTERVAL_MS'):
                if (parameter.value == self.INTERVAL_MS):
                    result = True
            if (parameter.name == "confidence/min_detection"):
                self.confidence_min_detection = parameter.value
                result = True
            if (parameter.name == "confidence/min_tracking"):
                self.confidence_min_tracking = parameter.value
                result = True
            if (parameter.name == "confidence/visibility_th"):
                self.confidence_visibility_th = parameter.value
                result = True
            if (parameter.name == "area/center_x"):
                self.area_center_x = parameter.value
                result = True
            if (parameter.name == "area/center_y"):
                self.area_center_y = parameter.value
                result = True
            if (parameter.name == "area/width"):
                self.area_width = parameter.value
                result = True
            if (parameter.name == "area/height"):
                self.area_height = parameter.value
                result = True
            if (parameter.name == 'mirror'):
                self.mirror = parameter.value
                result = True
            if (parameter.name == 'upside_down'):
                self.upside_down = parameter.value
                result = True
            if (parameter.name == 'clockwise'):
                value = parameter.value
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK):
                    self.clockwise = parameter.value
                    result = True
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_03_O_CLOCK):
                    self.clockwise = parameter.value
                    result = True
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_06_O_CLOCK):
                    self.clockwise = parameter.value
                    result = True
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_09_O_CLOCK):
                    self.clockwise = parameter.value
                    result = True
            if (parameter.name == "notify/message/verbose"):
                self.info_verbose = parameter.value
                result = True
        return SetParametersResult(successful=result)

    def get_parameter(self, node: Node):
        self.confidence_min_detection = node.get_parameter("confidence/min_detection").get_parameter_value().double_value
        self.confidence_min_tracking = node.get_parameter("confidence/min_tracking").get_parameter_value().double_value
        self.confidence_visibility_th = node.get_parameter("confidence/visibility_th").get_parameter_value().double_value
        self.area_center_x = node.get_parameter("area/center_x").get_parameter_value().integer_value
        self.area_center_y = node.get_parameter("area/center_y").get_parameter_value().integer_value
        self.area_width = node.get_parameter("area/width").get_parameter_value().integer_value
        self.area_height = node.get_parameter("area/height").get_parameter_value().integer_value
        self.mirror = node.get_parameter("mirror").get_parameter_value().bool_value
        self.upside_down = node.get_parameter("upside_down").get_parameter_value().bool_value
        self.clockwise = node.get_parameter("clockwise").get_parameter_value().integer_value
        self.info_verbose = node.get_parameter("notify/message/verbose").get_parameter_value().bool_value

    def init(self, node: Node):
        node.declare_parameter("INTERVAL_MS", self.INTERVAL_MS)
        node.declare_parameter("confidence/min_detection", self.confidence_min_detection)
        node.declare_parameter("confidence/min_tracking", self.confidence_min_tracking)
        node.declare_parameter("confidence/visibility_th", self.confidence_visibility_th)
        node.declare_parameter("area/center_x", self.area_center_x)
        node.declare_parameter("area/center_y", self.area_center_y)
        node.declare_parameter("area/width", self.area_width)
        node.declare_parameter("area/height", self.area_height)
        node.declare_parameter("mirror", self.mirror)
        node.declare_parameter("upside_down", self.upside_down)
        node.declare_parameter("clockwise", self.clockwise)
        node.declare_parameter("notify/message/verbose", self.info_verbose)
        # ### get parameter
        self.INTERVAL_MS = node.get_parameter("INTERVAL_MS").get_parameter_value().integer_value
        self.get_parameter(node)
        node.add_on_set_parameters_callback(self._callback_on_params)

    def __init__(self):
        self.INTERVAL_MS = 100
        self.confidence_min_detection = 0.5
        self.confidence_min_tracking = 0.5
        self.confidence_visibility_th = 0.5
        self.area_center_x = 0
        self.area_center_y = 0
        self.area_width = 0
        self.area_height = 0
        self.mirror = False
        self.upside_down = False
        self.clockwise = int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK)
        self.info_verbose = False


class MediapipeNode(Node):
    _report_detected = True

    _in_srv_name = 'in_service_image'
    _out_srv_name = 'out_service_data'
    _out_landmarks_name = 'out_topic_landmarks'
    _out_landmarks_queue_size = 3
    _timeout_ms = 5000
    _request_width = 640
    _request_height = 512
    _elapsed = deque(maxlen=30)

    def __init__(self, node_name):
        super().__init__(node_name)
        self._param = MediapipeNodeParam()
        self._bridge = CvBridge()
        self._msg = MrsSrv.MediaPipePoseLandmarkDetection.Response()
        self._previous_human_detected = False
        self._timer_output_information_ms = 5.0 * 1000.0
        self._fps_size = (5 * 60)
        self._display_fps = 0

    def open(self):
        result = False
        try:
            self._param.init(self)
            self._fps_calc = CvFpsCalc(buffer_len=self._fps_size)
            # ##################################################################
            self._ia = ImageAnalysis(self._param.confidence_min_detection,
                                     self._param.confidence_min_tracking)
            # ##################################################################

            self._create_service_client()
            self._create_publisher()
            self._create_timer()
            result = True
        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()
        return result

    def closing(self):
        return True

    ##################################################################################
    def _create_service_client(self):
        self._service_data = self.create_service(MrsSrv.MediaPipePoseLandmarkDetection, self._out_srv_name, self._callback_srv_data)
        self._client = self.create_client(MrsSrv.VideoCapture, self._in_srv_name)
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service({}) not available, waiting again...'.format(self._client.srv_name))
        self._request = MrsSrv.VideoCapture.Request()
        self._request_image(self.get_clock().now().nanoseconds)

    def _create_publisher(self):
        self._pub = self.create_publisher(MrsMsg.PoseDetection, self._out_landmarks_name, self._out_landmarks_queue_size)

    def _create_timer(self):
        # set ros callback
        self._timer_recognition = self.create_timer(float(self._param.INTERVAL_MS / 1000.0), self._callback_recognition)
        self._timer_output_information = self.create_timer(float(self._timer_output_information_ms / 1000.0), self._callback_output_information)

    ##################################################################################

    def _request_image(self, current_ns):
        self._request.resize_width = self._request_width
        self._request.resize_height = self._request_height
        self._response = self._client.call_async(self._request)
        self._next_time = (self._timeout_ms * 1000 * 1000) + current_ns

    def _area_cutting(self, image, center_x, center_y, area_width, area_height):
        height, width = image.shape[:2]
        if ((width != 0) and (height != 0)):
            if (area_width == 0):
                area_width = width
            if (area_height == 0):
                area_height = height
            area_x = int((width / 2) - (area_width / 2) + center_x)
            area_y = int((height / 2) - (area_height / 2) + center_y)
            start_x = int(min(max(area_x, 0), width))
            start_y = int(min(max(area_y, 0), height))
            end_x = int(max(0, min((area_x + area_width), width)))
            end_y = int(max(0, min((area_y + area_height), height)))
            if ((end_x - start_x) <= 0):
                start_x = 0
                end_x = width
            if ((end_y - start_y) <= 0):
                start_y = 0
                end_y = height
            return image[start_y:end_y, start_x:end_x]
        else:
            return image

    ##################################################################################
    def _callback_srv_data(self, request, response):
        response.data = self._msg.data
        response.image = self._msg.image
        return response

    def _callback_recognition(self):
        try:
            current_ns = self.get_clock().now().nanoseconds
            if (self._response is not None):
                if (self._response.done() is True):
                    msg = self._response.result()
                    if (msg.image is not None):
                        cv_image = self._bridge.imgmsg_to_cv2(msg.image, "bgr8")
                        cv_image = self._area_cutting(cv_image,
                                                    self._param.area_center_x, self._param.area_center_y,
                                                    self._param.area_width, self._param.area_height)
                        if ((self._param.upside_down is True) or (self._param.mirror is True)):
                            if (self._param.upside_down is False):
                                # mirror
                                op_flip = 1
                            elif ((self._param.mirror is False)):
                                # upside_down
                                op_flip = 0
                            else:
                                # upside_down and mirror
                                op_flip = -1
                            cv_image = cv.flip(cv_image, op_flip)
                            if (self._param.clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK):
                                pass
                            if (self._param.clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_03_O_CLOCK):
                                cv_image = cv.rotate(cv_image, cv.ROTATE_90_CLOCKWISE)
                            if (self._param.clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_06_O_CLOCK):
                                cv_image = cv.rotate(cv_image, cv.ROTATE_180)
                            if (self._param.clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_09_O_CLOCK):
                                cv_image = cv.rotate(cv_image, cv.ROTATE_90_COUNTERCLOCKWISE)
                        pose_landmarks = self._ia.detect_holistic(cv_image,
                                                                self._param.confidence_min_detection,
                                                                self._param.confidence_min_tracking)
                        self._msg.image = self._bridge.cv2_to_imgmsg(np.array(cv_image), "bgr8")
                        if pose_landmarks is not None:
                            self._msg.data = self.repackaging(pose_landmarks,
                                                            self._param.confidence_min_detection,
                                                            self._param.confidence_min_tracking,
                                                            self._param.confidence_visibility_th)
                            self._pub.publish(self._msg.data)
                            if (self._report_detected is True):
                                if (self._msg.data.human_detected is not self._previous_human_detected):
                                    self.get_logger().info('[{}] HUMAN_DETECTED'.format(self.get_name()))
                                    self._previous_human_detected = self._msg.data.human_detected
                        else:
                            self._msg.data.human_detected = False
                            if (self._report_detected is True):
                                if (self._msg.data.human_detected is not self._previous_human_detected):
                                    self._pub.publish(self._msg.data)
                                    self.get_logger().info('[{}] LOSE_TRACKING'.format(self.get_name()))
                                    self._previous_human_detected = self._msg.data.human_detected
                        #####################################################
                        self._request_image(current_ns)
                        self._display_fps = self._fps_calc.get()
                        self._elapsed.append(self.get_clock().now().nanoseconds - current_ns)
                        #####################################################

            if (self._next_time <= current_ns):
                self._request_image(current_ns)
                self.get_logger().warning('Timeout')

        except Exception as exception:
            self.get_logger().error('Exception (_callback_recognition) : ' + str(exception))
            traceback.print_exc()

    def _callback_output_information(self):
        if (self._param.info_verbose is True):
            self.get_logger().info('[{}/{}] FPS : {:5.02f} Elapsed : {:7.02f} ms'.format(self.get_namespace(),
                                                                                         self.get_name(),
                                                                                         self._display_fps,
                                                                                         round((sum(self._elapsed) / len(self._elapsed)) / (1000.0 * 1000.0), 2)))

    ##################################################################################

    def repackaging(self, landmarks, min_detection: float, min_tracking: float, visibility_th: float):
        data: MrsMsg.PoseDetection = MrsMsg.PoseDetection()
        data.human_detected = False
        data.min_detection = min_detection
        data.min_tracking = min_tracking
        data.visibility_th = visibility_th
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
            if landmark_visibility < visibility_th:
                landmark_exist = False
            else:
                data.human_detected = True
                box_x_min = min(box_x_min, float(landmark_x))
                box_y_min = min(box_y_min, float(landmark_y))
                box_x_max = max(box_x_max, float(landmark_x))
                box_y_max = max(box_y_max, float(landmark_y))

            if PoseLandmark.NOSE == index:
                data.landmark.nose.x = landmark_x
                data.landmark.nose.y = landmark_y
                data.landmark.nose.z = landmark_z
                data.landmark.nose.exist = landmark_exist
                data.landmark.nose.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_INNER == index:
                data.landmark.left.eye_inner.x = landmark_x
                data.landmark.left.eye_inner.y = landmark_y
                data.landmark.left.eye_inner.z = landmark_z
                data.landmark.left.eye_inner.exist = landmark_exist
                data.landmark.left.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE == index:
                data.landmark.left.eye.x = landmark_x
                data.landmark.left.eye.y = landmark_y
                data.landmark.left.eye.z = landmark_z
                data.landmark.left.eye.exist = landmark_exist
                data.landmark.left.eye.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_OUTER == index:
                data.landmark.left.eye_outer.x = landmark_x
                data.landmark.left.eye_outer.y = landmark_y
                data.landmark.left.eye_outer.z = landmark_z
                data.landmark.left.eye_outer.exist = landmark_exist
                data.landmark.left.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_INNER == index:
                data.landmark.right.eye_inner.x = landmark_x
                data.landmark.right.eye_inner.y = landmark_y
                data.landmark.right.eye_inner.z = landmark_z
                data.landmark.right.eye_inner.exist = landmark_exist
                data.landmark.right.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE == index:
                data.landmark.right.eye.x = landmark_x
                data.landmark.right.eye.y = landmark_y
                data.landmark.right.eye.z = landmark_z
                data.landmark.right.eye.exist = landmark_exist
                data.landmark.right.eye.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_OUTER == index:
                data.landmark.right.eye_outer.x = landmark_x
                data.landmark.right.eye_outer.y = landmark_y
                data.landmark.right.eye_outer.z = landmark_z
                data.landmark.right.eye_outer.exist = landmark_exist
                data.landmark.right.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EAR == index:
                data.landmark.left.ear.x = landmark_x
                data.landmark.left.ear.y = landmark_y
                data.landmark.left.ear.z = landmark_z
                data.landmark.left.ear.exist = landmark_exist
                data.landmark.left.ear.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EAR == index:
                data.landmark.right.ear.x = landmark_x
                data.landmark.right.ear.y = landmark_y
                data.landmark.right.ear.z = landmark_z
                data.landmark.right.ear.exist = landmark_exist
                data.landmark.right.ear.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_LEFT == index:
                data.landmark.left.mouth.x = landmark_x
                data.landmark.left.mouth.y = landmark_y
                data.landmark.left.mouth.z = landmark_z
                data.landmark.left.mouth.exist = landmark_exist
                data.landmark.left.mouth.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_RIGHT == index:
                data.landmark.right.mouth.x = landmark_x
                data.landmark.right.mouth.y = landmark_y
                data.landmark.right.mouth.z = landmark_z
                data.landmark.right.mouth.exist = landmark_exist
                data.landmark.right.mouth.visibility = landmark_visibility
            elif PoseLandmark.LEFT_SHOULDER == index:
                data.landmark.left.shoulder.x = landmark_x
                data.landmark.left.shoulder.y = landmark_y
                data.landmark.left.shoulder.z = landmark_z
                data.landmark.left.shoulder.exist = landmark_exist
                data.landmark.left.shoulder.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_SHOULDER == index:
                data.landmark.right.shoulder.x = landmark_x
                data.landmark.right.shoulder.y = landmark_y
                data.landmark.right.shoulder.z = landmark_z
                data.landmark.right.shoulder.exist = landmark_exist
                data.landmark.right.shoulder.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ELBOW == index:
                data.landmark.left.elbow.x = landmark_x
                data.landmark.left.elbow.y = landmark_y
                data.landmark.left.elbow.z = landmark_z
                data.landmark.left.elbow.exist = landmark_exist
                data.landmark.left.elbow.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ELBOW == index:
                data.landmark.right.elbow.x = landmark_x
                data.landmark.right.elbow.y = landmark_y
                data.landmark.right.elbow.z = landmark_z
                data.landmark.right.elbow.exist = landmark_exist
                data.landmark.right.elbow.visibility = landmark_visibility
            elif PoseLandmark.LEFT_WRIST == index:
                data.landmark.left.wrist.x = landmark_x
                data.landmark.left.wrist.y = landmark_y
                data.landmark.left.wrist.z = landmark_z
                data.landmark.left.wrist.exist = landmark_exist
                data.landmark.left.wrist.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_WRIST == index:
                data.landmark.right.wrist.x = landmark_x
                data.landmark.right.wrist.y = landmark_y
                data.landmark.right.wrist.z = landmark_z
                data.landmark.right.wrist.exist = landmark_exist
                data.landmark.right.wrist.visibility = landmark_visibility
            elif PoseLandmark.LEFT_PINKY == index:
                data.landmark.left.pinky.x = landmark_x
                data.landmark.left.pinky.y = landmark_y
                data.landmark.left.pinky.z = landmark_z
                data.landmark.left.pinky.exist = landmark_exist
                data.landmark.left.pinky.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_PINKY == index:
                data.landmark.right.pinky.x = landmark_x
                data.landmark.right.pinky.y = landmark_y
                data.landmark.right.pinky.z = landmark_z
                data.landmark.right.pinky.exist = landmark_exist
                data.landmark.right.pinky.visibility = landmark_visibility
            elif PoseLandmark.LEFT_INDEX == index:
                data.landmark.left.index.x = landmark_x
                data.landmark.left.index.y = landmark_y
                data.landmark.left.index.z = landmark_z
                data.landmark.left.index.exist = landmark_exist
                data.landmark.left.index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_INDEX == index:
                data.landmark.right.index.x = landmark_x
                data.landmark.right.index.y = landmark_y
                data.landmark.right.index.z = landmark_z
                data.landmark.right.index.exist = landmark_exist
                data.landmark.right.index.visibility = landmark_visibility
            elif PoseLandmark.LEFT_THUMB == index:
                data.landmark.left.thumb.x = landmark_x
                data.landmark.left.thumb.y = landmark_y
                data.landmark.left.thumb.z = landmark_z
                data.landmark.left.thumb.exist = landmark_exist
                data.landmark.left.thumb.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_THUMB == index:
                data.landmark.right.thumb.x = landmark_x
                data.landmark.right.thumb.y = landmark_y
                data.landmark.right.thumb.z = landmark_z
                data.landmark.right.thumb.exist = landmark_exist
                data.landmark.right.thumb.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HIP == index:
                data.landmark.left.hip.x = landmark_x
                data.landmark.left.hip.y = landmark_y
                data.landmark.left.hip.z = landmark_z
                data.landmark.left.hip.exist = landmark_exist
                data.landmark.left.hip.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HIP == index:
                data.landmark.right.hip.x = landmark_x
                data.landmark.right.hip.y = landmark_y
                data.landmark.right.hip.z = landmark_z
                data.landmark.right.hip.exist = landmark_exist
                data.landmark.right.hip.visibility = landmark_visibility
            elif PoseLandmark.LEFT_KNEE == index:
                data.landmark.left.knee.x = landmark_x
                data.landmark.left.knee.y = landmark_y
                data.landmark.left.knee.z = landmark_z
                data.landmark.left.knee.exist = landmark_exist
                data.landmark.left.knee.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_KNEE == index:
                data.landmark.right.knee.x = landmark_x
                data.landmark.right.knee.y = landmark_y
                data.landmark.right.knee.z = landmark_z
                data.landmark.right.knee.exist = landmark_exist
                data.landmark.right.knee.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ANKLE == index:
                data.landmark.left.ankle.x = landmark_x
                data.landmark.left.ankle.y = landmark_y
                data.landmark.left.ankle.z = landmark_z
                data.landmark.left.ankle.exist = landmark_exist
                data.landmark.left.ankle.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ANKLE == index:
                data.landmark.right.ankle.x = landmark_x
                data.landmark.right.ankle.y = landmark_y
                data.landmark.right.ankle.z = landmark_z
                data.landmark.right.ankle.exist = landmark_exist
                data.landmark.right.ankle.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HEEL == index:
                data.landmark.left.heel.x = landmark_x
                data.landmark.left.heel.y = landmark_y
                data.landmark.left.heel.z = landmark_z
                data.landmark.left.heel.exist = landmark_exist
                data.landmark.left.heel.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HEEL == index:
                data.landmark.right.heel.x = landmark_x
                data.landmark.right.heel.y = landmark_y
                data.landmark.right.heel.z = landmark_z
                data.landmark.right.heel.exist = landmark_exist
                data.landmark.right.heel.visibility = landmark_visibility
            elif PoseLandmark.LEFT_FOOT_INDEX == index:
                data.landmark.left.foot_index.x = landmark_x
                data.landmark.left.foot_index.y = landmark_y
                data.landmark.left.foot_index.z = landmark_z
                data.landmark.left.foot_index.exist = landmark_exist
                data.landmark.left.foot_index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_FOOT_INDEX == index:
                data.landmark.right.foot_index.x = landmark_x
                data.landmark.right.foot_index.y = landmark_y
                data.landmark.right.foot_index.z = landmark_z
                data.landmark.right.foot_index.exist = landmark_exist
                data.landmark.right.foot_index.visibility = landmark_visibility
        ##############################################
        if (data.human_detected is False):
            box_x_min = 0
            box_y_min = 0
            box_x_max = 0
            box_y_max = 0
        data.detected_area.x = box_x_min
        data.detected_area.y = box_y_min
        data.detected_area.width = box_x_max - box_x_min
        data.detected_area.height = box_y_max - box_y_min
        return data

    ##################################################################################


def main(args=None):
    rclpy.init(args=args)
    node_name = 'mediapipe_node'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = MediapipeNode(node_name)

    try:
        if (node.open() is True):
            rclpy.spin(node)

    except Exception as exception:
        traceback_logger.error(exception)
        traceback.print_exc()
        traceback_logger.error(traceback.format_exc())
    finally:
        node.closing()
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
