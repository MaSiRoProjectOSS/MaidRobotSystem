#!/usr/bin/env python3.10

import rclpy
import traceback
import queue
import cv2 as cv
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from utils.cv_fps_calc import CvFpsCalc
from mediapipe.python.solutions.pose import PoseLandmark
from mediapipe.python.solutions.holistic import Holistic as mp_holistic
from maid_robot_system_interfaces.msg._pose_detection import PoseDetection
from maid_robot_system_interfaces.msg._pose_landmark_model import PoseLandmarkModel


class ImageAnalysis:
    _holistic = None

    def __init__(self, confidence_min_detection: float, confidence_min_tracking: float):
        self._load_model(confidence_min_detection, confidence_min_tracking)

    def _load_model(self, confidence_min_detection: float, confidence_min_tracking: float):
        self._holistic = mp_holistic(
            min_detection_confidence=confidence_min_detection,
            min_tracking_confidence=confidence_min_tracking,
        )

    def detect_holistic(self, image):
        landmarks = None
        if self._holistic is not None:
            image.flags.writeable = False
            results = self._holistic.process(image)
            image.flags.writeable = True
            if results is not None:
                landmarks = results.pose_landmarks
        return landmarks


class VideoCaptureNodeParam():

    def _callback_on_params(self, parameter_list):
        result = False
        for parameter in parameter_list:
            self.get_logger().info('[{}/{}] Got {}={}'.format(
                self._namespace, self._node_name, parameter.name, parameter.value))
            if (parameter.name == "preference/info/verbose"):
                self.preference_info_verbose = parameter.value
                result = True
            if (parameter.name == "preference/confidence/min_detection"):
                self.preference_confidence_min_detection = parameter.value
                result = True
            if (parameter.name == "preference/confidence/min_tracking"):
                self.preference_confidence_min_tracking = parameter.value
                result = True
            if (parameter.name == "preference/confidence/visibility_th"):
                self.preference_confidence_visibility_th = parameter.value
                result = True
            if (parameter.name == "preference/video/area/center_x"):
                self.preference_video_area_center_x = parameter.value
                result = True
            if (parameter.name == "preference/video/area/center_y"):
                self.preference_video_area_center_y = parameter.value
                result = True
            if (parameter.name == "preference/video/area/width"):
                self.preference_video_area_width = parameter.value
                result = True
            if (parameter.name == "preference/video/area/height"):
                self.preference_video_area_height = parameter.value
                result = True
        return SetParametersResult(successful=result)

    def get_parameter(self, node: Node):
        self.preference_info_verbose = node.get_parameter("preference/info/verbose").get_parameter_value().bool_value
        self.preference_confidence_min_detection = node.get_parameter("preference/confidence/min_detection").get_parameter_value().double_value
        self.preference_confidence_min_tracking = node.get_parameter("preference/confidence/min_tracking").get_parameter_value().double_value
        self.preference_confidence_visibility_th = node.get_parameter("preference/confidence/visibility_th").get_parameter_value().double_value
        self.preference_video_area_center_x = node.get_parameter("preference/video/area/center_x").get_parameter_value().integer_value
        self.preference_video_area_center_y = node.get_parameter("preference/video/area/center_y").get_parameter_value().integer_value
        self.preference_video_area_width = node.get_parameter("preference/video/area/width").get_parameter_value().integer_value
        self.preference_video_area_height = node.get_parameter("preference/video/area/height").get_parameter_value().integer_value
        self.configuration_publisher_interval_fps = node.get_parameter("configuration/publisher/interval_fps").get_parameter_value().double_value

    def init(self, node: Node):
        self._namespace = node.get_namespace()
        self._node_name = node.get_name()
        node.declare_parameter("preference/info/verbose", self.preference_info_verbose)
        node.declare_parameter("preference/confidence/min_detection", self.preference_confidence_min_detection)
        node.declare_parameter("preference/confidence/min_tracking", self.preference_confidence_min_tracking)
        node.declare_parameter("preference/confidence/visibility_th", self.preference_confidence_visibility_th)
        node.declare_parameter("preference/video/area/center_x", self.preference_video_area_center_x)
        node.declare_parameter("preference/video/area/center_y", self.preference_video_area_center_y)
        node.declare_parameter("preference/video/area/width", self.preference_video_area_width)
        node.declare_parameter("preference/video/area/height", self.preference_video_area_height)
        node.declare_parameter("preference/image/publish", self.preference_image_publish)
        node.declare_parameter("configuration/publisher/interval_fps", self.configuration_publisher_interval_fps)
        self.get_parameter(node)
        node.add_on_set_parameters_callback(self._callback_on_params)

    def __init__(self):
        self._namespace = ''
        self._node_name = ''
        self.preference_info_verbose = True
        self.preference_confidence_min_detection = 0.5
        self.preference_confidence_min_tracking = 0.5
        self.preference_confidence_visibility_th = 0.5
        self.preference_video_area_center_x = 0
        self.preference_video_area_center_y = 0
        self.preference_video_area_width = 0
        self.preference_video_area_height = 0
        self.configuration_publisher_interval_fps = 10.0


class MediapipeNode(Node):
    _lifo = None
    _param = None
    _sub_queue_size = 2
    _sub_name = 'in'
    _pub = None
    _pub_queue_size = 5
    _pub_name = 'out'
    _pose_detection_msg = PoseDetection()
    _previous_human_detected = False
    _fps_calc = None
    _report_detected = True
    _timer_output_information = None
    _timer_output_information_period = (1.0 / 5.0)
    _timer_output_information_size = (5 * 60)
    _display_fps = 0

    def __init__(self, node_name):
        super().__init__(node_name)
        self._lifo = queue.LifoQueue(self._sub_queue_size)
        self._param = VideoCaptureNodeParam()

    def open(self):
        result = False
        try:
            self._param.init(self)
            self._pose_detection_msg = PoseDetection()
            self._fps_calc = CvFpsCalc(
                buffer_len=self._timer_output_information_size)
            # ##################################################################
            self._ia = ImageAnalysis(self._param.preference_confidence_min_detection,
                                     self._param.preference_confidence_min_tracking)
            # ##################################################################

            self._create_publisher()
            self._create_subscription()
            self._create_timer()
            result = True
        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()
        return result

    def closing(self):
        return True

    def _create_publisher(self):
        self._pub = self.create_publisher(PoseDetection, self._pub_name, self._pub_queue_size)

    def _create_subscription(self):
        self.subscription = self.create_subscription(Image, self._sub_name, self._callback_listener_image, self._sub_queue_size)

    def _create_timer(self):
        # set ros callback
        self._timer_recognition = self.create_timer((1.0 / float(self._param.configuration_publisher_interval_fps)), self._callback_timer_detect)
        self._timer_output_information = self.create_timer((1.0 / float(self._timer_output_information_period)), self._callback_output_information)

    def _callback_listener_image(self, msg):
        if (self._lifo.full() is True):
            self._lifo.get()
        self._lifo.put(msg)
        pass

    def _callback_timer_detect(self):
        try:
            if (self._lifo.empty() is False):
                self._pose_detection_msg.human_detected = False
                msg = self._lifo.get()
                cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                pose_landmarks = self._ia.detect_holistic(cv_image)
                if pose_landmarks is not None:
                    self.repackaging(pose_landmarks)
                    self._pub.publish(self._pose_detection_msg)
                    #####################################################
                    if (self._report_detected is True):
                        if (self._pose_detection_msg.human_detected is not self._previous_human_detected):
                            if (self._pose_detection_msg.human_detected is True):
                                self.get_logger().info(
                                    '[{}] HUMAN_DETECTED'.format(self.get_name()))
                            else:
                                self.get_logger().info(
                                    '[{}] LOSE_TRACKING'.format(self.get_name()))
                            self._previous_human_detected = self._pose_detection_msg.human_detected
                #####################################################
                self._display_fps = self._fps_calc.get()
                #####################################################

            while not self._lifo.empty():
                self._lifo.get()
        except Exception as exception:
            self.get_logger().error('Exception (_callback_recognition) : ' + str(exception))
            traceback.print_exc()

    def _callback_output_information(self):
        if (self._param.preference_info_verbose is True):
            self.get_logger().info('[{}/{}] FPS : {:.4g}'.format(self.get_namespace(), self.get_name(), self._display_fps))

    ##################################################################################

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
            if landmark_visibility < self._param.preference_confidence_visibility_th:
                landmark_exist = False
            else:
                human_detected = True
                box_x_min = min(box_x_min, float(landmark_x))
                box_y_min = min(box_y_min, float(landmark_y))
                box_x_max = max(box_x_max, float(landmark_x))
                box_y_max = max(box_y_max, float(landmark_y))

            if PoseLandmark.NOSE == index:
                self._pose_detection_msg.landmark.nose.x = landmark_x
                self._pose_detection_msg.landmark.nose.y = landmark_y
                self._pose_detection_msg.landmark.nose.z = landmark_z
                self._pose_detection_msg.landmark.nose.exist = landmark_exist
                self._pose_detection_msg.landmark.nose.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_INNER == index:
                self._pose_detection_msg.landmark.left.eye_inner.x = landmark_x
                self._pose_detection_msg.landmark.left.eye_inner.y = landmark_y
                self._pose_detection_msg.landmark.left.eye_inner.z = landmark_z
                self._pose_detection_msg.landmark.left.eye_inner.exist = landmark_exist
                self._pose_detection_msg.landmark.left.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE == index:
                self._pose_detection_msg.landmark.left.eye.x = landmark_x
                self._pose_detection_msg.landmark.left.eye.y = landmark_y
                self._pose_detection_msg.landmark.left.eye.z = landmark_z
                self._pose_detection_msg.landmark.left.eye.exist = landmark_exist
                self._pose_detection_msg.landmark.left.eye.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EYE_OUTER == index:
                self._pose_detection_msg.landmark.left.eye_outer.x = landmark_x
                self._pose_detection_msg.landmark.left.eye_outer.y = landmark_y
                self._pose_detection_msg.landmark.left.eye_outer.z = landmark_z
                self._pose_detection_msg.landmark.left.eye_outer.exist = landmark_exist
                self._pose_detection_msg.landmark.left.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_INNER == index:
                self._pose_detection_msg.landmark.right.eye_inner.x = landmark_x
                self._pose_detection_msg.landmark.right.eye_inner.y = landmark_y
                self._pose_detection_msg.landmark.right.eye_inner.z = landmark_z
                self._pose_detection_msg.landmark.right.eye_inner.exist = landmark_exist
                self._pose_detection_msg.landmark.right.eye_inner.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE == index:
                self._pose_detection_msg.landmark.right.eye.x = landmark_x
                self._pose_detection_msg.landmark.right.eye.y = landmark_y
                self._pose_detection_msg.landmark.right.eye.z = landmark_z
                self._pose_detection_msg.landmark.right.eye.exist = landmark_exist
                self._pose_detection_msg.landmark.right.eye.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EYE_OUTER == index:
                self._pose_detection_msg.landmark.right.eye_outer.x = landmark_x
                self._pose_detection_msg.landmark.right.eye_outer.y = landmark_y
                self._pose_detection_msg.landmark.right.eye_outer.z = landmark_z
                self._pose_detection_msg.landmark.right.eye_outer.exist = landmark_exist
                self._pose_detection_msg.landmark.right.eye_outer.visibility = landmark_visibility
            elif PoseLandmark.LEFT_EAR == index:
                self._pose_detection_msg.landmark.left.ear.x = landmark_x
                self._pose_detection_msg.landmark.left.ear.y = landmark_y
                self._pose_detection_msg.landmark.left.ear.z = landmark_z
                self._pose_detection_msg.landmark.left.ear.exist = landmark_exist
                self._pose_detection_msg.landmark.left.ear.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_EAR == index:
                self._pose_detection_msg.landmark.right.ear.x = landmark_x
                self._pose_detection_msg.landmark.right.ear.y = landmark_y
                self._pose_detection_msg.landmark.right.ear.z = landmark_z
                self._pose_detection_msg.landmark.right.ear.exist = landmark_exist
                self._pose_detection_msg.landmark.right.ear.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_LEFT == index:
                self._pose_detection_msg.landmark.left.mouth.x = landmark_x
                self._pose_detection_msg.landmark.left.mouth.y = landmark_y
                self._pose_detection_msg.landmark.left.mouth.z = landmark_z
                self._pose_detection_msg.landmark.left.mouth.exist = landmark_exist
                self._pose_detection_msg.landmark.left.mouth.visibility = landmark_visibility
            elif PoseLandmark.MOUTH_RIGHT == index:
                self._pose_detection_msg.landmark.right.mouth.x = landmark_x
                self._pose_detection_msg.landmark.right.mouth.y = landmark_y
                self._pose_detection_msg.landmark.right.mouth.z = landmark_z
                self._pose_detection_msg.landmark.right.mouth.exist = landmark_exist
                self._pose_detection_msg.landmark.right.mouth.visibility = landmark_visibility
            elif PoseLandmark.LEFT_SHOULDER == index:
                self._pose_detection_msg.landmark.left.shoulder.x = landmark_x
                self._pose_detection_msg.landmark.left.shoulder.y = landmark_y
                self._pose_detection_msg.landmark.left.shoulder.z = landmark_z
                self._pose_detection_msg.landmark.left.shoulder.exist = landmark_exist
                self._pose_detection_msg.landmark.left.shoulder.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_SHOULDER == index:
                self._pose_detection_msg.landmark.right.shoulder.x = landmark_x
                self._pose_detection_msg.landmark.right.shoulder.y = landmark_y
                self._pose_detection_msg.landmark.right.shoulder.z = landmark_z
                self._pose_detection_msg.landmark.right.shoulder.exist = landmark_exist
                self._pose_detection_msg.landmark.right.shoulder.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ELBOW == index:
                self._pose_detection_msg.landmark.left.elbow.x = landmark_x
                self._pose_detection_msg.landmark.left.elbow.y = landmark_y
                self._pose_detection_msg.landmark.left.elbow.z = landmark_z
                self._pose_detection_msg.landmark.left.elbow.exist = landmark_exist
                self._pose_detection_msg.landmark.left.elbow.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ELBOW == index:
                self._pose_detection_msg.landmark.right.elbow.x = landmark_x
                self._pose_detection_msg.landmark.right.elbow.y = landmark_y
                self._pose_detection_msg.landmark.right.elbow.z = landmark_z
                self._pose_detection_msg.landmark.right.elbow.exist = landmark_exist
                self._pose_detection_msg.landmark.right.elbow.visibility = landmark_visibility
            elif PoseLandmark.LEFT_WRIST == index:
                self._pose_detection_msg.landmark.left.wrist.x = landmark_x
                self._pose_detection_msg.landmark.left.wrist.y = landmark_y
                self._pose_detection_msg.landmark.left.wrist.z = landmark_z
                self._pose_detection_msg.landmark.left.wrist.exist = landmark_exist
                self._pose_detection_msg.landmark.left.wrist.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_WRIST == index:
                self._pose_detection_msg.landmark.right.wrist.x = landmark_x
                self._pose_detection_msg.landmark.right.wrist.y = landmark_y
                self._pose_detection_msg.landmark.right.wrist.z = landmark_z
                self._pose_detection_msg.landmark.right.wrist.exist = landmark_exist
                self._pose_detection_msg.landmark.right.wrist.visibility = landmark_visibility
            elif PoseLandmark.LEFT_PINKY == index:
                self._pose_detection_msg.landmark.left.pinky.x = landmark_x
                self._pose_detection_msg.landmark.left.pinky.y = landmark_y
                self._pose_detection_msg.landmark.left.pinky.z = landmark_z
                self._pose_detection_msg.landmark.left.pinky.exist = landmark_exist
                self._pose_detection_msg.landmark.left.pinky.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_PINKY == index:
                self._pose_detection_msg.landmark.right.pinky.x = landmark_x
                self._pose_detection_msg.landmark.right.pinky.y = landmark_y
                self._pose_detection_msg.landmark.right.pinky.z = landmark_z
                self._pose_detection_msg.landmark.right.pinky.exist = landmark_exist
                self._pose_detection_msg.landmark.right.pinky.visibility = landmark_visibility
            elif PoseLandmark.LEFT_INDEX == index:
                self._pose_detection_msg.landmark.left.index.x = landmark_x
                self._pose_detection_msg.landmark.left.index.y = landmark_y
                self._pose_detection_msg.landmark.left.index.z = landmark_z
                self._pose_detection_msg.landmark.left.index.exist = landmark_exist
                self._pose_detection_msg.landmark.left.index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_INDEX == index:
                self._pose_detection_msg.landmark.right.index.x = landmark_x
                self._pose_detection_msg.landmark.right.index.y = landmark_y
                self._pose_detection_msg.landmark.right.index.z = landmark_z
                self._pose_detection_msg.landmark.right.index.exist = landmark_exist
                self._pose_detection_msg.landmark.right.index.visibility = landmark_visibility
            elif PoseLandmark.LEFT_THUMB == index:
                self._pose_detection_msg.landmark.left.thumb.x = landmark_x
                self._pose_detection_msg.landmark.left.thumb.y = landmark_y
                self._pose_detection_msg.landmark.left.thumb.z = landmark_z
                self._pose_detection_msg.landmark.left.thumb.exist = landmark_exist
                self._pose_detection_msg.landmark.left.thumb.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_THUMB == index:
                self._pose_detection_msg.landmark.right.thumb.x = landmark_x
                self._pose_detection_msg.landmark.right.thumb.y = landmark_y
                self._pose_detection_msg.landmark.right.thumb.z = landmark_z
                self._pose_detection_msg.landmark.right.thumb.exist = landmark_exist
                self._pose_detection_msg.landmark.right.thumb.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HIP == index:
                self._pose_detection_msg.landmark.left.hip.x = landmark_x
                self._pose_detection_msg.landmark.left.hip.y = landmark_y
                self._pose_detection_msg.landmark.left.hip.z = landmark_z
                self._pose_detection_msg.landmark.left.hip.exist = landmark_exist
                self._pose_detection_msg.landmark.left.hip.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HIP == index:
                self._pose_detection_msg.landmark.right.hip.x = landmark_x
                self._pose_detection_msg.landmark.right.hip.y = landmark_y
                self._pose_detection_msg.landmark.right.hip.z = landmark_z
                self._pose_detection_msg.landmark.right.hip.exist = landmark_exist
                self._pose_detection_msg.landmark.right.hip.visibility = landmark_visibility
            elif PoseLandmark.LEFT_KNEE == index:
                self._pose_detection_msg.landmark.left.knee.x = landmark_x
                self._pose_detection_msg.landmark.left.knee.y = landmark_y
                self._pose_detection_msg.landmark.left.knee.z = landmark_z
                self._pose_detection_msg.landmark.left.knee.exist = landmark_exist
                self._pose_detection_msg.landmark.left.knee.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_KNEE == index:
                self._pose_detection_msg.landmark.right.knee.x = landmark_x
                self._pose_detection_msg.landmark.right.knee.y = landmark_y
                self._pose_detection_msg.landmark.right.knee.z = landmark_z
                self._pose_detection_msg.landmark.right.knee.exist = landmark_exist
                self._pose_detection_msg.landmark.right.knee.visibility = landmark_visibility
            elif PoseLandmark.LEFT_ANKLE == index:
                self._pose_detection_msg.landmark.left.ankle.x = landmark_x
                self._pose_detection_msg.landmark.left.ankle.y = landmark_y
                self._pose_detection_msg.landmark.left.ankle.z = landmark_z
                self._pose_detection_msg.landmark.left.ankle.exist = landmark_exist
                self._pose_detection_msg.landmark.left.ankle.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_ANKLE == index:
                self._pose_detection_msg.landmark.right.ankle.x = landmark_x
                self._pose_detection_msg.landmark.right.ankle.y = landmark_y
                self._pose_detection_msg.landmark.right.ankle.z = landmark_z
                self._pose_detection_msg.landmark.right.ankle.exist = landmark_exist
                self._pose_detection_msg.landmark.right.ankle.visibility = landmark_visibility
            elif PoseLandmark.LEFT_HEEL == index:
                self._pose_detection_msg.landmark.left.heel.x = landmark_x
                self._pose_detection_msg.landmark.left.heel.y = landmark_y
                self._pose_detection_msg.landmark.left.heel.z = landmark_z
                self._pose_detection_msg.landmark.left.heel.exist = landmark_exist
                self._pose_detection_msg.landmark.left.heel.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_HEEL == index:
                self._pose_detection_msg.landmark.right.heel.x = landmark_x
                self._pose_detection_msg.landmark.right.heel.y = landmark_y
                self._pose_detection_msg.landmark.right.heel.z = landmark_z
                self._pose_detection_msg.landmark.right.heel.exist = landmark_exist
                self._pose_detection_msg.landmark.right.heel.visibility = landmark_visibility
            elif PoseLandmark.LEFT_FOOT_INDEX == index:
                self._pose_detection_msg.landmark.left.foot_index.x = landmark_x
                self._pose_detection_msg.landmark.left.foot_index.y = landmark_y
                self._pose_detection_msg.landmark.left.foot_index.z = landmark_z
                self._pose_detection_msg.landmark.left.foot_index.exist = landmark_exist
                self._pose_detection_msg.landmark.left.foot_index.visibility = landmark_visibility
            elif PoseLandmark.RIGHT_FOOT_INDEX == index:
                self._pose_detection_msg.landmark.right.foot_index.x = landmark_x
                self._pose_detection_msg.landmark.right.foot_index.y = landmark_y
                self._pose_detection_msg.landmark.right.foot_index.z = landmark_z
                self._pose_detection_msg.landmark.right.foot_index.exist = landmark_exist
                self._pose_detection_msg.landmark.right.foot_index.visibility = landmark_visibility
        ##############################################
        if (human_detected is False):
            box_x_min = 0
            box_y_min = 0
            box_x_max = 0
            box_y_max = 0
        self._pose_detection_msg.detected_area.x = box_x_min
        self._pose_detection_msg.detected_area.y = box_y_min
        self._pose_detection_msg.detected_area.width = box_x_max - box_x_min
        self._pose_detection_msg.detected_area.height = box_y_max - box_y_min
        self._pose_detection_msg.human_detected = human_detected

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
