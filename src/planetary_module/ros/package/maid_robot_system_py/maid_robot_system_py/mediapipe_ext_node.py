#!/usr/bin/env python3.10

import rclpy
import traceback
import queue
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from features.mp.schematic_diagram import SchematicDiagram
from utils.cv_fps_calc import CvFpsCalc
from maid_robot_system_interfaces.msg._pose_landmark_model import PoseLandmarkModel
from maid_robot_system_interfaces.msg._pose_detection import PoseDetection
from maid_robot_system_interfaces.msg._rect_float import RectFloat
from maid_robot_system_interfaces.msg._landmark import Landmark


class MediapipeExtNodeParam():

    def _callback_on_params(self, parameter_list):
        result = False
        for parameter in parameter_list:
            self.get_logger().info('[{}/{}] Got {}={}'.format(
                self._namespace, self._node_name, parameter.name, parameter.value))

            if (parameter.name == 'preference/info/verbose'):
                self.info_verbose = parameter.value
                result = True
            if (parameter.name == 'preference/image/width'):
                self.width = parameter.value
                result = True
            if (parameter.name == 'preference/image/height'):
                self.height = parameter.value
                result = True
            if (parameter.name == 'preference/image/drawing_box'):
                self.drawing_box = parameter.value
                result = True

        return SetParametersResult(successful=result)

    def get_parameter(self, node: Node):
        self.info_verbose = node.get_parameter('preference/info/verbose').get_parameter_value().bool_value
        self.width = node.get_parameter('preference/image/width').get_parameter_value().integer_value
        self.height = node.get_parameter('preference/image/height').get_parameter_value().integer_value
        self.drawing_box = node.get_parameter('preference/image/drawing_box').get_parameter_value().integer_value

    def init(self, node: Node):
        self._namespace = node.get_namespace()
        self._node_name = node.get_name()
        node.declare_parameter('configuration/publisher/interval_fps', self.fps)
        node.declare_parameter('preference/info/verbose', self.info_verbose)
        node.declare_parameter('preference/image/width', self.width)
        node.declare_parameter('preference/image/height', self.height)
        node.declare_parameter('preference/image/drawing_box', self.drawing_box)
        self.get_parameter(node)
        node.add_on_set_parameters_callback(self._callback_on_params)

    def __init__(self):
        self._namespace = ''
        self._node_name = ''

        self.fps = 4.0
        self.info_verbose = False
        self.width = False
        self.height = False
        self.drawing_box = True


class MediapipeExtNode(Node):
    _param = None
    _bridge = None
    _sa = None

    _sub_image = None
    _sub_image_lifo = None
    _sub_image_queue_size = 2
    _sub_image_name = 'in/image'

    _sub_pose = None
    _sub_pose_lifo = None
    _sub_pose_queue_size = 2
    _sub_pose_name = 'in/pose_data'

    _pub_image = None
    _pub_image_queue_size = 2
    _pub_image_name = 'out/image'
    _send_msg = Image()

    _fps_calc = 0
    _timer_output_information = None
    _timer_output_information_size = (5 * 60)
    _timer_output_information_period_fps = 0.2  # 5 seconds
    _display_fps = 0

    def __init__(self, node_name):
        super().__init__(node_name)
        self._sub_image_lifo = queue.LifoQueue(self._sub_image_queue_size)
        self._sub_pose_lifo = queue.LifoQueue(self._sub_pose_queue_size)
        self._param = MediapipeExtNodeParam()

    def open(self):
        result = True
        try:
            self._param.init(self)
            self._bridge = CvBridge()
            self._sa = SchematicDiagram()
            self._fps_calc = CvFpsCalc(buffer_len=self._timer_output_information_size)
            # ##################################################################
            self._create_publisher()
            self._create_subscription()
            self._create_timer()
        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()
        return result

    def closing(self):
        return True

    def _create_publisher(self):
        self._pub_image = self.create_publisher(Image, self._pub_image_name, self._pub_image_queue_size)

    def _create_subscription(self):
        self._sub_image = self.create_subscription(Image, self._sub_image_name, self._callback_listener_image, self._sub_image_queue_size)
        self._sub_pose = self.create_subscription(Image, self._sub_pose_name, self._callback_listener_pose, self._sub_pose_queue_size)

    def _create_timer(self):
        # set ros callback
        self._timer_recognition = self.create_timer((1.0 / float(self._param.fps)), self._callback_timer)
        self._timer_output_information = self.create_timer(
            (1.0 / self._timer_output_information_period_fps), self._callback_output_information)

    def _callback_listener_image(self, msg):
        self.get_logger().error('_callback_listener_image')
        self._sub_image_lifo.put(msg)

    def _callback_listener_pose(self, msg):
        self.get_logger().error('_callback_listener_pose')
        self._sub_pose_lifo.put(msg)

    def _resize(self, image, width, height):
        im_h, im_w = image.shape[:2]
        if ((width == 0) or (height == 0)):
            width = im_w
            height = im_h
        aspect = im_w / im_h
        if (width / height >= aspect):
            new_h = height
            new_w = round(new_h * aspect)
        else:
            new_w = width
            new_h = round(new_w / aspect)
        return cv.resize(image, dsize=(new_w, new_h))

    def _callback_timer(self):
        try:
            self.get_logger().error('_callback_timer')
            if (self._sub_image_lifo.empty() is False):
                if (self._sub_pose_lifo.empty() is False):
                    image_msg = self._sub_image_lifo.get()
                    cv_image = self._bridge.imgmsg_to_cv2(image_msg, "bgr8")
                    height, width = cv_image.shape[:2]
                    pose_msg: PoseDetection = self._sub_pose_lifo.get()
                    schematic_diagram = self._sa.drawing(cv_image,
                                                         float(width),
                                                         float(height),
                                                         pose_msg.landmark,
                                                         pose_msg.detected_area,
                                                         pose_msg.human_detected,
                                                         self._param.drawing_box)
                    # float(self._ros_com.param_video_area_end_x - self._ros_com.param_video_area_start_x),
                    # float(self._ros_com.param_video_area_end_y - self._ros_com.param_video_area_start_y),
                    schematic_diagram = self._resize(schematic_diagram, self._param.width, self._param.height)
                    self._send_msg = self._bridge.cv2_to_imgmsg(np.array(schematic_diagram), "bgr8")
                    self._pub_image.publish(self._send_msg)

                    #####################################################
                    while not self._sub_image_lifo.empty():
                        self._sub_image_lifo.get()
                    while not self._sub_pose_lifo.empty():
                        self._sub_pose_lifo.get()
                    #####################################################
                    self._display_fps = self._fps_calc.get()
                    #####################################################
        except Exception as exception:
            self.get_logger().error('Exception (_callback_timer) : ' + str(exception))
            traceback.print_exc()

    def _callback_output_information(self):
        if (self._param.info_verbose is True):
            # Output information
            self.get_logger().info('[{}/{}] FPS : {:.4g}'.format(self.get_namespace(), self.get_name(), self._display_fps))


def main(args=None):
    rclpy.init(args=args)
    node_name = 'mediapipe_ext_node'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = MediapipeExtNode(node_name)

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
