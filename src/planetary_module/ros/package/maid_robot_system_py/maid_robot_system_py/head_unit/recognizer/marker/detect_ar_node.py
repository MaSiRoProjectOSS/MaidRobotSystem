#!/usr/bin/env python3.10

import rclpy
import traceback
import cv2 as cv
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from maid_robot_system_interfaces.msg._ar_markers import ArMarkers
from maid_robot_system_interfaces.srv._video_capture import VideoCapture
from utils.cv_fps_calc import CvFpsCalc


class DetectARNodeParam():
    def _callback_on_params(self, parameter_list):
        result = False
        for parameter in parameter_list:
            if (parameter.name == 'timeout_ms'):
                self.timeout_ms = parameter.value
                result = True
            if (parameter.name == 'mirror'):
                self.mirror = parameter.value
                result = True
            if (parameter.name == 'upside_down'):
                self.upside_down = parameter.value
                result = True
            if (parameter.name == 'width'):
                self.width = parameter.value
                result = True
            if (parameter.name == 'heigh'):
                self.height = parameter.value
                result = True
            if (parameter.name == 'notify/message/verbose'):
                self.info_verbose = parameter.value
                result = True
        return SetParametersResult(successful=result)

    def get_parameter(self, node: Node):
        self.INTERVAL_MS = node.get_parameter('INTERVAL_MS').get_parameter_value().integer_value
        self.timeout_ms = node.get_parameter('timeout_ms').get_parameter_value().integer_value
        self.mirror = node.get_parameter('mirror').get_parameter_value().bool_value
        self.upside_down = node.get_parameter('upside_down').get_parameter_value().bool_value
        self.width = node.get_parameter('width').get_parameter_value().integer_value
        self.height = node.get_parameter('heigh').get_parameter_value().integer_value
        self.info_verbose = node.get_parameter('notify/message/verbose').get_parameter_value().bool_value

    def init(self, node: Node):
        node.declare_parameter('INTERVAL_MS', self.INTERVAL_MS)
        node.declare_parameter('timeout_ms', self.timeout_ms)
        node.declare_parameter('mirror', self.mirror)
        node.declare_parameter('upside_down', self.upside_down)
        node.declare_parameter('width', self.width)
        node.declare_parameter('heigh', self.height)
        node.declare_parameter('notify/message/verbose', self.info_verbose)
        self.get_parameter(node)
        node.add_on_set_parameters_callback(self._callback_on_params)

    def __init__(self):
        self.INTERVAL_MS = 500
        self.timeout_ms = 5000
        self.mirror = False
        self.upside_down = False
        self.width = 640
        self.height = 512
        self.info_verbose = False


class DetectARNode(Node):
    _in_image_name = 'in_srv'
    _out_data_name = 'out'
    _send_msg = ArMarkers()
    _out_data_queue_size = 5
    #################################
    _bridge = None
    _param = DetectARNodeParam()
    _timer_output_info_time_ms = 5000
    _display_fps = 0
    _detector = None
    _request = VideoCapture.Request()
    _fps_calc = None
    _timer_output_information_size = (5 * 60)
    #################################
    _debug_mode = False
    _debug_pub_name = 'out_image'
    #################################

    def __init__(self, node_name):
        super().__init__(node_name)

    def open(self):
        result = False
        try:
            self._param.init(self)
            self._bridge = CvBridge()
            self._fps_calc = CvFpsCalc(buffer_len=self._timer_output_information_size)

            # ##################################################################
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters()
            self._detector = aruco.ArucoDetector(dictionary, parameters)
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
        self._client = self.create_client(VideoCapture, self._in_image_name)
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service({}) not available, waiting again...'.format(self._client.srv_name))
        self._request = VideoCapture.Request()
        self._request_image(self.get_clock().now().nanoseconds)

    def _create_publisher(self):
        self._pub = self.create_publisher(ArMarkers, self._out_data_name, self._out_data_queue_size)
        if (self._debug_mode is True):
            self._pub_image = self.create_publisher(Image, self._debug_pub_name, 2)

    def _create_timer(self):
        # set ros callback
        self._timer_recognition = self.create_timer(float(self._param.INTERVAL_MS / 1000.0), self._callback_timer_detect)
        self._timer_output_info = self.create_timer(float(self._timer_output_info_time_ms / 1000.0), self._callback_output_information)

    ##################################################################################
    def detect_ar(self, image):
        corners, ids, rejectedImgPoints = self._detector.detectMarkers(image)
        return corners, ids

    ##################################################################################
    def _request_image(self, current_ns):
        self._request.resize_width = self._param.width
        self._request.resize_height = self._param.height
        self._response = self._client.call_async(self._request)
        self._next_time = (self._param.timeout_ms * 1000 * 1000) + current_ns

    def _callback_timer_detect(self):
        try:
            current_ns = self.get_clock().now().nanoseconds
            if (self._response is not None):
                if (self._response.done() is True):
                    msg = self._response.result()
                    cv_image = self._bridge.imgmsg_to_cv2(msg.image, "bgr8")
                    gray = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
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
                        gray = cv.flip(gray, op_flip)
                    if (self._debug_mode is True):
                        if (self._pub_image is not None):
                            debug_image = cv.cvtColor(gray, cv.COLOR_GRAY2RGB)
                            debug_send_data = self._bridge.cv2_to_imgmsg(np.array(debug_image), "bgr8")
                            self._pub_image.publish(debug_send_data)
                    corners, ids = self.detect_ar(gray)
                    if (ids is not None):
                        if (self._param.info_verbose is True):
                            self.get_logger().info('[{}/{}] size:{}, id:{}'.format(self.get_namespace(), self.get_name(), ids.size, str(ids)))
                        data = []
                        for num in range(ids.size):
                            if (ids[num] is not None):
                                data.append(int(ids[num]))
                            if (corners[num] is not None):
                                pass
                        self._send_msg.ids = (data)
                        self._pub.publish(self._send_msg)
                    #####################################################
                    self._request_image(current_ns)
                    self._display_fps = self._fps_calc.get()
                    #####################################################

            if (self._next_time <= current_ns):
                self._request_image(current_ns)
                self.get_logger().warning('Timeout')
        except Exception as exception:
            self.get_logger().error('Exception (_callback_recognition) : ' + str(exception))
            traceback.print_exc()

    def _callback_output_information(self):
        if (self._param.info_verbose is True):
            self.get_logger().info('[{}/{}] FPS : {:.4g}'.format(self.get_namespace(), self.get_name(), self._display_fps))


def main(args=None):
    rclpy.init(args=args)
    node_name = 'detect_ar_node'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = DetectARNode(node_name)

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
