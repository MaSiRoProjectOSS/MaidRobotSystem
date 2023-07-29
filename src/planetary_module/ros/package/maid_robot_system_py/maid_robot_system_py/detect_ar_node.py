#!/usr/bin/env python3.10

import rclpy
import traceback
import queue
import cv2 as cv
import cv2.aruco as aruco
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from maid_robot_system_interfaces.msg._ar_markers import ArMarkers


class DetectARNodeParam():

    def _callback_on_params(self, parameter_list):
        result = False
        for parameter in parameter_list:
            self.get_logger().info('[{}/{}] Got {}={}'.format(
                self._namespace, self._node_name, parameter.name, parameter.value))

            if (parameter.name == 'preference/info/verbose'):
                self.info_verbose = parameter.value
                result = True

            if (parameter.name == 'preference/video/mirror'):
                self.mirror = parameter.value
                result = True

        return SetParametersResult(successful=result)

    def get_parameter(self, node: Node):
        self.info_verbose = node.get_parameter('preference/info/verbose').get_parameter_value().bool_value
        self.mirror = node.get_parameter('preference/video/mirror').get_parameter_value().bool_value

    def init(self, node: Node):
        self._namespace = node.get_namespace()
        self._node_name = node.get_name()
        node.declare_parameter('confidence/fps', self.fps)
        node.declare_parameter('preference/info/verbose', self.info_verbose)
        node.declare_parameter('preference/video/mirror', self.mirror)
        self.get_parameter(node)
        node.add_on_set_parameters_callback(self._callback_on_params)

    def __init__(self):
        self._namespace = ''
        self._node_name = ''
        self.info_verbose = False
        self.mirror = False
        self.fps = 10.0


class DetectARNode(Node):
    _lifo = None
    _param = None
    _sub_queue_size = 2
    _sub_name = 'in'
    _pub = None
    _pub_queue_size = 5
    _pub_name = 'out'
    _send_msg = ArMarkers()
    _bridge = None

    def __init__(self, node_name):
        super().__init__(node_name)
        self._lifo = queue.LifoQueue(self._sub_queue_size)
        self._param = DetectARNodeParam()

    def open(self):
        result = True
        try:
            self._param.init(self)
            self._bridge = CvBridge()

            # ##################################################################
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters()
            self._detector = aruco.ArucoDetector(dictionary, parameters)
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
        self._pub = self.create_publisher(ArMarkers, self._pub_name, self._pub_queue_size)

    def _create_subscription(self):
        self.subscription = self.create_subscription(Image, self._sub_name, self._callback_listener_image, self._sub_queue_size)

    def _create_timer(self):
        # set ros callback
        self._timer_recognition = self.create_timer((1.0 / float(self._param.fps)), self._callback_timer_detect)

    ##################################################################################

    def detect_ar(self, image):
        corners, ids, rejectedImgPoints = self._detector.detectMarkers(image)
        return corners, ids
    ##################################################################################

    def _callback_listener_image(self, msg):
        # self.get_logger().info('_callback_listener_image')
        self._lifo.put(msg)
        pass

    def _callback_timer_detect(self):
        try:
            if (self._lifo.empty() is False):
                msg = self._lifo.get()
                cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                gray = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)
                if (self._param.mirror is True):
                    gray = cv.flip(gray, 1)
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

            while not self._lifo.empty():
                self._lifo.get()
        except Exception as exception:
            self.get_logger().error('Exception (_callback_recognition) : ' + str(exception))
            traceback.print_exc()


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
