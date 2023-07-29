#!/usr/bin/env python3.10

import rclpy
import traceback
import threading
import cv2 as cv
import numpy as np
from rclpy.node import Node
from utils.usb_video_device import UsbVideoDevice
from utils.cv_fps_calc import CvFpsCalc
from rclpy.exceptions import ParameterNotDeclaredException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class VideoCaptureNodeParam(Node):

    def __init__(self):
        self.update = False


class MediapipeNode(Node):

    _param = VideoCaptureNodeParam()

    def __init__(self, node_name):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._init_param()

    def open(self):
        return True

    def closing(self):
        return True


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
