#!/usr/bin/env python3.10

import rclpy
import traceback
import threading
import copy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import maid_robot_system_interfaces.srv as MrsSrv


class VideoCaptureNodeParam():
    # ############################################
    class ParamDevice():
        def __init__(self):
            self.TYPE = 'v4l'
            self.NAME_BY_PATH = ''
            self.PATH = ''
            self.ID = -1
            self.FORMAT = "MJPG"
            self.WIDTH = 1280
            self.HEIGHT = 1024
            self.ANGLE_X = 140
            self.ANGLE_Y = 140
            self.FPS = 30.0
            pass

    class ParamSettings():
        def __init__(self):
            self.mirror = False
            self.upside_down = False
            self.clockwise = int(MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK)
            self.AREA_CENTER_X = 0
            self.AREA_CENTER_Y = 0
            self.AREA_WIDTH = 0
            self.AREA_HEIGHT = 0
            #######################
            self.calc_start_x = 0
            self.calc_start_y = 0
            self.calc_end_x = 0
            self.calc_end_y = 0

    # ############################################

    def print_parameter(self, node: Node):
        node.get_logger().info('<Parameter>')
        node.get_logger().info(' area: ')
        node.get_logger().info('   mirror      : {}'.format(self.settings.mirror))
        node.get_logger().info('   upside_down : {}'.format(self.settings.upside_down))
        node.get_logger().info('   clockwise   : {}'.format(self.settings.clockwise))
        node.get_logger().info('   center      : [{}, {}]'.format(self.settings.AREA_CENTER_X, self.settings.AREA_CENTER_Y))
        node.get_logger().info('   w/h         : [{}, {}]'.format(self.settings.AREA_WIDTH, self.settings.AREA_HEIGHT))
        node.get_logger().info('   start       : [{}, {}]'.format(self.settings.calc_start_x, self.settings.calc_start_y))
        node.get_logger().info('   end         : [{}, {}]'.format(self.settings.calc_end_x, self.settings.calc_end_y))

    def _calc_area(self, width, height, area_center_x, area_center_y, area_width, area_height):
        area_x = int((width / 2) - (area_width / 2) + area_center_x)
        area_y = int((height / 2) - (area_height / 2) + area_center_y)
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

        self.settings.calc_start_x = start_x
        self.settings.calc_start_y = start_y
        self.settings.calc_end_x = end_x
        self.settings.calc_end_y = end_y

    def _callback_on_params(self, parameter_list):
        result = False
        buf_area_x = self.settings.AREA_CENTER_X
        buf_area_y = self.settings.AREA_CENTER_Y
        buf_area_width = self.settings.AREA_WIDTH
        buf_area_height = self.settings.AREA_HEIGHT
        for parameter in parameter_list:
            # settings
            if (parameter.name == 'settings/area/mirror'):
                self.settings.mirror = parameter.value
                result = True
            if (parameter.name == 'settings/area/upside_down'):
                self.settings.upside_down = parameter.value
                result = True
            if (parameter.name == 'settings/area/clockwise'):
                value = parameter.value
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK):
                    self.settings.clockwise = parameter.value
                    result = True
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_03_O_CLOCK):
                    self.settings.clockwise = parameter.value
                    result = True
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_06_O_CLOCK):
                    self.settings.clockwise = parameter.value
                    result = True
                if (value is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_09_O_CLOCK):
                    self.settings.clockwise = parameter.value
                    result = True
            if (parameter.name == 'settings/area/center_x'):
                buf_area_x = parameter.value
                result = True
            if (parameter.name == 'settings/area/center_y'):
                buf_area_y = parameter.value
                result = True
            if (parameter.name == 'settings/area/width'):
                buf_area_width = parameter.value
                result = True
            if (parameter.name == 'settings/area/height'):
                buf_area_height = parameter.value
                result = True
        if (result is True):
            self.settings.AREA_CENTER_X = buf_area_x
            self.settings.AREA_CENTER_Y = buf_area_y
            self.settings.AREA_WIDTH = buf_area_width
            self.settings.AREA_HEIGHT = buf_area_height
            self._calc_area(self.device.WIDTH, self.device.HEIGHT,
                            self.settings.AREA_CENTER_X, self.settings.AREA_CENTER_Y,
                            self.settings.AREA_WIDTH, self.settings.AREA_HEIGHT)
        return SetParametersResult(successful=result)

    def _init_param(self, node: Node):
        # settings
        node.declare_parameter('settings/area/mirror', self.settings.mirror)
        node.declare_parameter('settings/area/upside_down', self.settings.upside_down)
        node.declare_parameter('settings/area/clockwise', self.settings.clockwise)
        node.declare_parameter('settings/area/center_x', self.settings.AREA_CENTER_X)
        node.declare_parameter('settings/area/center_y', self.settings.AREA_CENTER_Y)
        node.declare_parameter('settings/area/width', self.settings.AREA_WIDTH)
        node.declare_parameter('settings/area/height', self.settings.AREA_HEIGHT)
        ####################
        self.get_parameter(node)
        # set callback
        node.add_on_set_parameters_callback(self._callback_on_params)

    def get_parameter(self, node: Node):
        # settings
        self.settings.mirror = node.get_parameter('settings/area/mirror').get_parameter_value().bool_value
        self.settings.upside_down = node.get_parameter('settings/area/upside_down').get_parameter_value().bool_value
        self.settings.clockwise = node.get_parameter('settings/area/clockwise').get_parameter_value().integer_value
        self.settings.AREA_CENTER_X = node.get_parameter('settings/area/center_x').get_parameter_value().integer_value
        self.settings.AREA_CENTER_Y = node.get_parameter('settings/area/center_y').get_parameter_value().integer_value
        self.settings.AREA_WIDTH = node.get_parameter('settings/area/width').get_parameter_value().integer_value
        self.settings.AREA_HEIGHT = node.get_parameter('settings/area/height').get_parameter_value().integer_value
        # publisher
        self._calc_area(self.device.WIDTH, self.device.HEIGHT,
                        self.settings.AREA_CENTER_X, self.settings.AREA_CENTER_Y,
                        self.settings.AREA_WIDTH, self.settings.AREA_HEIGHT)

    def init(self, node: Node):
        self.device = self.ParamDevice()
        self.settings = self.ParamSettings()
        self._init_param(node)

    def __init__(self):
        pass


class VideoCaptureNode(Node):
    _service_name_info = 'out_info'
    _service_name_capture = 'out_srv'
    _topic_in_name = 'in_topic'
    _timeout_ms = 5000
    ##########################################################################
    _param = None
    _video = None
    ##########################################################################
    _bridge = None
    ##########################################################################
    _topic_queue_size = 2
    ##########################################################################
    _is_running = False

    def __init__(self, node_name):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._param = VideoCaptureNodeParam()

    def is_running(self):
        return self._is_running

    def closing(self):
        self._request_shutdown()

    def open(self):
        result = False
        try:
            self._param.init(self)
            ###################################################################
            # set instance
            self._bridge = CvBridge()

            # create ros
            self._create_service()
            self._create_subscription()
            result = True

        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()

        self._is_running = result
        return result

    ##########################################################################
    # Image
    def _repack(self, image, start_x, start_y, end_x, end_y, mirror, upside_down, clockwise):
        dst = image[start_y:end_y, start_x:end_x]
        if ((upside_down is True) or (mirror is True)):
            if (upside_down is False):
                # mirror
                op_flip = 1
            elif ((mirror is False)):
                # upside_down
                op_flip = 0
            else:
                # upside_down and mirror
                op_flip = -1
            dst = cv.flip(dst, op_flip)
        if (clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_12_O_CLOCK):
            pass
        if (clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_03_O_CLOCK):
            dst = cv.rotate(dst, cv.ROTATE_90_CLOCKWISE)
        if (clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_06_O_CLOCK):
            dst = cv.rotate(dst, cv.ROTATE_180)
        if (clockwise is MrsSrv.VideoCapture.Request.ROTATE_CLOCKWISE_09_O_CLOCK):
            dst = cv.rotate(dst, cv.ROTATE_90_COUNTERCLOCKWISE)
        return dst

    def _set_image(self, image):
        return self._bridge.cv2_to_imgmsg(np.array(image), "bgr8")

    def _resize(self, image, width, height):
        im_h, im_w = image.shape[:2]
        if ((im_w != 0) and (im_h != 0)):
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
        else:
            return image

    ##########################################################################
    # ROS
    def _create_service(self):
        self._service_info = self.create_service(MrsSrv.VideoDeviceInfo, self._service_name_info, self._callback_srv_video_device_info)
        self._service_capture = self.create_service(MrsSrv.VideoCapture, self._service_name_capture, self._callback_srv_video_capture)

    def _create_subscription(self):
        self._sub_image = self.create_subscription(
            Image,
            self._topic_in_name,
            self._callback_video_capture,
            self._topic_queue_size)

    def _request_shutdown(self):
        self._is_running = False

    ##########################################################################
    # ## ROS callback
    def _callback_srv_video_device_info(self,
                                        request: MrsSrv.VideoDeviceInfo.Request,
                                        response: MrsSrv.VideoDeviceInfo.Response):
        response.data.id = self._param.device.ID
        response.data.type = str(self._param.device.TYPE)
        response.data.by_path_name = str(self._param.device.NAME_BY_PATH)
        response.data.path = str(self._param.device.PATH)
        response.data.angle_x = self._param.device.ANGLE_X
        response.data.angle_y = self._param.device.ANGLE_Y
        response.data.width = self._param.device.WIDTH
        response.data.height = self._param.device.HEIGHT
        response.data.fps = self._param.device.FPS
        response.data.format = str(self._param.device.FORMAT)
        return response

    # ## video capture
    def _callback_srv_video_capture(self,
                                    request: MrsSrv.VideoCapture.Request,
                                    response: MrsSrv.VideoCapture.Response):
        with self._lock:
            image = copy.deepcopy(self._image)
        if (image is not None):
            image = self._resize(image, request.resize_width, request.resize_height)
        response.image = self._set_image(image)
        return response

    # ## video capture
    def _callback_video_capture(self, msg):
        with self._lock:
            self._image = self._bridge.imgmsg_to_cv2(msg, "bgr8")


def main(args=None):
    rclpy.init(args=args)
    node_name = 'video_topic_to_service'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = VideoCaptureNode(node_name)

    try:
        if (node.open() is True):
            while (node.is_running() is True):
                rclpy.spin_once(node)

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
