#!/usr/bin/env python3.10

import rclpy
import traceback
import threading
import copy
import cv2 as cv
import numpy as np
import os
from rclpy.node import Node
from utils.usb_video_device import UsbVideoDevice
from utils.cv_fps_calc import CvFpsCalc
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
            self.img = None
            self.path = ''
            self.WIDTH = 1280
            self.HEIGHT = 1024
            pass

    class ParamNotify():
        def __init__(self):
            self.mess_verbose = False

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

    class ParamPublisher():
        def __init__(self):
            self.INTERVAL_FPS = 10.0
            self.width = 0
            self.height = 0
            self.enable = True
    # ############################################

    def print_parameter(self, node: Node):
        node.get_logger().info('<Parameter>')

        node.get_logger().info(' video: ')
        node.get_logger().info('   width  : ' + str(self.device.WIDTH))
        node.get_logger().info('   height : ' + str(self.device.HEIGHT))
        node.get_logger().info('   angle  : [{}, {}]'.format(self.device.ANGLE_X, self.device.ANGLE_Y))
        node.get_logger().info('   fps    : ' + str(self.device.FPS))

        node.get_logger().info(' area: ')
        node.get_logger().info('   mirror      : {}'.format(self.settings.mirror))
        node.get_logger().info('   upside_down : {}'.format(self.settings.upside_down))
        node.get_logger().info('   clockwise   : {}'.format(self.settings.clockwise))
        node.get_logger().info('   center      : [{}, {}]'.format(self.settings.AREA_CENTER_X, self.settings.AREA_CENTER_Y))
        node.get_logger().info('   w/h         : [{}, {}]'.format(self.settings.AREA_WIDTH, self.settings.AREA_HEIGHT))
        node.get_logger().info('   start       : [{}, {}]'.format(self.settings.calc_start_x, self.settings.calc_start_y))
        node.get_logger().info('   end         : [{}, {}]'.format(self.settings.calc_end_x, self.settings.calc_end_y))

        node.get_logger().info(' publisher : ')
        node.get_logger().info('  enable  : ' + str(self.publisher.enable))
        node.get_logger().info('  width   : ' + str(self.publisher.width))
        node.get_logger().info('  height  : ' + str(self.publisher.height))
        node.get_logger().info('  fps     : ' + str(self.publisher.INTERVAL_FPS))

        node.get_logger().info(' info : ')
        node.get_logger().info('   verbose : {}'.format(self.notify.mess_verbose))

    def _calc_area(self, width, height,
                   area_center_x, area_center_y, area_width, area_height):

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

    def update_device_info(self, path):
        result = False
        if os.path.isfile(path):
            self.device.path = path
            self.device.img = cv.imread(path)
            self.device.WIDTH = self.device.img.shape[1]
            self.device.HEIGHT = self.device.img.shape[0]
            result = True
        else:
            self.device.path = ""
            pass
        return result

    def _callback_on_params(self, parameter_list):
        result = False
        buf_area_x = self.settings.AREA_CENTER_X
        buf_area_y = self.settings.AREA_CENTER_Y
        buf_area_width = self.settings.AREA_WIDTH
        buf_area_height = self.settings.AREA_HEIGHT
        for parameter in parameter_list:
            # device
            if (parameter.name == 'photo'):
                result = self.update_device_info(parameter.value)
            # notify
            if (parameter.name == 'notify/message/verbose'):
                self.notify.mess_verbose = parameter.value
                result = True
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
            # publisher
            if (parameter.name == 'publisher/INTERVAL_FPS'):
                if (parameter.value == self.publisher.INTERVAL_FPS):
                    result = True
            if (parameter.name == 'publisher/resize/width'):
                self.publisher.width = parameter.value
                result = True
            if (parameter.name == 'publisher/resize/height'):
                self.publisher.height = parameter.value
                result = True
            if (parameter.name == 'publisher/enable'):
                self.publisher.enable = parameter.value
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
        # device
        node.declare_parameter('photo', self.device.path)
        # notify
        node.declare_parameter('notify/message/verbose', self.notify.mess_verbose)
        # settings
        node.declare_parameter('settings/area/mirror', self.settings.mirror)
        node.declare_parameter('settings/area/upside_down', self.settings.upside_down)
        node.declare_parameter('settings/area/clockwise', self.settings.clockwise)
        node.declare_parameter('settings/area/center_x', self.settings.AREA_CENTER_X)
        node.declare_parameter('settings/area/center_y', self.settings.AREA_CENTER_Y)
        node.declare_parameter('settings/area/width', self.settings.AREA_WIDTH)
        node.declare_parameter('settings/area/height', self.settings.AREA_HEIGHT)
        # publisher
        node.declare_parameter('publisher/INTERVAL_FPS', self.publisher.INTERVAL_FPS)
        node.declare_parameter('publisher/resize/width', self.publisher.width)
        node.declare_parameter('publisher/resize/height', self.publisher.height)
        node.declare_parameter('publisher/enable', self.publisher.enable)
        ####################
        self.get_parameter(node)
        # set callback
        node.add_on_set_parameters_callback(self._callback_on_params)

    def get_parameter(self, node: Node):
        self.device.path = node.get_parameter('photo').get_parameter_value().string_value
        # notify
        self.notify.mess_verbose = node.get_parameter('notify/message/verbose').get_parameter_value().bool_value
        # settings
        self.settings.mirror = node.get_parameter('settings/area/mirror').get_parameter_value().bool_value
        self.settings.upside_down = node.get_parameter('settings/area/upside_down').get_parameter_value().bool_value
        self.settings.clockwise = node.get_parameter('settings/area/clockwise').get_parameter_value().integer_value
        self.settings.AREA_CENTER_X = node.get_parameter('settings/area/center_x').get_parameter_value().integer_value
        self.settings.AREA_CENTER_Y = node.get_parameter('settings/area/center_y').get_parameter_value().integer_value
        self.settings.AREA_WIDTH = node.get_parameter('settings/area/width').get_parameter_value().integer_value
        self.settings.AREA_HEIGHT = node.get_parameter('settings/area/height').get_parameter_value().integer_value
        # publisher
        self.publisher.width = node.get_parameter('publisher/resize/width').get_parameter_value().integer_value
        self.publisher.height = node.get_parameter('publisher/resize/height').get_parameter_value().integer_value
        self.publisher.enable = node.get_parameter('publisher/enable').get_parameter_value().bool_value
        self._calc_area(self.device.WIDTH, self.device.HEIGHT,
                        self.settings.AREA_CENTER_X, self.settings.AREA_CENTER_Y,
                        self.settings.AREA_WIDTH, self.settings.AREA_HEIGHT)

    def init(self, node: Node):
        self.device = self.ParamDevice()
        self.notify = self.ParamNotify()
        self.settings = self.ParamSettings()
        self.publisher = self.ParamPublisher()
        self._init_param(node)

    def __init__(self):
        pass


class VideoCaptureNode(Node):
    _service_name_info = 'out_info'
    _service_name_capture = 'out_srv'
    _topic_out_name = 'out_topic'
    _topic_in_name = 'in_topic'
    _timeout_ms = 5000
    ##########################################################################
    _debug = False
    ##########################################################################
    _param = None
    _video = None
    ##########################################################################
    _fps_calc = None
    _bridge = None
    _display_fps = 0

    ##########################################################################
    _topic_queue_size = 2
    _timer_output_information_period_fps = 0.2  # 5 seconds
    _timer_output_information_size = (5 * 60)
    ##########################################################################
    _timer_capture = None
    _timer_send = None
    _timer_output_information = None
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
            self._fps_calc = CvFpsCalc(buffer_len=self._timer_output_information_size)
            self._bridge = CvBridge()

            # blank image
            self._blank_image = self._set_image(np.zeros((self._param.device.HEIGHT, self._param.device.WIDTH, 3), np.uint8))

            # create ros
            self._create_service()
            self._create_publisher()
            self._create_timer()
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

    def _create_publisher(self):
        self._pub_image = self.create_publisher(Image, self._topic_out_name, self._topic_queue_size)

    def _create_timer(self):
        self._timer_send = self.create_timer((1.0 / self._param.publisher.INTERVAL_FPS), self._callback_send)

    def _request_shutdown(self):
        if (self._timer_capture is not None):
            self._timer_capture.cancel()
        if (self._timer_send is not None):
            self._timer_send.cancel()
        if (self._timer_output_information is not None):
            self._timer_output_information.cancel()
        self._is_running = False

    ##########################################################################
    # ## ROS callback
    def _callback_srv_video_device_info(self,
                                        request: MrsSrv.VideoDeviceInfo.Request,
                                        response: MrsSrv.VideoDeviceInfo.Response):
        response.id = -1
        response.type = "image"
        response.by_path_name = ""
        response.path = str(self._param.device.path)
        response.video_info.angle_x = 0
        response.video_info.angle_y = 0
        response.video_info.width = self._param.device.WIDTH
        response.video_info.height = self._param.device.HEIGHT
        response.video_info.fps = float(0.0)
        ext = os.path.splitext(self._param.device.path)[1][1:]
        response.video_info.format = ext
        return response

    # ## video capture
    def _callback_srv_video_capture(self,
                                    request: MrsSrv.VideoCapture.Request,
                                    response: MrsSrv.VideoCapture.Response):
        try:
            image = None
            with self._lock:
                if (self._param.device.img is not None):
                    image = copy.deepcopy(self._param.device.img)
            if (image is not None):
                image = self._resize(image, request.resize_width, request.resize_height)
                response.image = self._set_image(image)
            else:
                response.image = self._blank_image
            return response
        except Exception as exception:
            self.get_logger().error('Exception (_callback_srv_video_capture) : ' + str(exception))
            return response

    # ##
    def _callback_send(self):
        try:
            if (self._param.publisher.enable is True):
                with self._lock:
                    image = copy.deepcopy(self._param.device.img)
                if (image is not None):
                    image = self._resize(image, self._param.publisher.width, self._param.publisher.height)
                    self._send_data = self._set_image(image)
                    if (self._send_data is not None):
                        self._pub_image.publish(self._send_data)
        except Exception as exception:
            self.get_logger().error('Exception (_callback_send) : ' + str(exception))
            traceback.print_exc()
            pass


def main(args=None):
    rclpy.init(args=args)
    node_name = 'photo_to_video_node'
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
