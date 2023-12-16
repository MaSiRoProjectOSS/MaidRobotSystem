#!/usr/bin/env python3.10

import rclpy
import traceback
import threading
import copy
import cv2 as cv
import numpy as np
import os
import time
from datetime import datetime
from rclpy.node import Node
from utils.usb_video_device import UsbVideoDevice
from utils.cv_fps_calc import CvFpsCalc
from rclpy.exceptions import ParameterNotDeclaredException
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import maid_robot_system_interfaces.srv as MrsSrv


class VideoDeviceManager():
    cap = None

    def __init__(self):
        pass

    def search_video(self, type, id, path):
        result = -1
        usbVideoDevice = UsbVideoDevice(type)
        result = usbVideoDevice.get_id_by_path(path)
        if (-1 == result):
            result = usbVideoDevice.get_id_from_id(id)
        return usbVideoDevice.get_info(result)

    def open(self, device_id):
        self.cap = cv.VideoCapture(device_id)

    def closing(self):
        if (self.cap is not None):
            self.cap.release()

    def isOpened(self):
        if (self.cap is not None):
            return self.cap.isOpened()
        else:
            return False

    def set_video_setting(self, format, width, height, fps=30.0):
        if (self.cap is not None):
            if (format == 'MJPG'):
                self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # .avi
            if (format == 'H264'):
                self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('H', '2', '6', '4'))
            if (format == 'YUYV'):
                self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            if (format == 'BGR3'):
                self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('B', 'G', 'R', '3'))
            if (format == 'MP4V'):
                self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'P', '4', 'V'))  # .mp4
            if (format == 'MP4S'):
                self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'P', '4', 'S'))  # .mp4

            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv.CAP_PROP_FPS, fps)

    def txt_cv_fourcc(self, ):
        v = int(self.cap.get(cv.CAP_PROP_FOURCC))
        return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

    def _video_to_txt_cv_mode(self, mode):
        if (0 == mode):
            return "BGR"
        elif (1 == mode):
            return "RGB"
        elif (2 == mode):
            return "GRAY"
        elif (3 == mode):
            return "YUYV"
        else:
            return "Unknown" + str(mode)


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

    class ParamNotify():
        def __init__(self):
            self.mess_verbose = False

    class ParamSettings():
        def __init__(self):
            self.save_folder = ""
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

        node.get_logger().info(' device: ')
        node.get_logger().info('   type    : ' + str(self.device.TYPE))
        node.get_logger().info('   id      : ' + str(self.device.ID))
        node.get_logger().info('   by_path : ' + str(self.device.NAME_BY_PATH))
        node.get_logger().info('   path    : ' + str(self.device.PATH))

        node.get_logger().info(' video: ')
        node.get_logger().info('   format : ' + str(self.device.FORMAT))
        node.get_logger().info('   width  : ' + str(self.device.WIDTH))
        node.get_logger().info('   height : ' + str(self.device.HEIGHT))
        node.get_logger().info('   angle  : [{}, {}]'.format(self.device.ANGLE_X, self.device.ANGLE_Y))
        node.get_logger().info('   fps    : ' + str(self.device.FPS))

        node.get_logger().info(' area: ')
        node.get_logger().info('   save folder : ' + str(self.settings.save_folder))
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

    def update_device_info(self, node: Node, type, by_path, id, path, fourcc_text, width, height, fps):
        self.device.TYPE = type
        self.device.NAME_BY_PATH = by_path
        self.device.ID = int(id)
        self.device.PATH = path
        self.device.FORMAT = fourcc_text
        self.device.WIDTH = int(width)
        self.device.HEIGHT = int(height)
        # self.device.ANGLE = self.device.ANGLE
        self.device.FPS = float(fps)

        new_parameters = [Parameter('device/TYPE', Parameter.Type.STRING, self.device.TYPE),
                          Parameter('device/BY_PATH', Parameter.Type.STRING, self.device.NAME_BY_PATH),
                          Parameter('device/ID', Parameter.Type.INTEGER, self.device.ID),
                          Parameter('device/PATH', Parameter.Type.STRING, self.device.PATH),
                          Parameter('device/settings/FORMAT', Parameter.Type.STRING, self.device.FORMAT),
                          Parameter('device/settings/WIDTH', Parameter.Type.INTEGER, self.device.WIDTH),
                          Parameter('device/settings/HEIGHT', Parameter.Type.INTEGER, self.device.HEIGHT),
                          Parameter('device/settings/ANGLE_X', Parameter.Type.INTEGER, self.device.ANGLE_X),
                          Parameter('device/settings/ANGLE_Y', Parameter.Type.INTEGER, self.device.ANGLE_Y),
                          Parameter('device/settings/FPS', Parameter.Type.DOUBLE, self.device.FPS),
                          Parameter('save_folder', Parameter.Type.STRING, self.settings.save_folder)
                          ]
        node.set_parameters(new_parameters)

    def _callback_on_params(self, parameter_list):
        result = False
        buf_area_x = self.settings.AREA_CENTER_X
        buf_area_y = self.settings.AREA_CENTER_Y
        buf_area_width = self.settings.AREA_WIDTH
        buf_area_height = self.settings.AREA_HEIGHT
        for parameter in parameter_list:
            # device
            if (parameter.name == 'device/TYPE'):
                if (parameter.value == self.device.TYPE):
                    result = True
            if (parameter.name == 'device/BY_PATH'):
                if (parameter.value == self.device.NAME_BY_PATH):
                    result = True
            if (parameter.name == 'device/PATH'):
                if (parameter.value == self.device.PATH):
                    result = True
            if (parameter.name == 'device/ID'):
                if (parameter.value == self.device.ID):
                    result = True
            if (parameter.name == 'device/settings/FORMAT'):
                if (parameter.value == self.device.FORMAT):
                    result = True
            if (parameter.name == 'device/settings/WIDTH'):
                if (parameter.value == self.device.WIDTH):
                    result = True
            if (parameter.name == 'device/settings/HEIGHT'):
                if (parameter.value == self.device.HEIGHT):
                    result = True
            if (parameter.name == 'device/settings/ANGLE_X'):
                if (parameter.value == self.device.ANGLE_X):
                    result = True
            if (parameter.name == 'device/settings/ANGLE_Y'):
                if (parameter.value == self.device.ANGLE_Y):
                    result = True
            if (parameter.name == 'device/settings/FPS'):
                if (parameter.value == self.device.FPS):
                    result = True
            # notify
            if (parameter.name == 'notify/message/verbose'):
                self.notify.mess_verbose = parameter.value
                result = True
            # settings
            if (parameter.name == 'save_folder'):
                self.settings.save_folder = parameter.value
                result = True
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
        node.declare_parameter('device/TYPE', self.device.TYPE)
        node.declare_parameter('device/BY_PATH', self.device.NAME_BY_PATH)
        node.declare_parameter('device/PATH', self.device.PATH)
        node.declare_parameter('device/ID', self.device.ID)
        node.declare_parameter('device/settings/FORMAT', self.device.FORMAT)
        node.declare_parameter('device/settings/WIDTH', self.device.WIDTH)
        node.declare_parameter('device/settings/HEIGHT', self.device.HEIGHT)
        node.declare_parameter('device/settings/ANGLE_X', self.device.ANGLE_X)
        node.declare_parameter('device/settings/ANGLE_Y', self.device.ANGLE_Y)
        node.declare_parameter('device/settings/FPS', self.device.FPS)
        # notify
        node.declare_parameter('notify/message/verbose', self.notify.mess_verbose)
        # settings
        node.declare_parameter('save_folder', self.settings.save_folder)
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
        # get parameter
        self.device.TYPE = node.get_parameter('device/TYPE').get_parameter_value().string_value
        self.device.NAME_BY_PATH = node.get_parameter('device/BY_PATH').get_parameter_value().string_value
        self.device.ID = node.get_parameter('device/ID').get_parameter_value().integer_value
        self.device.FORMAT = node.get_parameter('device/settings/FORMAT').get_parameter_value().string_value
        self.device.WIDTH = node.get_parameter('device/settings/WIDTH').get_parameter_value().integer_value
        self.device.HEIGHT = node.get_parameter('device/settings/HEIGHT').get_parameter_value().integer_value
        self.device.ANGLE_X = node.get_parameter('device/settings/ANGLE_X').get_parameter_value().integer_value
        self.device.ANGLE_Y = node.get_parameter('device/settings/ANGLE_Y').get_parameter_value().integer_value
        self.device.FPS = node.get_parameter('device/settings/FPS').get_parameter_value().double_value
        self.publisher.INTERVAL_FPS = node.get_parameter('publisher/INTERVAL_FPS').get_parameter_value().double_value
        self.get_parameter(node)
        # set callback
        node.add_on_set_parameters_callback(self._callback_on_params)

    def get_parameter(self, node: Node):
        # notify
        self.notify.mess_verbose = node.get_parameter('notify/message/verbose').get_parameter_value().bool_value
        # settings
        self.settings.save_folder = node.get_parameter('save_folder').get_parameter_value().string_value
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
    _service_name_capture = 'out_srv'
    _subscribe_save = 'save'
    _topic_name = 'out_topic'
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
        self._video = VideoDeviceManager()

    def is_running(self):
        return self._is_running

    def closing(self):
        self._request_shutdown()
        if (self._video is not None):
            self._video.closing()

    def open(self):
        result = False
        try:
            self._param.init(self)
            ###################################################################
            # set instance
            self._fps_calc = CvFpsCalc(buffer_len=self._timer_output_information_size)
            self._bridge = CvBridge()

            # set video
            self._param.device.ID, self._param.device.NAME_BY_PATH, self._param.device.PATH = self._video.search_video(self._param.device.TYPE, self._param.device.ID, self._param.device.NAME_BY_PATH)
            if (-1 != self._param.device.ID):
                try:
                    self._video.open(self._param.device.ID)
                    if (not self._video.isOpened()):
                        self._param.device.ID = -1
                    else:
                        self._video.set_video_setting(self._param.device.FORMAT,
                                                      self._param.device.WIDTH,
                                                      self._param.device.HEIGHT,
                                                      self._param.device.FPS)
                        self._video.cap.grab()
                        self._next_time = (self._timeout_ms * 1000 * 1000) + self.get_clock().now().nanoseconds
                        ret = self._callback_video_capture()
                        if (ret is False):
                            self._param.device.ID = -1

                        # check video device
                        if (0 > self._param.device.ID):
                            self.get_logger().error('Not found Camera')
                        else:
                            self._param.update_device_info(self,
                                                           self._param.device.TYPE,
                                                           self._param.device.NAME_BY_PATH,
                                                           self._param.device.ID,
                                                           self._param.device.PATH,
                                                           self._video.txt_cv_fourcc(),
                                                           self._video.cap.get(cv.CAP_PROP_FRAME_WIDTH),
                                                           self._video.cap.get(cv.CAP_PROP_FRAME_HEIGHT),
                                                           self._video.cap.get(cv.CAP_PROP_FPS))
                            if (self._debug is True):
                                self._param.print_parameter(self)
                            self.get_logger().info('Open Camera : {} ({})'.format(self._param.device.PATH, self._param.device.NAME_BY_PATH))
                            # create ros
                            self._create_service()
                            self._create_subscription()
                            self._create_publisher()
                            self._create_timer()
                            result = True
                except Exception as exception:
                    self.get_logger().error('Not open Camera : /dev/video' + str(self._param.device.ID) + ' : ' + str(exception))
                    self._param.device.ID = -1
                except:
                    self.get_logger().error('Not open Camera : /dev/video' + str(self._param.device.ID))
                    self._param.device.ID = -1

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
        self._service_capture = self.create_service(MrsSrv.VideoCapture, self._service_name_capture, self._callback_srv_video_capture)

    def _create_subscription(self):
        self._sub_image = self.create_subscription(
            String,
            self._subscribe_save,
            self._callback_save,
            self._topic_queue_size)

    def _create_publisher(self):
        self._pub_image = self.create_publisher(Image, self._topic_name, self._topic_queue_size)

    def _create_timer(self):
        self._timer_capture = self.create_timer((1.0 / self._param.device.FPS), self._callback_video_capture)
        self._timer_send = self.create_timer((1.0 / self._param.publisher.INTERVAL_FPS), self._callback_send)
        self._timer_output_information = self.create_timer((1.0 / self._timer_output_information_period_fps), self._callback_output_information)

    def _request_shutdown(self):
        if (self._timer_capture is not None):
            self._timer_capture.cancel()
        if (self._timer_send is not None):
            self._timer_send.cancel()
        if (self._timer_output_information is not None):
            self._timer_output_information.cancel()
        self._is_running = False

    ##########################################################################

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
    def _callback_save(self, msg: String):
        try:
            if not self._param.settings.save_folder:
                self.get_logger().warning('[{}/{}] {}'.format(self.get_namespace(), self.get_name(), "The folder to save is not set."))
            else:
                with self._lock:
                    image = copy.deepcopy(self._image)
                if (image is not None):
                    folder = os.path.join(self._param.settings.save_folder, '{:%Y%m%d}'.format(datetime.now()))
                    if not os.path.exists(folder):
                        os.makedirs(folder)
                    name = ""
                    if (msg.data is not None):
                        if (msg.data != ""):
                            name = "_" + msg.data
                    path = os.path.join(folder, time.strftime(
                        "%H%M%S", time.localtime()) + name + ".jpg")
                    self.get_logger().info('Save image' + str(path))
                    cv.imwrite(path, image)
        except Exception as exception:
            self.get_logger().error('Exception (_callback_save) : ' + str(exception))
            traceback.print_exc()

    # ## video capture
    def _callback_video_capture(self):
        result = False
        try:
            current_ns = self.get_clock().now().nanoseconds
            result, sc = self._video.cap.read()
            #####################################################
            if (result is True):
                with self._lock:
                    self._image = self._repack(sc,
                                               self._param.settings.calc_start_x,
                                               self._param.settings.calc_start_y,
                                               self._param.settings.calc_end_x,
                                               self._param.settings.calc_end_y,
                                               self._param.settings.mirror,
                                               self._param.settings.upside_down,
                                               self._param.settings.clockwise)
                #####################################################
                self._next_time = (self._timeout_ms * 1000 * 1000) + current_ns
                self._display_fps = self._fps_calc.get()
                #####################################################
            if (self._next_time <= current_ns):
                self.get_logger().warning('[{}/{}] Timeout : [{}]'.format(self.get_namespace(), self.get_name(), current_ns))
                self._request_shutdown()
        except Exception as exception:
            self.get_logger().error('Exception (_callback_video_capture) : ' + str(exception))
            traceback.print_exc()
        return result

    # ##
    def _callback_send(self):
        try:
            if (self._param.publisher.enable is True):
                with self._lock:
                    image = copy.deepcopy(self._image)
                if (image is not None):
                    image = self._resize(image, self._param.publisher.width, self._param.publisher.height)
                    self._send_data = self._set_image(image)
                    if (self._send_data is not None):
                        self._pub_image.publish(self._send_data)
        except Exception as exception:
            self.get_logger().error('Exception (_callback_send) : ' + str(exception))
            traceback.print_exc()
            pass

    # ## Output message
    def _callback_output_information(self):
        if (self._param.notify.mess_verbose is True):
            # Output information
            self.get_logger().info('[{}/{}] FPS : {:.4g}'.format(self.get_namespace(), self.get_name(), self._display_fps))


def main(args=None):
    rclpy.init(args=args)
    node_name = 'video_capture_node'
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
