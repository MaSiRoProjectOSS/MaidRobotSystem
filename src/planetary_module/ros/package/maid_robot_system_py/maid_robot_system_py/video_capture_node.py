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


class VideoCaptureNodeParam():
    class ParamInfo():
        def __init__(self):
            self.verbose = True

    class ParamDevice():
        def __init__(self):
            self.type = 'v4l'
            self.id = 1
            self.name_by_path = ''
            self.path = ''

    class ParamVideo():
        class ParamVideoSettings():
            def __init__(self):
                self.format = ''
                self.width = 1280
                self.height = 1024
                self.angle = 140
                self.fps = 30.0

        class ParamVideoArea():
            def __init__(self):
                self.mirror = True
                self.start_x = 0
                self.start_y = 0
                self.end_x = 0
                self.end_y = 0
                self.center_x = 0
                self.center_y = 0
                self.width = 0
                self.height = 0

        def __init__(self):
            self.settings = self.ParamVideoSettings()
            self.area = self.ParamVideoArea()

    class ParamVideoSender():
        def __init__(self):
            self.width = 0
            self.height = 0
            self.fps = 10.0

    def __init__(self):
        self.info = self.ParamInfo()
        self.device = self.ParamDevice()
        self.video = self.ParamVideo()
        self.sender = self.ParamVideoSender()

    def print_parameter(self, node):
        node.get_logger().debug('<Parameter>')
        node.get_logger().debug(' device: ')
        node.get_logger().debug('   type   : ' + str(self.device.type))
        node.get_logger().debug('   id     : ' + str(self.device.id))
        node.get_logger().debug('   by_path: ' + str(self.device.name_by_path))
        node.get_logger().debug(' video: ')
        node.get_logger().debug('   format : ' + str(self.video.settings.format))
        node.get_logger().debug('   mirror : {}'.format(self.video.area.mirror))
        node.get_logger().debug('   width  : ' + str(self.video.settings.width))
        node.get_logger().debug('   height : ' + str(self.video.settings.height))
        node.get_logger().debug('   angle  : ' + str(self.video.settings.angle))
        node.get_logger().debug('   fps    : ' + str(self.video.settings.fps))
        node.get_logger().debug(' area: ')
        node.get_logger().debug('   start  : [{}, {}]'.format(self.video.area.start_x, self.video.area.start_y))
        node.get_logger().debug('   end    : [{}, {}]'.format(self.video.area.end_x, self.video.area.end_y))
        node.get_logger().debug(' publisher : ')
        node.get_logger().debug('  width   : ' + str(self.sender.width))
        node.get_logger().debug('  height  : ' + str(self.sender.height))
        node.get_logger().debug('  fps     : ' + str(self.sender.fps))
        node.get_logger().debug(' info : ')
        node.get_logger().debug('   verbose : {}'.format(self.info.verbose))


class VideoCaptureNode(Node):
    ##########################################################################
    _debug_mode = False

    ##########################################################################
    _timer_recognition = None
    _timer_output_information = None
    _timer_output_information_period_fps = 0.2  # 5 seconds

    ##########################################################################
    _lock = None
    _cap = None
    _param = None
    ##########################################################################
    _fps_calc = None
    _timer_output_information = None
    _timer_output_information_period = 5.0  # seconds
    _timer_output_information_size = (5 * 60)
    _display_fps = 0

    ##########################################################################
    _topic_queue_size = 2
    _topic_name = 'image'
    _pub_image = None
    _data_image = None
    _bridge = None
    _param = VideoCaptureNodeParam()

    ##########################################################################
    # finalize
    def closing(self):
        if (self._cap is not None):
            self._cap .release()

    ##########################################################################
    # initialize
    def _develop(self):
        self._timer_output_information_period_fps = 0.2  # seconds

    def __init__(self, node_name):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._init_param()

    def open(self):
        result = True
        if (self._debug_mode is True):
            self._develop()
        try:
            ###################################################################
            # set instance
            self._fps_calc = CvFpsCalc(buffer_len=self._timer_output_information_size)
            self._bridge = CvBridge()

            # set video
            self._param.device.id, self._param.device.name_by_path, self._param.device.path = self._video_search_video(self._param.device.type, self._param.device.id, self._param.device.name_by_path)

            if (-1 != self._param.device.id):
                try:
                    self._cap = cv.VideoCapture(self._param.device.id)
                    if (not self._cap .isOpened()):
                        self._param.device.id = -1
                    else:
                        self._video_set_cv_setting(self._param.video.settings.format,
                                                   self._param.video.settings.width, self._param.video.settings.height,
                                                   self._param.video.settings.fps)

                        self._cap.grab()
                        ret, image = self._cap.read()
                        if (ret is False):
                            self._param.device.id = -1
                        else:
                            self._resizing(image)
                except Exception as exception:
                    self.get_logger().error('Not open Camera : /dev/video' + str(self._param.device.id) + ' : ' + str(exception))
                    self._param.device.id = -1
                except:
                    self.get_logger().error('Not open Camera : /dev/video' + str(self._param.device.id))
                    self._param.device.id = -1

            # check video device
            if (0 > self._param.device.id):
                self.get_logger().error('Not found Camera')
                result = False
            else:
                self._update_device_info()
                self.get_logger().info('Open Camera : /dev/video' + str(self._param.device.id)
                                       + ' : ' + str(self._param.device.name_by_path))
                # create ros
                self._create_publisher()
                self._create_timer()

        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()
        return result

    ##########################################################################
    # ROS
    def _resizing(self, image):
        with self._lock:
            # dst = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            dst = image[self._param.video.area.start_y:self._param.video.area.end_y,
                        self._param.video.area.start_x:self._param.video.area.end_x]
            if (self._param.video.area.mirror is True):
                dst = cv.flip(dst, 1)
            else:
                dst = cv.flip(dst, 0)
            dst = self._resize(dst, self._param.sender.width, self._param.sender.height)
        return dst

    def _set_image(self, image):
        self._data_image = self._bridge.cv2_to_imgmsg(np.array(image), "bgr8")

    def _send_image(self):
        if (self._data_image is not None):
            self._pub_image.publish(self._data_image)

    def _create_publisher(self):
        self._pub_image = self.create_publisher(Image, self._topic_name, self._topic_queue_size)

    def _create_timer(self):
        # set ros callback
        self._timer_recognition = self.create_timer(
            (1.0 / self._param.sender.fps), self._callback_recognition)
        self._timer_output_information = self.create_timer(
            (1.0 / self._timer_output_information_period_fps), self._callback_output_information)

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

    def _callback_recognition(self):
        try:
            #####################################################
            # Get image
            ret, sc = self._cap.read()
            #####################################################
            if (ret is True):
                image = self._resizing(sc)
                self._set_image(image)
                self._send_image()
                #####################################################
                self._display_fps = self._fps_calc.get()
                #####################################################

        except Exception as exception:
            self.get_logger().error('Exception (_callback_recognition) : ' + str(exception))
            traceback.print_exc()

    def _callback_output_information(self):
        if (self._param.info.verbose is True):
            # Output information
            self.get_logger().info('[{}/{}] FPS : {:.4g}'.format(self.get_namespace(), self.get_name(), self._display_fps))

    ##########################################################################
    # ROS Parameter
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

        self._param.video.area.start_x = start_x
        self._param.video.area.start_y = start_y
        self._param.video.area.end_x = end_x
        self._param.video.area.end_y = end_y

    def _update_device_info(self):
        self._param.video.settings.format = self._video_to_txt_cv_fourcc(self._cap.get(cv.CAP_PROP_FOURCC))
        self._param.video.settings.width = int(self._cap.get(cv.CAP_PROP_FRAME_WIDTH))
        self._param.video.settings.height = int(self._cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        self._param.video.settings.fps = float(self._cap.get(cv.CAP_PROP_FPS))

        param_type = Parameter('configuration/device/type', Parameter.Type.STRING, self._param.device.type)
        param_id = Parameter('configuration/device/id', Parameter.Type.INTEGER, self._param.device.id)
        param_name_by_path = Parameter('configuration/device/by_path', Parameter.Type.STRING, self._param.device.name_by_path)

        param_format = Parameter('configuration/video/settings/format', Parameter.Type.STRING, self._param.video.settings.format)
        param_device_width = Parameter('configuration/video/settings/width', Parameter.Type.INTEGER, self._param.video.settings.width)
        param_device_height = Parameter('configuration/video/settings/height', Parameter.Type.INTEGER, self._param.video.settings.height)
        param_device_fps = Parameter('configuration/video/settings/fps', Parameter.Type.DOUBLE, float(self._param.video.settings.fps))

        new_parameters = [param_type, param_id, param_name_by_path,
                          param_format, param_device_width, param_device_height, param_device_fps]

        self.set_parameters(new_parameters)

    def _callback_on_params(self, parameter_list):
        result = False
        with self._lock:
            for parameter in parameter_list:
                self.get_logger().info('[{}/{}] Got {}={}'.format(
                    self.get_namespace(), self.get_name(), parameter.name, parameter.value))

                if (parameter.name == 'preference/info/verbose'):
                    self._param.info.verbose = parameter.value
                    result = True
                if (parameter.name == 'preference/publisher/resize/width'):
                    self._param.sender.width = parameter.value
                    result = True
                if (parameter.name == 'preference/publisher/resize/height'):
                    self._param.sender.height = parameter.value
                    result = True
                if (parameter.name == 'preference/video/area/mirror'):
                    self._param.video.area.mirror = parameter.value
                    result = True
                if (parameter.name == 'preference/video/area/center_x'):
                    self._param.video.area.center_x = parameter.value
                    result = True
                if (parameter.name == 'preference/video/area/center_y'):
                    self._param.video.area.center_y = parameter.value
                    result = True
                if (parameter.name == 'preference/video/area/width'):
                    self._param.video.area.width = parameter.value
                    result = True
                if (parameter.name == 'preference/video/area/height'):
                    self._param.video.area.height = parameter.value
                    result = True
                if (parameter.name == 'configuration/device/id'):
                    if (parameter.value == self._param.device.id):
                        result = True
                if (parameter.name == 'configuration/device/by_path'):
                    if (parameter.value == self._param.device.name_by_path):
                        result = True
                if (parameter.name == 'configuration/device/type'):
                    if (parameter.value == self._param.device.type):
                        result = True
                if (parameter.name == 'configuration/video/settings/format'):
                    if (parameter.value == self._param.video.settings.format):
                        result = True
                if (parameter.name == 'configuration/video/settings/width'):
                    if (parameter.value == self._param.video.settings.width):
                        result = True
                if (parameter.name == 'configuration/video/settings/height'):
                    if (parameter.value == self._param.video.settings.height):
                        result = True
                if (parameter.name == 'configuration/video/settings/angle'):
                    if (parameter.value == self._param.video.settings.angle):
                        result = True
                if (parameter.name == 'configuration/video/settings/fps'):
                    if (parameter.value == self._param.video.settings.fps):
                        result = True
            if (result is True):
                self._calc_area(self._param.video.settings.width, self._param.video.settings.height,
                                self._param.video.area.center_x, self._param.video.area.center_y,
                                self._param.video.area.width, self._param.video.area.height)
                self._param.print_parameter(self)
                self.get_logger().info('<Parameter Update> : {}/{}'.format(self.get_namespace(), self.get_name()))
            else:
                self.get_logger().warn('<Parameter NOT Update> : {}/{}'.format(self.get_namespace(), self.get_name()))
        return SetParametersResult(successful=result)

    def _get_parameter(self):
        with self._lock:
            self._param.info.verbose = self.get_parameter('preference/info/verbose').get_parameter_value().bool_value
            self._param.sender.width = self.get_parameter('preference/publisher/resize/width').get_parameter_value().integer_value
            self._param.sender.height = self.get_parameter('preference/publisher/resize/height').get_parameter_value().integer_value
            self._param.sender.fps = self.get_parameter('configuration/publisher/interval_fps').get_parameter_value().double_value
            self._param.video.settings.format = self.get_parameter('configuration/video/settings/format').get_parameter_value().string_value
            self._param.video.settings.width = self.get_parameter('configuration/video/settings/width').get_parameter_value().integer_value
            self._param.video.settings.height = self.get_parameter('configuration/video/settings/height').get_parameter_value().integer_value
            self._param.video.settings.angle = self.get_parameter('configuration/video/settings/angle').get_parameter_value().integer_value
            self._param.video.settings.fps = self.get_parameter('configuration/video/settings/fps').get_parameter_value().double_value
            self._param.video.area.mirror = self.get_parameter('preference/video/area/mirror').get_parameter_value().bool_value
            self._param.video.area.center_x = self.get_parameter('preference/video/area/center_x').get_parameter_value().integer_value
            self._param.video.area.center_y = self.get_parameter('preference/video/area/center_y').get_parameter_value().integer_value
            self._param.video.area.width = self.get_parameter('preference/video/area/width').get_parameter_value().integer_value
            self._param.video.area.height = self.get_parameter('preference/video/area/height').get_parameter_value().integer_value
            self._calc_area(self._param.video.settings.width, self._param.video.settings.height,
                            self._param.video.area.center_x, self._param.video.area.center_y,
                            self._param.video.area.width, self._param.video.area.height)
        self._param.print_parameter(self)

    def _init_param(self):
        # info
        self.declare_parameter('preference/info/verbose', self._param.info.verbose)
        # device
        self.declare_parameter('configuration/device/type', self._param.device.type)
        self.declare_parameter('configuration/device/id', self._param.device.id)
        self.declare_parameter('configuration/device/by_path', self._param.device.name_by_path)
        # publisher
        self.declare_parameter('preference/publisher/resize/width', self._param.sender.width)
        self.declare_parameter('preference/publisher/resize/height', self._param.sender.height)
        self.declare_parameter('configuration/publisher/interval_fps', self._param.sender.fps)
        # video
        self.declare_parameter('configuration/video/settings/format', self._param.video.settings.format)
        self.declare_parameter('configuration/video/settings/width', self._param.video.settings.width)
        self.declare_parameter('configuration/video/settings/height', self._param.video.settings.height)
        self.declare_parameter('configuration/video/settings/angle', self._param.video.settings.angle)
        self.declare_parameter('configuration/video/settings/fps', self._param.video.settings.fps)

        width = (self._param.video.area.end_x - self._param.video.area.start_x)
        height = (self._param.video.area.end_y - self._param.video.area.end_x)
        center_x = self._param.video.area.start_x - ((self._param.video.settings.width / 2) - (width / 2))
        center_y = self._param.video.area.start_y - ((self._param.video.settings.height / 2) - (height / 2))
        self.declare_parameter('preference/video/area/center_x', int(center_x))
        self.declare_parameter('preference/video/area/center_y', int(center_y))
        self.declare_parameter('preference/video/area/width', int(width))
        self.declare_parameter('preference/video/area/height', int(height))
        self.declare_parameter('preference/video/area/mirror', self._param.video.area.mirror)
        # init parameter
        self._param.device.type = self.get_parameter('configuration/device/type').get_parameter_value().string_value
        self._param.device.name_by_path = self.get_parameter('configuration/device/by_path').get_parameter_value().string_value
        self._param.device.id = self.get_parameter('configuration/device/id').get_parameter_value().integer_value
        # set callback
        self._get_parameter()
        self.add_on_set_parameters_callback(self._callback_on_params)

    ##########################################################################
    # Video
    def _video_set_cv_setting(self, format, width, height, fps=30):
        if (format == 'MJPG'):
            self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # .avi
        if (format == 'H264'):
            self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('H', '2', '6', '4'))
        if (format == 'YUYV'):
            self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        if (format == 'BGR3'):
            self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('B', 'G', 'R', '3'))
        if (format == 'MP4V'):
            self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'P', '4', 'V'))  # .mp4
        if (format == 'MP4S'):
            self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'P', '4', 'S'))  # .mp4

        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv.CAP_PROP_FPS, fps)
        self._video_output_video_information()

    def _video_output_video_information(self):
        if (self._cap is not None):
            fourcc = self._video_to_txt_cv_fourcc(self._cap.get(cv.CAP_PROP_FOURCC))
            width = self._cap.get(cv.CAP_PROP_FRAME_WIDTH)
            height = self._cap.get(cv.CAP_PROP_FRAME_HEIGHT)
            fps = self._cap.get(cv.CAP_PROP_FPS)
            mode = self._video_to_txt_cv_mode(self._cap.get(cv.CAP_PROP_MODE))
            format = self._cap.get(cv.CAP_PROP_FORMAT)

            self.get_logger().info("fourcc:{}, fps:{} [{:.4g} ms], width:{}, height:{}, mode:{}, format:{}".format(
                fourcc, fps, (1 / float(fps)), width, height, mode, format))

    def _video_search_video(self, type, id, path):
        result = -1
        usbVideoDevice = UsbVideoDevice(type)
        result = usbVideoDevice.get_id_by_path(path)
        if (-1 == result):
            result = usbVideoDevice.get_id_from_id(id)
        return usbVideoDevice.get_info(result)

    def _video_to_txt_cv_fourcc(self, v):
        v = int(v)
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


def main(args=None):
    rclpy.init(args=args)
    node_name = 'video_capture_node'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = VideoCaptureNode(node_name)

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
