#!/usr/bin/env python3.10

import rclpy
import traceback
import cv2 as cv
from rclpy.node import Node
from utils.usbvideodevice import UsbVideoDevice
from maid_robot_system_py.features.ros.face_recognition_ros import FaceRecognitionRos
from features.mp.schematicdiagram import SchematicDiagram
from features.mp.imageanalysis import ImageAnalysis
from utils.cvfpscalc import CvFpsCalc
from enum import Enum
import copy
# import threading


class ModelNode(Node):
    _debug_mode = False

    ##########################################################################
    _ros_com = None
    _timer_recognition = None
    _timer_recognition_period = 0.0334  # seconds / FPS : 30
    _timer_drawing = None
    _timer_drawing_period = 0.5  # seconds / FPS : 2

    ##########################################################################
    _fps_calc = None
    _timer_output_information = None
    _timer_output_information_period = 5.0  # seconds
    _timer_output_information_size = (5 * 60)
    _display_fps = 0
    ##########################################################################
    _video_device_id = -1
    _cap = None
    ##########################################################################
    _draw_box = True
    _frame = None
    ##########################################################################
    _ar_list = None
    ##########################################################################

    def _develop(self):
        self._timer_recognition_period = 1  # seconds
        self._timer_drawing_period = 5  # seconds

    def _reset_video_id(self):
        path = ""
        result = -1
        usbVideoDevice = UsbVideoDevice(self._ros_com.param_device_type)
        result = usbVideoDevice.get_id_by_path(
            self._ros_com.param_device_by_path)
        if -1 == result:
            result = usbVideoDevice.get_id_from_id(
                self._ros_com.param_device_id)
        self._ros_com.param_device_id, self._ros_com.param_device_by_path, path = usbVideoDevice.get_info(
            result)
        return self._ros_com.param_device_id

    def __init__(self, node_name):
        super().__init__(node_name)

    def fin(self):
        if self._cap is not None:
            self._cap .release()

    def init(self):
        result = True
        if (self._debug_mode is True):
            self._develop()
        try:
            ###################################################################
            # set instance
            self._ros_com = FaceRecognitionRos(self)
            self._ia = ImageAnalysis(self._ros_com.param_confidence_min_detection,
                                     self._ros_com.param_confidence_min_tracking)
            self._sa = SchematicDiagram()
            self._fps_calc = CvFpsCalc(
                buffer_len=self._timer_output_information_size)

            # set video
            self._video_device_id = self._reset_video_id()
            if -1 != self._video_device_id:
                try:
                    self._cap = cv.VideoCapture(self._video_device_id)
                    self._cap.set(cv.CAP_PROP_FRAME_WIDTH,
                                  self._ros_com.param_video_width)
                    self._cap.set(
                        cv.CAP_PROP_FRAME_HEIGHT, self._ros_com.param_video_height)
                    ret, image = self._cap.read()
                    if (ret is False):
                        self._video_device_id = -1
                    else:
                        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
                        self._ros_com.set_image(image)
                except Exception as exception:
                    self.get_logger().error('Not open Camera : /dev/video'
                                            + str(self._video_device_id)
                                            + " "
                                            + exception.message)
                    self._video_device_id = -1

            # check video device
            if 0 > self._video_device_id:
                rclpy.shutdown()
            else:
                self.get_logger().info('Open Camera : /dev/video' + str(self._video_device_id)
                                       + ' : ' + str(self._ros_com.param_device_by_path))
                # init lock
                # self._lock = threading.Lock()
                # set los
                self._ros_com.create_publisher(self, self._video_device_id)
                # set ros callback
                self._callback_recognition()
                self._timer_recognition = self.create_timer(
                    self._timer_recognition_period, self._callback_recognition)
                self._timer_drawing = self.create_timer(
                    self._timer_drawing_period, self._callback_drawing)
                # set fps_calc
                self._timer_output_information = self.create_timer(
                    self._timer_output_information_period, self._callback_output_information)

        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()
        return result

    def _callback_recognition(self):
        try:
            #####################################################
            self._ros_com.get_parameter(self, False)
            #####################################################
            self._ros_com.person_data.human_detected = False
            #####################################################
            # Get image
            ret, image = self._cap.read()
            #####################################################
            if (ret is True):
                image = cv.flip(image, 1)
                image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
                if (self._ros_com.param_image_overlay_information is True):
                    self._frame = copy.deepcopy(image)
                #####################################################
                self._ros_com.set_image(image)
                pose_landmarks = self._ia.detect_holistic(image)
                if pose_landmarks is not None:
                    self._ros_com.repackaging(pose_landmarks)
                #####################################################
                if (self._ros_com.features_detect_markers is True):
                    gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
                    self._ar_list = self._ia.detect_ar(self, gray)
                #####################################################

                if (self._debug_mode is True):
                    if (self._ros_com.person_data.human_detected is True):
                        self.get_logger().info(
                            ' HUMAN_DETECTED [{}]'.format(str(self._ros_com.param_topic_sub_name)))
                #####################################################
            #####################################################
            self._display_fps = self._fps_calc.get()
            #####################################################

        except Exception as exception:
            self.get_logger().error('Exception (_callback_recognition) : ' + str(exception))
            traceback.print_exc()
        finally:
            self._ros_com.send_recognition()

    def _callback_output_information(self):
        if (self._ros_com.param_info_verbose is True):
            text_detect_markers = str(self._ar_list) if (
                self._ros_com.features_detect_markers is True) else 'DISABLED'
            # Output information
            self.get_logger().info('[{}] FPS : {}, AR : {}'.format(
                str(self._ros_com.param_topic_sub_name),
                str(self._display_fps),
                text_detect_markers
            ))

    def _callback_drawing(self):
        try:
            if (self._ros_com.param_image_publish is True):
                if (self._ros_com.param_image_overlay_information is True):
                    if (self._frame is not None):
                        schematic_diagram = self._sa.drawing(copy.deepcopy(self._frame),
                                                             self._ros_com.param_video_width,
                                                             self._ros_com.param_video_height,
                                                             self._ros_com.person_data,
                                                             self._draw_box
                                                             )
                        self._ros_com.set_image_overlay(schematic_diagram)
                # ##########################################################
                # send image
                self._ros_com.send_image()
                # ##########################################################
        except Exception as exception:
            self.get_logger().error('Exception (_callback_drawing) : ' + str(exception))
            traceback.print_exc()
        finally:
            self._ros_com.get_parameter_update(self)


def main(args=None):
    rclpy.init(args=args)
    node_name = 'face_recognition_in_node'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = ModelNode(node_name)

    try:
        if (node.init() is True):
            rclpy.spin(node)

    except Exception as exception:
        print(exception)
        traceback.print_exc()
        traceback_logger.error(traceback.format_exc())
    finally:
        node.fin()
        node.destroy_node()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
