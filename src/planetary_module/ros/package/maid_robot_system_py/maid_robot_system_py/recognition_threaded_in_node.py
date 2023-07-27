#!/usr/bin/env python3.10

import rclpy
import traceback
import copy
import threading
import cv2 as cv
from rclpy.node import Node
from utils.usbvideodevice import UsbVideoDevice
from maid_robot_system_py.features.ros.ros_comm_recognition import RosCommRecognition
from features.mp.schematicdiagram import SchematicDiagram
from features.mp.imageanalysis import ImageAnalysis
from utils.cvfpscalc import CvFpsCalc


class ModelNode(Node):

    _get_image_timer = None
    _get_image_callback_fps = 30.0  # 0.0334 seconds
    _get_image_fps = 0.0
    _get_image_tick = 0.0

    _recognition_enable = True
    _recognition_timer = None
    _recognition_callback_fps = 30.0  # 0.0334 seconds
    _recognition_ar_list = None
    _recognition_fps = 0.0
    _recognition_tick = 0.0
    _report_detected = True
    _previous_human_detected = False

    _output_information_timer = None
    _output_information_callback_fps = 0.2  # 5 seconds

    _drawing_flag_draw_box = True
    _drawing_timer = None
    _drawing_callback_fps = 2.0  # 0.5 seconds

    ##########################################################################
    _fps_calc_get_image = None
    _fps_calc_get_image_size = (5 * 60)

    _fps_calc_recognition = None
    _fps_calc_recognition_size = (5 * 60)

    ##########################################################################
    _lock = None
    _cap = None
    _ros_com = None
    _ia = None
    _sa = None
    _frame = None
    ##########################################################################
    _debug_mode = True

    def _develop(self):
        self.get_logger().info('================= DEBUG MODE =================')
        self._drawing_callback_fps = 0.2  # 5 seconds
        # self._recognition_callback_fps = 1  # 1 seconds
    ##########################################################################

    def _to_txt_cv_fourcc(self, v):
        v = int(v)
        return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

    def _to_txt_cv_mode(self, mode):
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

    def _output_video_information(self):
        if self._cap is not None:
            fourcc = self._to_txt_cv_fourcc(self._cap.get(cv.CAP_PROP_FOURCC))
            width = self._cap.get(cv.CAP_PROP_FRAME_WIDTH)
            height = self._cap.get(cv.CAP_PROP_FRAME_HEIGHT)
            fps = self._cap.get(cv.CAP_PROP_FPS)
            mode = self._to_txt_cv_mode(self._cap.get(cv.CAP_PROP_MODE))
            format = self._cap.get(cv.CAP_PROP_FORMAT)

            self.get_logger().info("fourcc:{}, fps:{}, width:{}, height:{}, mode:{}, format:{}".format(
                fourcc, fps, width, height, mode, format))

    def _set_cv_setting(self, width, height):
        self._output_video_information()
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'P', '4', 'S')) #.mp4
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'P', '4', 'V')) # .mp4
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('D', 'I', 'V', '3')) # .avi
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('D', 'I', 'V', 'X')) # .avi
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('I', 'Y', 'U', 'V')) # .avi
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G')) # .avi
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('X', 'V', 'I', 'D')) # .avi
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('H', '2', '6', '3')) # .wmv
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('m', 'p', '4', 'v')) # .mov
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('B', 'G', 'R', '3'))
        self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('H', '2', '6', '4'))
        # self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv.CAP_PROP_FPS, 30)
        # self._cap.set(cv.CAP_PROP_MODE, 1)

    def _reset_video_id(self):
        path = ""
        result = -1
        usbVideoDevice = UsbVideoDevice(self._ros_com.param_device_type)
        result = usbVideoDevice.get_id_by_path(self._ros_com.param_device_by_path)
        if -1 == result:
            result = usbVideoDevice.get_id_from_id(self._ros_com.param_device_id)
        self._ros_com.param_device_id, self._ros_com.param_device_by_path, path = usbVideoDevice.get_info(result)
        return self._ros_com.param_device_id

    def fin(self):
        if self._cap is not None:
            self._cap .release()

    def __init__(self, node_name):
        super().__init__(node_name)

    def _set_video_image(self, image):
        with self._lock:
            # self._frame = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            self._frame = copy.deepcopy(image[self._ros_com.param_video_area_start_y:self._ros_com.param_video_area_end_y,
                                              self._ros_com.param_video_area_start_x:self._ros_com.param_video_area_end_x])
            self._get_image_tick = cv.getTickCount()
        self._ros_com.set_image(self._frame)

    def init(self):
        result = True
        if (self._debug_mode is True):
            self._develop()
        try:
            ###################################################################
            # set instance
            self._lock = threading.Lock()
            self._ros_com = RosCommRecognition(self)
            self._ia = ImageAnalysis(self._ros_com.param_confidence_min_detection,
                                     self._ros_com.param_confidence_min_tracking)
            self._sa = SchematicDiagram()
            # *********************************************************
            self._fps_calc_get_image = CvFpsCalc(
                buffer_len=self._fps_calc_get_image_size)
            self._fps_calc_recognition = CvFpsCalc(
                buffer_len=self._fps_calc_recognition_size)
            # *********************************************************
            # set video
            video_device_id = self._reset_video_id()
            if -1 != video_device_id:
                try:
                    self._cap = cv.VideoCapture(video_device_id)
                    if (not self._cap .isOpened()):
                        video_device_id = -1
                    else:
                        self._set_cv_setting(self._ros_com.param_video_width, self._ros_com.param_video_height)
                        self._cap.grab()
                        ret, image = self._cap.read()
                        if (ret is False):
                            video_device_id = -1
                        else:
                            self._set_video_image(image)
                except Exception as exception:
                    self.get_logger().error('Not open Camera : /dev/video'
                                            + str(video_device_id)
                                            + ' : '
                                            + str(exception))
                    video_device_id = -1
                except:
                    self.get_logger().error('Not open Camera : /dev/video'
                                            + str(video_device_id)
                                            )
                    video_device_id = -1

            # check video device
            if 0 > video_device_id:
                self.get_logger().error('Not fond Camera')
                result = False
            else:
                self.get_logger().info('Open Camera : /dev/video' + str(video_device_id)
                                       + ' : ' + str(self._ros_com.param_device_by_path))
                if (self._debug_mode is True):
                    self._output_video_information()

                # set los
                self._ros_com.create_publisher(self, video_device_id)

                # set ros callback
                self._get_image_callback()
                self._recognition_callback()
                # set ros callback
                # ## TODO : 実施したい処理を記述 ###
                self._get_image_timer = self.create_timer(
                    1.0 / self._get_image_callback_fps, self._get_image_callback)
                self._recognition_timer = self.create_timer(
                    1.0 / self._recognition_callback_fps, self._recognition_callback)

                self._drawing_timer = self.create_timer(
                    1.0 / self._drawing_callback_fps, self._drawing_callback)
                self._output_information_timer = self.create_timer(
                    1.0 / self._output_information_callback_fps, self._output_information_callback)

        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()
        return result

    def _get_image_callback(self):
        try:
            #####################################################
            self._ros_com.get_parameter(self, False)
            #####################################################
            # Get image
            ret, image = self._cap.read()
            if (ret is True):
                self._set_video_image(image)
                #####################################################
                self._get_image_fps = self._fps_calc_get_image.get()
                #####################################################
        except Exception as exception:
            self.get_logger().error('Exception (_get_image_callback) : ' + str(exception))
            traceback.print_exc()

    def _recognition_callback(self):
        try:
            if (self._recognition_enable is True):
                if (self._get_image_tick is not self._recognition_tick):
                    self._recognition_tick = self._get_image_tick
                    with self._lock:
                        image = copy.deepcopy(self._frame)
                    if (image is not None):
                        gray = None if (self._ros_com.features_detect_markers is False) else cv.cvtColor(image, cv.COLOR_RGB2GRAY)

                        ###################################################################
                        self._ros_com.pose_detection.human_detected = False
                        ###################################################################

                        #####################################################
                        pose_landmarks = self._ia.detect_holistic(image)
                        if pose_landmarks is not None:
                            self._ros_com.repackaging(pose_landmarks)
                        #####################################################
                        if (gray is not None):
                            self._recognition_ar_list = self._ia.detect_ar(gray)
                        #####################################################
                        if (self._report_detected is True):
                            if (self._ros_com.pose_detection.human_detected is not self._previous_human_detected):
                                if (self._ros_com.pose_detection.human_detected is True):
                                    self.get_logger().info(
                                        '[{}] HUMAN_DETECTED'.format(str(self._ros_com.param_topic_sub_name)))
                                else:
                                    self.get_logger().info(
                                        '[{}] LOSE_TRACKING'.format(str(self._ros_com.param_topic_sub_name)))
                                self._previous_human_detected = self._ros_com.pose_detection.human_detected
                        #####################################################
                        self._ros_com.send_recognition()
                        #####################################################
                        self._recognition_fps = self._fps_calc_recognition.get()
        except Exception as exception:
            self.get_logger().error('Exception (_recognition_callback) : ' + str(exception))
            traceback.print_exc()

    def _drawing_callback(self):
        try:
            if (self._ros_com.param_image_publish is True):
                if (self._frame is not None):
                    with self._lock:
                        image = copy.deepcopy(self._frame)
                    if (image is not None):
                        if (self._ros_com.param_image_overlay_information is True):
                            schematic_diagram = self._sa.drawing(image,
                                                                 float(self._ros_com.param_video_area_end_x - self._ros_com.param_video_area_start_x),
                                                                 float(self._ros_com.param_video_area_end_y - self._ros_com.param_video_area_start_y),
                                                                 self._ros_com.pose_detection.landmark,
                                                                 self._ros_com.pose_detection.detected_area,
                                                                 self._ros_com.pose_detection.human_detected,
                                                                 self._drawing_flag_draw_box)

                            self._ros_com.set_image_overlay(schematic_diagram)
                        else:
                            self._ros_com.set_image(image)
                    # ##########################################################
                    # send image
                    self._ros_com.send_image()
                    # ##########################################################
        except Exception as exception:
            self.get_logger().error('Exception (_drawing_callback) : ' + str(exception))
            traceback.print_exc()
        finally:
            self._ros_com.get_parameter_update(self)

    def _output_information_callback(self):
        try:
            if (self._ros_com.param_info_verbose is True):
                text_detect_markers = 'DISABLED' if (self._ros_com.features_detect_markers is False) else str(self._recognition_ar_list)
                # Output information
                self.get_logger().info('[{}] FPS [image] : {:.4g}, FPS [holistic] : {:.4g}, {}, AR : {}'.format(
                    str(self._ros_com.param_topic_sub_name),
                    self._get_image_fps,
                    self._recognition_fps,
                    "HUMAN_DETECTED" if (self._ros_com.pose_detection.human_detected is True) else "--",
                    text_detect_markers
                ))
        except Exception as exception:
            self.get_logger().error('Exception (_output_information_callback) : ' + str(exception))
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node_name = 'recognition_threaded_in_node'
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
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
