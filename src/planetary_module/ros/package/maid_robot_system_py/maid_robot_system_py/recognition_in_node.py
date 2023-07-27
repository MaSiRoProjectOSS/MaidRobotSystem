#!/usr/bin/env python3.10

import rclpy
import traceback
import copy
import cv2 as cv
from rclpy.node import Node
from utils.cv_fps_calc import UsbVideoDevice
from features.ros.ros_comm_recognition import RosCommRecognition
from features.mp.schematic_diagram import SchematicDiagram
from features.mp.image_analysis import ImageAnalysis
from utils.cv_fps_calc import CvFpsCalc


class ModelNode(Node):
    _debug_mode = False
    _enable_holistic = True
    ##########################################################################
    _report_detected = True
    _previous_human_detected = False

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
    _flag_draw_box = True
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

    def init(self):
        result = True
        if (self._debug_mode is True):
            self._develop()
        try:
            ###################################################################
            # set instance
            self._ros_com = RosCommRecognition(self)
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
                    if (not self._cap .isOpened()):
                        self._video_device_id = -1
                    else:
                        self._set_cv_setting(self._ros_com.param_video_width, self._ros_com.param_video_height)

                        self._cap.grab()
                        ret, image = self._cap.read()
                        if (ret is False):
                            self._video_device_id = -1
                        else:
                            # image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
                            self._ros_com.set_image(image)
                except Exception as exception:
                    self.get_logger().error('Not open Camera : /dev/video'
                                            + str(self._video_device_id)
                                            + ' : '
                                            + str(exception))
                    self._video_device_id = -1
                except:
                    self.get_logger().error('Not open Camera : /dev/video'
                                            + str(self._video_device_id)
                                            )
                    self._video_device_id = -1

            # check video device
            if 0 > self._video_device_id:
                self.get_logger().error('Not fond Camera')
                result = False
            else:
                self.get_logger().info('Open Camera : /dev/video' + str(self._video_device_id)
                                       + ' : ' + str(self._ros_com.param_device_by_path))
                if (self._debug_mode is True):
                    self._output_video_information()
                self._output_video_information()
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
            self._ros_com.pose_detection.human_detected = False
            #####################################################
            # Get image
            ret, sc = self._cap.read()
            #####################################################
            if (ret is True):
                gray = None if (self._ros_com.features_detect_markers is False) else cv.cvtColor(sc, cv.COLOR_RGB2GRAY)
                #####################################################
                # image = sc.cvtColor(image, cv.COLOR_BGR2RGB)
                image = sc[self._ros_com.param_video_area_start_y:self._ros_com.param_video_area_end_y,
                           self._ros_com.param_video_area_start_x:self._ros_com.param_video_area_end_x]
                image = cv.flip(image, 1)
                if (self._ros_com.param_image_publish is True):
                    self._frame = copy.deepcopy(image)
                #####################################################
                if (self._enable_holistic is True):
                    pose_landmarks = self._ia.detect_holistic(image)
                    if pose_landmarks is not None:
                        self._ros_com.repackaging(pose_landmarks)
                #####################################################
                if (gray is not None):
                    self._ar_list = self._ia.detect_ar(gray)
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
            self.get_logger().info('[{}] FPS : {:.4g}, {}, AR : {}'.format(
                str(self._ros_com.param_topic_sub_name),
                self._display_fps,
                "HUMAN_DETECTED" if (self._ros_com.pose_detection.human_detected is True) else "--",
                text_detect_markers
            ))

    def _callback_drawing(self):
        try:
            if (self._ros_com.param_image_publish is True):
                if (self._frame is not None):
                    if (self._ros_com.param_image_overlay_information is True):
                        schematic_diagram = self._sa.drawing(copy.deepcopy(self._frame),
                                                             float(self._ros_com.param_video_area_end_x - self._ros_com.param_video_area_start_x),
                                                             float(self._ros_com.param_video_area_end_y - self._ros_com.param_video_area_start_y),
                                                             self._ros_com.pose_detection.landmark,
                                                             self._ros_com.pose_detection.detected_area,
                                                             self._ros_com.pose_detection.human_detected,
                                                             self._flag_draw_box)

                        self._ros_com.set_image_overlay(schematic_diagram)
                    else:
                        self._ros_com.set_image(copy.deepcopy(self._frame))
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
    node_name = 'recognition_in_node'
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
