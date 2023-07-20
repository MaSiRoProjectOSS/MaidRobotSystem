#!/usr/bin/env python3.10

import rclpy
import time
import math
import copy
import traceback
import cv2 as cv
from rclpy.node import Node
from utils.usbvideodevice import UsbVideoDevice
from maid_robot_system_py.features.ros.face_recognition_ros import FaceRecognitionRos
from features.mp.poseinformation import PoseInformation
from features.mp.schematicdiagram import SchematicDiagram
from features.mp.imageanalysis import ImageAnalysis


class ModelNode(Node):
    #############################################################################
    OFFSET_ANGLE_Z_LEFT = -40.0
    OFFSET_ANGLE_Y_LEFT = 0.0
    OFFSET_ANGLE_Z_RIGHT = -12.0
    OFFSET_ANGLE_Y_RIGHT = 0.0
    #############################################################################
    target_angle_z_left = OFFSET_ANGLE_Z_LEFT
    target_angle_z_right = OFFSET_ANGLE_Z_RIGHT
    target_angle_y_left = OFFSET_ANGLE_Y_LEFT
    target_angle_y_right = OFFSET_ANGLE_Y_RIGHT
    #############################################################################
    _ros_com = None
    _timer = None
    _timer_period = 0.016  # seconds
    _timer_info = None
    _timer_info_period = 0.5  # seconds
    TIME_TIMER_TRACKING_TIMEOUT = 3.0

    _video_device_left = -1
    _video_device_right = -1
    _cap_left = None
    _cap_right = None
    _tracking_time = None

    #############################################################################
    upper_body_only = True
    use_brect = True

    #############################################################################

    def _reset_video_id(self, video_type, id, by_path):
        result = -1
        usbVideoDevice = UsbVideoDevice(video_type)
        if -1 == id:
            result = usbVideoDevice.get_id_from_name(by_path)
        else:
            result = usbVideoDevice.get_id_from_id(id)
        return result

    def _debug(self):
        self._timer_period = 2
        self._timer_info_period = 2

    def __init__(self, node_name):
        super().__init__(node_name)
        self._debug()

    def fin(self):
        if -1 != self._video_device_left:
            self._cap_left .release()
        if -1 != self._video_device_right:
            self._cap_right.release()

    def init(self):
        result = True
        try:
            #############################################################################
            # set instance
            self._ros_com = FaceRecognitionRos(self)
            self._ia = ImageAnalysis(self._ros_com.param_confidence_min_detection, self._ros_com.param_confidence_min_tracking)

            # set video

            self._video_device_left = self._reset_video_id(self._ros_com.param_device_left_type, self._ros_com.param_device_left_id, self._ros_com.param_device_left_by_path)
            self._video_device_right = self._reset_video_id(self._ros_com.param_device_right_type, self._ros_com.param_device_right_id, self._ros_com.param_device_right_by_path)
            if -1 != self._video_device_left:
                self._cap_left = cv.VideoCapture(self._video_device_left)
                self._cap_left.set(cv.CAP_PROP_FRAME_WIDTH, self._ros_com.param_device_left_width)
                self._cap_left.set(cv.CAP_PROP_FRAME_HEIGHT, self._ros_com.param_device_left_height)
            if -1 != self._video_device_right:
                self._cap_right = cv.VideoCapture(self._video_device_right)
                self._cap_right.set(cv.CAP_PROP_FRAME_WIDTH, self._ros_com.param_device_right_width)
                self._cap_right.set(cv.CAP_PROP_FRAME_HEIGHT, self._ros_com.param_device_right_height)
            init_ros_func = True
            if 0 > self._video_device_left:
                if 0 > self._video_device_right:
                    init_ros_func = False
                    rclpy.shutdown()

            if [True == init_ros_func]:
                # self._ia.load_model()

                self._timer = self.create_timer(self._timer_period, self._callback_calculate)

                self._tracking_time = time.time() + self. TIME_TIMER_TRACKING_TIMEOUT
                self._timer_info = self.create_timer(self._timer_info_period, self._callback_info)

        except Exception as exception:
            result = False
            traceback.print_exc()
        return result

    _running_cap_id = -1

    def _callback_calculate(self):
        try:
            self._ros_com.get_parameter(self)
            if (0 == self._running_cap_id):
                self._running_cap_id = 1
            else:
                self._running_cap_id = 0
            if 0 > self._video_device_left:
                if 0 > self._video_device_right:
                    rclpy.shutdown()
            if (0 == self._running_cap_id):
                ret, image = self._cap_left.read()
            if (1 == self._running_cap_id):
                ret, image = self._cap_right.read()
            if True == ret:
                image = cv.flip(image, 1)
                debug_image = copy.deepcopy(image)
                # Get image #############################################################
                image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
                # read image
                frame = image
                frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)

                # Detect AR
                # self._ia.detect_ar(gray)
                # aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
                # parameters = aruco.DetectorParameters_create()
                # corners, ids, rejectedImgPoints = aruco.detectMarkers(                    gray, aruco_dict, parameters=parameters)
                # print("AR : " +ids)

                # holistic
                image.flags.writeable = False
                if (0 == self._running_cap_id):
                    results = self._ia.holistic_left.process(image)
                if (1 == self._running_cap_id):
                    results = self._ia.holistic_right.process(image)
                image.flags.writeable = True

                # Pose ###############################################################
                pose_landmarks = results.pose_landmarks
                human_up_hand_flag = 0
                human_confront_flag = 0
                flag_human_clear = 0

                if pose_landmarks is not None:
                    flag_human_clear = 1

                    # 外接矩形の計算
                    brect = SchematicDiagram.calc_bounding_rect(debug_image, pose_landmarks)
                    # 描画
                    Get_person_pos = [0, 0, 0, 0]
                    get_person_data = PoseInformation()

                    Get_eye_pos_x = [0, 0]
                    debug_image = SchematicDiagram.draw_pose_landmarks(
                        debug_image, pose_landmarks, self. upper_body_only, get_person_data, Get_eye_pos_x)

                    target_position = PoseInformation.Axis3()

                    # 見る人の部位を決める 手を上げられたら手を見る
                    see_nose_flag = 1

                    person_vertical_flag = 0
                    # 人間の直立判定  逆転判定したら膝枕
                    if (get_person_data.nose.y < get_person_data.shoulder_left.y and get_person_data.nose.y < get_person_data.shoulder_right.y):
                        person_vertical_flag = 1

                    # eye_cmd_angle.linear.x = 0
                    if (person_vertical_flag == 1 and get_person_data.hand_left.get_flag == 1 and get_person_data.shoulder_left.get_flag == 1 and get_person_data.hand_left.y < get_person_data.shoulder_left.y):
                        target_position = get_person_data.hand_left
                        # eye_cmd_angle.linear.x = 1   # eye smile
                        see_nose_flag = 0
                    if (person_vertical_flag == 1 and get_person_data.hand_right.get_flag == 1 and get_person_data.shoulder_right.get_flag == 1 and get_person_data.hand_right.y < get_person_data.shoulder_right.y):
                        target_position = get_person_data.hand_right
                        # eye_cmd_angle.linear.x = 6     #eye close
                        see_nose_flag = 0
                    if (see_nose_flag == 1):
                        target_position = get_person_data.nose

                    Get_face_x = target_position.x
                    Get_face_y = target_position.y
                    Get_face_z = target_position.z

                    # 人が正対するとき 右目ｘ座標は左目ｘ座標より大きい 人が後ろ向きでも目の位置は推定される
                    if (person_vertical_flag == 1):
                        if (Get_eye_pos_x[0] != 0 and Get_eye_pos_x[1] != 0 and Get_eye_pos_x[0] > Get_eye_pos_x[1]):
                            human_confront_flag = 1
                        else:
                            human_confront_flag = 0

                    # ロール角度を計算
                    target_roll = 0
                    if 1 == human_confront_flag:
                        if (get_person_data.R_eye.get_flag == 1 and get_person_data.L_eye.get_flag == 1):
                            if (human_confront_flag == 1 and person_vertical_flag == 1):
                                eye_dx = get_person_data.R_eye.x - get_person_data.L_eye.x
                                eye_dy = get_person_data.R_eye.y - get_person_data.L_eye.y

                                target_roll = (math.degrees(math.atan2(eye_dy, eye_dx)))

                    To_face_yaw = ((Get_face_x / self._ros_com .CAMERA_PIXEL_X) - 0.5) * self._ros_com .CAMERA_ANGLE_X
                    To_face_pitch = ((Get_face_y / self._ros_com .CAMERA_PIXEL_Y) - 0.5) * self._ros_com . CAMERA_ANGLE_Y

                    self._ros_com.set_angle(To_face_yaw, To_face_pitch, target_roll, self._ia.flag_human_confront, self._ia.flag_person_vertical)

                    if (0 == self._running_cap_id):
                        target_angle_z_R = To_face_yaw + self. OFFSET_ANGLE_Z_LEFT
                        target_angle_y_R = To_face_pitch + self. OFFSET_ANGLE_Y_LEFT
                    if (1 == self._running_cap_id):
                        target_angle_z_L = To_face_yaw + self.OFFSET_ANGLE_Z_RIGHT
                        target_angle_y_L = To_face_pitch + self. OFFSET_ANGLE_Y_RIGHT
                    target_angle_y = (target_angle_y_L + target_angle_y_R)/2.0
                    target_angle_z = (target_angle_z_L + target_angle_z_R)/2.0

                    self._ros_com.set_eye_angle((target_angle_y_L + target_angle_y_R)/2.0, (target_angle_z_L + target_angle_z_R)/2.0)

                    if (self._ia.flag_human_confront == 1):
                        self._ros_com.set_neck_confronted(target_angle_y, target_angle_z, target_roll)
                    else:
                        flag_human_clear = 1

                    debug_image = SchematicDiagram.draw_bounding_rect(self.use_brect, debug_image, brect)

                else:  # 人物が認識されなかったら
                    flag_human_clear = 1

                if (0 == self._running_cap_id):
                    self._ros_com.set_image_left(debug_image)
                if (1 == self._running_cap_id):
                    self._ros_com.set_image_right(debug_image)

                if (1 == flag_human_clear):
                    if (time.time() > self._tracking_time):
                        self._tracking_time = time.time() + self._ros_com.param_confidence_tracking_timeout
                        self._ros_com.set_pose_clear()
            self._ros_com.send()

        except Exception as exception:
            traceback.print_exc()

    def _callback_info(self):
        self._ros_com.send_info()
        self._ros_com.get_parameter_update(self)


def main(args=None):
    rclpy.init(args=args)
    node_name = 'face_recognition_node'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = ModelNode(node_name)

    try:
        if (True == node.init()):
            rclpy.spin(node)

    except Exception as exception:
        traceback_logger.error(traceback.format_exc())
    finally:
        node.fin()
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
