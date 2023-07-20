#!/usr/bin/env python3.10

import mediapipe as mp
import numpy
import cv2 as cv

from enum import Enum
from features.mp.poseinformation import PoseInformation


class PoseLandmarkNum(Enum):
    NOSE = 0
    LEFT_EYE_INNER = 1
    LEFT_EYE = 2
    LEFT_EYE_OUTER = 3
    RIGHT_EYE_INNER = 4
    RIGHT_EYE = 5
    RIGHT_EYE_OUTER = 6
    LEFT_EAR = 7
    RIGHT_EAR = 8
    MOUTH_LEFT = 9
    MOUTH_RIGHT = 10
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12
    LEFT_ELBOW = 13
    RIGHT_ELBOW = 14
    LEFT_WRIST = 15
    RIGHT_WRIST = 16
    LEFT_PINKY = 17
    RIGHT_PINKY = 18
    LEFT_INDEX = 19
    RIGHT_INDEX = 20
    LEFT_THUMB = 21
    RIGHT_THUMB = 22
    LEFT_HIP = 23
    RIGHT_HIP = 24
    LEFT_KNEE = 25
    RIGHT_KNEE = 26
    LEFT_ANKLE = 27
    RIGHT_ANKLE = 28
    LEFT_HEEL = 29
    RIGHT_HEEL = 30
    LEFT_FOOT_INDEX = 31
    RIGHT_FOOT_INDEX = 32
    MAX = 33


class ImageAnalysis:
    path_model = '/opt/masiro_ros_framework/data/blaze_face_short_range.tflite'
    detector = None
    frame_timestamp_ms = 10
    pose_data = PoseInformation()
    flag_human_confront = 0
    flag_person_vertical = 0
    holistic_left = None
    holistic_right = None

    def __init__(self, min_detection_confidence, min_tracking_confidence):
        self.mp_face_detection = mp.solutions.face_detection
        self.holistic_left = mp.solutions.holistic.Holistic(
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.holistic_right = mp.solutions.holistic.Holistic(
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        # static_image_mode=False,
        # model_complexity=1,
        # smooth_landmarks=True,
        # enable_segmentation=False,
        # smooth_segmentation=True,
        # refine_face_landmarks=False,
        #  min_detection_confidence=0.5,
        # min_tracking_confidence=0.5

    def detect_ar(self, image):
        # dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        # ar_parameters = aruco.DetectorParameters()
        # corners, ids, rejectedImgPoints = aruco.detectMarkers(image, dict_aruco)
        print("detect_ar")
        # au_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        # parameters = aruco.DetectorParameters_create()
        # corners, ids, rejectedImgPoints = aruco.detectMarkers(image, au_dict, parameters=parameters)

    def calc_bounding_rect(self, image, landmarks, pose_data, eye_flag, visibility_th=0.5):
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_array = []
        landmark_point = []

        landmark_array = numpy.empty((0, 2), int)
        for mark, row in enumerate(landmarks.landmark):
            landmark_x = min(int(row.x * image_width), image_width - 1)
            landmark_y = min(int(row.y * image_height), image_height - 1)
            landmark_z = row.z
            # landmark_array = numpy.append(landmark_array, landmark_point, axis=0)
            landmark_point.append([row.visibility, (landmark_x, landmark_y)])
            landmark_point.append([landmark_x, landmark_y])

            if row.visibility < visibility_th:
                continue
            index = PoseLandmarkNum(mark)
            if PoseLandmarkNum.NOSE == index:
                pose_data.nose.set(landmark_x, landmark_y, landmark_z)
                # print("  [NOSE]")
            # elif PoseLandmarkNum.LEFT_EYE_INNER == index:
            elif PoseLandmarkNum.LEFT_EYE == index:
                eye_flag[0] = landmark_x
                pose_data.eye_left.set(landmark_x, landmark_y, landmark_z)
                # print("  [LEFT_EYE]")
            # elif  PoseLandmarkNum.LEFT_EYE_OUTER ==index:
            # elif  PoseLandmarkNum.RIGHT_EYE_INNER ==index:
            elif PoseLandmarkNum.RIGHT_EYE == index:
                eye_flag[1] = landmark_x
                pose_data.eye_right.set(landmark_x, landmark_y, landmark_z)
                # print("  [RIGHT_EYE]")
            # elif  PoseLandmarkNum.RIGHT_EYE_OUTER ==index:
            # elif  PoseLandmarkNum.LEFT_EAR ==index:
            # elif  PoseLandmarkNum.RIGHT_EAR ==index:
            # elif  PoseLandmarkNum.MOUTH_LEFT ==index:
            # elif  PoseLandmarkNum.MOUTH_RIGHT ==index:
            elif PoseLandmarkNum.LEFT_SHOULDER == index:
                pose_data.shoulder_left.set(landmark_x, landmark_y, landmark_z)
                # print("  [LEFT_SHOULDER]")
            elif PoseLandmarkNum.RIGHT_SHOULDER == index:
                pose_data.shoulder_right.set(landmark_x, landmark_y, landmark_z)
                # print("  [RIGHT_SHOULDER]")
            # elif  PoseLandmarkNum.LEFT_ELBOW ==index:
            # elif  PoseLandmarkNum.RIGHT_ELBOW ==index:
            # elif  PoseLandmarkNum.LEFT_WRIST ==index:
            # elif  PoseLandmarkNum.RIGHT_WRIST ==index:
            # elif  PoseLandmarkNum.LEFT_PINKY ==index:
            # elif  PoseLandmarkNum.RIGHT_PINKY ==index:
            elif PoseLandmarkNum.LEFT_INDEX == index:
                pose_data.hand_left.set(landmark_x, landmark_y, landmark_z)
                # print("  [LEFT_INDEX]")
            elif PoseLandmarkNum.RIGHT_INDEX == index:
                pose_data.hand_right.set(landmark_x, landmark_y, landmark_z)
                # print("  [RIGHT_INDEX]")
            # elif  PoseLandmarkNum.LEFT_THUMB ==index:
            # elif  PoseLandmarkNum.RIGHT_THUMB ==index:
            # elif  PoseLandmarkNum.LEFT_HIP ==index:
            # elif  PoseLandmarkNum.RIGHT_HIP ==index:
            # elif  PoseLandmarkNum.LEFT_KNEE ==index:
            # elif  PoseLandmarkNum.RIGHT_KNEE==index:
            # elif  PoseLandmarkNum.LEFT_ANKLE ==index:
            # elif  PoseLandmarkNum.RIGHT_ANKLE ==index:
            # elif  PoseLandmarkNum.LEFT_HEEL==index:
            # elif  PoseLandmarkNum.RIGHT_HEEL ==index:
            # elif  PoseLandmarkNum.LEFT_FOOT_INDEX ==index:
            # elif  PoseLandmarkNum.RIGHT_FOOT_INDEX ==index:
            elif PoseLandmarkNum.MAX == index:
                break

        # x, y, w, h = cv.boundingRect(landmark_array)

        # return [x, y, x + w, y + h]

    def detect_face(self, mode, image):
        image.flags.writeable = False
        if ("left" == mode):
            results = self.holistic_left.process(image)
        else:
            results = self.holistic_right.process(image)
        image.flags.writeable = True
        if results is not None:
            eye_flag = [0, 0]
            if results.pose_landmarks is not None:
                rect = self.calc_bounding_rect(image, results.pose_landmarks, self.pose_data, eye_flag)

            self.pose_data.gaze.copy(self.pose_data.nose)

            self.flag_human_confront = 0
            self.flag_person_vertical = 0
            # Look at your hands when you're on my knees
            if (self.pose_data.nose.y < self.pose_data.shoulder_left.y):
                if (self.pose_data.nose.y < self.pose_data.shoulder_right.y):
                    self.flag_person_vertical = 1
                    if (self.pose_data.hand_left.get_flag == 1):
                        if (self.pose_data.shoulder_left.get_flag == 1):
                            if (self.pose_data.hand_left.y < self.pose_data.shoulder_left.y):
                                self.pose_data.gaze.copy(self.pose_data.hand_left)
                    if (self.pose_data.hand_right.get_flag == 1):
                        if (self.pose_data.shoulder_right.get_flag == 1):
                            if (self.pose_data.hand_right.y < self.pose_data.shoulder_right.y):
                                self.pose_data.gaze.copy(self.pose_data.hand_right)
                    if (eye_flag[0] != 0 and eye_flag[1] != 0 and eye_flag[0] > eye_flag[1]):
                        self.flag_human_confront = 1
            # Look at your hands when you're on my knees
            return 1
        return 0

###############################################################


class ImageAnalysis2:
    path_model = '/opt/masiro_ros_framework/data/blaze_face_short_range.tflite'
    detector = None
    frame_timestamp_ms = 10

    def __init__(self):
        self.mp_face_detection = mp.solutions.face_detection

    def set_data(self, image):
        with self.mp_face_detection.FaceDetection(min_detection_confidence=0.5) as face_detection:
            image.flags.writeable = False
            results = face_detection.process(image)
            # 画像に顔検出アノテーションを描画
            image.flags.writeable = True

            if results.detections:
                print('MediaPipe Face Detection')
