#!/usr/bin/env python3.10

from mediapipe.python.solutions.holistic import Holistic as mp_holistic
import cv2 as cv
import cv2.aruco as aruco


class ImageAnalysis:
    _holistic = None

    def __init__(self, confidence_min_detection: float, confidence_min_tracking: float):
        self._load_model(confidence_min_detection, confidence_min_tracking)

    def _load_model(self, confidence_min_detection: float, confidence_min_tracking: float):
        self._holistic = mp_holistic(
            min_detection_confidence=confidence_min_detection,
            min_tracking_confidence=confidence_min_tracking,
        )

    def detect_ar(self, image):
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        return ids

    def detect_holistic(self, image):
        landmarks = None
        if self._holistic is not None:
            image.flags.writeable = False
            results = self._holistic.process(image)
            image.flags.writeable = True
            if results is not None:
                landmarks = results.pose_landmarks
        return landmarks
