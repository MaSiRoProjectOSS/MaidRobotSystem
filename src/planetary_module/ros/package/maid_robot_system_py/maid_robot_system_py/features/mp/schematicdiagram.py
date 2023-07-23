#!/usr/bin/env python3.10

import cv2 as cv
import numpy as np
from maid_robot_system_interfaces.msg._pose_landmark_model import PoseLandmarkModel
from maid_robot_system_interfaces.msg._landmark import Landmark


class SchematicDiagram():
    _radius = 5
    _color = (0, 255, 0)
    _thickness = 2

    def __init__(self):
        self._init = False

    def drawing_point(self, image, w, h, data: Landmark):
        if (data.exist is True):
            cv.circle(image, (int(data.x * w), int(data.y * h)), self._radius, self._color, self._thickness)

    def drawing_points(self, image, w, h, person_data: PoseLandmarkModel):
        self.drawing_point(image, w, h, person_data.nose)
        self.drawing_point(image, w, h, person_data.left.eye_inner)
        self.drawing_point(image, w, h, person_data.left.eye)
        self.drawing_point(image, w, h, person_data.left.eye_outer)
        self.drawing_point(image, w, h, person_data.right.eye_inner)
        self.drawing_point(image, w, h, person_data.right.eye)
        self.drawing_point(image, w, h, person_data.right.eye_outer)
        self.drawing_point(image, w, h, person_data.left.ear)
        self.drawing_point(image, w, h, person_data.right.ear)
        self.drawing_point(image, w, h, person_data.left.mouth)
        self.drawing_point(image, w, h, person_data.right.mouth)
        self.drawing_point(image, w, h, person_data.left.shoulder)
        self.drawing_point(image, w, h, person_data.right.shoulder)
        self.drawing_point(image, w, h, person_data.left.elbow)
        self.drawing_point(image, w, h, person_data.right.elbow)
        self.drawing_point(image, w, h, person_data.left.wrist)
        self.drawing_point(image, w, h, person_data.right.wrist)
        self.drawing_point(image, w, h, person_data.left.pinky)
        self.drawing_point(image, w, h, person_data.right.pinky)
        self.drawing_point(image, w, h, person_data.left.index)
        self.drawing_point(image, w, h, person_data.right.index)
        self.drawing_point(image, w, h, person_data.left.thumb)
        self.drawing_point(image, w, h, person_data.right.thumb)
        self.drawing_point(image, w, h, person_data.left.hip)
        self.drawing_point(image, w, h, person_data.right.hip)
        self.drawing_point(image, w, h, person_data.left.knee)
        self.drawing_point(image, w, h, person_data.right.knee)
        self.drawing_point(image, w, h, person_data.left.ankle)
        self.drawing_point(image, w, h, person_data.right.ankle)
        self.drawing_point(image, w, h, person_data.left.heel)
        self.drawing_point(image, w, h, person_data.right.heel)
        self.drawing_point(image, w, h, person_data.left.foot_index)
        self.drawing_point(image, w, h, person_data.right.foot_index)

    def drawing_line(self, image, w, h, start: Landmark, end: Landmark):
        if ((start.exist is True) and (end.exist is True)):
            cv.line(image,
                    (int(start.x * w), int(start.y * h)),
                    (int(end.x * w), int(end.y * h)),
                    self._color, self._thickness)

    def drawing_lines(self, image, w, h, data: PoseLandmarkModel):
        # face line
        self.drawing_line(image, w, h, data.nose, data.left.eye)
        self.drawing_line(image, w, h, data.nose, data.right.eye)
        self.drawing_line(image, w, h, data.left.ear, data.left.eye)
        self.drawing_line(image, w, h, data.right.ear, data.right.eye)
        # eye
        self.drawing_line(image, w, h, data.left.eye, data.left.eye_inner)
        self.drawing_line(image, w, h, data.left.eye, data.left.eye_outer)
        self.drawing_line(image, w, h, data.right.eye, data.right.eye_inner)
        self.drawing_line(image, w, h, data.right.eye, data.right.eye_outer)
        # mouth
        self.drawing_line(image, w, h, data.left.mouth, data.right.mouth)
        # body
        self.drawing_line(image, w, h, data.left.shoulder, data.right.shoulder)
        self.drawing_line(image, w, h, data.left.hip, data.right.hip)
        self.drawing_line(image, w, h, data.left.shoulder, data.left.hip)
        self.drawing_line(image, w, h, data.right.shoulder, data.right.hip)
        # arm - left
        self.drawing_line(image, w, h, data.left.shoulder, data.left.elbow)
        self.drawing_line(image, w, h, data.left.wrist, data.left.elbow)
        self.drawing_line(image, w, h, data.left.wrist, data.left.thumb)
        self.drawing_line(image, w, h, data.left.wrist, data.left.pinky)
        self.drawing_line(image, w, h, data.left.wrist, data.left.index)
        self.drawing_line(image, w, h, data.left.pinky, data.left.index)
        # arm - right
        self.drawing_line(image, w, h, data.right.shoulder, data.right.elbow)
        self.drawing_line(image, w, h, data.right.wrist, data.right.elbow)
        self.drawing_line(image, w, h, data.right.wrist, data.right.thumb)
        self.drawing_line(image, w, h, data.right.wrist, data.right.pinky)
        self.drawing_line(image, w, h, data.right.wrist, data.right.index)
        self.drawing_line(image, w, h, data.right.pinky, data.right.index)
        # leg - left
        self.drawing_line(image, w, h, data.left.hip, data.left.knee)
        self.drawing_line(image, w, h, data.left.ankle, data.left.knee)
        self.drawing_line(image, w, h, data.left.ankle, data.left.thumb)
        self.drawing_line(image, w, h, data.left.ankle, data.left.heel)
        self.drawing_line(image, w, h, data.left.heel, data.left.foot_index)
        # leg - right
        self.drawing_line(image, w, h, data.right.hip, data.right.knee)
        self.drawing_line(image, w, h, data.right.ankle, data.right.knee)
        self.drawing_line(image, w, h, data.right.ankle, data.right.thumb)
        self.drawing_line(image, w, h, data.right.ankle, data.right.heel)
        self.drawing_line(image, w, h, data.right.heel, data.right.foot_index)

    def draw_box(self, image, x, y, w, h):
        cv.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), self._color, self._thickness)

    def drawing(self, image, width, height, person_data: PoseLandmarkModel, flag_box):
        if person_data.human_detected is True:
            self.drawing_points(image, width, height, person_data)
            self.drawing_lines(image, width, height, person_data)

            if flag_box is True:
                box_01 = (width * person_data.area.x)
                box_02 = (height * person_data.area.y)
                box_03 = (width * person_data.area.width)
                box_04 = (height * person_data.area.height)
                self.draw_box(image, box_01, box_02, box_03, box_04)
        return image
