#!/usr/bin/env python3.10

class PoseInformation:
    class Axis3:
        x = 0
        y = 0
        z = 0

        def __init__(self):
            self.x = 0
            self.y = 0
            self.z = 0
            self.get_flag = 0

        def check(self):
            if (self.x == 0 and self.y == 0):
                self.get_flag = 0
            else:
                self.get_flag = 1

        def set(self, xx, yy, zz):
            self.x = xx
            self.y = yy
            self.z = zz
            self.check()

        def copy(self, pose_data):
            self.x = pose_data.x
            self.y = pose_data.y
            self.z = pose_data.z
            self.check()

    nose = Axis3()
    eye_left = Axis3()
    eye_right = Axis3()
    shoulder_left = Axis3()
    shoulder_right = Axis3()
    hand_left = Axis3()
    hand_right = Axis3()
    gaze = Axis3()

    def __init__(self):
        self.nose = self.Axis3()
        self.eye_left = self.Axis3()
        self.eye_right = self.Axis3()
        self.shoulder_left = self.Axis3()
        self.shoulder_right = self.Axis3()
        self.hand_left = self.Axis3()
        self.hand_right = self.Axis3()
        self.gaze = self.Axis3()
