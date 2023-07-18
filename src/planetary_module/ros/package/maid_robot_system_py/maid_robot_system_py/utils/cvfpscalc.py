from collections import deque
import cv2 as cv


class CvFpsCalc(object):
    def __init__(self, buffer_len=30):
        self._start_tick = cv.getTickCount()
        self._freq = 1000.0 / cv.getTickFrequency()
        self._diff_times = deque(maxlen=buffer_len)

    def get(self):
        current_tick = cv.getTickCount()
        different_time = (current_tick - self._start_tick) * self._freq
        self._start_tick = current_tick

        self._diff_times.append(different_time)

        fps = 1000.0 / ((sum(self._diff_times) / len(self._diff_times)))
        fps_rounded = round(fps, 2)

        return fps_rounded
