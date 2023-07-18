#!/usr/bin/env python3.10

import rclpy
import time
import math
import subprocess
import cv2 as cv
from rclpy.node import Node
from std_msgs.msg import String
from utils.cvfpscalc import CvFpsCalc
from features.roscommunication import RosCommunication


class RosCommunication2():
    #####################################
    param_device_left_id = -1
    param_device_left_by_path = "--"
    param_device_left_type = "v4l"
    param_device_left_width = 960
    param_device_left_height = 540
    param_device_right_id = -1
    param_device_right_by_path = "--"
    param_device_right_type = "v4l"
    param_device_right_width = 960
    param_device_right_height = 540
    param_confidence_min_detection = 0.5
    param_confidence_min_tracking = 0.5
    param_update= True

    param_upper_body_only = True
    param_box_rect = True
    #####################################

    def __init__(self,node):
        node.declare_parameter("device/left/id", self.param_device_left_id)
        node.declare_parameter("device/left/by_path", self.param_device_left_by_path)
        node.declare_parameter("device/left/type", self.param_device_left_type)
        node.declare_parameter("device/left/width", self.param_device_left_width)
        node.declare_parameter("device/left/height", self.param_device_left_height)

        node.declare_parameter("device/right/id", self.param_device_right_id)
        node.declare_parameter("device/right/by_path", self.param_device_right_by_path)
        node.declare_parameter("device/right/type", self.param_device_right_type)
        node.declare_parameter("device/right/width", self.param_device_right_width)
        node.declare_parameter("device/right/height", self.param_device_right_height)

        node.declare_parameter("confidence/min_detection", self.param_confidence_min_detection)
        node.declare_parameter("confidence/min_tracking", self.param_confidence_min_tracking)

        node.declare_parameter("update", self.param_update)

    def get_parameter(self, node):

        self.param_device_left_id = node.get_parameter_or("device/left/id", self.param_device_left_id).get_parameter_value().integer_value
        self.param_device_left_by_path = node.get_parameter_or("device/left/by_path", self.param_device_left_by_path).get_parameter_value().string_value
        self.param_device_left_type = node.get_parameter_or("device/left/type", self.param_device_left_type).get_parameter_value().string_value
        self.param_device_left_width = node.get_parameter_or("device/left/width", self.param_device_left_width).get_parameter_value().integer_value
        self.param_device_left_height = node.get_parameter_or("device/left/height", self.param_device_left_height).get_parameter_value().integer_value

        self.param_device_right_id = node.get_parameter_or("device/right/id", self.param_device_right_id).get_parameter_value().integer_value
        self.param_device_right_by_path = node.get_parameter_or("device/right/by_path", self.param_device_right_by_path).get_parameter_value().string_value
        self.param_device_right_type = node.get_parameter_or("device/right/type", self.param_device_right_type).get_parameter_value().string_value
        self.param_device_right_width = node.get_parameter_or("device/right/width", self.param_device_right_width).get_parameter_value().integer_value
        self.param_device_right_height = node.get_parameter_or("device/right/height", self.param_device_right_height).get_parameter_value().integer_value

        self.param_confidence_min_detection = node.get_parameter_or("confidence/min_detection", self.param_confidence_min_detection).get_parameter_value().double_value
        self.param_confidence_min_tracking = node.get_parameter_or("confidence/min_tracking", self.param_confidence_min_tracking).get_parameter_value().double_value
        self.param_update = node.get_parameter_or("update", self.param_update).get_parameter_value().bool_value

        node.get_logger().info('Parameter: ')
        node.get_logger().info(' device: ')
        node.get_logger().info('  left: ')
        node.get_logger().info('   type   : ' + str(self.param_device_left_type))
        node.get_logger().info('   id     : ' + str(self.param_device_left_id))
        node.get_logger().info('   by_path: ' + str(self.param_device_left_by_path))
        node.get_logger().info('   width  : ' + str(self.param_device_left_width))
        node.get_logger().info('   height : ' + str(self.param_device_left_height))
        node.get_logger().info('  right: ')
        node.get_logger().info('   type   : ' + str(self.param_device_right_type))
        node.get_logger().info('   id     : ' + str(self.param_device_right_id))
        node.get_logger().info('   by_path: ' + str(self.param_device_right_by_path))
        node.get_logger().info('   width  : ' + str(self.param_device_right_width))
        node.get_logger().info('   height : ' + str(self.param_device_right_height))
        node.get_logger().info('  confidence: ')
        node.get_logger().info('   min_detection : ' + str(self.param_confidence_min_detection))
        node.get_logger().info('   min_tracking  : ' + str(self.param_confidence_min_tracking))
        node.get_logger().info('  update  : ' + str( self.param_update))


class UsbVideoDevice():
    _deviceList = []

    def display(self):
        for (row_id, row_name, row_path) in self._deviceList:
            print("{} : {}".format(row_path, row_name))

    def get_id_from_name(self, name):
        for (row_id, row_name, row_path) in self._deviceList:
            if (name in row_name):
                return row_id
        return -1

    def get_id_from_id(self, id):
        for (row_id, row_name, row_path) in self._deviceList:
            if (id == row_id):
                return row_id
        return -1

    def get_path(self, path):
        for (_, name, p) in self._deviceList:
            if (path in p):
                return p
        return ''

    def __init__(self, device_name):
        self._deviceList = []

        try:
            cmd = 'ls -la /dev/' + device_name + '/by-path'
            res = subprocess.check_output(cmd.split())
            by_path = res.decode()

            for line in by_path.split('\n'):
                if ('usb' in line):
                    tmp = self._split(line, ' ')
                    name = tmp[9]
                    tmp2 = self._split(tmp[11], '../../video')
                    deviceId = int(tmp2[0])
                    self._deviceList.append(
                        (deviceId, name, '/dev/video' + str(deviceId)))
        except:
            return

    def _split(self, str, val):
        tmp = str.split(val)
        if ('' in tmp):
            tmp.remove('')
        return tmp


class ModelNode(Node):
    _timer_period = 1  # seconds

    def __init__(self, node_name):
        #############################################################################
        # ROS : Setup
        super().__init__(node_name)
        #############################################################################
        # set instance
        self.ros_com = RosCommunication2(self)
        # ia = ImageAnalysis(ros_com.param_min_detection_confidence, ros_com.param_min_tracking_confidence)

        # get param
        self.ros_com.get_parameter(self)

        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.timer = self.create_timer(
            self._timer_period, self._timer_callback)

        self.i = 0


    def _timer_callback(self):
        device_left_id = self.get_parameter(            "device/left/id").get_parameter_value().integer_value
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i = device_left_id


def main(args=None):
    rclpy.init(args=args)
    node = ModelNode('face_recognition_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
