#!/usr/bin/env python3.10

import rclpy
import traceback
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
import maid_robot_system_interfaces.srv as MrsSrv
import maid_robot_system_interfaces.msg as MrsMsg


class MediapipeExtNodeParam():

    def _callback_on_params(self, parameter_list):
        result = False
        for parameter in parameter_list:
            if (parameter.name == 'INTERVAL_MS'):
                if (parameter.value == self.INTERVAL_MS):
                    result = True
            if (parameter.name == 'timeout_ms'):
                self.timeout_ms = parameter.value
                result = True
            if (parameter.name == 'publisher/resize/width'):
                self.width = parameter.value
                result = True
            if (parameter.name == 'publisher/resize/height'):
                self.height = parameter.value
                result = True
            if (parameter.name == 'drawing_box'):
                self.drawing_box = parameter.value
                result = True
            if (parameter.name == 'drawing_posture'):
                self.drawing_posture = parameter.value
                result = True
        return SetParametersResult(successful=result)

    def get_parameter(self, node: Node):
        self.timeout_ms = node.get_parameter('timeout_ms').get_parameter_value().integer_value
        self.drawing_box = node.get_parameter('drawing_box').get_parameter_value().bool_value
        self.drawing_posture = node.get_parameter('drawing_posture').get_parameter_value().bool_value
        self.width = node.get_parameter('publisher/resize/width').get_parameter_value().integer_value
        self.height = node.get_parameter('publisher/resize/height').get_parameter_value().integer_value

    def init(self, node: Node):
        self._namespace = node.get_namespace()
        self._node_name = node.get_name()
        node.declare_parameter('INTERVAL_MS', self.INTERVAL_MS)
        node.declare_parameter('timeout_ms', self.timeout_ms)
        node.declare_parameter('drawing_box', self.drawing_box)
        node.declare_parameter('drawing_posture', self.drawing_posture)
        node.declare_parameter('publisher/resize/width', self.width)
        node.declare_parameter('publisher/resize/height', self.height)
        self.INTERVAL_MS = node.get_parameter('INTERVAL_MS').get_parameter_value().integer_value
        self.get_parameter(node)
        node.add_on_set_parameters_callback(self._callback_on_params)

    def __init__(self):
        self.INTERVAL_MS = 500
        self.timeout_ms = 5000
        self.width = 640
        self.height = 512
        self.drawing_box = True
        self.drawing_posture = True


class SchematicDiagram():
    _radius = 5
    _color_green = (0, 255, 0)
    _thickness = 2

    def __init__(self):
        pass

    def drawing_point(self, image, w, h, data: MrsMsg.Landmark):
        if (data.exist is True):
            cv.circle(image, (int(data.x * w), int(data.y * h)), self._radius, self._color_green, self._thickness)

    def drawing_points(self, image, w, h, person_data: MrsMsg.PoseLandmarkModel):
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

    def drawing_line(self, image, w, h, start: MrsMsg.Landmark, end: MrsMsg.Landmark):
        if ((start.exist is True) and (end.exist is True)):
            cv.line(image,
                    (int(start.x * w), int(start.y * h)),
                    (int(end.x * w), int(end.y * h)),
                    self._color_green, self._thickness)

    def drawing_lines(self, image, w, h, data: MrsMsg.PoseLandmarkModel):
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
        self.drawing_line(image, w, h, data.left.ankle, data.left.foot_index)
        self.drawing_line(image, w, h, data.left.ankle, data.left.heel)
        self.drawing_line(image, w, h, data.left.heel, data.left.foot_index)
        # leg - right
        self.drawing_line(image, w, h, data.right.hip, data.right.knee)
        self.drawing_line(image, w, h, data.right.ankle, data.right.knee)
        self.drawing_line(image, w, h, data.right.ankle, data.right.foot_index)
        self.drawing_line(image, w, h, data.right.ankle, data.right.heel)
        self.drawing_line(image, w, h, data.right.heel, data.right.foot_index)

    def draw_box(self, image, x, y, w, h):
        cv.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), self._color_green, self._thickness)

    def drawing(self, image, width: float, height:
                float, person_data: MrsMsg.PoseLandmarkModel, area: MrsMsg.RectFloat,
                human_detected: bool, flag_box: bool, flag_posture: bool):

        if human_detected is True:
            if (flag_posture is True):
                self.drawing_points(image, width, height, person_data)
                self.drawing_lines(image, width, height, person_data)

            if (flag_box is True):
                box_01 = (width * area.x)
                box_02 = (height * area.y)
                box_03 = (width * area.width)
                box_04 = (height * area.height)
                self.draw_box(image, box_01, box_02, box_03, box_04)
        return image


class MediapipeExtNode(Node):
    ##############################################################################
    _in_srv_name = 'in_srv'
    _pub_image_name = 'out'

    _pub_image_queue_size = 2
    ##############################################################################

    def __init__(self, node_name):
        super().__init__(node_name)
        self._param = MediapipeExtNodeParam()
        self._sa = SchematicDiagram()
        self._bridge = CvBridge()

    def open(self):
        result = True
        try:
            self._param.init(self)
            # ##################################################################
            self._create_service_client()
            self._create_publisher()
            self._create_timer()
        except Exception as exception:
            self.get_logger().error('Exception : ' + str(exception))
            result = False
            traceback.print_exc()
        return result

    def closing(self):
        return True

    ##################################################################
    def _create_service_client(self):
        self._client = self.create_client(MrsSrv.MediaPipePoseLandmarkDetection, self._in_srv_name)
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service({}) not available, waiting again...'.format(self._client.srv_name))
        self._request = MrsSrv.MediaPipePoseLandmarkDetection.Request()
        self._request_image(self.get_clock().now().nanoseconds)

    def _create_publisher(self):
        self._pub_image = self.create_publisher(Image, self._pub_image_name, self._pub_image_queue_size)

    def _create_timer(self):
        self._timer = self.create_timer(float(self._param.INTERVAL_MS) / 1000.0, self._callback_timer)
    ##################################################################

    def _request_image(self, current_ns):
        self._response = self._client.call_async(self._request)
        self._next_time = (self._param.timeout_ms * 1000 * 1000) + current_ns

    def _resize(self, image, width, height):
        im_h, im_w = image.shape[:2]
        if ((im_w != 0) and (im_h != 0)):
            if ((width == 0) or (height == 0)):
                width = im_w
                height = im_h
            aspect = im_w / im_h
            if (width / height >= aspect):
                new_h = height
                new_w = round(new_h * aspect)
            else:
                new_w = width
                new_h = round(new_w / aspect)
            return cv.resize(image, dsize=(new_w, new_h))
        else:
            return image

    ##################################################################

    def _callback_timer(self):
        try:
            current_ns = self.get_clock().now().nanoseconds
            if (self._response is not None):
                if (self._response.done() is True):
                    msg: MrsSrv.MediaPipePoseLandmarkDetection.Response = self._response.result()
                    if ((msg.image.height != 0) and (msg.image.width != 0)):
                        pose_landmarks: MrsMsg.PoseDetection = msg.data
                        cv_image = self._bridge.imgmsg_to_cv2(msg.image, "bgr8")
                        if (pose_landmarks.human_detected is True):
                            image_height, image_wight = cv_image.shape[:2]
                            cv_image = self._sa.drawing(cv_image,
                                                        float(image_wight),
                                                        float(image_height),
                                                        pose_landmarks.landmark,
                                                        pose_landmarks.detected_area,
                                                        pose_landmarks.human_detected,
                                                        self._param.drawing_box,
                                                        self._param.drawing_posture)
                        #####################################################
                        cv_image = self._resize(cv_image, self._param.width, self._param.height)
                        self._pub_msg = self._bridge.cv2_to_imgmsg(np.array(cv_image), "bgr8")
                        self._pub_image.publish(self._pub_msg)
                        #####################################################
                    self._request_image(current_ns)
                    #####################################################
            if (self._next_time <= current_ns):
                self._request_image(current_ns)
                self.get_logger().warning('Timeout')

        except Exception as exception:
            self.get_logger().error('Exception (_callback_timer) : ' + str(exception))
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node_name = 'mediapipe_ext_node'
    traceback_logger = rclpy.logging.get_logger(node_name + '_logger')
    node = MediapipeExtNode(node_name)

    try:
        if (node.open() is True):
            rclpy.spin(node)

    except Exception as exception:
        traceback_logger.error(exception)
        traceback.print_exc()
        traceback_logger.error(traceback.format_exc())
    finally:
        node.closing()
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
