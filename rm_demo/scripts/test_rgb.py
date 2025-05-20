import sys
import time
import os
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo as msg_CameraInfo
from nav_msgs.msg import Odometry as msg_Odometry
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import argparse
import message_filters
import cv2
import matplotlib.pyplot as plt
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation as R


class CWaitForMessage:
    def __init__(self, opt):
        self.height = opt.img_height
        self.width = opt.img_width
        self.node_name = "rs2_listener"
        self.bridge = CvBridge()
        self.listener = None
        self.prev_msg_time = 0
        self.is_init = False
        # x->pitch, y->yaw, z->roll
        self.theta = {}
        self.theta["pitch"], self.theta["yaw"], self.theta["roll"] = None, None, None
        self.external_mat = None
        self.rotation_mat = None
        self.t_mat = None
        self.internal_mat = None

    def rotation_estimator(self, pos_data):
        w = pos_data.pose.pose.orientation.w
        x = pos_data.pose.pose.orientation.x
        y = pos_data.pose.pose.orientation.y
        z = pos_data.pose.pose.orientation.z
        self.theta["pitch"] = -np.arcsin(2.0 * (x*z - w*y)) * 180.0 / np.pi
        self.theta["roll"] = np.arctan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / np.pi
        self.theta["yaw"] = np.arctan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / np.pi

    def get_camera_external_param(self, pos_data):
        rotation_quaternion = np.asarray([pos_data.pose.pose.orientation.w, pos_data.pose.pose.orientation.x,
                                          pos_data.pose.pose.orientation.y, pos_data.pose.pose.orientation.z])
        translation = np.asarray([pos_data.pose.pose.position.x, pos_data.pose.pose.position.y,
                                  pos_data.pose.pose.position.z])
        T_qua2rota = RigidTransform(rotation_quaternion, translation)
        self.rotation_mat = T_qua2rota._rotation
        self.t_mat = T_qua2rota._translation

    def get_camera_internal_param(self, camera_info):
        k = camera_info.K
        self.internal_mat = np.zeros((3, 3))
        self.internal_mat[0, :] = k[:3]
        self.internal_mat[1, :] = k[3:6]
        self.internal_mat[2, :] = k[6:]

    def check(self, img_data, depth_data):
        cv_color_img = self.bridge.imgmsg_to_cv2(img_data, img_data.encoding)
        cv_depth_img = self.bridge.imgmsg_to_cv2(depth_data, depth_data.encoding)
        assert cv_color_img.shape == (self.height, self.width, 3)
        assert cv_depth_img.shape == (self.height, self.width)

    def estimate_absolute_pos(self, obj_pos, obj_depth):
        uv_pos = np.array([[obj_pos[0]], [obj_pos[1]], [1]])
        abs_pos = np.dot(np.linalg.inv(self.internal_mat), obj_depth * uv_pos)
        axes_trans = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        abs_pos = np.dot(axes_trans, abs_pos)
        # abs_pos[:, 0] -= self.t_mat * 1000
        abs_pos = np.dot(self.rotation_mat, abs_pos)
        abs_pos[:, 0] += self.t_mat * 1000
        # abs_pos = np.dot(np.linalg.inv(self.rotation_mat), abs_pos)
        return abs_pos.flatten()

    def callback(self, img_data, depth_data, pos_data, camera_info):
        # self.rotation_estimator(pos_data)
        # print(pos_data)
        if not self.is_init:
            self.check(img_data, depth_data)
            self.get_camera_internal_param(camera_info)
        cv_color_img = self.bridge.imgmsg_to_cv2(img_data, img_data.encoding)
        cv_depth_img = self.bridge.imgmsg_to_cv2(depth_data, depth_data.encoding)
        cv_color_img = cv2.cvtColor(cv_color_img, cv2.COLOR_RGB2BGR)
        obj_pos = [100, 100]
        self.get_camera_external_param(pos_data)
        obj_absolute_pos = self.estimate_absolute_pos(obj_pos, cv_depth_img[obj_pos[0], obj_pos[1]])
        cv2.rectangle(cv_color_img, (obj_pos[0] - 5, obj_pos[1] - 5), (obj_pos[0] + 5, obj_pos[1] + 5), (0, 255, 0), 2)
        pos_str = "x: {:.2f}, y: {:.2f}, z: {:.2f}".format(obj_absolute_pos[0], obj_absolute_pos[1], obj_absolute_pos[2])
        cv2.putText(cv_color_img, pos_str, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("RGB", cv_color_img)
        cv2.imshow("depth", cv_depth_img)

        if int(time.time()) % 10 == 0:
            print("pos")
            print(obj_absolute_pos)
            print(self.t_mat)
            print(self.rotation_mat)

        key = cv2.waitKey(1)
        # Press esc or "q" to close the image window
        if key & 0xFF == ord("q") or key == 27:
            cv2.destroyAllWindows()
            os._exit(0)

    def wait_for_messages(self):
        print("connect to ROS with name: %s" % self.node_name)
        rospy.init_node(self.node_name, anonymous=True)
        img_sub = message_filters.Subscriber("/camera/color/image_raw", msg_Image)
        depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", msg_Image)
        pos_sub = message_filters.Subscriber("/t265/odom/sample", msg_Odometry)
        carmera_sub = message_filters.Subscriber("/camera/color/camera_info", msg_CameraInfo)
        # topic synchronization by time
        ts = message_filters.ApproximateTimeSynchronizer([img_sub, depth_sub, pos_sub, carmera_sub],
                                                         queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.callback)
        rospy.spin()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_height", type=int, default=480, help="color image height")
    parser.add_argument("--img_width", type=int, default=640, help="color image width")
    opt = parser.parse_args()

    msg_retriever = CWaitForMessage(opt)
    msg_retriever.wait_for_messages()
