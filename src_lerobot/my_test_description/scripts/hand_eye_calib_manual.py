#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import xml.etree.ElementTree as ET
import numpy as np
import os
import yaml

# ------------------------------
# URDF 解析函数
# ------------------------------
def parse_urdf_joints(urdf_file):
    """
    解析 URDF 文件，返回机械臂关节名称列表 (joint_1 ~ joint_6)
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    joint_names = []
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name in [f"joint_{i}" for i in range(1, 7)]:
            joint_names.append(name)
    return joint_names

# ------------------------------
# ROS2 图像订阅节点
# ------------------------------
class ImageSubscriber(Node):
    def __init__(self, topic_name="/image_raw"):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_latest_image(self, timeout_sec=5.0):
        import time
        start_time = time.time()
        while self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                raise RuntimeError("没有接收到图像")
        return self.latest_image

# ------------------------------
# 主程序
# ------------------------------
def main():
    rclpy.init()
    image_node = ImageSubscriber(topic_name="/image_raw")

    urdf_file = "/home/ubuntu/my_test_ws/src/my_test_description/urdf/so101.urdf"
    joints = parse_urdf_joints(urdf_file)
    print("解析到的机械臂关节:", joints)

    # ------------------------------
    # 棋盘格参数
    # ------------------------------
    pattern_size = (9, 6)  # 内角数
    square_size = 0.025    # 单位米

    # 生成棋盘格世界坐标
    objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # ------------------------------
    # 相机内参
    # ------------------------------
    camera_matrix = np.array([
        [421.18549, 0.0, 340.20154],
        [0.0, 421.1219, 257.31452],
        [0.0, 0.0, 1.0]
    ])
    dist_coeffs = np.array([-0.386929, 0.114206, -0.011538, -0.004773, 0.0])

    # ------------------------------
    # 数据采集
    # ------------------------------
    num_positions = int(input("请输入手眼标定采集位置数: "))
    joint_positions = []
    rvecs = []
    tvecs = []

    for i in range(num_positions):
        input(f"\n手动移动机械臂到第{i+1}个位置，按Enter继续...")
        angles = input(f"输入6个关节角(rad)，用空格分开: ")
        angles = list(map(float, angles.strip().split()))
        if len(angles) != 6:
            print("错误：请输入6个关节角")
            return
        joint_positions.append(angles)

        # 获取图像
        try:
            img = image_node.get_latest_image()
        except RuntimeError as e:
            print(f"获取图像失败: {e}")
            continue

        # 转为灰度图
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            # 精细化角点
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11,11), (-1,-1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            # solvePnP 获取相机到棋盘格的位姿
            ret, rvec, tvec = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)
            rvecs.append(rvec)
            tvecs.append(tvec)

            # 保存图像
            filename = f"image_{i+1}.png"
            cv2.imwrite(filename, img)
            print(f"第{i+1}张图像及角点位姿已保存: {filename}")
        else:
            print(f"第{i+1}张图像未检测到棋盘格，请重新采集")
            continue

    # ------------------------------
    # 保存采集数据
    # ------------------------------
    save_dir = os.path.expanduser("~/handeye_data")
    os.makedirs(save_dir, exist_ok=True)

    # 保存关节角
    np.savetxt(os.path.join(save_dir, "joint_positions.txt"), np.array(joint_positions))
    # 保存相机位姿
    np.save(os.path.join(save_dir, "rvecs.npy"), np.array(rvecs))
    np.save(os.path.join(save_dir, "tvecs.npy"), np.array(tvecs))

    print("\n手眼标定数据采集完成，保存在:", save_dir)
    print("joint_positions.txt -> 关节角")
    print("rvecs.npy / tvecs.npy -> 相机到棋盘格位姿 (solvePnP)")

    rclpy.shutdown()

if __name__ == "__main__":
    main()