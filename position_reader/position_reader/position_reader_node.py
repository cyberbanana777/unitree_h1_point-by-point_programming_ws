#!/usr/bin/env python3
import rclpy
import json
import os
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from unitree_go.msg import (
    LowState, MotorStates
)
from ament_index_python.packages import get_package_share_directory


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Параметры для разграничения проверки позы
        self.max_num_joint_body = 19
        self.max_num_joint_fingers = 0
        self.max_num_joint_wrists = 0
        self.impact_joint = 9
        
        # Параметры
        self.declare_parameter('file_name', 'poses')
        file_name = self.get_parameter('file_name').get_parameter_value().string_value

        package_share_dir = get_package_share_directory("position_reader")
        file_path = os.path.join(
            package_share_dir, "resource", (file_name + '.txt')
        )
        
        # Загрузка поз из файла
        self.poses = self.load_poses(file_path)
        self.current_pose_index = 0

        self.control_dt = 0.01  # 100 Hz
        
        # Публикатор и подписчики
        self.publisher = self.create_publisher(
            String, 
            'positions_to_unitree', 
            10)
        
        self.subscription = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback_body_states, 
            10)
        
        self.subscription_wrist_states = self.create_subscription(
            MotorStates, 
            "wrist/states",
            self.listener_callback_wrists_states, 
            10
        )

        self.subscription_fingers_states = self.create_subscription(
            MotorStates, 
            "inspire/state", 
            self.listener_callback_fingers_states, 
            10
        )

                # Timers
        self.timer = self.create_timer(
            self.control_dt, self.timer_callback
        )
        # Точность совпадения позы
        self.position_threshold_wrists = 0.3
        self.position_threshold_body = 0.3
        self.position_threshold_fingers = 0.1

        self.achieve_body_pose = False
        self.achieve_fingers_pose = False
        self.achieve_wrist_pose = False


        self.pose_buffer = {
            0: 0.0,  # right_hip_roll_joint
            1: 0.0,  # right_hip_pitch_joint
            2: 0.0,  # right_knee_joint
            3: 0.0,  # left_hip_roll_joint
            4: 0.0,  # left_hip_pitch_joint
            5: 0.0,  # left_knee_joint
            6: 0.0,  # torso_joint
            7: 0.0,  # left_hip_yaw_joint
            8: 0.0,  # right_hip_yaw_joint
            9: 0.0, # IMPACT
            10: 0.0,  # left_ankle_joint
            11: 0.0,  # right_ankle_joint
            12: 0.0,  # right_shoulder_pitch_joint
            13: 0.0,  # right_shoulder_roll_joint
            14: 0.0, # right_shoulder_yaw_joint
            15: 0.0,  # right_elbow_joint
            16: 0.0,  # left_shoulder_pitch_joint
            17: 0.0,  # left_shoulder_roll_joint
            18: 0.0, # left_shoulder_yaw_joint
            19: 0.0,  # left_elbow_joint
            20: 0.0, #right_pinky
            21: 0.0, #right_ring
            22: 0.0, #right_middle
            23: 0.0, #right_index
            24: 0.0, #right_thumb_bend
            25: 0.0, #right_thumb_rotation
            26: 0.0, #left_pinky
            27: 0.0, #left_ring
            28: 0.0, #left_middle
            29: 0.0, #left_index
            30: 0.0, #left_thumb_bend
            31: 0.0, #left_thumb_rotation
            32: 0.0, # left_wrist
            33: 0.0,  # right_wrist
        }

        self.checked_joints = []
        
        # Опубликовать первую позу
        self.publish_current_pose()
    
    def load_poses(self, file_path):
        """Загружает позы из файла"""
        poses = []
        try:
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    poses.append(line)  # Добавляем строку как есть
            self.get_logger().info(f"Loaded {len(poses)} poses from {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load poses: {str(e)}")
            poses = []
        return poses
    
    def publish_current_pose(self):
        """Публикует текущую позу"""
        if self.current_pose_index < len(self.poses):
            msg = String()
            msg.data = self.poses[self.current_pose_index]
            self.publisher.publish(msg)
            self.get_logger().debug(f"Published pose {self.current_pose_index}: {msg.data}")
            self.update_pose_buffer()
        else:
            self.get_logger().info("All poses published")

    def update_pose_buffer(self):
        self.checked_joints = []
        raw_data = self.poses[self.current_pose_index]

        # Split message into position data and impact value
        parts = raw_data.split("$")
        if len(parts) != 2:
            self.get_logger().error(f"Invalid message format: {raw_data}")
            return

        data_part, impact_part = parts

        # Skip if no position data
        if not data_part or data_part == "{}":
            self.get_logger().debug("Empty pose data received, skipping processing")
            return

        # Process position data
        try:
            pose = json.loads(data_part)
            self.get_logger().debug(f"data = {pose}")

            for key, value in pose.items():
                if key.isdigit():  # Verify key is a numeric string
                    index = int(key)
                    self.pose_buffer[index] = value
                    self.checked_joints.append(index)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    
    def listener_callback_body_states(self, msg):
            """Обрабатывает входящие сообщения с состоянием суставов"""
            if self.current_pose_index >= len(self.poses):
                return  # Все позы уже опубликованы
            
            differences = []

            for i in self.checked_joints:
                if i == self.impact_joint:
                    continue

                if i > self.max_num_joint_body:
                    break

                differences.append(np.abs(msg.motor_state[i].q - self.pose_buffer[i]))

            max_diff = max(differences)

            if max_diff < self.position_threshold_body:
                self.achieve_body_pose = True

    def listener_callback_wrists_states(self, msg):
        """Обрабатывает входящие сообщения с состоянием суставов"""
        if self.current_pose_index >= len(self.poses)  or self.max_num_joint_wrists == 0:
            self.achieve_wrist_pose = True
            return  # Все позы уже опубликованы
        
        differences = []

        for i in self.checked_joints:
            if i == self.impact_joint or i <= self.max_num_joint_wrists:
                continue

            if i > self.max_num_joint_body:
                break

            differences.append(np.abs(msg.motor_state[i-20].q - self.pose_buffer[i]))

        max_diff = max(differences)

        if max_diff < self.position_threshold_wrists:
            self.achieve_wrist_pose = True

    def listener_callback_fingers_states(self, msg):
        """Обрабатывает входящие сообщения с состоянием суставов"""
        if self.current_pose_index >= len(self.poses) or self.max_num_joint_fingers == 0:
            self.achieve_fingers_pose = True
            return  # Все позы уже опубликованы
        
        differences = []

        for i in self.checked_joints:
            if i == self.impact_joint:
                continue

            if i > self.max_num_joint_fingers:
                break

            differences.append(np.abs(msg.motor_state[i-32].q - self.pose_buffer[i]))

        max_diff = max(differences)

        if max_diff < self.position_threshold_fingers:
            self.achieve_fingers_pose = True


    def timer_callback(self):
        if self.current_pose_index >= len(self.poses):
            self.get_logger().info("All poses completed")
            return
        
        # Добавим логирование для отладки
        self.get_logger().debug(
            f"Current pose: {self.current_pose_index}, "
            f"Body: {self.achieve_body_pose}, "
            f"Fingers: {self.achieve_fingers_pose}, "
            f"Wrists: {self.achieve_wrist_pose}"
        )
        
        # Проверяем условия для перехода к следующей позе
        body_ok = self.achieve_body_pose
        fingers_ok = self.max_num_joint_fingers == 0 or self.achieve_fingers_pose
        wrists_ok = self.max_num_joint_wrists == 0 or self.achieve_wrist_pose
        
        if body_ok and fingers_ok and wrists_ok:
            self.current_pose_index += 1
            self.achieve_body_pose = False
            self.achieve_fingers_pose = False
            self.achieve_wrist_pose = False
            
            if self.current_pose_index < len(self.poses):
                self.publish_current_pose()
            else:
                self.get_logger().info("All poses completed")

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
