#!/usr/bin/env python3

'''
АННОТАЦИЯ
ROS2 нода для обработки нажатий кнопок и управления последовательностями
движений. Считывает макросы из файлов, публикует позиции для робота Unitree H1
с регулируемым воздействием. Требует заготовленных файлов-макросов с
использованием рук (inspire hands).

ANNOTATION
ROS2 node for processing button presses and managing motion sequences. Reads
macros from files, publishes positions for Unitree H1 robot with adjustable
impact. Requires prepared macros fileswith using hands (inspire hands).
'''

import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

TOPIC_POS = "positions_to_unitree"
TOPIC_BUTTON = "button_status"
FREQUENCY = 333.3  # Monitoring frequency in Hertz


class ButtonAnalyzer(Node):
    """
    ROS2 node for reading values from buttons and sending values from macros.
    """

    def __init__(self):
        super().__init__("button_analyzer")

        self.impact = 0.0
        self.time_for_return_control = 1.0
        self.control_dt = 1 / FREQUENCY
        self.count_progress = 0

        package_share_dir = get_package_share_directory('button_analyzer')

        self.macroses = {
            "wave": 0,
            "photo": 1,
            "attention": 2,
            "selfie": 3,
            "1_offer_hand": 4,
            "2_shake_hand": 5,
            "1_offer_docs": 6,
            "2_grip_docs": 7,
            "3_hold_docs": 8,
            "4_give_docs": 9,
            "5_release_docs": 10,
        }

        self.index = None

        self.files = []
        for macros_name in self.macroses:
            file_path = os.path.join(
                package_share_dir, f"{macros_name}_with_hands.txt"
            )
            with open(file_path, 'r') as f:
                data = [line.rstrip('\n') for line in f]
                self.files.append(data)

        self.iters = []
        for file in self.files:
            self.iters.append(iter(file))

        for i in self.macroses.keys():
            self.get_logger().info(f"macros_founded: {i}")
            # self.get_logger().info(
            #     f"macros_include: {self.files[self.macroses[i]]}")

        self.last_pressed_button = "None"

        self.subscription = self.create_subscription(
            String, TOPIC_BUTTON, self.listener_callback, 10
        )

        self.create_timer(self.control_dt, self.timer_callback)
        self.publisher = self.create_publisher(String, TOPIC_POS, 10)

        self.msg = String()
        self.last_data = 'None'

    def listener_callback(self, msg):
        self.last_pressed_button = msg.data
        self.get_logger().info(f'listener_callback = {msg.data}')
        if self.last_pressed_button != "None":
            self.impact = 1.0

        if self.index is not None:
            self.iters[self.index] = iter(self.files[self.index])
            self.count_progress = 0

    def timer_callback(self):
        """Timer callback to process data."""
        # Check if there is an active button pressed
        if self.last_pressed_button == 'None':
            if self.impact != 0.0:
                self.return_control()
                self.get_logger().info('Drop_control')
            return

        # Check if a macro exists for the current button
        if self.last_pressed_button not in self.macroses:
            self.get_logger().error(
                f"Unknown button: {self.last_pressed_button}"
            )
            return

        self.index = self.macroses[self.last_pressed_button]
        # self.get_logger().info(f'index = {self.index}')
        data = self.iters[self.index]

        # self.get_logger().info(
        #   f'Selected macros = {self.last_pressed_button}'
        # )

        try:
            self.last_data = next(data)
            self.count_progress += 1
        except StopIteration:
            self.get_logger().info("Function ends")
            self.iters[self.index] = iter(self.files[self.index])
            data = self.iters[self.index]
            self.last_data = next(data)
            self.count_progress = 0

        self.msg.data = self.last_data + '$' + str(self.impact)
        self.get_logger().info(
            f'Progress: {self.count_progress}/\
                {len(self.files[self.index]) - 1}; \
                Impact = {round(self.impact, 3)}'
        )
        self.publisher.publish(self.msg)

    def return_control(self):
        '''Soft shutdown of our control'''
        if self.last_data != 'None':
            steps = self.time_for_return_control / self.control_dt
            value_of_step = 1.0 / steps
            for _ in range(int(steps) + 1):
                self.impact -= value_of_step
                self.impact = np.clip(self.impact, 0.0, 1.0)
                self.msg.data = (
                    self.last_data + '$' + str(round(self.impact, 3))
                )
                self.get_logger().info(f'Impact = {round(self.impact, 3)}')
                self.publisher.publish(self.msg)
                time.sleep(self.control_dt)


def main(args=None):
    """The main function for starting a node."""
    rclpy.init(args=args)
    node = ButtonAnalyzer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(e)

    finally:
        if node.impact != 0.0:
            node.get_logger().info('down_9')  ### ?????
            node.return_control()
        node.return_control()  ### ?????
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
