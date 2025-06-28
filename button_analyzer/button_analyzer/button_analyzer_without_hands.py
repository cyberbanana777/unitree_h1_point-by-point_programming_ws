#!/usr/bin/env python3

'''
АННОТАЦИЯ
'''

'''
ANNOTATION
'''

import os
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String


TOPIC_POS = "positions_to_unitree"
TOPIC_BUTTON = "button_status"
FREQUENCY = 333.3  # Частота мониторинга в Герцах


class ButtonAnalyzer(Node):
    """ROS2 нода для чтения показания с кнопок и отправки значений из макросов."""

    def __init__(self):
        super().__init__("button_analyzer")

        self.impact = 0.0
        self.time_for_return_control = 1.0
        self.control_dt = 1 / FREQUENCY
        self.count_progress = 0

        package_share_dir = get_package_share_directory('button_analyzer')

        self.macroses = {
            "selfie": 0,
            "wave": 1,
            "attention": 2,
            "photo": 3,
            
        }

        self.index = None

        self.files = []
        for macros_name in self.macroses:
            file_path = os.path.join(package_share_dir, f'{macros_name}_without_hands.txt')
            with open(file_path, 'r') as f:
                data = [line.rstrip('\n') for line in f]
                self.files.append(data)

        self.iters = []
        for file in self.files:
            self.iters.append(iter(file))

        for i in self.macroses.keys():
            self.get_logger().info(f"macros_founded: {i}")
            self.get_logger().info(
                f"macros_include: {self.files[self.macroses[i]]}")

        self.last_pressed_button = "None"

        self.subscription = self.create_subscription(
            String,
            TOPIC_BUTTON,
            self.listener_callback,
            10
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

        if self.index != None:
            self.iters[self.index] = iter(self.files[self.index])
            self.count_progress = 0

    def timer_callback(self):
        """Обратный вызов таймера для обработки данных."""
        # Проверяем, есть ли активная кнопка
        if self.last_pressed_button == 'None':
            if self.impact != 0.0:
                self.return_control()
                self.get_logger().info('Drop_control')
            return

        # Проверяем, существует ли макрос для текущей кнопки
        if self.last_pressed_button not in self.macroses:
            self.get_logger().error(
                f"Unknown button: {self.last_pressed_button}")
            return

        self.index = self.macroses[self.last_pressed_button]
        # self.get_logger().info(f'index = {self.index}')
        data = self.iters[self.index]

        # self.get_logger().info(f'Selected macros = {self.last_pressed_button}')

        try:
            self.last_data = next(data)
            self.count_progress += 1
        except StopIteration:
            self.get_logger().info(f"Функция завершилась")
            self.iters[self.index] = iter(self.files[self.index])
            data = self.iters[self.index]
            self.last_data = next(data)
            self.count_progress = 0

        self.msg.data = self.last_data + '$' + str(self.impact)
        self.get_logger().info(
            f'Progress: {self.count_progress}/{len(self.files[self.index]) - 1}; Impact = {round(self.impact, 3)}')
        self.publisher.publish(self.msg)

    def return_control(self):
        if self.last_data != 'None':
            steps = self.time_for_return_control / self.control_dt
            value_of_step = 1.0 / steps
            for _ in range(int(steps) + 1):
                self.impact -= value_of_step
                self.impact = np.clip(self.impact, 0.0, 1.0)
                self.msg.data = self.last_data + \
                    '$' + str(round(self.impact, 3))
                self.get_logger().info(f'Impact = {round(self.impact, 3)}')
                self.publisher.publish(self.msg)
                time.sleep(self.control_dt)


def main(args=None):
    """Основная функция для запуска ноды."""
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
            node.get_logger().info('down_9')
            node.return_control()
        node.return_control()
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
