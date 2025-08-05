#!/usr/bin/env python3

'''
АННОТАЦИЯ
Этот скрипт создаёт ROS2-ноду "macros_writer_node", которая слушает 
топик "positions_to_unitree" и записывает содержимое в файл, имя и 
путь которого задаются в ручном режиме во время запуска.
За 1 запуск программы возможно записать только 1 файл-дамп. Для 
записи нескольких файлов запустите скрипт несколько несколько. 
'''

'''
ANNOTATION
This script creates a ROS2 node "macros_writer_node" that listens to the 
topic "positions_to_unitree" and writes the contents to a file whose name and
path are specified manually during startup.
Only 1 dump file can be written per program startup. To write multiple files,
run the script multiple times.
'''

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "positions_to_unitree"


class MacrosWriterNode(Node):
    def __init__(self, macros_name: str, path_to_file: str):
        super().__init__('macros_writer_node')

        self.macros_name = macros_name
        self.abs_path = os.path.join(path_to_file, macros_name)
        print(self.abs_path)
        self.file = open(f"{self.abs_path}.txt", "w")

        self.subscription = self.create_subscription(
            String, TOPIC, self.listener_callback, 10
        )

        self.get_logger().info(f"Запуск записи макроса {self.macros_name}")
        self.get_logger().info("Нажмите Ctrl+C для остановки")

    def listener_callback(self, msg):
        """Запись в файл"""
        raw_data = msg.data
        data = raw_data.split('$')[0]
        self.file.write(f'{data}\n')
        self.file.flush()


def main(args=None):
    rclpy.init(args=args)

    macros_name = input("Введите имя нового макроса (без расширения): ")
    path_to_file = input("Введите АБСОЛЮТНЫЙ путь сохранения файла: ")

    macros_writer_node = MacrosWriterNode(macros_name, path_to_file)

    try:
        rclpy.spin(macros_writer_node)

    except KeyboardInterrupt:
        macros_writer_node.get_logger().info("Запись остановлена.")
        macros_writer_node.file.close()
        macros_writer_node.get_logger().info("Файл успешно сохранён и закрыт.")
        macros_writer_node.get_logger().info(
            f"Файл сохранён по пути: {macros_writer_node.abs_path}.txt"
        )

    finally:
        macros_writer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
