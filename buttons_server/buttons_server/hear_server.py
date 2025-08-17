#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS 2-узел для мониторинга состояния кнопок через HTTP-API. Получает данные с 
локального сервера (порт 5000), преобразует ID кнопок в читаемые названия и 
публикует изменения в топик button_status (тип String) с частотой 20 Гц. 
Сообщение содержит ID активной кнопки или None при отсутствии нажатий. 
Обрабатывает ошибки соединения и преобразования данных. Требует запущенного 
сервера и библиотек ROS 2, requests.

ANNOTATION
ROS 2 node for monitoring button states via HTTP API. Retrieves data from local 
server (port 5000), converts button IDs to readable names and publishes changes 
to button_status topic (String type) at 20 Hz rate. Message contains active 
button ID or None if no presses detected. Handles connection and data processing 
errors. Requires running server and ROS 2, requests libraries.
'''


import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String

# Configuration constants
SERVER_URL     = 'http://localhost:5000'
MONITOR_RATE   = 20.0  # Monitoring frequency in Hz
STATUS_TOPIC   = "button_status"


class ButtonStatusReaderNode(Node):
    """ROS 2 node that monitors and publishes button status from a web API."""
    
    def __init__(self, server_url='http://localhost:5000'):
        super().__init__('button_status_reader_node')
        self.server_url        = server_url
        self.last_status       = None
        self.last_active_button = None

        # Set up timer and publisher
        self.create_timer(1 / MONITOR_RATE, self.timer_callback)
        self.publisher = self.create_publisher(String, STATUS_TOPIC, 10)

        self.get_logger().info("Button status monitoring started")
        self.get_logger().info("Press Ctrl+C to stop")

    def get_current_status(self):
        """Fetch current button status from the server."""
        try:
            response = requests.get(f'{self.server_url}/api/status')
            response.raise_for_status()
            self.last_status = response.json()
            return self.last_status

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Connection error: {e}")
            return None

    def timer_callback(self):
        """Timer callback that checks for button state changes."""
        try:
            status = self.get_current_status()
            if not status:
                return

            if status['active_button'] != self.last_active_button:
                # Handle button state change
                if status['active_button']:
                    active_button_name = self._get_button_name(status['active_button'])
                    self.get_logger().info(f"Active button: {active_button_name}")
                else:
                    self.get_logger().info("No active buttons")

                # Log all button states
                self.get_logger().info(f"Button states: {status['buttons']}")

                # Publish the change
                self.last_active_button = status['active_button']
                msg = String()
                msg.data = str(self.last_active_button)
                self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Data processing error: {e}")

    def _get_button_name(self, button_id):
        """Convert button ID to human-readable name."""
        button_names = {
            'selfie': 'Selfie mode',
            'wave':   'Wave hand',
            'photo':  'Take photo',
            'free':   'Free button',
        }
        return button_names.get(button_id, button_id)


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    button_status_reader_node = ButtonStatusReaderNode(SERVER_URL)

    try:
        rclpy.spin(button_status_reader_node)

    except KeyboardInterrupt:
        button_status_reader_node.get_logger().info("Monitoring stopped")

    finally:
        button_status_reader_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()