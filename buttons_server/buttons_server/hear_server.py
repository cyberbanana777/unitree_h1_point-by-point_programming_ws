#!/usr/bin/env python3

'''
АННОТАЦИЯ
Данный скрипт обращатеся по API к локальному серверу и получает информацию о 
нажатых кнопках. Программа создаёт ноду "button_status_reader_node"
и публикует в топик "button_status" соббщения типа "String", в котором
содержится информация о нажатой кнопке, или "None", если ни нажата ни одна. 
'''

'''
ANNOTATION
This script accesses the local server via API and receives information about
the pressed buttons. The program creates a node "button_status_reader_node"
and publishes a message of type "String" to the topic "button_status", which
contains information about the pressed button, or "None" if none were pressed.
'''

import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import String

SERVER_URL = 'http://localhost:5000'
FREQUENCY = 20.0  # Частота мониторинга в Герцах
TOPIC = "button_status"


class ButtonStatusReaderNode(Node):
    def __init__(self, server_url='http://localhost:5000'):
        super().__init__('button_status_reader_node')
        self.server_url = server_url
        self.last_status = None
        self.last_active_button = None

        self.create_timer(1 / FREQUENCY, self.timer_callback)
        self.publisher = self.create_publisher(String, TOPIC, 10)

        self.get_logger().info("Запуск мониторинга состояний кнопок.")
        self.get_logger().info("Нажмите Ctrl+C для остановки")

    def get_current_status(self):
        """Получить текущее состояние кнопок"""
        try:
            response = requests.get(f'{self.server_url}/api/status')
            response.raise_for_status()
            self.last_status = response.json()
            return self.last_status

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Ошибка подключения: {e}")
            return None

    def timer_callback(self):
        """Мониторинг изменений с заданным интервалом по таймеру"""
        try:
            status = self.get_current_status()
            if status:
                if status['active_button'] != self.last_active_button:
                    if status['active_button']:
                        active_button_name = self._get_button_name(
                            status['active_button']
                        )
                        self.get_logger().info(
                            f"Активна кнопка: {active_button_name}"
                        )

                    else:
                        self.get_logger().info("Нет активных кнопок")

                    button_states = status['buttons']
                    self.get_logger().info(
                        'Button states = ' + str(button_states)
                    )

                    self.last_active_button = status['active_button']
                    msg = String()
                    msg.data = str(self.last_active_button)
                    self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Ошибка при получении данных: {e}")

    def _get_button_name(self, button_id):
        """Преобразовать ID кнопки в читаемое название"""
        names = {
            'selfie': 'Сэлфи',
            'wave': 'Помахать рукой',
            'photo': 'Приглашаю на фото',
            'free': 'Свободная кнопка',
        }
        return names.get(button_id, button_id)


def main(args=None):
    rclpy.init(args=args)
    button_status_reader_node = ButtonStatusReaderNode(SERVER_URL)

    try:
        rclpy.spin(button_status_reader_node)

    except KeyboardInterrupt:
        button_status_reader_node.get_logger().info("Мониторинг остановлен.")

    finally:
        button_status_reader_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)
