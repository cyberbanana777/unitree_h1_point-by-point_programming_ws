#!/usr/bin/env python3

'''
АННОТАЦИЯ
ROS 2-узел для записи движений робота Unitree. Создает ноду 
"macros_writer_node", которая подписывается на топик "positions_to_unitree" 
и записывает полученные данные в текстовый файл. Требует ручного ввода имени 
макроса и абсолютного пути сохранения при запуске. За один запуск записывает 
данные в один файл (расширение .txt). Для записи нескольких файлов требуется 
многократный запуск. Автоматически закрывает файл при завершении работы.

ANNOTATION
ROS 2 node for recording Unitree robot motion. Creates "macros_writer_node" 
that subscribes to "positions_to_unitree" topic and writes received data to a text 
file. Requires manual input of macro name and absolute save path during startup. 
Writes data to a single file (.txt extension) per execution. Multiple runs required 
for multiple file recording. Automatically closes file on shutdown.
'''


import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Constants
TOPIC_NAME = "positions_to_unitree"


class MacrosWriterNode(Node):
    """ROS2 node for recording position data to a file."""
    
    def __init__(self, macro_name: str, output_path: str):
        """
        Initialize the macro writer node.
        
        Args:
            macro_name: Name of the macro (without extension)
            output_path: Absolute path for saving the file
        """
        super().__init__('macros_writer_node')
        
        self.macro_name = macro_name
        self.output_path = os.path.join(output_path, macro_name)
        
        # Initialize output file
        self.output_file = open(f"{self.output_path}.txt", "w")
        
        # Set up topic subscription
        self.subscription = self.create_subscription(
            String,
            TOPIC_NAME,
            self.data_callback,
            10
        )
        
        self.get_logger().info(f"Started recording macro: {self.macro_name}")
        self.get_logger().info("Press Ctrl+C to stop recording")

    def data_callback(self, msg):
        """
        Process incoming messages and write to file.
        
        Args:
            msg: Incoming String message from ROS topic
        """
        # Extract and clean data
        raw_data = msg.data
        cleaned_data = raw_data.split('$')[0]
        
        # Write to file
        self.output_file.write(f'{cleaned_data}\n')
        self.output_file.flush()


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    
    # Get user input for file details
    macro_name = input("Enter new macro name (without extension): ")
    output_path = input("Enter ABSOLUTE path for saving the file: ")
    
    # Create and run node
    node = MacrosWriterNode(macro_name, output_path)
    
    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.get_logger().info("Recording stopped")
        node.output_file.close()
        node.get_logger().info("File successfully saved and closed")
        node.get_logger().info(
            f"File saved at: {node.output_path}.txt"
        )
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()