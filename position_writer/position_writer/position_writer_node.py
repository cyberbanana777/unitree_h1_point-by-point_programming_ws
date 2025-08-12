#!/usr/bin/env python3

'''
АННОТАЦИЯ
ROS 2-приложение с графическим интерфейсом для записи поз робота.
Включает:
- Ноду pose_writer_node для обработки данных из топика positions_to_unitree
- Tkinter GUI с кнопками записи текущей позы, удаления и восстановления последней позы
- Сохранение поз в формате JSON с параметром impact в текстовые файлы
- Автоматическое создание директории poses для хранения данных
Требует ручного ввода имени файла при запуске. Поддерживает отмену/повтор последнего действия.

ANNOTATION
ROS 2 GUI application for recording robot poses.
Includes:
- pose_writer_node node for processing data from the positions_to_unitree topic
- Tkinter GUI with buttons for recording the current pose, deleting, and restoring the last pose
- Saving poses in JSON format with the impact parameter to text files
- Automatically creating a poses directory for storing data
Requires manual input of the file name at startup. Supports undo/redo of the last action.
'''


import json
import os
import threading
import tkinter as tk
import traceback
from tkinter import ttk

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class PoseWriterGUI:
    """Tkinter GUI for controlling pose recording operations."""
    
    def __init__(self, node):
        """Initialize the GUI window and widgets."""
        self.node = node
        self.root = tk.Tk()
        self.root.title("Pose Writer Control")
        self.root.geometry("400x200")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Configure styles
        self.style = ttk.Style()
        self.style.configure('TButton', font=('Arial', 10))
        self.style.configure('Big.TButton', font=('Arial', 14, 'bold'))

        # Main write button
        self.write_btn = ttk.Button(
            self.root,
            text="Record Pose",
            style='Big.TButton',
            command=self.write_pose,
        )
        self.write_btn.pack(pady=20, ipadx=20, ipady=10)

        # Secondary buttons frame
        self.small_btns_frame = ttk.Frame(self.root)
        self.small_btns_frame.pack(pady=10)

        # Delete button
        self.delete_btn = ttk.Button(
            self.small_btns_frame,
            text="Delete Last",
            command=self.delete_last_pose,
        )
        self.delete_btn.pack(side=tk.LEFT, padx=10, ipadx=5, ipady=5)

        # Restore button
        self.return_btn = ttk.Button(
            self.small_btns_frame,
            text="Restore Last",
            command=self.restore_last_pose,
        )
        self.return_btn.pack(side=tk.LEFT, padx=10, ipadx=5, ipady=5)

        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = ttk.Label(
            self.root,
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W,
        )
        self.status_bar.pack(fill=tk.X, pady=(10, 0))

        # file path


    def update_status(self, message):
        """Update status bar with temporary message."""
        self.status_var.set(message)
        self.root.after(3000, lambda: self.status_var.set("Ready"))

    def write_pose(self):
        """Handle pose recording request."""
        try:
            if self.node.write_pose_to_file():
                self.update_status("Pose recorded")
            else:
                self.update_status("Recording failed")
        except Exception as e:
            self.update_status("Recording error")
            self.node.get_logger().error(f"Pose recording error: {str(e)}")

    def delete_last_pose(self):
        """Handle last pose deletion request."""
        try:
            if self.node.delete_pose_from_file():
                self.update_status("Pose deleted")
            else:
                self.update_status("No poses to delete")
        except Exception as e:
            self.update_status("Deletion error")
            self.node.get_logger().error(f"Pose deletion error: {str(e)}")

    def restore_last_pose(self):
        """Handle last pose restoration request."""
        try:
            if self.node.restore_last_pose():
                self.update_status("Pose restored")
            else:
                self.update_status("No poses to restore")
        except Exception as e:
            self.update_status("Restoration error")
            self.node.get_logger().error(f"Pose restoration error: {str(e)}")

    def on_close(self):
        """Handle window close event."""
        self.root.destroy()

    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()


class PoseWriterNode(Node):
    """ROS2 node for managing pose recording operations."""
    
    def __init__(self):
        """Initialize the node and pose storage."""
        super().__init__('pose_writer_node')
        self.file_name = input('Enter filename for pose storage: ')

        # ROS2 subscription
        self.subscription = self.create_subscription(
            String, 
            'positions_to_unitree', 
            self.pose_callback, 
            10
        )

        self.impact = 0.0
        self.last_deleted_pose = None
        self.current_pose = {str(i): 0.0 for i in range(34)}

        package_name = 'position_writer'
        try:
            package_path = get_package_share_directory(package_name)
        except:
            # Fallback to current directory if package not found
            package_path = os.path.dirname(os.path.abspath(__file__))
            os.makedirs(os.path.join(package_path, 'poses'), exist_ok=True)

        self.poses_dir = os.path.join(package_path, 'poses')

    def pose_callback(self, msg):
        """Process incoming pose messages."""
        try:
            raw_data = msg.data
            parts = raw_data.split('$')
            if len(parts) != 2:
                return

            data_part, impact_part = parts
            self.impact = float(impact_part) if impact_part else 0.0

            if not data_part or data_part == '{}':
                return

            pose = json.loads(data_part)
            for key, value in pose.items():
                if key in self.current_pose:
                    self.current_pose[key] = float(value)

        except Exception as e:
            self.get_logger().error(f"Data processing error: {str(e)}")
            traceback.print_exc()

    def get_poses_file_path(self):
        """Determine the path to the poses file."""

        os.makedirs(self.poses_dir, exist_ok=True)
        return os.path.join(self.poses_dir, f"{self.file_name}.txt")

    def write_pose_to_file(self):
        """Record current pose to file."""
        try:
            file_path = self.get_poses_file_path()
            joint_data = json.dumps(self.current_pose)
            data = f"{joint_data}${self.impact:.2f}"

            with open(file_path, 'a') as file:
                file.write(data + '\n')
            return True
            
        except Exception as e:
            self.get_logger().error(f"File writing error: {str(e)}")
            traceback.print_exc()
            return False

    def delete_pose_from_file(self):
        """Remove last pose from file."""
        try:
            file_path = self.get_poses_file_path()

            if not os.path.exists(file_path):
                return False

            with open(file_path, 'r') as file:
                lines = file.readlines()

            if not lines:
                return False

            self.last_deleted_pose = lines[-1]
            with open(file_path, 'w') as file:
                file.writelines(lines[:-1])
            return True
            
        except Exception as e:
            self.get_logger().error(f"Pose deletion error: {str(e)}")
            traceback.print_exc()
            return False

    def restore_last_pose(self):
        """Restore last deleted pose to file."""
        try:
            if not self.last_deleted_pose:
                return False

            file_path = self.get_poses_file_path()
            with open(file_path, 'a') as file:
                file.write(self.last_deleted_pose)
            self.last_deleted_pose = None
            return True
            
        except Exception as e:
            self.get_logger().error(f"Pose restoration error: {str(e)}")
            traceback.print_exc()
            return False


def spin_node(node):
    """Spin the ROS2 node in a separate thread."""
    rclpy.spin(node)


def main(args=None):
    """Main function to initialize and run the application."""
    
    try:
        rclpy.init(args=args)
        node = PoseWriterNode()
        node.get_logger().info(f"Файл будет сохранён в: {node.poses_dir}")

        # Start ROS2 node in separate thread
        spin_thread = threading.Thread(
            target=spin_node, 
            args=(node,), 
            daemon=True
        )
        spin_thread.start()

        # Start GUI
        gui = PoseWriterGUI(node)
        gui.run()
    except KeyboardInterrupt:
        node.get_logger().info('Program stopped by user')
    finally:
        # Cleanup
        node.get_logger().info('Node stoped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()