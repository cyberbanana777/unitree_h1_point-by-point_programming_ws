#!/usr/bin/env python3

import os
import json
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import threading
import traceback

class PoseWriterGUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Pose Writer Control")
        self.root.geometry("400x200")
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self.style = ttk.Style()
        self.style.configure('TButton', font=('Arial', 10))
        self.style.configure('Big.TButton', font=('Arial', 14, 'bold'))
        
        self.write_btn = ttk.Button(
            self.root, 
            text="Записать позу", 
            style='Big.TButton',
            command=self.write_pose
        )
        self.write_btn.pack(pady=20, ipadx=20, ipady=10)
        
        self.small_btns_frame = ttk.Frame(self.root)
        self.small_btns_frame.pack(pady=10)
        
        self.delete_btn = ttk.Button(
            self.small_btns_frame, 
            text="Удалить последнюю", 
            command=self.delete_last_pose
        )
        self.delete_btn.pack(side=tk.LEFT, padx=10, ipadx=5, ipady=5)
        
        self.return_btn = ttk.Button(
            self.small_btns_frame, 
            text="Вернуть последнюю", 
            command=self.return_last_pose
        )
        self.return_btn.pack(side=tk.LEFT, padx=10, ipadx=5, ipady=5)
        
        self.status_var = tk.StringVar(value="Готов к работе")
        self.status_bar = ttk.Label(
            self.root, 
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W
        )
        self.status_bar.pack(fill=tk.X, pady=(10,0))
    
    def update_status(self, message):
        self.status_var.set(message)
        self.root.after(3000, lambda: self.status_var.set("Готов к работе"))
    
    def write_pose(self):
        try:
            if self.node.write_pose_to_file():
                self.update_status("Поза записана")
            else:
                self.update_status("Ошибка записи")
        except Exception as e:
            self.update_status("Ошибка записи")
            self.node.get_logger().error(f"Ошибка при записи позы: {str(e)}")
    
    def delete_last_pose(self):
        try:
            if self.node.delete_pose_from_file():
                self.update_status("Поза удалена")
            else:
                self.update_status("Нет поз для удаления")
        except Exception as e:
            self.update_status("Ошибка удаления")
            self.node.get_logger().error(f"Ошибка при удалении позы: {str(e)}")
    
    def return_last_pose(self):
        try:
            if self.node.return_last_pose():
                self.update_status("Поза возвращена")
            else:
                self.update_status("Нет поз для возврата")
        except Exception as e:
            self.update_status("Ошибка возврата")
            self.node.get_logger().error(f"Ошибка при возврате позы: {str(e)}")
    
    def on_close(self):
        self.node.get_logger().info('Завершение работы...')
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()
    
    def run(self):
        self.root.mainloop()

class PoseWriterNode(Node):
    def __init__(self):
        super().__init__('pose_writer_node')
        self.file_name = input('Введите имя файла для сохранения поз: ')
        
        self.subscription_positions_to_unitree = self.create_subscription(
            String,
            'positions_to_unitree',
            self.listener_callback,
            10)
        
        self.impact = 0.0
        self.last_line = None
        self.current_pose = {str(i): 0.0 for i in range(34)}
    
    def listener_callback(self, msg):
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
            self.get_logger().error(f"Ошибка обработки данных: {str(e)}")
            traceback.print_exc()

    def write_pose_to_file(self):
        try:
            package_name = 'position_writer'
            try:
                package_path = get_package_share_directory(package_name)
            except:
                # Если пакет не найден, используем текущую директорию
                package_path = os.path.dirname(os.path.abspath(__file__))
                os.makedirs(os.path.join(package_path, 'poses'), exist_ok=True)
            
            poses_dir = os.path.join(package_path, 'poses')
            os.makedirs(poses_dir, exist_ok=True)
            file_path = os.path.join(poses_dir, f"{self.file_name}.txt")
            
            joint_data = json.dumps(self.current_pose)
            data = f"{joint_data}${self.impact:.2f}"
            
            with open(file_path, 'a') as file:
                file.write(data + '\n')
            return True
        except Exception as e:
            self.get_logger().error(f"Ошибка записи в файл: {str(e)}")
            traceback.print_exc()
            return False
    
    def delete_pose_from_file(self):
        try:
            package_name = 'position_writer'
            try:
                package_path = get_package_share_directory(package_name)
            except:
                package_path = os.path.dirname(os.path.abspath(__file__))
            
            file_path = os.path.join(package_path, 'poses', f"{self.file_name}.txt")
            
            if not os.path.exists(file_path):
                return False
                
            with open(file_path, 'r') as file:
                lines = file.readlines()
            
            if not lines:
                return False
                
            self.last_line = lines[-1]
            with open(file_path, 'w') as file:
                file.writelines(lines[:-1])
            return True
        except Exception as e:
            self.get_logger().error(f"Ошибка удаления позы: {str(e)}")
            traceback.print_exc()
            return False
    
    def return_last_pose(self):
        try:
            if not self.last_line:
                return False
                
            package_name = 'position_writer'
            try:
                package_path = get_package_share_directory(package_name)
            except:
                package_path = os.path.dirname(os.path.abspath(__file__))
            
            file_path = os.path.join(package_path, 'poses', f"{self.file_name}.txt")
            
            with open(file_path, 'a') as file:
                file.write(self.last_line)
            self.last_line = None
            return True
        except Exception as e:
            self.get_logger().error(f"Ошибка возврата позы: {str(e)}")
            traceback.print_exc()
            return False

def spin_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = PoseWriterNode()
    
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()
    
    gui = PoseWriterGUI(node)
    gui.run()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()