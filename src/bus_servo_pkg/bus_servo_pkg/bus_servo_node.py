#!/usr/bin/env python3
import os
import yaml
import serial
import time
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory
import threading

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # 1. 讀 YAML 設定檔
        pkg_share = get_package_share_directory('bus_servo_pkg')
        cfg_path = os.path.join(pkg_share, 'config', 'arm_config.yaml')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        params = cfg['servo_controller']['ros__parameters']
        port          = params['port']
        baud          = params.get('baud', 115200)
        self.speed    = params.get('speed', 100)
        self.servo_cfgs = params['servo_configs']
        
        # Temperature monitoring parameters
        self.overheat_return_angle = params.get('overheat_return_angle_deg', 90.0)
        self.temp_motor_index = params.get('temp_motor_index', 3)
        self.temp_motor_id = self.servo_cfgs[self.temp_motor_index - 1]['id']  # Convert index to ID
        self.overheat_threshold = params.get('limit_temp', 75.0)  # Read from YAML or default to 75°C
        self.is_overheating = False
        self.temp_check_lock = threading.Lock()

        # 2. 打開串口
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened serial port {port} @ {baud}bps')
        except Exception as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        # 3. 訂閱 JointTrajectoryPoint
        self.create_subscription(
            JointTrajectoryPoint,
            'robot_arm',
            self.joint_callback,
            10
        )
        
        # 4. Start temperature monitoring thread
        self.temp_monitor_running = True
        self.temp_thread = threading.Thread(target=self.temperature_monitor, daemon=True)
        self.temp_thread.start()
        
        self.get_logger().info(f'Temperature monitoring started for servo ID {self.temp_motor_id} (index {self.temp_motor_index})')
        self.get_logger().info(f'Overheat protection: >{self.overheat_threshold}°C will move to {self.overheat_return_angle}°')

    def calc_checksum(self, data_bytes):
        return (~sum(data_bytes) & 0xFF)

    def send_packet(self, servo_id, cmd, params):
        frame = bytearray([0x55,0x55,servo_id,len(params)+3,cmd] + params)
        frame.append(self.calc_checksum(frame[2:]))
        self.ser.write(frame)

    def read_temperature(self, servo_id):
        """Read servo temperature using the same method as read_temp_voltage_servo.py"""
        try:
            self.ser.reset_input_buffer()
            # Temperature read command
            cmd_temp_read = 0x1A
            pkt = [0x55, 0x55, 0x03, servo_id, cmd_temp_read]
            checksum = (~sum(pkt[2:]) & 0xFF) & 0xFF
            pkt.append(checksum)
            
            self.ser.write(bytes(pkt))
            time.sleep(0.02)
            resp = self.ser.read(7)
            
            if len(resp) >= 7:
                return resp[5]  # Temperature in Celsius
            return None
        except Exception as e:
            self.get_logger().warning(f'Failed to read temperature from servo {servo_id}: {e}')
            return None

    def temperature_monitor(self):
        """Background thread to monitor temperature"""
        while self.temp_monitor_running:
            try:
                temp = self.read_temperature(self.temp_motor_id)
                
                if temp is not None:
                    with self.temp_check_lock:
                        if temp > self.overheat_threshold and not self.is_overheating:
                            # Temperature exceeded threshold - activate overheat protection
                            self.is_overheating = True
                            self.get_logger().warn(f'OVERHEAT DETECTED! Servo {self.temp_motor_id}: {temp}°C > {self.overheat_threshold}°C')
                            self.get_logger().warn(f'Moving servo {self.temp_motor_id} to safe position: {self.overheat_return_angle}°')
                            
                            # Move the specified servo to safe position
                            cfg = self.servo_cfgs[self.temp_motor_index - 1]
                            self.move_servo_to_angle(self.temp_motor_id, self.overheat_return_angle, cfg['min'], cfg['max'])
                            
                        elif temp <= self.overheat_threshold and self.is_overheating:
                            # Temperature dropped below threshold - deactivate overheat protection
                            self.is_overheating = False
                            self.get_logger().info(f'Temperature normalized: Servo {self.temp_motor_id}: {temp}°C <= {self.overheat_threshold}°C')
                            self.get_logger().info('Overheat protection deactivated - normal operation resumed')
                        
                        # Log temperature every 10 readings (every 10 seconds)
                        if hasattr(self, 'temp_log_counter'):
                            self.temp_log_counter += 1
                        else:
                            self.temp_log_counter = 1
                            
                        if self.temp_log_counter % 10 == 0:
                            status = "OVERHEATING" if self.is_overheating else "Normal"
                            self.get_logger().info(f'Servo {self.temp_motor_id} temp: {temp}°C [{status}]')
                
            except Exception as e:
                self.get_logger().error(f'Temperature monitoring error: {e}')
            
            time.sleep(1.0)  # Check every second

    def move_servo_to_angle(self, sid, angle_deg, min_ang, max_ang):
        # 1) 使用 YAML 的 min/max 作為保護限制
        clamped_deg = max(min_ang, min(angle_deg, max_ang))  # 超過範圍時停留在邊界值

        # 2) 將限制後的角度 (0~240度範圍) 轉換回伺服馬達的目標值 (0~1000)
        #    假設 0 度對應 0，240 度對應 1000
        target = int(round((clamped_deg / 240.0) * 1000))
        target = max(0, min(target, 1000))  # 確保目標值在 0~1000 範圍內

        # 3) 構造指令並發送給伺服馬達
        pL, pH = target & 0xFF, (target >> 8) & 0xFF
        sp = max(0, min(int(self.speed), 1000)) # 使用全域速度
        sL, sH = sp & 0xFF, (sp >> 8) & 0xFF

        self.send_packet(sid, 0x01, [pL, pH, sL, sH])
        return clamped_deg  # 返回限制後的角度 (度數)

    def joint_callback(self, msg: JointTrajectoryPoint):
        positions = msg.positions or []
        positions_deg = np.degrees(positions)
        n_cfg = len(self.servo_cfgs)

        # Check current temperature and overheat status before processing commands
        with self.temp_check_lock:
            current_overheat_status = self.is_overheating

        for i, ang in enumerate(positions_deg[:n_cfg]):
            cfg = self.servo_cfgs[i]
            sid = cfg['id']
            amin = cfg['min']
            amax = cfg['max']

            # Check if this is the temperature-monitored servo and if it's overheating
            if sid == self.temp_motor_id and current_overheat_status:
                # Override command - force servo to safe position during overheat
                clamped = self.move_servo_to_angle(sid, self.overheat_return_angle, amin, amax)
                self.get_logger().warn(f'ID {sid}: OVERHEAT PROTECTION - forced to {clamped:.1f}° (ignoring command {ang:.1f}°)')
            else:
                # Normal operation - use received command
                clamped = self.move_servo_to_angle(sid, ang, amin, amax)
                if sid == self.temp_motor_id:
                    # Log temperature-monitored servo status
                    self.get_logger().info(f'ID {sid}: {clamped:.1f}° [Temp monitored - Normal]')
                else:
                    self.get_logger().info(f'ID {sid}: {clamped:.1f}°')

        extra = len(positions_deg) - n_cfg
        if extra > 0:
            self.get_logger().warn(f'Ignored {extra} extra angle commands')

    def destroy_node(self):
        # Stop temperature monitoring thread
        self.temp_monitor_running = False
        if hasattr(self, 'temp_thread') and self.temp_thread.is_alive():
            self.temp_thread.join(timeout=2.0)
            
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info('Shutting down servo_controller node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()