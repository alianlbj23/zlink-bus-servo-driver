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

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # 1. 讀 YAML 設定檔
        pkg_share = get_package_share_directory('bus_servo_pkg')
        cfg_path = os.path.join(pkg_share, 'config', 'arm_config.yaml')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        params = cfg['servo_controller']['ros__parameters']
        # 從 params 裡拿所有設定
        port          = params['port']
        baud          = params.get('baud', 115200)
        self.speed    = params.get('speed', 100)
        self.servo_cfgs = params['servo_configs']  # list of dicts

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

    def calc_checksum(self, data_bytes):
        return (~sum(data_bytes) & 0xFF)

    def send_packet(self, servo_id, cmd, params):
        frame = bytearray([0x55,0x55,servo_id,len(params)+3,cmd] + params)
        frame.append(self.calc_checksum(frame[2:]))
        self.ser.write(frame)

    def move_servo_to_angle(self, sid, angle_deg, min_ang, max_ang):
        # 1) 先做 min/max 限制
        clamped = max(min_ang, min(angle_deg, max_ang))
        # 2) 轉到 0..1000
        target = int(round((clamped - min_ang) / (max_ang - min_ang) * 1000))
        pL, pH = target & 0xFF, (target >> 8) & 0xFF
        # 3) 處理速度
        sp = max(0, min(int(self.speed), 1000))
        sL, sH = sp & 0xFF, (sp >> 8) & 0xFF

        self.send_packet(sid, 0x01, [pL, pH, sL, sH])

    def joint_callback(self, msg: JointTrajectoryPoint):
        positions = msg.positions or []
        # Convert radians to degrees
        positions_deg = np.degrees(positions)
        n_cfg = len(self.servo_cfgs)

        # 只處理前 n_cfg 個
        for i, ang in enumerate(positions_deg[:n_cfg]):
            cfg = self.servo_cfgs[i]
            sid   = cfg['id']
            amin  = cfg['min']
            amax  = cfg['max']
            self.move_servo_to_angle(sid, ang, amin, amax)
            self.get_logger().info(f'→ ID {sid}: {ang:.1f}° (clamped to [{amin},{amax}])')

        # 若有超過的，就忽略
        extra = len(positions_deg) - n_cfg
        if extra > 0:
            self.get_logger().warn(f'Ignored {extra} extra angle commands')

    def destroy_node(self):
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
