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
        port          = params['port']
        baud          = params.get('baud', 115200)
        self.speed    = params.get('speed', 100)
        self.servo_cfgs = params['servo_configs']

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

        for i, ang in enumerate(positions_deg[:n_cfg]):
            cfg = self.servo_cfgs[i]
            sid = cfg['id']
            amin = cfg['min']
            amax = cfg['max']

            # 調用 move_servo_to_angle 並獲取限制後的角度
            clamped = self.move_servo_to_angle(sid, ang, amin, amax)
            # 簡化日誌輸出
            self.get_logger().info(f'ID {sid}: {clamped:.1f}°')

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