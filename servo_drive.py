import serial
import time

# 改成 Linux 下的裝置檔
PORT = "/dev/ttyUSB0"
BAUD = 115200  # 與 ZLink 上 CH340 的波特率要一致
TIMEOUT = 0.1  # 可視需求調整


def calc_checksum(data_bytes):
    """計算 STBus 封包的校驗和 = ~sum & 0xFF"""
    return ~sum(data_bytes) & 0xFF


def scan_connected_servos(ser, id_range=range(1, 31)):
    """
    掃描連接到串口的伺服馬達
    返回有回應的伺服馬達ID列表
    """
    connected_servos = []
    print("正在掃描連接的伺服馬達...")

    for servo_id in id_range:
        # 清空緩衝區
        ser.reset_input_buffer()

        # 發送讀取狀態命令
        send_packet(ser, servo_id, 0x02, [])

        # 增加等待時間，給串接的馬達更多響應時間
        time.sleep(0.1)

        # 嘗試讀取回應
        if ser.in_waiting:
            response = ser.read(ser.in_waiting)
            if response and len(response) > 4:  # 確保回應有效
                connected_servos.append(servo_id)
                print(f"發現伺服馬達 ID: {servo_id}")

    print(f"掃描完成，找到 {len(connected_servos)} 顆伺服馬達")
    return connected_servos


def send_packet(ser, servo_id, cmd, params):
    """
    送一個 STBus 封包
    Frame = [0x55,0x55][ID][LEN][CMD][PARAM…][CHK]
    """
    length = len(params) + 3
    frame = bytearray([0x55, 0x55, servo_id, length, cmd] + params)
    frame.append(calc_checksum(frame[2:]))
    ser.write(frame)


def move_servo_to_angle(ser, servo_id, angle_deg, max_angle=300.0, speed=100):
    """
    以角度方式驅動單顆舵機
      angle_deg : 目標角度 (0 .. max_angle)
      max_angle : 實際最大轉角 (預設 300°)
      speed     : 0..1000
    """
    angle = max(0.0, min(angle_deg, max_angle))
    target_pos = int(round(angle / max_angle * 1000))
    low_p, high_p = target_pos & 0xFF, (target_pos >> 8) & 0xFF

    sp = max(0, min(int(speed), 1000))
    low_s, high_s = sp & 0xFF, (sp >> 8) & 0xFF

    params = [low_p, high_p, low_s, high_s]
    send_packet(ser, servo_id, 0x01, params)


def move_multiple_servos(ser, commands, max_angle=300.0, speed=100):
    """
    同時驅動多顆舵機
      commands : dict {servo_id: angle_deg, ...}
      max_angle、speed 可在呼叫時指定通用值
    """
    for servo_id, angle in commands.items():
        move_servo_to_angle(ser, servo_id, angle, max_angle, speed)


if __name__ == "__main__":
    # 確保使用者在 dialout 群組才能開 /dev/ttyUSB0
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        connected_servos = scan_connected_servos(ser)
        print(f"已連接的伺服馬達: {connected_servos}")
        # 同時設定 ID=1 →  90°，ID=2 → 45°
        servo_commands = {1: 360, 2: 180}
        move_multiple_servos(
            ser, commands=servo_commands, max_angle=300, speed=50  # 依你舵機規格調整
        )
        print(f"已發送：{servo_commands}")
        time.sleep(0.2)
