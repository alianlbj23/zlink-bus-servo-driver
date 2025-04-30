import serial
import time

# 改成 Linux 下的裝置檔
PORT    = "/dev/ttyUSB0"
BAUD    = 115200   # 與 ZLink 上 CH340 的波特率要一致
TIMEOUT = 0.1      # 可視需求調整

def calc_checksum(data_bytes):
    """計算 STBus 封包的校驗和 = ~sum & 0xFF"""
    return (~sum(data_bytes) & 0xFF)

def send_packet(ser, servo_id, cmd, params):
    """
    送一個 STBus 封包
    Frame = [0x55,0x55][ID][LEN][CMD][PARAM…][CHK]
    """
    length = len(params) + 3
    frame = bytearray([0x55, 0x55, servo_id, length, cmd] + params)
    frame.append(calc_checksum(frame[2:]))
    ser.write(frame)

def scan_connected_servos(ser, id_range=range(1, 31)):
    """
    掃描連接到串口的伺服馬達 (HTD-45H)
    返回有回應的伺服馬達 ID 列表
    """
    found = []
    print("正在掃描 HTD-45H 伺服馬達...")
    for sid in id_range:
        ser.reset_input_buffer()
        send_packet(ser, sid, 0x02, [])   # 讀位置/狀態指令
        time.sleep(0.05)
        if ser.in_waiting:
            resp = ser.read(ser.in_waiting)
            # 任意有效回應就視為該 ID 存在
            if len(resp) >= 6:
                found.append(sid)
                print(f"  ▶ 發現 ID = {sid}")
    print(f"掃描完成，共找到 {len(found)} 顆伺服馬達")
    return found

def move_servo_to_angle(ser, servo_id,
                        angle_deg,
                        max_angle=240.0,
                        speed=100):
    """
    以角度方式驅動 HiWonder HTD-45H 舵機
      angle_deg : 目標角度 (0 .. max_angle)
      max_angle : 240° (HTD-45H 規格)
      speed     : 0..1000
    """
    # 限制角度
    angle = max(0.0, min(angle_deg, max_angle))
    # 轉到 0..1000
    target = int(round(angle / max_angle * 1000))
    pL, pH = target & 0xFF, (target >> 8) & 0xFF

    sp = max(0, min(int(speed), 1000))
    sL, sH = sp & 0xFF, (sp >> 8) & 0xFF

    params = [pL, pH, sL, sH]
    send_packet(ser, servo_id, 0x01, params)

def move_multiple_servos(ser,
                         commands,
                         max_angle=240.0,
                         speed=100):
    """
    同時驅動多顆 HTD-45H
      commands : dict {servo_id: angle_deg, ...}
    """
    for sid, ang in commands.items():
        move_servo_to_angle(ser, sid, ang, max_angle, speed)

def read_servo_position(ser, servo_id):
    """
    讀取 HTD-45H 當前位置並轉成度數
    回傳 angle_deg 或 None
    """
    ser.reset_input_buffer()
    send_packet(ser, servo_id, 0x02, [])
    time.sleep(0.05)
    if ser.in_waiting:
        resp = ser.read(ser.in_waiting)
        if len(resp) >= 8:
            # 位置在第 7、8 byte (低位在前)
            pos = (resp[7] << 8) + resp[6]
            return pos / 1000 * 240.0
    return None

if __name__ == "__main__":
    # 確保已在 dialout 群組，否則無法打開 /dev/ttyUSB0
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        servos = scan_connected_servos(ser)
        print("已連接:", servos)

        if servos:
            cmds = { servos[0]: 270 }  # 120° 範例
            if len(servos) > 1:
                cmds[servos[1]] = 180
            move_multiple_servos(ser, cmds, max_angle=240.0, speed=200)
            print("發送角度指令：", cmds)

            # 讀回並顯示位置
            for sid in servos:
                ang = read_servo_position(ser, sid)
                print(f"ID {sid} 當前角度: {ang:.1f}°")
            time.sleep(0.2)
