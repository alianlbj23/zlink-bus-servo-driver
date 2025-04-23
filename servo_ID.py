#!/usr/bin/env python3
# filepath: /home/user/tmp/set_servo_id.py
import serial
import time
import argparse
import sys


def calc_checksum(data_bytes):
    """計算 STBus 封包的校驗和 = ~sum & 0xFF"""
    return ~sum(data_bytes) & 0xFF


def send_packet(ser, servo_id, cmd, params):
    """送一個 STBus 封包"""
    length = len(params) + 3
    frame = bytearray([0x55, 0x55, servo_id, length, cmd] + params)
    frame.append(calc_checksum(frame[2:]))
    ser.write(frame)
    time.sleep(0.01)  # 確保命令發送完成


def check_servo_responds(ser, servo_id, max_attempts=3):
    """檢查特定ID的舵機是否有回應"""
    for attempt in range(max_attempts):
        ser.reset_input_buffer()
        send_packet(ser, servo_id, 0x02, [])  # 發送讀取狀態命令
        time.sleep(0.2)  # 給予足夠響應時間

        if ser.in_waiting:
            response = ser.read(ser.in_waiting)
            if response and len(response) > 4:
                return True
    return False


def set_id_direct(ser, current_id, new_id):
    """針對眾靈ZLink轉接板的舵機ID設定，嘗試多種協議"""
    print(f"正在修改舵機 ID: {current_id} → {new_id}")

    # 檢查原ID舵機
    if not check_servo_responds(ser, current_id):
        print(f"錯誤: 找不到 ID {current_id} 的舵機")
        return False

    # 嘗試所有可能的ID修改方法
    methods = [
        # 方法1: 標準STBus協議 (0x03)
        lambda: send_packet(ser, current_id, 0x03, [new_id]),
        # 方法2: 樂博LX-16A協議
        lambda: send_packet(ser, current_id, 0x0D, [new_id]),
        # 方法3: 觸發EEPROM寫入
        lambda: (
            send_packet(ser, current_id, 0x03, [new_id]),
            time.sleep(0.5),
            send_packet(ser, current_id, 0x06, []),
        ),  # 存儲命令
        # 方法4: 使用寫EEPROM命令
        lambda: send_packet(ser, current_id, 0x01, [0xD4, 0x00, new_id]),
        # 方法5: 使用廣播ID + 當前位置
        lambda: send_packet(ser, 0xFE, 0x03, [new_id, current_id]),
    ]

    print("開始嘗試多種ID修改方式...")

    for i, method in enumerate(methods):
        print(f"方法 {i+1}...")
        method()  # 執行修改方法
        time.sleep(1.0)  # 等待舵機處理

        # 確認是否成功
        if check_servo_responds(ser, new_id):
            print(f"成功! 使用方法 {i+1} 修改ID成功")
            return True

        # 檢查原ID是否還存在
        print(f"檢查原ID {current_id} 是否仍響應...")
        still_responds = check_servo_responds(ser, current_id)

        if not still_responds:
            print(f"注意: 舵機不再使用ID {current_id} 回應，但也沒有使用ID {new_id}")
            print("嘗試循環掃描可能的ID...")

            # 掃描可能的ID範圍
            for test_id in range(1, 30):
                if (
                    test_id != current_id
                    and test_id != new_id
                    and check_servo_responds(ser, test_id)
                ):
                    print(f"發現舵機現在使用ID {test_id} 回應!")
                    return True

    print("所有嘗試都失敗，無法修改舵機ID")
    return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="直接修改伺服馬達ID")
    parser.add_argument("current_id", type=int, help="當前舵機ID")
    parser.add_argument("new_id", type=int, help="新的舵機ID (1-253)")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="串口設備")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="串口波特率")
    args = parser.parse_args()

    if not 1 <= args.new_id <= 253:
        print("錯誤: 新ID必須在1到253之間")
        sys.exit(1)

    try:
        print(f"開啟串口 {args.port}, 波特率 {args.baud}")
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            result = set_id_direct(ser, args.current_id, args.new_id)
            if result:
                print("ID修改成功")
                sys.exit(0)
            else:
                print("ID修改失敗")
                sys.exit(1)

    except serial.SerialException as e:
        print(f"串口錯誤: {e}")
        print("請確認裝置是否已插入且有適當權限")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n程式被中斷")
        sys.exit(1)
