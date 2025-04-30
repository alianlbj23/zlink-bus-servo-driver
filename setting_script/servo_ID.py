#!/usr/bin/env python3
import serial
import time
# import argparse # 移除 argparse
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
    # 短暫延遲確保發送，但在掃描時不宜過長
    time.sleep(0.005)

def check_servo_responds(ser, servo_id):
    """檢查特定ID的舵機是否有回應 (單次嘗試)"""
    ser.reset_input_buffer()
    # 使用讀取狀態命令 (0x02) 作為 Ping
    send_packet(ser, servo_id, 0x02, [])
    # 給予響應時間，掃描時不能太長
    time.sleep(0.05) # 稍微增加掃描時的等待時間

    if ser.in_waiting:
        response = ser.read(ser.in_waiting)
        # 簡單檢查是否有任何回應即可判斷存在
        if response and len(response) >= 6: # 基本 STBus 回應至少6字節
             return True
    return False

def scan_for_servos(ser, id_range=(1, 254)):
    """掃描指定範圍內的舵機ID"""
    found_ids = []
    print(f"正在掃描舵機 ID 從 {id_range[0]} 到 {id_range[1]-1}...")
    for servo_id in range(id_range[0], id_range[1]):
        print(f"  測試 ID: {servo_id}...", end='\r') # 實時顯示測試進度
        # *** 修正: 將 servo_id 傳遞給 check_servo_responds ***
        if check_servo_responds(ser, servo_id):
            print(f"  找到舵機 ID: {servo_id}   ") # 清除行尾並打印找到的ID
            found_ids.append(servo_id)
        # 稍微減慢掃描速度，避免匯流排擁堵
        time.sleep(0.01)
    print("\n掃描完成.")
    return found_ids

def set_id_direct(ser, current_id, new_id):
    """針對眾靈ZLink轉接板的舵機ID設定，嘗試多種協議"""
    print(f"正在嘗試修改舵機 ID: {current_id} → {new_id}")

    # 再次確認原ID舵機是否存在 (掃描後可能狀態改變)
    print(f"確認當前 ID {current_id} 是否響應...")
    # 使用更長的超時和多次嘗試來確認
    if not check_servo_responds_robust(ser, current_id):
        print(f"錯誤: ID {current_id} 的舵機在嘗試修改前已無響應。")
        return False
    print(f"ID {current_id} 響應正常，開始修改。")

    # 嘗試所有可能的ID修改方法 (保持不變)
    methods = [
        lambda: send_packet(ser, current_id, 0x03, [new_id]), # 標準 STBus
        lambda: send_packet(ser, current_id, 0x0D, [new_id]), # 樂博 LX-16A
        lambda: (send_packet(ser, current_id, 0x03, [new_id]), time.sleep(0.5), send_packet(ser, current_id, 0x06, [])), # 標準 + 存儲
        lambda: send_packet(ser, current_id, 0x01, [0xD4, 0x00, new_id]), # 寫 EEPROM (地址可能不對)
        # lambda: send_packet(ser, 0xFE, 0x03, [new_id, current_id]), # 廣播 (風險較高，暫不啟用)
    ]

    print("開始嘗試多種ID修改方式...")
    success = False
    for i, method in enumerate(methods):
        print(f"方法 {i+1}...")
        method()
        # 等待舵機處理，給予更長的時間
        time.sleep(1.5)

        # 確認新ID是否成功
        print(f"檢查新 ID {new_id} 是否響應...")
        if check_servo_responds_robust(ser, new_id): # 使用更可靠的檢查
            print(f"成功! 使用方法 {i+1} 修改 ID 成功 ({current_id} -> {new_id})")
            success = True
            break # 成功即跳出循環

        # 如果新ID不響應，檢查原ID是否還在 (避免誤判)
        print(f"檢查原 ID {current_id} 是否仍響應...")
        if check_servo_responds_robust(ser, current_id): # 使用更可靠的檢查
            print(f"方法 {i+1} 失敗，原 ID {current_id} 仍然響應。")
        else:
            print(f"警告: 方法 {i+1} 後，原 ID {current_id} 無響應，新 ID {new_id} 也無響應。")
            # 可以選擇在這裡加入額外的掃描來尋找丟失的舵機

    if not success:
        print(f"所有嘗試都失敗，無法將舵機 ID 從 {current_id} 修改為 {new_id}")
    return success

def check_servo_responds_robust(ser, servo_id, max_attempts=3, delay=0.2):
    """更可靠地檢查特定ID的舵機是否有回應 (多次嘗試)"""
    for attempt in range(max_attempts):
        ser.reset_input_buffer()
        send_packet(ser, servo_id, 0x02, [])  # 發送讀取狀態命令
        time.sleep(delay)  # 給予足夠響應時間

        if ser.in_waiting:
            response = ser.read(ser.in_waiting)
            # 檢查是否有有效回應 (長度通常 > 4)
            if response and len(response) > 4:
                # print(f"ID {servo_id} 在嘗試 {attempt+1} 時響應: {response.hex()}") # 調試用
                return True
        # print(f"ID {servo_id} 在嘗試 {attempt+1} 時無響應") # 調試用
    return False


if __name__ == "__main__":
    # --- 移除 argparse ---
    # parser = argparse.ArgumentParser(description="掃描並修改伺服馬達ID")
    # parser.add_argument("-p", "--port", default="/dev/usb_robot_arm", help="串口設備 (例如 /dev/ttyUSB0 或 /dev/usb_robot_arm)")
    # parser.add_argument("-b", "--baud", type=int, default=115200, help="串口波特率")
    # parser.add_argument("-r", "--range", type=int, nargs=2, default=[1, 31], metavar=('START', 'END'), help="掃描的ID範圍 (例如 1 31)") # 預設掃描 1 到 30
    # args = parser.parse_args()

    # --- 設定預設值 ---
    port = "/dev/usb_robot_arm" # 預設串口
    baud = 115200               # 預設波特率
    scan_range_list = [1, 31]   # 預設掃描範圍 (1 到 30)
    # --- 結束設定預設值 ---

    scan_range = tuple(scan_range_list) # 轉換為元組
    if not (1 <= scan_range[0] < scan_range[1] <= 254):
         print("錯誤: 預設掃描範圍無效。請檢查腳本中的 scan_range_list。")
         sys.exit(1)

    try:
        # 使用預設值變數 port 和 baud
        print(f"開啟串口 {port}, 波特率 {baud}")
        # 增加串口超時時間，以配合 check_servo_responds_robust 中的 delay
        with serial.Serial(port, baud, timeout=0.3) as ser:
            # 使用預設值變數 scan_range
            found_ids = scan_for_servos(ser, scan_range)

            if not found_ids:
                print("在指定範圍內未找到任何舵機。請檢查連接、電源和腳本中的預設掃描範圍。")
                sys.exit(1)

            print("\n找到的舵機 ID:", ", ".join(map(str, found_ids)))

            # 詢問要修改哪個ID (保持不變)
            current_id_to_change = -1
            while True:
                try:
                    id_input = input("請輸入您想要修改的舵機 ID (從上面列表中選擇): ")
                    current_id_to_change = int(id_input)
                    if current_id_to_change in found_ids:
                        break
                    else:
                        print("輸入的 ID 不在找到的列表中，請重新輸入。")
                except ValueError:
                    print("輸入無效，請輸入數字。")

            # 詢問新的ID (保持不變)
            new_id = -1
            while True:
                try:
                    id_input = input(f"請輸入 ID {current_id_to_change} 的新 ID (1-253): ")
                    new_id = int(id_input)
                    if 1 <= new_id <= 253:
                        if new_id == current_id_to_change:
                             print("新舊 ID 相同，無需修改。")
                             continue # 重新詢問新ID
                        other_found_ids = [id for id in found_ids if id != current_id_to_change]
                        if new_id in other_found_ids:
                            confirm = input(f"警告：ID {new_id} 已被列表中的其他舵機使用。確定要繼續嗎? (y/n): ").lower()
                            if confirm == 'y':
                                break
                            else:
                                print("操作取消。")
                                continue # 重新詢問新ID
                        else:
                            break # ID有效且不衝突
                    else:
                        print("輸入的 ID 不在有效範圍 (1-253) 內，請重新輸入。")
                except ValueError:
                    print("輸入無效，請輸入數字。")

            # 執行修改 (保持不變)
            result = set_id_direct(ser, current_id_to_change, new_id)

            if result:
                print("\nID 修改成功完成。")
                sys.exit(0)
            else:
                print("\nID 修改失敗。")
                sys.exit(1)

    except serial.SerialException as e:
        # 使用預設值變數 port
        print(f"串口錯誤: {e}")
        print(f"請確認裝置 {port} 是否已插入、驅動正常且有適當權限。")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n程式被中斷")
        sys.exit(1)