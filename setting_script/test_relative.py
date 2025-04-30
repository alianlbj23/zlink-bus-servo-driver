import time
import serial
from lewansoul_lx16a import ServoController

# 串口初始化
# 請確保 '/dev/usb_robot_arm' 是正確的串口設備名稱
try:
    ser = serial.Serial('/dev/usb_robot_arm', 115200, timeout=0.1)
    controller = ServoController(ser)
except serial.SerialException as e:
    print(f"無法開啟串口: {e}")
    exit()

servo_id = 1      # 要操作的舵機 ID
interval = 0.1    # 每隔 0.1 秒读取一次

print(f"正在使用舵機 ID: {servo_id}")

# 讓使用者輸入定義為 0 度的 Raw Position
while True:
    try:
        zero_pos_input = input("請輸入定義為 0 度的 Raw Position (0-1000): ")
        zero_pos = int(zero_pos_input)
        if 0 <= zero_pos <= 1000:
            print(f"已設定 Raw pos {zero_pos} 為 0°")
            break
        else:
            print("輸入無效，請輸入 0 到 1000 之間的整數。")
    except ValueError:
        print("輸入無效，請輸入一個整數。")

# 讓使用者決定 Raw Position 增加的方向
while True:
    direction_input = input("當 Raw Position 增加時，代表 'cw' (順時針) 還是 'ccw' (逆時針)? ").lower()
    if direction_input in ['cw', 'ccw']:
        pos_increase_is_clockwise = (direction_input == 'cw')
        if pos_increase_is_clockwise:
            print("已設定 Raw Position 增加代表順時針。")
        else:
            print("已設定 Raw Position 增加代表逆時針。")
        break
    else:
        print("輸入無效，請輸入 'cw' 或 'ccw'。")


print(f"\n開始實時讀取舵機相對角度 (相對於 Raw pos {zero_pos} = 0°)，按 Ctrl+C 停止")

try:
    while True:
        try:
            # 讀取當前原始位置 (0~1000)
            current_pos = controller.get_position(servo_id)

            if current_pos is not None:
                # 計算相對於使用者定義零點的位置差
                pos_diff = current_pos - zero_pos

                # --- 修改開始 ---
                # 計算角度差的 *絕對值* (永遠從 0 開始遞增)
                abs_angle_diff = abs(pos_diff * 240.0 / 1000.0)

                # 根據使用者定義和實際位置差判斷 *方向標籤*
                direction = "靜止"
                if pos_diff != 0: # 只有在移動時才判斷方向
                    if pos_increase_is_clockwise: # 使用者定義 raw pos 增加為 CW
                        direction = "順時針" if pos_diff > 0 else "逆時針"
                    else: # 使用者定義 raw pos 增加為 CCW
                        direction = "逆時針" if pos_diff > 0 else "順時針"
                # --- 修改結束 ---

                # 輸出: 顯示絕對角度差 (從0遞增) 和方向標籤
                # 注意 abs_angle_diff 前面移除了正負號格式 '+'
                print(f"[ID={servo_id}] Current Raw pos = {current_pos:4d}, Relative Angle ≈ {abs_angle_diff:7.2f}°, Direction: {direction}")
            else:
                print(f"[ID={servo_id}] 讀取錯誤: 未收到舵機響應")

        except Exception as e:
            print(f"讀取或計算錯誤: {e}")

        time.sleep(interval)

except KeyboardInterrupt:
    print("\n用戶已停止循環")
except Exception as e:
    print(f"發生未預期的錯誤: {e}")
finally:
    if ser.is_open:
        ser.close()
        print("串口已關閉")