import time
import serial
from lewansoul_lx16a import ServoController
import sys

# --- 設定 ---
# target_pos = 0    # 移除硬編碼的目標位置，將由使用者輸入
move_time = 1000  # 移動到目標位置所需的時間 (毫秒)，可以根據需要調整
RAW_TO_DEG = 240.0 / 1000.0 # 原始位置轉換為角度的係數

# --- 串口初始化 ---
try:
    # 確保使用正確的串口設備名稱
    ser = serial.Serial('/dev/usb_robot_arm', 115200, timeout=0.1)
    controller = ServoController(ser)
    print(f"成功開啟串口 /dev/usb_robot_arm")
except serial.SerialException as e:
    print(f"無法開啟串口: {e}")
    sys.exit(1)

# --- 詢問使用者要操作哪個 ID ---
servo_id_to_move = None
while True:
    try:
        id_input = input("請輸入您想要移動的舵機 ID (1-253): ")
        selected_id = int(id_input)
        if 1 <= selected_id <= 253:
            print(f"正在嘗試與 ID {selected_id} 通信...")
            if controller.get_position(selected_id) is not None:
                 print(f"成功連接到 ID {selected_id}。")
                 servo_id_to_move = selected_id
                 break
            else:
                 confirm = input(f"警告：無法從 ID {selected_id} 獲取響應。您確定要繼續操作此 ID 嗎? (y/n): ").lower()
                 if confirm == 'y':
                     print(f"將繼續操作 ID {selected_id} (即使目前無響應)。")
                     servo_id_to_move = selected_id
                     break
                 else:
                     print("操作取消，請重新輸入 ID。")
        else:
            print("輸入的 ID 不在有效範圍 (1-253) 內，請重新輸入。")
    except ValueError:
        print("輸入無效，請輸入一個數字 ID。")
    except Exception as e:
        print(f"嘗試連接舵機時發生錯誤: {e}")

# --- 詢問使用者要移動到哪個位置 ---
target_pos = None
while True:
    try:
        pos_input = input(f"請輸入您想讓舵機 ID {servo_id_to_move} 移動到的目標原始位置 (0-1000): ")
        selected_pos = int(pos_input)
        if 0 <= selected_pos <= 1000:
            target_pos = selected_pos
            # 同時計算目標角度以供參考
            target_angle = target_pos * RAW_TO_DEG
            print(f"目標原始位置: {target_pos} (約 {target_angle:.2f}°)")
            break
        else:
            print("輸入的位置不在有效範圍 (0-1000) 內，請重新輸入。")
    except ValueError:
        print("輸入無效，請輸入一個數字位置。")

# --- 發送移動命令 (使用使用者輸入的 servo_id_to_move 和 target_pos) ---
try:
    print(f"\n正在命令舵機 ID {servo_id_to_move} 移動到原始位置 {target_pos}...")
    # 使用 move 命令舵機在指定時間內移動到目標位置
    controller.move(servo_id_to_move, position=target_pos, time=move_time)
    print(f"移動命令已發送。舵機應在 {move_time / 1000.0:.1f} 秒內到達位置 {target_pos}。")

    # 等待舵機完成移動 (可選，但建議)
    print("等待舵機移動...")
    time.sleep(move_time / 1000.0 + 0.2) # 等待移動時間再加上一點緩衝

    # 可選：讀取當前位置確認是否到達
    print("讀取最終位置...")
    final_pos = controller.get_position(servo_id_to_move)
    if final_pos is not None:
        # --- 修改：計算並打印角度 ---
        final_angle = final_pos * RAW_TO_DEG
        print(f"舵機 ID {servo_id_to_move} 當前位置: {final_pos} (約 {final_angle:.2f}°)")
        # --- 修改結束 ---
        if abs(final_pos - target_pos) <= 10: # 允許一點誤差
             print("舵機已到達目標位置附近。")
        else:
             print(f"警告：舵機當前位置 {final_pos} 與目標 {target_pos} 相差較大。")
    else:
        print("無法讀取舵機的最終位置。")

except Exception as e:
    print(f"發送移動命令或讀取位置時發生錯誤: {e}")

finally:
    # --- 關閉串口 ---
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("串口已關閉")