import time
import serial
from lewansoul_lx16a import ServoController
import sys
import curses # Import curses for keyboard input

# --- 設定 ---
STEP_SIZE = 10    # 每按一次上下鍵調整的原始位置量
MOVE_TIME = 100   # 移動命令的時間 (毫秒)，較小的值響應更快
RAW_TO_DEG = 240.0 / 1000.0 # 原始位置轉換為角度的係數 (新增)

# --- 串口初始化 ---
try:
    ser = serial.Serial('/dev/usb_robot_arm', 115200, timeout=0.1)
    controller = ServoController(ser)
    print(f"成功開啟串口 /dev/usb_robot_arm")
except serial.SerialException as e:
    print(f"無法開啟串口: {e}")
    sys.exit(1)

# --- 詢問使用者要操作哪個 ID ---
servo_id_to_adjust = None
current_pos_check = None # 用於存儲初始位置
while True:
    try:
        id_input = input("請輸入您想要用鍵盤調整的舵機 ID (1-253): ")
        selected_id = int(id_input)
        if 1 <= selected_id <= 253:
            print(f"正在嘗試與 ID {selected_id} 通信...")
            current_pos_check = controller.get_position(selected_id)
            if current_pos_check is not None:
                 # --- 修改：同時顯示初始角度 ---
                 initial_angle = current_pos_check * RAW_TO_DEG
                 print(f"成功連接到 ID {selected_id}。當前位置: {current_pos_check} (約 {initial_angle:.2f}°)")
                 # --- 修改結束 ---
                 servo_id_to_adjust = selected_id
                 break
            else:
                 confirm = input(f"警告：無法從 ID {selected_id} 獲取響應。您確定要繼續操作此 ID 嗎? (y/n): ").lower()
                 if confirm == 'y':
                     print(f"將繼續操作 ID {selected_id} (即使目前無響應)。")
                     servo_id_to_adjust = selected_id
                     # 嘗試讀取位置失敗，將目標位置初始化為中間值
                     current_pos_check = 500
                     break
                 else:
                     print("操作取消，請重新輸入 ID。")
        else:
            print("輸入的 ID 不在有效範圍 (1-253) 內，請重新輸入。")
    except ValueError:
        print("輸入無效，請輸入一個數字 ID。")
    except Exception as e:
        print(f"嘗試連接舵機時發生錯誤: {e}")
        ser.close()
        sys.exit(1)

# --- 鍵盤控制主函數 ---
def main(stdscr):
    # curses 初始化
    curses.curs_set(0) # 隱藏光標
    stdscr.nodelay(True) # 非阻塞讀取鍵盤
    stdscr.keypad(True)  # 啟用特殊鍵 (如方向鍵)
    stdscr.timeout(100) # 等待輸入的超時時間 (毫秒)

    # 使用之前讀取的 current_pos_check 作為初始目標位置
    target_pos = current_pos_check
    # 如果初始讀取失敗 (current_pos_check 會是 500)
    if controller.get_position(servo_id_to_adjust) is None and target_pos == 500:
         stdscr.addstr(0, 0, f"警告：無法讀取初始位置，從 {target_pos} 開始。")
    else:
         initial_angle = target_pos * RAW_TO_DEG
         stdscr.addstr(0, 0, f"舵機 ID: {servo_id_to_adjust} | 初始位置: {target_pos} (約 {initial_angle:.2f}°)")

    stdscr.addstr(1, 0, "使用 [↑] / [↓] 調整位置 | 按 [q] 退出")
    # --- 修改：初始顯示也包含角度 ---
    current_angle = target_pos * RAW_TO_DEG
    stdscr.addstr(2, 0, f"當前目標: {target_pos:4d} (約 {current_angle:6.2f}°) ")
    # --- 修改結束 ---
    stdscr.refresh()

    last_sent_pos = target_pos # 記錄上次發送的位置

    while True:
        try:
            key = stdscr.getch() # 獲取按鍵，非阻塞

            new_target_pos = target_pos # 假設目標位置不變

            if key == curses.KEY_UP:
                new_target_pos += STEP_SIZE
            elif key == curses.KEY_DOWN:
                new_target_pos -= STEP_SIZE
            elif key == ord('q'): # 按 'q' 退出
                break

            # 限制目標位置在 0 到 1000 之間
            new_target_pos = max(0, min(1000, new_target_pos))

            # 如果目標位置改變，則更新顯示並發送命令
            if new_target_pos != target_pos:
                target_pos = new_target_pos
                # --- 修改：更新顯示時包含角度 ---
                current_angle = target_pos * RAW_TO_DEG
                stdscr.addstr(2, 0, f"當前目標: {target_pos:4d} (約 {current_angle:6.2f}°) ") # 更新顯示 (加空格清除舊內容)
                # --- 修改結束 ---
                stdscr.refresh()

            # 只有當目標位置與上次發送的位置不同時才發送移動命令
            if target_pos != last_sent_pos:
                 controller.move(servo_id_to_adjust, position=target_pos, time=MOVE_TIME)
                 last_sent_pos = target_pos

            # 短暫延遲，避免 CPU 占用過高
            time.sleep(0.02)

        except Exception as e:
            curses.endwin()
            print(f"\n控制循環中發生錯誤: {e}")
            return

# --- 執行主邏輯並確保 curses 正確退出 ---
try:
    curses.wrapper(main)
    print("\n程式已退出。")
except Exception as e:
    print(f"\n啟動或運行 curses 時發生錯誤: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("串口已關閉")