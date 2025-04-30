#!/usr/bin/env python3
import serial
import time
import logging
import os

# 設置日誌格式
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)


def read_devices_from_txt(file_path):
    """Read device paths from text file, one per line"""
    devices = []
    try:
        with open(file_path, "r") as f:
            for line in f:
                # Strip whitespace and skip empty lines or comments
                line = line.strip()
                if line and not line.startswith("#"):
                    devices.append(line)
        return devices
    except Exception as e:
        logging.error(f"Failed to read file {file_path}: {e}")
        return []


def reset_esp32(device):
    """嘗試對 ESP32 進行 DTR 重置"""
    try:
        with serial.Serial(device) as ser:
            logging.info(f"Opened {device}")

            ser.dtr = False
            time.sleep(0.1)

            ser.dtr = True
            time.sleep(1)

            logging.info(f"ESP32 reset complete on {device}!")

    except serial.SerialException as e:
        logging.error(f"Failed to open {device}: {e}")
    except Exception as e:
        logging.error(f"Unexpected error with {device}: {e}")


if __name__ == "__main__":
    # Get the directory of the script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    txt_path = os.path.join(script_dir, "devices.txt")

    devices = read_devices_from_txt(txt_path)
    if not devices:
        logging.error("No devices found in the text file!")
    else:
        for dev in devices:
            reset_esp32(dev)
