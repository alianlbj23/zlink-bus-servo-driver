#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import sys
import os
from lewansoul_lx16a import ServoController

# Non-blocking keyboard reading (Linux/macOS terminal)
if os.name != "nt":
    import termios, tty, select

def kb_hit():
    if os.name == "nt":
        return False
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(dr)

def getch_nonblock():
    if os.name == "nt":
        return None
    if not kb_hit():
        return None
    return sys.stdin.read(1)

def checksum(data):
    return (~sum(data) & 0xFF) & 0xFF

def make_cmd(sid, cmd):
    pkt = [0x55, 0x55, 0x03, sid, cmd]
    pkt.append(checksum(pkt[2:]))
    return bytes(pkt)

class BusServo:
    CMD_TEMP_READ = 0x1A
    CMD_VIN_READ  = 0x1B

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.2):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.controller = ServoController(self.ser)

    def _query(self, sid, cmd, expect_len):
        self.ser.reset_input_buffer()
        self.ser.write(make_cmd(sid, cmd))
        time.sleep(0.02)
        resp = self.ser.read(expect_len)
        return resp if len(resp) >= expect_len else None

    def read_temperature(self, sid):
        resp = self._query(sid, self.CMD_TEMP_READ, 7)
        if resp:
            return resp[5]
        return None

    def read_voltage(self, sid):
        resp = self._query(sid, self.CMD_VIN_READ, 8)
        if resp:
            vin = resp[5] | (resp[6] << 8)
            return vin / 1000.0
        return None

    def read_position(self, sid):
        """Read servo position in degrees (0-360)"""
        try:
            # Use lewansoul_lx16a library to get position
            raw_pos = self.controller.get_position(sid)
            if raw_pos is not None:
                # LX-16A servo: raw position 0-1000 maps to 0-240 degrees
                # Convert to 0-240 degree range first
                angle_240 = (raw_pos / 1000.0) * 240.0
                
                # Map 0-240 to 0-360 if you want full circle range
                # Or keep 0-240 if that's the actual servo range
                angle = angle_240  # Keep original 0-240 range
                
                return angle
            return None
        except Exception as e:
            # Fallback to manual position reading if library fails
            return self._read_position_manual(sid)
    
    def _read_position_manual(self, sid):
        """Manual position reading using direct serial commands"""
        resp = self._query(sid, 0x02, 8)  # CMD_POS_READ = 0x02
        if resp and len(resp) >= 8:
            # Position is in bytes 6 and 7 (low byte first)
            pos = resp[6] | (resp[7] << 8)
            # Convert from 0-1000 range to 0-240 degrees
            angle = (pos / 1000.0) * 240.0
            return angle
        return None


if __name__ == "__main__":
    LAST_ID = 3
    PORT = "/dev/usb_robot_arm"

    servo = BusServo(PORT, 115200)
    times, temps, volts, angles = [], [], [], []
    t0 = time.time()

    print("Starting monitoring, press 'q' or Ctrl+C to exit and generate charts")

    old_settings = None
    if os.name != "nt":
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    try:
        while True:
            now = time.time() - t0
            temp = servo.read_temperature(LAST_ID)
            vin  = servo.read_voltage(LAST_ID)
            angle = servo.read_position(LAST_ID)

            times.append(now)
            temps.append(temp if temp is not None else float("nan"))
            volts.append(vin if vin is not None else float("nan"))
            angles.append(angle if angle is not None else float("nan"))

            if temp is not None and vin is not None and angle is not None:
                print(f"Servo {LAST_ID}: {temp} °C, {vin:.2f} V, {angle:.1f}°")
            else:
                print(f"Servo {LAST_ID}: [Read failed]")

            ch = getch_nonblock()
            if ch is not None and ch.lower() == "q":
                break

            time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        if os.name != "nt" and old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # ===== Generate charts after exit =====
    try:
        import matplotlib.pyplot as plt
        
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        
        # First chart: Time vs Temperature
        fig1, ax1 = plt.subplots(figsize=(10, 6))
        ax1.plot(times, temps, "r-", linewidth=2, marker='o', markersize=3)
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Temperature (°C)")
        ax1.grid(True, linestyle="--", alpha=0.5)
        ax1.set_title(f"Servo {LAST_ID} Temperature vs Time")
        fig1.tight_layout()
        
        filename1 = f"servo_{LAST_ID}_temperature_{timestamp}.png"
        plt.savefig(filename1, dpi=300, bbox_inches='tight')
        print(f"Temperature chart saved as: {filename1}")
        plt.close()
        
        # Second chart: Time vs Voltage
        fig2, ax2 = plt.subplots(figsize=(10, 6))
        ax2.plot(times, volts, "b-", linewidth=2, marker='s', markersize=3)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Voltage (V)")
        ax2.grid(True, linestyle="--", alpha=0.5)
        ax2.set_title(f"Servo {LAST_ID} Voltage vs Time")
        fig2.tight_layout()
        
        filename2 = f"servo_{LAST_ID}_voltage_{timestamp}.png"
        plt.savefig(filename2, dpi=300, bbox_inches='tight')
        print(f"Voltage chart saved as: {filename2}")
        plt.close()
        
        # Third chart: Time vs Angle
        fig3, ax3 = plt.subplots(figsize=(10, 6))
        ax3.plot(times, angles, "g-", linewidth=2, marker='^', markersize=3)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Angle (degrees)")
        ax3.grid(True, linestyle="--", alpha=0.5)
        ax3.set_title(f"Servo {LAST_ID} Angle vs Time")
        fig3.tight_layout()
        
        filename3 = f"servo_{LAST_ID}_angle_{timestamp}.png"
        plt.savefig(filename3, dpi=300, bbox_inches='tight')
        print(f"Angle chart saved as: {filename3}")
        plt.close()
        
    except ImportError:
        print("Please install matplotlib first: pip install matplotlib")
