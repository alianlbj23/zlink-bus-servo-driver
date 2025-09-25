#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time

def checksum(data):
    return (~sum(data) & 0xFF) & 0xFF

def make_cmd(sid, cmd):
    pkt = [0x55, 0x55, 0x03, sid, cmd]
    pkt.append(checksum(pkt[2:]))
    return bytes(pkt)

class BusServo:
    CMD_TEMP_READ = 0x1A
    CMD_VIN_READ  = 0x1B

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.2)

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
            return vin / 1000.0  # V
        return None


if __name__ == "__main__":
    LAST_ID = 3   # 這裡換成最後一顆馬達的 ID
    servo = BusServo("/dev/usb_robot_arm", 115200)

    while True:
        temp = servo.read_temperature(LAST_ID)
        vin  = servo.read_voltage(LAST_ID)

        if temp is not None and vin is not None:
            print(f"Servo {LAST_ID}: {temp} °C, {vin:.2f} V")
        else:
            print(f"Servo {LAST_ID}: [讀取失敗]")

        time.sleep(1)  # 每秒更新一次
