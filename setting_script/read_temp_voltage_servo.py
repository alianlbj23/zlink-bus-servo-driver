import serial
import time

def checksum(data):
    return (~sum(data) & 0xFF)

class BusServo:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)

    def read_temperature(self, sid):
        # CMD: SERVO_TEMP_READ = 0x1A
        packet = [0x55, 0x55, 0x03, sid, 0x1A]
        packet.append(checksum(packet[2:]))
        self.ser.write(bytes(packet))
        resp = self.ser.read(7)
        if len(resp) >= 7:
            return resp[5]  # °C
        return None

    def read_voltage(self, sid):
        # CMD: SERVO_VIN_READ = 0x1B
        packet = [0x55, 0x55, 0x03, sid, 0x1B]
        packet.append(checksum(packet[2:]))
        self.ser.write(bytes(packet))
        resp = self.ser.read(8)
        if len(resp) >= 8:
            vin = resp[5] | (resp[6] << 8)  # mV
            return vin / 1000.0
        return None


if __name__ == "__main__":
    servo = BusServo("/dev/usb_robot_arm")  # 換成你的串口
    servo_ids = [1, 2, 3, 4]          # 這裡填上實際有的伺服 ID

    while True:
        print("==== Servo Status ====")
        for sid in servo_ids:
            temp = servo.read_temperature(sid)
            vin = servo.read_voltage(sid)

            if temp is not None and vin is not None:
                print(f"ID {sid}: {temp} °C, {vin:.2f} V")
            else:
                print(f"ID {sid}: [讀取失敗]")
        print()
        time.sleep(1)  # 每秒更新一次
