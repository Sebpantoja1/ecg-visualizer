import serial
from PyQt6.QtCore import QThread, pyqtSignal
import time


class SerialService(QThread):
    data_received = pyqtSignal(int)  # Signal emitted with ADC value

    def __init__(self, port='COM3', baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.serial_conn = None

    def run(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            buffer = bytearray()
            while self.running:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(data)
                    self.process_buffer(buffer)
                time.sleep(0.001)  # Small delay to prevent CPU hogging
        except Exception as e:
            print(f"Serial error: {e}")
        finally:
            if self.serial_conn:
                self.serial_conn.close()

    def process_buffer(self, buffer):
        # Packet format: [start_byte, lsb, msb, checksum]
        # checksum = start ^ lsb ^ msb
        DATA_START_BYTE = 0xAA  # Assuming from ESP32 code, need to check
        PACKET_SIZE = 4

        while len(buffer) >= PACKET_SIZE:
            if buffer[0] == DATA_START_BYTE:
                packet = buffer[:PACKET_SIZE]
                checksum = packet[0] ^ packet[1] ^ packet[2]
                if checksum == packet[3]:
                    adc_value = (packet[2] << 8) | packet[1]
                    self.data_received.emit(adc_value)
                    buffer[:] = buffer[PACKET_SIZE:]
                else:
                    # Invalid checksum, shift by 1
                    buffer.pop(0)
            else:
                # Not start byte, shift by 1
                buffer.pop(0)

    def stop(self):
        self.running = False
        self.wait()