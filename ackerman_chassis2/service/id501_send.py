import can
import time

bus = can.interface.Bus(channel='can0', interface ='socketcan')

data = bytes.fromhex('2F00000004000000')
msg = can.Message(arbitration_id=0x501, data=data, is_extended_id=False)

try:
    while True:
        bus.send(msg)
        time.sleep(0.02)  # 20ms
except KeyboardInterrupt:
    print("Stopped.")
