import serial, time
PORT = "/dev/ttyACM0"
ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(1.5)
for _ in range(8):
    if ser.in_waiting:
        print(ser.readline().decode(errors="ignore").strip())
    time.sleep(0.15)
ser.close()
